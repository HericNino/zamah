import { BroadPhase } from './collision/broadphase.ts';
import type { Shape } from './collision/shape.ts';
import { Body, type BodyDef } from './dynamics/body.ts';
import { Contact, pairKey } from './dynamics/contact.ts';
import {
  prepareContacts,
  refreshContacts,
  solveContacts,
  solveImpacts,
  warmStartContacts,
} from './dynamics/contactSolver.ts';
import { type Joint, makeSoft, type StepContext } from './dynamics/joint.ts';
import { DistanceJoint, type DistanceJointDef } from './dynamics/joints/distance.ts';
import { MouseJoint, type MouseJointDef } from './dynamics/joints/mouse.ts';
import { RevoluteJoint, type RevoluteJointDef } from './dynamics/joints/revolute.ts';
import { StateHash } from './math/hash.ts';
import { cos, sin } from './math/trig.ts';
import type { Vec2 } from './math/vec2.ts';
import { Fluid } from './particles/fluid.ts';
import { SoftSystem } from './particles/soft.ts';
import {
  MAX_LINEAR_SPEED,
  MAX_ROTATION_PER_SUBSTEP,
  SLEEP_ANGULAR_SPEED,
  SLEEP_LINEAR_SPEED,
  SPECULATIVE_DISTANCE,
  TIME_TO_SLEEP,
} from './settings.ts';

export interface WorldOptions {
  gravity?: Vec2;
  /** Fixed time step in seconds. Every call to step() advances the world by exactly this much. */
  dt?: number;
  /** Solver sub-steps per step. More sub-steps make stacks and joints stiffer. */
  substeps?: number;
  /** Stiffness of the soft contact constraint, in hertz. */
  contactHertz?: number;
  contactDampingRatio?: number;
  jointHertz?: number;
  jointDampingRatio?: number;
  /** Fastest speed at which overlap is pushed out, m/s. */
  pushVelocity?: number;
  /** Impacts slower than this do not bounce, which lets bouncy things come to rest. */
  restitutionThreshold?: number;
  warmStarting?: boolean;
  sleeping?: boolean;
}

/** Milliseconds spent in each phase of the last step. */
export interface Profile {
  broadphase: number;
  narrowphase: number;
  solve: number;
  soft: number;
  fluid: number;
  total: number;
}

const now = (): number => performance.now();

/**
 * The simulation. Owns the bodies, joints and contacts, plus the two particle systems, and
 * advances all of them by one fixed time step per call to step().
 *
 * Nothing in here touches the DOM, the wall clock (other than for profiling) or Math.random,
 * so the same sequence of calls always produces the same state, in a browser or in Node.
 */
export class World {
  gravityX: number;
  gravityY: number;
  readonly dt: number;
  substeps: number;
  contactHertz: number;
  contactDampingRatio: number;
  jointHertz: number;
  jointDampingRatio: number;
  pushVelocity: number;
  restitutionThreshold: number;
  warmStarting: boolean;
  sleeping: boolean;

  readonly bodies: Body[] = [];
  readonly joints: Joint[] = [];
  readonly contacts = new Map<number, Contact>();
  readonly broadphase = new BroadPhase();

  /** Cloth, ropes and soft bodies (XPBD). */
  readonly soft: SoftSystem;
  /** Liquid (position based fluids). */
  readonly fluid: Fluid;

  /** The contacts the solver worked on in the last step; exposed for debug drawing. */
  activeContacts: Contact[] = [];
  stepCount = 0;
  readonly profile: Profile = { broadphase: 0, narrowphase: 0, solve: 0, soft: 0, fluid: 0, total: 0 };

  private awakeBodies: Body[] = [];
  private activeJoints: Joint[] = [];
  private impactContacts: Contact[] = [];
  private deadContacts: Contact[] = [];
  private islandParent = new Int32Array(64);
  private islandFlag = new Uint8Array(64);
  private islandSleep = new Float64Array(64);

  constructor(options: WorldOptions = {}) {
    this.gravityX = options.gravity?.x ?? 0;
    this.gravityY = options.gravity?.y ?? -10;
    this.dt = options.dt ?? 1 / 60;
    this.substeps = options.substeps ?? 4;
    this.contactHertz = options.contactHertz ?? 30;
    this.contactDampingRatio = options.contactDampingRatio ?? 10;
    this.jointHertz = options.jointHertz ?? 60;
    this.jointDampingRatio = options.jointDampingRatio ?? 2;
    this.pushVelocity = options.pushVelocity ?? 3;
    this.restitutionThreshold = options.restitutionThreshold ?? 1;
    this.warmStarting = options.warmStarting ?? true;
    this.sleeping = options.sleeping ?? true;
    this.soft = new SoftSystem();
    this.fluid = new Fluid();
  }

  // ---- construction ----------------------------------------------------------------------------

  createBody(def: BodyDef): Body {
    const body = new Body(def);
    this.bodies.push(body);
    for (const shape of body.shapes) this.broadphase.createProxy(shape);
    return body;
  }

  destroyBody(body: Body): void {
    const index = this.bodies.indexOf(body);
    if (index < 0) return;
    for (const joint of body.joints.slice()) this.destroyJoint(joint);
    for (const contact of this.contacts.values()) {
      if (contact.bodyA !== body && contact.bodyB !== body) continue;
      // Whatever was resting on this body must notice that it is gone.
      contact.bodyA.wake();
      contact.bodyB.wake();
      this.deadContacts.push(contact);
    }
    for (const contact of this.deadContacts) this.contacts.delete(contact.key);
    this.deadContacts.length = 0;
    for (const shape of body.shapes) this.broadphase.destroyProxy(shape);
    this.bodies.splice(index, 1);
  }

  createRevoluteJoint(def: RevoluteJointDef): RevoluteJoint {
    return this.addJoint(new RevoluteJoint(def));
  }

  createDistanceJoint(def: DistanceJointDef): DistanceJoint {
    return this.addJoint(new DistanceJoint(def));
  }

  createMouseJoint(def: MouseJointDef): MouseJoint {
    return this.addJoint(new MouseJoint(def));
  }

  destroyJoint(joint: Joint): void {
    const index = this.joints.indexOf(joint);
    if (index < 0) return;
    this.joints.splice(index, 1);
    for (const body of joint.bodyA === joint.bodyB ? [joint.bodyA] : [joint.bodyA, joint.bodyB]) {
      body.joints.splice(body.joints.indexOf(joint), 1);
      body.wake();
      // The pair may have been filtered out because of this joint; let it be found again.
      for (const shape of body.shapes) this.broadphase.touch(shape);
    }
  }

  private addJoint<J extends Joint>(joint: J): J {
    this.joints.push(joint);
    joint.bodyA.joints.push(joint);
    if (joint.bodyB !== joint.bodyA) joint.bodyB.joints.push(joint);
    joint.bodyA.wake();
    joint.bodyB.wake();
    if (!joint.collideConnected) {
      for (const contact of this.contacts.values()) {
        const linked =
          (contact.bodyA === joint.bodyA && contact.bodyB === joint.bodyB) ||
          (contact.bodyA === joint.bodyB && contact.bodyB === joint.bodyA);
        if (linked) this.deadContacts.push(contact);
      }
      for (const contact of this.deadContacts) this.contacts.delete(contact.key);
      this.deadContacts.length = 0;
    }
    return joint;
  }

  setGravity(x: number, y: number): void {
    this.gravityX = x;
    this.gravityY = y;
    for (const body of this.bodies) body.wake();
  }

  // ---- queries ---------------------------------------------------------------------------------

  /** Visits every shape whose (inflated) bounding box overlaps the given box. */
  queryAABB(minX: number, minY: number, maxX: number, maxY: number, visit: (shape: Shape) => boolean | void): void {
    const tree = this.broadphase.tree;
    tree.query(minX, minY, maxX, maxY, (id) => visit(tree.getData(id)));
  }

  /** The shape under a world point, preferring dynamic bodies over static ones. */
  shapeAt(x: number, y: number): Shape | null {
    let found: Shape | null = null;
    this.queryAABB(x, y, x, y, (shape) => {
      if (!shape.containsPoint(x, y)) return;
      if (found === null || (shape.body.isDynamic && !found.body.isDynamic)) found = shape;
    });
    return found;
  }

  kineticEnergy(): number {
    let energy = 0;
    for (const body of this.bodies) energy += body.kineticEnergy();
    return energy;
  }

  /** Gravitational potential energy of the rigid bodies, relative to the origin. */
  potentialEnergy(): number {
    let energy = 0;
    for (const body of this.bodies) {
      energy -= body.mass * body.gravityScale * (this.gravityX * body.x + this.gravityY * body.y);
    }
    return energy;
  }

  /**
   * A fingerprint of the complete dynamic state, down to the last bit of every float.
   * Two worlds with the same hash are in exactly the same state.
   */
  hash(): string {
    const hash = new StateHash();
    for (const body of this.bodies) {
      hash.add(body.x);
      hash.add(body.y);
      hash.add(body.c);
      hash.add(body.s);
      hash.add(body.vx);
      hash.add(body.vy);
      hash.add(body.w);
    }
    this.soft.hashInto(hash);
    this.fluid.hashInto(hash);
    return hash.digest();
  }

  // ---- stepping --------------------------------------------------------------------------------

  /** Advances the world by `dt`. */
  step(): void {
    const t0 = now();
    this.applyPendingImpulses();
    this.syncBroadphase();
    this.broadphase.updatePairs(this.addPair);
    const t1 = now();
    this.updateContacts();
    const t2 = now();
    this.wakeIslands();
    this.solve();
    this.finish();
    const t3 = now();
    this.soft.step(this);
    const t4 = now();
    this.fluid.step(this);
    const t5 = now();
    this.stepCount++;

    const profile = this.profile;
    profile.broadphase = t1 - t0;
    profile.narrowphase = t2 - t1;
    profile.solve = t3 - t2;
    profile.soft = t4 - t3;
    profile.fluid = t5 - t4;
    profile.total = t5 - t0;
  }

  /**
   * Particle systems push awake bodies directly, but on a sleeping body they only keep score.
   * If the total would have moved it noticeably, it wakes up and feels them from now on.
   */
  private applyPendingImpulses(): void {
    for (const body of this.bodies) {
      if (body.pendingX === 0 && body.pendingY === 0 && body.pendingAngular === 0) continue;
      if (body.isDynamic && !body.awake) {
        const dvx = body.invMass * body.pendingX;
        const dvy = body.invMass * body.pendingY;
        const dw = body.invInertia * body.pendingAngular;
        if (
          dvx * dvx + dvy * dvy > SLEEP_LINEAR_SPEED * SLEEP_LINEAR_SPEED ||
          dw * dw > SLEEP_ANGULAR_SPEED * SLEEP_ANGULAR_SPEED
        ) {
          body.wake();
        }
      }
      body.pendingX = body.pendingY = body.pendingAngular = 0;
    }
  }

  private syncBroadphase(): void {
    const dt = this.dt;
    for (const body of this.bodies) {
      if (!body.awake) continue;
      for (const shape of body.shapes) this.broadphase.moveProxy(shape, body.vx * dt, body.vy * dt);
    }
  }

  private addPair = (a: Shape, b: Shape): void => {
    const A = a.body;
    const B = b.body;
    if (A === B) return;
    if (!A.isDynamic && !B.isDynamic) return;
    if ((a.mask & b.category) === 0 || (b.mask & a.category) === 0) return;
    const key = pairKey(a, b);
    if (this.contacts.has(key)) return;
    const joints = A.joints.length <= B.joints.length ? A.joints : B.joints;
    for (const joint of joints) {
      if (joint.collideConnected) continue;
      if ((joint.bodyA === A && joint.bodyB === B) || (joint.bodyA === B && joint.bodyB === A)) return;
    }
    this.contacts.set(key, new Contact(a, b));
  };

  /** Narrowphase: recomputes the manifold of every contact that has an awake body. */
  private updateContacts(): void {
    const dt = this.dt;
    const dead = this.deadContacts;
    for (const contact of this.contacts.values()) {
      const A = contact.bodyA;
      const B = contact.bodyB;
      if (!A.awake && !B.awake) continue;
      if (!this.broadphase.overlaps(contact.shapeA, contact.shapeB)) {
        dead.push(contact);
        continue;
      }
      // Speculative margin grows with closing speed, so fast bodies get a contact a step
      // before they would otherwise tunnel into each other.
      const rvx = B.vx - A.vx;
      const rvy = B.vy - A.vy;
      const speed =
        Math.sqrt(rvx * rvx + rvy * rvy) +
        Math.abs(A.w) * contact.shapeA.boundRadius +
        Math.abs(B.w) * contact.shapeB.boundRadius;
      contact.update(SPECULATIVE_DISTANCE + dt * speed);
    }
    for (const contact of dead) this.contacts.delete(contact.key);
    dead.length = 0;
  }

  /**
   * Groups bodies into islands (sets connected by touching contacts or joints) with a
   * union-find, then wakes every island that contains at least one awake body.
   */
  private wakeIslands(): void {
    const bodies = this.bodies;
    const count = bodies.length;
    if (this.islandParent.length < count) {
      const size = Math.max(count, this.islandParent.length * 2);
      this.islandParent = new Int32Array(size);
      this.islandFlag = new Uint8Array(size);
      this.islandSleep = new Float64Array(size);
    }
    const parent = this.islandParent;
    const flag = this.islandFlag;
    for (let i = 0; i < count; i++) {
      bodies[i].islandIndex = i;
      parent[i] = i;
      flag[i] = 0;
    }
    for (const contact of this.contacts.values()) {
      if (contact.manifold.count === 0) continue;
      if (contact.bodyA.isDynamic && contact.bodyB.isDynamic) {
        this.union(contact.bodyA.islandIndex, contact.bodyB.islandIndex);
      }
    }
    for (const joint of this.joints) {
      if (joint.bodyA.isDynamic && joint.bodyB.isDynamic) {
        this.union(joint.bodyA.islandIndex, joint.bodyB.islandIndex);
      }
    }
    for (let i = 0; i < count; i++) if (bodies[i].awake) flag[this.find(i)] = 1;
    for (let i = 0; i < count; i++) {
      const body = bodies[i];
      // Woken by a neighbour: becomes awake but keeps its sleep timer, as its own motion has
      // not changed yet.
      if (!body.awake && body.isDynamic && flag[this.find(i)]) body.awake = true;
    }
  }

  private find(i: number): number {
    const parent = this.islandParent;
    while (parent[i] !== i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  }

  private union(a: number, b: number): void {
    a = this.find(a);
    b = this.find(b);
    if (a === b) return;
    if (a < b) this.islandParent[b] = a;
    else this.islandParent[a] = b;
  }

  private solve(): void {
    const awake = this.awakeBodies;
    awake.length = 0;
    for (const body of this.bodies) {
      if (!body.awake) continue;
      body.prevX = body.x;
      body.prevY = body.y;
      body.prevC = body.c;
      body.prevS = body.s;
      awake.push(body);
    }

    const joints = this.activeJoints;
    joints.length = 0;
    for (const joint of this.joints) if (joint.bodyA.awake || joint.bodyB.awake) joints.push(joint);

    const contacts = this.activeContacts;
    const impacts = this.impactContacts;
    contacts.length = 0;
    impacts.length = 0;
    for (const contact of this.contacts.values()) {
      if (contact.manifold.count === 0) continue;
      if (!contact.bodyA.awake && !contact.bodyB.awake) continue;
      contacts.push(contact);
      if (contact.restitution > 0) impacts.push(contact);
    }

    const substeps = this.substeps;
    const h = this.dt / substeps;
    // A contact spring stiffer than a quarter of the sub-step rate would be unstable.
    const contactHertz = Math.min(this.contactHertz, 0.25 / h);
    const context: StepContext = {
      dt: this.dt,
      h,
      invH: 1 / h,
      warmStarting: this.warmStarting,
      contactSoftness: makeSoft(contactHertz, this.contactDampingRatio, h),
      staticSoftness: makeSoft(2 * contactHertz, this.contactDampingRatio, h),
      jointSoftness: makeSoft(Math.min(this.jointHertz, 0.5 / h), this.jointDampingRatio, h),
      pushVelocity: this.pushVelocity,
      restitutionThreshold: this.restitutionThreshold,
    };

    for (let i = 0; i < joints.length; i++) joints[i].prepare(context);
    prepareContacts(contacts, context);

    const gx = this.gravityX;
    const gy = this.gravityY;
    const maxSpin = MAX_ROTATION_PER_SUBSTEP / h;
    for (let step = 0; step < substeps; step++) {
      // 1. Forces change velocities.
      for (let i = 0; i < awake.length; i++) {
        const body = awake[i];
        const linear = 1 / (1 + h * body.linearDamping);
        const vx = (body.vx + h * (body.invMass * body.fx + body.gravityScale * gx)) * linear;
        const vy = (body.vy + h * (body.invMass * body.fy + body.gravityScale * gy)) * linear;
        body.vx = vx;
        body.vy = vy;
        body.w = (body.w + h * body.invInertia * body.torque) / (1 + h * body.angularDamping);
      }

      // 2. Elastic impacts, as discrete collisions.
      if (impacts.length > 0) solveImpacts(impacts, context);

      // 3. Constraints change velocities so that nothing penetrates or comes apart.
      for (let i = 0; i < joints.length; i++) joints[i].warmStart();
      warmStartContacts(contacts);
      for (let i = 0; i < joints.length; i++) joints[i].solve(context, true);
      solveContacts(contacts, context, true);

      // 4. Velocities change positions.
      for (let i = 0; i < awake.length; i++) {
        const body = awake[i];
        // The solver may have pushed a velocity past the limit; nothing moves faster than it.
        const speed2 = body.vx * body.vx + body.vy * body.vy;
        if (speed2 > MAX_LINEAR_SPEED * MAX_LINEAR_SPEED) {
          const scale = MAX_LINEAR_SPEED / Math.sqrt(speed2);
          body.vx *= scale;
          body.vy *= scale;
        }
        body.x += h * body.vx;
        body.y += h * body.vy;
        // Likewise for spin: at most a quarter turn per sub-step.
        if (body.w > maxSpin) body.w = maxSpin;
        else if (body.w < -maxSpin) body.w = -maxSpin;
        const rotation = h * body.w;
        const cd = cos(rotation);
        const sd = sin(rotation);
        const c = body.c * cd - body.s * sd;
        const s = body.s * cd + body.c * sd;
        // Renormalise so rounding never lets the rotation drift away from unit length.
        const inv = 1 / Math.sqrt(c * c + s * s);
        body.c = c * inv;
        body.s = s * inv;
      }
      refreshContacts(contacts);

      // 5. Relax: solve once more without position correction, removing the velocity it added.
      //    The relax pass also looks one sub-step ahead through speculative contacts, so
      //    impacts that are now imminent must be claimed first, or it would absorb them.
      if (impacts.length > 0) solveImpacts(impacts, context);
      for (let i = 0; i < joints.length; i++) joints[i].solve(context, false);
      solveContacts(contacts, context, false);
    }
  }

  /** Refreshes shape caches, decides who falls asleep, and updates the broadphase. */
  private finish(): void {
    const awake = this.awakeBodies;
    const dt = this.dt;
    for (let i = 0; i < awake.length; i++) {
      const body = awake[i];
      body.fx = body.fy = body.torque = 0;
      const speed2 = body.vx * body.vx + body.vy * body.vy;
      if (speed2 > MAX_LINEAR_SPEED * MAX_LINEAR_SPEED) {
        const scale = MAX_LINEAR_SPEED / Math.sqrt(speed2);
        body.vx *= scale;
        body.vy *= scale;
      }
      for (const shape of body.shapes) shape.sync();
    }

    if (this.sleeping) {
      const sleepTime = this.islandSleep;
      for (let i = 0; i < awake.length; i++) {
        const body = awake[i];
        const moving =
          !body.allowSleep ||
          body.vx * body.vx + body.vy * body.vy > SLEEP_LINEAR_SPEED * SLEEP_LINEAR_SPEED ||
          body.w * body.w > SLEEP_ANGULAR_SPEED * SLEEP_ANGULAR_SPEED;
        body.sleepTime = moving ? 0 : body.sleepTime + dt;
        sleepTime[this.find(body.islandIndex)] = Infinity;
      }
      // An island sleeps only when its most restless body has been still for long enough.
      for (let i = 0; i < awake.length; i++) {
        const root = this.find(awake[i].islandIndex);
        if (awake[i].sleepTime < sleepTime[root]) sleepTime[root] = awake[i].sleepTime;
      }
      for (let i = 0; i < awake.length; i++) {
        const body = awake[i];
        if (sleepTime[this.find(body.islandIndex)] >= TIME_TO_SLEEP) {
          body.awake = false;
          body.vx = body.vy = body.w = 0;
          body.prevX = body.x;
          body.prevY = body.y;
          body.prevC = body.c;
          body.prevS = body.s;
        }
      }
    }

    // Keep the tree current for the particle systems, which query it right after this.
    this.syncBroadphase();
  }
}
