import { Shape, type ShapeDef } from '../collision/shape.ts';
import { atan2, cos, sin } from '../math/trig.ts';
import type { Vec2 } from '../math/vec2.ts';
import type { Joint } from './joint.ts';

export type BodyType = 'static' | 'dynamic';

export interface BodyDef {
  type?: BodyType;
  position?: Vec2;
  angle?: number;
  linearVelocity?: Vec2;
  angularVelocity?: number;
  shapes: ShapeDef[];
  linearDamping?: number;
  angularDamping?: number;
  gravityScale?: number;
  fixedRotation?: boolean;
  allowSleep?: boolean;
  userData?: unknown;
}

let nextBodyId = 1;

/**
 * A rigid body: a position, an orientation, velocities and mass, plus the shapes attached to it.
 *
 * `x, y` is always the centre of mass. If the shapes in the definition are off-centre, the body
 * origin is moved onto their combined centre of mass when the body is created, which keeps the
 * solver free of "local centre" offsets.
 *
 * Orientation is stored as a unit vector (c, s) = (cos, sin) rather than an angle, so rotating
 * a point never needs a trigonometric call inside the simulation loop.
 */
export class Body {
  readonly id: number;
  readonly type: BodyType;

  x: number;
  y: number;
  c: number;
  s: number;
  vx: number;
  vy: number;
  /** Angular velocity, radians per second, counter-clockwise positive. */
  w: number;

  /** Forces accumulated by applyForce; cleared at the end of every step. */
  fx = 0;
  fy = 0;
  torque = 0;

  mass = 0;
  invMass = 0;
  inertia = 0;
  invInertia = 0;

  linearDamping: number;
  angularDamping: number;
  gravityScale: number;

  readonly shapes: Shape[] = [];
  readonly joints: Joint[] = [];

  /** Static bodies are never awake; sleeping dynamic bodies are skipped by the solver. */
  awake: boolean;
  sleepTime = 0;
  allowSleep: boolean;

  /** Transform at the start of the last step, for render interpolation. */
  prevX: number;
  prevY: number;
  prevC: number;
  prevS: number;

  /**
   * While a body sleeps, the particle systems (cloth, liquid) add the impulses they would
   * have applied here instead. The next step looks at the total to decide whether to wake it.
   */
  pendingX = 0;
  pendingY = 0;
  pendingAngular = 0;

  userData: unknown;

  /** Scratch index used while building islands. */
  islandIndex = 0;

  constructor(def: BodyDef) {
    this.id = nextBodyId++;
    this.type = def.type ?? 'dynamic';
    const angle = def.angle ?? 0;
    this.x = def.position?.x ?? 0;
    this.y = def.position?.y ?? 0;
    this.c = cos(angle);
    this.s = sin(angle);
    this.vx = def.linearVelocity?.x ?? 0;
    this.vy = def.linearVelocity?.y ?? 0;
    this.w = def.angularVelocity ?? 0;
    this.linearDamping = def.linearDamping ?? 0;
    this.angularDamping = def.angularDamping ?? 0;
    this.gravityScale = def.gravityScale ?? 1;
    this.allowSleep = def.allowSleep ?? true;
    this.awake = this.type === 'dynamic';
    this.userData = def.userData;

    for (const shapeDef of def.shapes) {
      const shape = new Shape(shapeDef);
      shape.body = this;
      this.shapes.push(shape);
    }

    if (this.type === 'dynamic') {
      let mass = 0;
      let cx = 0;
      let cy = 0;
      let inertia = 0;
      for (const shape of this.shapes) {
        const data = shape.computeMass();
        mass += data.mass;
        cx += data.mass * data.centerX;
        cy += data.mass * data.centerY;
        inertia += data.inertia;
      }
      if (mass > 0) {
        cx /= mass;
        cy /= mass;
        // Parallel axis theorem: inertia about the centre of mass.
        inertia -= mass * (cx * cx + cy * cy);
        for (const shape of this.shapes) shape.translateLocal(-cx, -cy);
        this.x += this.c * cx - this.s * cy;
        this.y += this.s * cx + this.c * cy;
      } else {
        // Weightless shapes still need to respond to the solver.
        mass = 1;
        inertia = 1;
      }
      this.mass = mass;
      this.invMass = 1 / mass;
      this.inertia = inertia;
      this.invInertia = def.fixedRotation || !(inertia > 0) ? 0 : 1 / inertia;
    }

    for (const shape of this.shapes) {
      shape.computeBoundRadius();
      shape.sync();
    }

    this.prevX = this.x;
    this.prevY = this.y;
    this.prevC = this.c;
    this.prevS = this.s;
  }

  get isDynamic(): boolean {
    return this.invMass > 0;
  }

  /** Orientation in radians, in (-pi, pi]. Derived from (c, s); nothing in the solver reads it. */
  get angle(): number {
    return atan2(this.s, this.c);
  }

  wake(): void {
    if (this.type !== 'dynamic') return;
    this.awake = true;
    this.sleepTime = 0;
  }

  setTransform(x: number, y: number, angle: number): void {
    this.x = this.prevX = x;
    this.y = this.prevY = y;
    this.c = this.prevC = cos(angle);
    this.s = this.prevS = sin(angle);
    for (const shape of this.shapes) shape.sync();
    this.wake();
  }

  setVelocity(vx: number, vy: number, w = this.w): void {
    this.vx = vx;
    this.vy = vy;
    this.w = w;
    this.wake();
  }

  /** A force in newtons at a world point (the centre of mass when omitted), for this step only. */
  applyForce(fx: number, fy: number, px = this.x, py = this.y): void {
    this.fx += fx;
    this.fy += fy;
    this.torque += (px - this.x) * fy - (py - this.y) * fx;
    this.wake();
  }

  /** An instantaneous change of momentum at a world point. */
  applyImpulse(ix: number, iy: number, px = this.x, py = this.y): void {
    if (this.invMass === 0) return;
    this.vx += this.invMass * ix;
    this.vy += this.invMass * iy;
    this.w += this.invInertia * ((px - this.x) * iy - (py - this.y) * ix);
    this.wake();
  }

  /** Records an impulse a particle system would apply to this sleeping body; see `pendingX`. */
  queueImpulse(ix: number, iy: number, px: number, py: number): void {
    this.pendingX += ix;
    this.pendingY += iy;
    this.pendingAngular += (px - this.x) * iy - (py - this.y) * ix;
  }

  worldPoint(localX: number, localY: number): Vec2 {
    return { x: this.x + this.c * localX - this.s * localY, y: this.y + this.s * localX + this.c * localY };
  }

  localPoint(worldX: number, worldY: number): Vec2 {
    const dx = worldX - this.x;
    const dy = worldY - this.y;
    return { x: this.c * dx + this.s * dy, y: -this.s * dx + this.c * dy };
  }

  /** Velocity of the material point currently at a world position. */
  velocityAt(px: number, py: number): Vec2 {
    return { x: this.vx - this.w * (py - this.y), y: this.vy + this.w * (px - this.x) };
  }

  kineticEnergy(): number {
    return 0.5 * this.mass * (this.vx * this.vx + this.vy * this.vy) + 0.5 * this.inertia * this.w * this.w;
  }
}
