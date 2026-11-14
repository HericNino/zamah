import type { Vec2 } from '../../math/vec2.ts';
import type { Body } from '../body.ts';
import { Joint, makeSoft, RIGID, type Softness, type StepContext } from '../joint.ts';

export interface DistanceJointDef {
  bodyA: Body;
  bodyB: Body;
  /** Attachment points in world coordinates. */
  anchorA: Vec2;
  anchorB: Vec2;
  /** Rest length; defaults to the current distance between the anchors. */
  length?: number;
  /**
   * `rod` keeps the anchors exactly `length` apart, `rope` only stops them getting further
   * apart, `spring` pulls them towards `length` with the given frequency and damping.
   */
  mode?: 'rod' | 'rope' | 'spring';
  hertz?: number;
  dampingRatio?: number;
  collideConnected?: boolean;
}

/** Constrains the distance between a point on each body. */
export class DistanceJoint extends Joint {
  readonly kind = 'distance';

  localAx: number;
  localAy: number;
  localBx: number;
  localBy: number;
  length: number;
  mode: 'rod' | 'rope' | 'spring';
  hertz: number;
  dampingRatio: number;

  private impulse = 0;
  private springSoftness: Softness = RIGID;

  constructor(def: DistanceJointDef) {
    super(def.bodyA, def.bodyB, def.collideConnected ?? true);
    const a = def.bodyA.localPoint(def.anchorA.x, def.anchorA.y);
    const b = def.bodyB.localPoint(def.anchorB.x, def.anchorB.y);
    this.localAx = a.x;
    this.localAy = a.y;
    this.localBx = b.x;
    this.localBy = b.y;
    const dx = def.anchorB.x - def.anchorA.x;
    const dy = def.anchorB.y - def.anchorA.y;
    this.length = def.length ?? Math.sqrt(dx * dx + dy * dy);
    this.mode = def.mode ?? 'rod';
    this.hertz = def.hertz ?? 2;
    this.dampingRatio = def.dampingRatio ?? 0.3;
  }

  get anchorA(): Vec2 {
    return this.bodyA.worldPoint(this.localAx, this.localAy);
  }

  get anchorB(): Vec2 {
    return this.bodyB.worldPoint(this.localBx, this.localBy);
  }

  prepare(context: StepContext): void {
    if (!context.warmStarting) this.impulse = 0;
    this.springSoftness = this.mode === 'spring' ? makeSoft(this.hertz, this.dampingRatio, context.h) : RIGID;
  }

  warmStart(): void {
    this.apply(this.impulse);
  }

  solve(context: StepContext, useBias: boolean): void {
    const A = this.bodyA;
    const B = this.bodyB;
    const rAx = A.c * this.localAx - A.s * this.localAy;
    const rAy = A.s * this.localAx + A.c * this.localAy;
    const rBx = B.c * this.localBx - B.s * this.localBy;
    const rBy = B.s * this.localBx + B.c * this.localBy;
    const dx = B.x + rBx - A.x - rAx;
    const dy = B.y + rBy - A.y - rAy;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len < 1e-9) return;
    const ax = dx / len;
    const ay = dy / len;

    const crA = rAx * ay - rAy * ax;
    const crB = rBx * ay - rBy * ax;
    const k = A.invMass + B.invMass + A.invInertia * crA * crA + B.invInertia * crB * crB;
    if (k === 0) return;
    const mass = 1 / k;

    const vr = ax * (B.vx - B.w * rBy - A.vx + A.w * rAy) + ay * (B.vy + B.w * rBx - A.vy - A.w * rAx);
    const c = len - this.length;
    const old = this.impulse;

    if (this.mode === 'spring') {
      // A spring is a soft constraint that is *always* soft, relax pass included.
      const soft = this.springSoftness;
      this.impulse = old - soft.massScale * mass * (vr + soft.biasRate * c) - soft.impulseScale * old;
    } else if (this.mode === 'rope') {
      // One-sided: only resists stretching. While slack it lets the anchors drift apart just
      // fast enough to arrive at full length, like a speculative contact.
      let bias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (c < 0) {
        bias = c * context.invH;
      } else if (useBias) {
        const soft = context.jointSoftness;
        bias = soft.biasRate * c;
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }
      this.impulse = Math.min(old - massScale * mass * (vr + bias) - impulseScale * old, 0);
    } else {
      let bias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (useBias) {
        const soft = context.jointSoftness;
        bias = soft.biasRate * c;
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }
      this.impulse = old - massScale * mass * (vr + bias) - impulseScale * old;
    }

    const impulse = this.impulse - old;
    const px = impulse * ax;
    const py = impulse * ay;
    A.vx -= A.invMass * px;
    A.vy -= A.invMass * py;
    A.w -= A.invInertia * (rAx * py - rAy * px);
    B.vx += B.invMass * px;
    B.vy += B.invMass * py;
    B.w += B.invInertia * (rBx * py - rBy * px);
  }

  private apply(impulse: number): void {
    const A = this.bodyA;
    const B = this.bodyB;
    const rAx = A.c * this.localAx - A.s * this.localAy;
    const rAy = A.s * this.localAx + A.c * this.localAy;
    const rBx = B.c * this.localBx - B.s * this.localBy;
    const rBy = B.s * this.localBx + B.c * this.localBy;
    const dx = B.x + rBx - A.x - rAx;
    const dy = B.y + rBy - A.y - rAy;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len < 1e-9) return;
    const px = (impulse * dx) / len;
    const py = (impulse * dy) / len;
    A.vx -= A.invMass * px;
    A.vy -= A.invMass * py;
    A.w -= A.invInertia * (rAx * py - rAy * px);
    B.vx += B.invMass * px;
    B.vy += B.invMass * py;
    B.w += B.invInertia * (rBx * py - rBy * px);
  }
}
