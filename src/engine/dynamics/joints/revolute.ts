import { atan2 } from '../../math/trig.ts';
import type { Vec2 } from '../../math/vec2.ts';
import type { Body } from '../body.ts';
import { Joint, type StepContext } from '../joint.ts';

export interface RevoluteJointDef {
  bodyA: Body;
  bodyB: Body;
  /** The hinge point, in world coordinates. */
  anchor: Vec2;
  collideConnected?: boolean;
  enableLimit?: boolean;
  /** Limits on the angle of B relative to A, measured from the pose at creation. Radians. */
  lowerAngle?: number;
  upperAngle?: number;
  enableMotor?: boolean;
  /** Target relative angular velocity, radians per second. */
  motorSpeed?: number;
  maxMotorTorque?: number;
}

/**
 * A hinge: pins a point of body A to a point of body B, leaving only relative rotation.
 * Optionally limits that rotation to a range and drives it with a motor.
 */
export class RevoluteJoint extends Joint {
  readonly kind = 'revolute';

  localAx: number;
  localAy: number;
  localBx: number;
  localBy: number;
  /** Relative rotation at creation, as (cos, sin); joint angle zero means this pose. */
  private refC: number;
  private refS: number;

  enableLimit: boolean;
  lowerAngle: number;
  upperAngle: number;
  enableMotor: boolean;
  motorSpeed: number;
  maxMotorTorque: number;

  private impulseX = 0;
  private impulseY = 0;
  private motorImpulse = 0;
  private lowerImpulse = 0;
  private upperImpulse = 0;
  private axialMass = 0;

  constructor(def: RevoluteJointDef) {
    super(def.bodyA, def.bodyB, def.collideConnected ?? false);
    const a = def.bodyA.localPoint(def.anchor.x, def.anchor.y);
    const b = def.bodyB.localPoint(def.anchor.x, def.anchor.y);
    this.localAx = a.x;
    this.localAy = a.y;
    this.localBx = b.x;
    this.localBy = b.y;
    const A = def.bodyA;
    const B = def.bodyB;
    this.refC = B.c * A.c + B.s * A.s;
    this.refS = B.s * A.c - B.c * A.s;
    this.enableLimit = def.enableLimit ?? false;
    this.lowerAngle = def.lowerAngle ?? 0;
    this.upperAngle = def.upperAngle ?? 0;
    this.enableMotor = def.enableMotor ?? false;
    this.motorSpeed = def.motorSpeed ?? 0;
    this.maxMotorTorque = def.maxMotorTorque ?? 0;
  }

  /** Current hinge position in world coordinates (as seen from body B). */
  get anchor(): Vec2 {
    return this.bodyB.worldPoint(this.localBx, this.localBy);
  }

  /** Angle of B relative to A, zero at the creation pose. */
  get angle(): number {
    const A = this.bodyA;
    const B = this.bodyB;
    const c = B.c * A.c + B.s * A.s;
    const s = B.s * A.c - B.c * A.s;
    return atan2(s * this.refC - c * this.refS, c * this.refC + s * this.refS);
  }

  prepare(context: StepContext): void {
    const k = this.bodyA.invInertia + this.bodyB.invInertia;
    this.axialMass = k > 0 ? 1 / k : 0;
    if (!context.warmStarting) {
      this.impulseX = this.impulseY = 0;
      this.motorImpulse = this.lowerImpulse = this.upperImpulse = 0;
    }
    if (!this.enableMotor) this.motorImpulse = 0;
    if (!this.enableLimit) this.lowerImpulse = this.upperImpulse = 0;
  }

  warmStart(): void {
    const A = this.bodyA;
    const B = this.bodyB;
    const rAx = A.c * this.localAx - A.s * this.localAy;
    const rAy = A.s * this.localAx + A.c * this.localAy;
    const rBx = B.c * this.localBx - B.s * this.localBy;
    const rBy = B.s * this.localBx + B.c * this.localBy;
    const axial = this.motorImpulse + this.lowerImpulse - this.upperImpulse;
    A.vx -= A.invMass * this.impulseX;
    A.vy -= A.invMass * this.impulseY;
    A.w -= A.invInertia * (rAx * this.impulseY - rAy * this.impulseX + axial);
    B.vx += B.invMass * this.impulseX;
    B.vy += B.invMass * this.impulseY;
    B.w += B.invInertia * (rBx * this.impulseY - rBy * this.impulseX + axial);
  }

  solve(context: StepContext, useBias: boolean): void {
    const A = this.bodyA;
    const B = this.bodyB;
    const mA = A.invMass;
    const mB = B.invMass;
    const iA = A.invInertia;
    const iB = B.invInertia;

    if (this.enableMotor) {
      const cdot = B.w - A.w - this.motorSpeed;
      const max = this.maxMotorTorque * context.h;
      const old = this.motorImpulse;
      this.motorImpulse = Math.max(-max, Math.min(max, old - this.axialMass * cdot));
      const impulse = this.motorImpulse - old;
      A.w -= iA * impulse;
      B.w += iB * impulse;
    }

    if (this.enableLimit) {
      const angle = this.angle;
      const soft = context.jointSoftness;
      {
        // Lower limit: keep (angle - lower) >= 0.
        const c = angle - this.lowerAngle;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (c > 0) {
          bias = c * context.invH;
        } else if (useBias) {
          bias = soft.biasRate * c;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        const cdot = B.w - A.w;
        const old = this.lowerImpulse;
        this.lowerImpulse = Math.max(old - this.axialMass * massScale * (cdot + bias) - impulseScale * old, 0);
        const impulse = this.lowerImpulse - old;
        A.w -= iA * impulse;
        B.w += iB * impulse;
      }
      {
        // Upper limit: keep (upper - angle) >= 0.
        const c = this.upperAngle - angle;
        let bias = 0;
        let massScale = 1;
        let impulseScale = 0;
        if (c > 0) {
          bias = c * context.invH;
        } else if (useBias) {
          bias = soft.biasRate * c;
          massScale = soft.massScale;
          impulseScale = soft.impulseScale;
        }
        const cdot = A.w - B.w;
        const old = this.upperImpulse;
        this.upperImpulse = Math.max(old - this.axialMass * massScale * (cdot + bias) - impulseScale * old, 0);
        const impulse = this.upperImpulse - old;
        A.w += iA * impulse;
        B.w -= iB * impulse;
      }
    }

    // Point constraint: the two anchors must coincide. Solved as one 2x2 system.
    const rAx = A.c * this.localAx - A.s * this.localAy;
    const rAy = A.s * this.localAx + A.c * this.localAy;
    const rBx = B.c * this.localBx - B.s * this.localBy;
    const rBy = B.s * this.localBx + B.c * this.localBy;

    let cdotX = B.vx - B.w * rBy - A.vx + A.w * rAy;
    let cdotY = B.vy + B.w * rBx - A.vy - A.w * rAx;
    let massScale = 1;
    let impulseScale = 0;
    if (useBias) {
      const soft = context.jointSoftness;
      cdotX += soft.biasRate * (B.x + rBx - A.x - rAx);
      cdotY += soft.biasRate * (B.y + rBy - A.y - rAy);
      massScale = soft.massScale;
      impulseScale = soft.impulseScale;
    }

    const k11 = mA + mB + iA * rAy * rAy + iB * rBy * rBy;
    const k12 = -iA * rAx * rAy - iB * rBx * rBy;
    const k22 = mA + mB + iA * rAx * rAx + iB * rBx * rBx;
    const det = k11 * k22 - k12 * k12;
    if (det === 0) return;
    const bx = (k22 * cdotX - k12 * cdotY) / det;
    const by = (k11 * cdotY - k12 * cdotX) / det;

    const ix = -massScale * bx - impulseScale * this.impulseX;
    const iy = -massScale * by - impulseScale * this.impulseY;
    this.impulseX += ix;
    this.impulseY += iy;

    A.vx -= mA * ix;
    A.vy -= mA * iy;
    A.w -= iA * (rAx * iy - rAy * ix);
    B.vx += mB * ix;
    B.vy += mB * iy;
    B.w += iB * (rBx * iy - rBy * ix);
  }
}
