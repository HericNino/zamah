import type { Vec2 } from '../../math/vec2.ts';
import type { Body } from '../body.ts';
import { Joint, makeSoft, RIGID, type Softness, type StepContext } from '../joint.ts';

export interface MouseJointDef {
  body: Body;
  /** Where the body is grabbed, in world coordinates. */
  target: Vec2;
  hertz?: number;
  dampingRatio?: number;
  /** Cap on the pulling force, so a grabbed body cannot be dragged through a wall. */
  maxForce?: number;
}

/**
 * Drags a point of one body towards a moving target with a damped spring. This is what the
 * sandbox's grab tool uses: releasing the joint leaves the body with whatever velocity the
 * spring gave it, which is all "throwing" is.
 */
export class MouseJoint extends Joint {
  readonly kind = 'mouse';

  localX: number;
  localY: number;
  targetX: number;
  targetY: number;
  hertz: number;
  dampingRatio: number;
  maxForce: number;

  private impulseX = 0;
  private impulseY = 0;
  private softness: Softness = RIGID;

  constructor(def: MouseJointDef) {
    super(def.body, def.body, true);
    const local = def.body.localPoint(def.target.x, def.target.y);
    this.localX = local.x;
    this.localY = local.y;
    this.targetX = def.target.x;
    this.targetY = def.target.y;
    this.hertz = def.hertz ?? 6;
    this.dampingRatio = def.dampingRatio ?? 0.9;
    this.maxForce = def.maxForce ?? 1000 * def.body.mass;
  }

  setTarget(x: number, y: number): void {
    this.targetX = x;
    this.targetY = y;
    this.bodyB.wake();
  }

  get anchor(): Vec2 {
    return this.bodyB.worldPoint(this.localX, this.localY);
  }

  prepare(context: StepContext): void {
    this.softness = makeSoft(this.hertz, this.dampingRatio, context.h);
    if (!context.warmStarting) this.impulseX = this.impulseY = 0;
    // A held body must not fall asleep in the user's hand.
    this.bodyB.sleepTime = 0;
  }

  warmStart(): void {
    const B = this.bodyB;
    const rx = B.c * this.localX - B.s * this.localY;
    const ry = B.s * this.localX + B.c * this.localY;
    B.vx += B.invMass * this.impulseX;
    B.vy += B.invMass * this.impulseY;
    B.w += B.invInertia * (rx * this.impulseY - ry * this.impulseX);
  }

  solve(context: StepContext): void {
    const B = this.bodyB;
    const m = B.invMass;
    const i = B.invInertia;
    const rx = B.c * this.localX - B.s * this.localY;
    const ry = B.s * this.localX + B.c * this.localY;
    const soft = this.softness;

    const cdotX = B.vx - B.w * ry + soft.biasRate * (B.x + rx - this.targetX);
    const cdotY = B.vy + B.w * rx + soft.biasRate * (B.y + ry - this.targetY);

    const k11 = m + i * ry * ry;
    const k12 = -i * rx * ry;
    const k22 = m + i * rx * rx;
    const det = k11 * k22 - k12 * k12;
    if (det === 0) return;
    const bx = (k22 * cdotX - k12 * cdotY) / det;
    const by = (k11 * cdotY - k12 * cdotX) / det;

    const oldX = this.impulseX;
    const oldY = this.impulseY;
    this.impulseX += -soft.massScale * bx - soft.impulseScale * oldX;
    this.impulseY += -soft.massScale * by - soft.impulseScale * oldY;
    const max = this.maxForce * context.h;
    const mag2 = this.impulseX * this.impulseX + this.impulseY * this.impulseY;
    if (mag2 > max * max) {
      const scale = max / Math.sqrt(mag2);
      this.impulseX *= scale;
      this.impulseY *= scale;
    }
    const ix = this.impulseX - oldX;
    const iy = this.impulseY - oldY;
    B.vx += m * ix;
    B.vy += m * iy;
    B.w += i * (rx * iy - ry * ix);
  }
}
