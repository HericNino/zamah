import { TWO_PI } from '../math/trig.ts';
import type { Body } from './body.ts';

/**
 * Coefficients of a "soft" constraint: an implicit spring-damper folded into the impulse
 * formula. A rigid constraint solved with these behaves like a critically (or over-) damped
 * spring of the given frequency, which removes position error smoothly instead of snapping,
 * and never adds energy the way a plain Baumgarte velocity bias can.
 */
export interface Softness {
  /** Fraction of the position error fed back per second. */
  biasRate: number;
  /** Scales the impulse computed from the velocity error. */
  massScale: number;
  /** Bleeds off the accumulated impulse, which is what makes the constraint springy. */
  impulseScale: number;
}

export const RIGID: Softness = { biasRate: 0, massScale: 1, impulseScale: 0 };

export function makeSoft(hertz: number, dampingRatio: number, h: number): Softness {
  if (hertz === 0) return RIGID;
  const omega = TWO_PI * hertz;
  const a1 = 2 * dampingRatio + h * omega;
  const a2 = h * omega * a1;
  const a3 = 1 / (1 + a2);
  return { biasRate: omega / a1, massScale: a2 * a3, impulseScale: a3 };
}

/** Everything a constraint needs to know about the step being solved. */
export interface StepContext {
  /** Full step, seconds. */
  dt: number;
  /** Sub-step, seconds. */
  h: number;
  invH: number;
  warmStarting: boolean;
  contactSoftness: Softness;
  /** Contacts against static bodies are stiffer, so things do not sink into the ground. */
  staticSoftness: Softness;
  jointSoftness: Softness;
  /** Largest speed at which overlapping shapes are pushed apart. */
  pushVelocity: number;
  restitutionThreshold: number;
}

let nextJointId = 1;

/**
 * Base class of every joint. A joint removes degrees of freedom between two bodies by
 * applying impulses; the three methods mirror the phases of the solver.
 */
export abstract class Joint {
  readonly id = nextJointId++;
  abstract readonly kind: string;
  readonly bodyA: Body;
  readonly bodyB: Body;
  /** When false (the default), the two bodies pass through each other. */
  collideConnected: boolean;
  userData: unknown;

  constructor(bodyA: Body, bodyB: Body, collideConnected: boolean) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.collideConnected = collideConnected;
  }

  /** Called once per step, before the sub-steps. */
  abstract prepare(context: StepContext): void;
  /** Re-applies last step's impulses as a starting guess. */
  abstract warmStart(): void;
  /** One solver iteration. With `useBias` the joint also corrects position error. */
  abstract solve(context: StepContext, useBias: boolean): void;
}
