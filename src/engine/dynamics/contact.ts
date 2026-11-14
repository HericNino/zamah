import { collideCircles, collidePolygonCircle, collidePolygons, Manifold } from '../collision/manifold.ts';
import type { Shape } from '../collision/shape.ts';
import type { Body } from './body.ts';
import type { Softness } from './joint.ts';
import { RIGID } from './joint.ts';

/** Shape ids are unique and small, so a pair fits in one number that can key a Map. */
export function pairKey(a: Shape, b: Shape): number {
  return a.id < b.id ? a.id * 0x4000000 + b.id : b.id * 0x4000000 + a.id;
}

/**
 * A potential contact between two shapes whose broadphase boxes overlap. It lives as long as
 * they do, which is what lets it carry impulses from one step to the next.
 */
export class Contact {
  readonly key: number;
  readonly shapeA: Shape;
  readonly shapeB: Shape;
  readonly bodyA: Body;
  readonly bodyB: Body;
  readonly manifold = new Manifold();
  friction: number;
  restitution: number;
  softness: Softness = RIGID;

  constructor(a: Shape, b: Shape) {
    this.key = pairKey(a, b);
    // The polygon-circle routine expects the polygon first.
    if (a.kind === 'circle' && b.kind === 'polygon') {
      const swap = a;
      a = b;
      b = swap;
    }
    this.shapeA = a;
    this.shapeB = b;
    this.bodyA = a.body;
    this.bodyB = b.body;
    // Friction mixes geometrically, restitution takes the bouncier of the two.
    this.friction = Math.sqrt(a.friction * b.friction);
    this.restitution = Math.max(a.restitution, b.restitution);
  }

  /** Recomputes the manifold, keeping the impulses of points that persist. */
  update(margin: number): void {
    const m = this.manifold;
    const p0 = m.points[0];
    const p1 = m.points[1];
    const oldCount = m.count;
    const id0 = p0.id;
    const n0 = p0.normalImpulse;
    const t0 = p0.tangentImpulse;
    const id1 = p1.id;
    const n1 = p1.normalImpulse;
    const t1 = p1.tangentImpulse;

    const a = this.shapeA;
    const b = this.shapeB;
    if (a.kind === 'polygon') {
      if (b.kind === 'polygon') collidePolygons(m, a, b, margin);
      else collidePolygonCircle(m, a, b, margin);
    } else {
      collideCircles(m, a, b, margin);
    }

    for (let i = 0; i < m.count; i++) {
      const p = m.points[i];
      if (oldCount > 0 && p.id === id0) {
        p.normalImpulse = n0;
        p.tangentImpulse = t0;
      } else if (oldCount > 1 && p.id === id1) {
        p.normalImpulse = n1;
        p.tangentImpulse = t1;
      } else {
        p.normalImpulse = 0;
        p.tangentImpulse = 0;
      }
    }
  }
}
