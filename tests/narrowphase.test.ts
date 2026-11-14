import { describe, expect, it } from 'vitest';
import {
  Body,
  box,
  circle,
  collideCircles,
  collidePolygonCircle,
  collidePolygons,
  Manifold,
  polygon,
  Random,
  type Shape,
} from '../src/engine/index.ts';

function shapeOf(def: ConstructorParameters<typeof Body>[0]): Shape {
  return new Body({ type: 'static', ...def }).shapes[0];
}

/** Polygon overlap without SAT: do any edges cross, or is one polygon inside the other? */
function polygonsOverlap(a: Shape, b: Shape): boolean {
  const points = (s: Shape) => Array.from({ length: s.count }, (_, i) => [s.worldVertices[2 * i], s.worldVertices[2 * i + 1]]);
  const pa = points(a);
  const pb = points(b);
  const cross = (o: number[], p: number[], q: number[]) => (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0]);
  for (let i = 0; i < pa.length; i++) {
    for (let j = 0; j < pb.length; j++) {
      const a1 = pa[i];
      const a2 = pa[(i + 1) % pa.length];
      const b1 = pb[j];
      const b2 = pb[(j + 1) % pb.length];
      if (cross(a1, a2, b1) * cross(a1, a2, b2) < 0 && cross(b1, b2, a1) * cross(b1, b2, a2) < 0) return true;
    }
  }
  const inside = (p: number[], poly: number[][]) => poly.every((v, i) => cross(v, poly[(i + 1) % poly.length], p) > 0);
  return inside(pa[0], pb) || inside(pb[0], pa);
}

function randomPolygon(rnd: Random, x: number, y: number): Shape {
  const count = 3 + rnd.int(6);
  const points = Array.from({ length: count }, () => ({ x: rnd.range(-1, 1), y: rnd.range(-1, 1) }));
  // Reject slivers: they make "is it overlapping" a question about rounding errors.
  const def = polygon(points);
  const body = new Body({ type: 'static', position: { x, y }, angle: rnd.range(-3, 3), shapes: [def] });
  return body.shapes[0];
}

describe('polygon vs polygon', () => {
  it('finds two contact points and an upward normal for a box resting on a box', () => {
    const lower = shapeOf({ position: { x: 0, y: 0 }, shapes: [box(2, 0.5)] });
    const upper = shapeOf({ position: { x: 0.5, y: 0.99 }, shapes: [box(0.5, 0.5)] });
    const m = new Manifold();
    collidePolygons(m, lower, upper, 0.02);
    expect(m.count).toBe(2);
    expect(m.nx).toBeCloseTo(0, 12);
    expect(m.ny).toBeCloseTo(1, 12);
    for (const p of m.points) {
      expect(p.separation).toBeCloseTo(-0.01, 12);
      expect(p.y).toBeCloseTo(0.495, 12);
    }
    expect(m.points.map((p) => p.x).sort()).toEqual([0, 1]);
    expect(m.points[0].id).not.toBe(m.points[1].id);
  });

  it('reports speculative points inside the margin and nothing beyond it', () => {
    const lower = shapeOf({ position: { x: 0, y: 0 }, shapes: [box(2, 0.5)] });
    const near = shapeOf({ position: { x: 0, y: 1.015 }, shapes: [box(0.5, 0.5)] });
    const far = shapeOf({ position: { x: 0, y: 1.2 }, shapes: [box(0.5, 0.5)] });
    const m = new Manifold();
    collidePolygons(m, lower, near, 0.02);
    expect(m.count).toBe(2);
    expect(m.points[0].separation).toBeCloseTo(0.015, 12);
    collidePolygons(m, lower, far, 0.02);
    expect(m.count).toBe(0);
  });

  it('keeps contact ids stable while a box slides, so impulses can be matched across steps', () => {
    const lower = shapeOf({ position: { x: 0, y: 0 }, shapes: [box(4, 0.5)] });
    const m = new Manifold();
    let ids: number[] | null = null;
    for (let i = 0; i < 50; i++) {
      const upper = shapeOf({ position: { x: -1 + i * 0.04, y: 0.995 }, shapes: [box(0.5, 0.5)] });
      collidePolygons(m, lower, upper, 0.02);
      expect(m.count).toBe(2);
      const current = [m.points[0].id, m.points[1].id];
      if (ids) expect(current).toEqual(ids);
      ids = current;
    }
  });

  it('agrees with an independent overlap test on 20,000 random pairs', () => {
    const rnd = new Random(2024);
    const m = new Manifold();
    let overlapping = 0;
    let tested = 0;
    for (let i = 0; i < 20000; i++) {
      const a = randomPolygon(rnd, 0, 0);
      const b = randomPolygon(rnd, rnd.range(-2, 2), rnd.range(-2, 2));
      collidePolygons(m, a, b, 0);
      let deepest = Infinity;
      for (let k = 0; k < m.count; k++) deepest = Math.min(deepest, m.points[k].separation);
      const expected = polygonsOverlap(a, b);
      // Grazing contact is a coin toss for any floating point test; skip the knife edge.
      if (m.count > 0 && Math.abs(deepest) < 1e-9) continue;
      tested++;
      if (expected) overlapping++;
      expect(m.count > 0 && deepest < 0, `pair ${i}`).toBe(expected);
    }
    expect(tested).toBeGreaterThan(19000);
    expect(overlapping).toBeGreaterThan(3000);
    expect(tested - overlapping).toBeGreaterThan(3000);
  });

  it('reports a normal and depth that actually separate the shapes', () => {
    const rnd = new Random(77);
    const m = new Manifold();
    let checked = 0;
    for (let i = 0; i < 5000; i++) {
      const a = randomPolygon(rnd, 0, 0);
      const bx = rnd.range(-1.5, 1.5);
      const by = rnd.range(-1.5, 1.5);
      const angle = rnd.range(-3, 3);
      const points = Array.from({ length: 5 }, () => ({ x: rnd.range(-1, 1), y: rnd.range(-1, 1) }));
      const make = (x: number, y: number) => new Body({ type: 'static', position: { x, y }, angle, shapes: [polygon(points)] }).shapes[0];
      const b = make(bx, by);
      collidePolygons(m, a, b, 0);
      if (m.count === 0) continue;
      let deepest = 0;
      for (let k = 0; k < m.count; k++) deepest = Math.min(deepest, m.points[k].separation);
      if (deepest > -1e-3) continue;
      // The normal points from A to B: moving B along it by the depth must end the overlap ...
      const moved = make(bx + m.nx * (-deepest + 1e-6), by + m.ny * (-deepest + 1e-6));
      expect(polygonsOverlap(a, moved), `pair ${i} still overlaps`).toBe(false);
      // ... and moving it only half way must not.
      const half = make(bx + m.nx * -0.5 * deepest, by + m.ny * -0.5 * deepest);
      expect(polygonsOverlap(a, half), `pair ${i} separated early`).toBe(true);
      checked++;
    }
    expect(checked).toBeGreaterThan(1000);
  });
});

describe('circles', () => {
  it('computes circle vs circle exactly', () => {
    const a = shapeOf({ position: { x: 0, y: 0 }, shapes: [circle(1)] });
    const b = shapeOf({ position: { x: 3, y: 4 }, shapes: [circle(2)] });
    const m = new Manifold();
    collideCircles(m, a, b, 2.5);
    expect(m.count).toBe(1);
    expect(m.nx).toBeCloseTo(0.6, 12);
    expect(m.ny).toBeCloseTo(0.8, 12);
    expect(m.points[0].separation).toBeCloseTo(2, 12);
    // The point sits halfway between the two surfaces.
    expect(m.points[0].x).toBeCloseTo(0.6 * 2, 12);
    collideCircles(m, a, b, 1.9);
    expect(m.count).toBe(0);
  });

  it('distinguishes face and corner regions for polygon vs circle', () => {
    const square = shapeOf({ position: { x: 0, y: 0 }, shapes: [box(1, 1)] });
    const m = new Manifold();

    const onFace = shapeOf({ position: { x: 0.3, y: 1.4 }, shapes: [circle(0.5)] });
    collidePolygonCircle(m, square, onFace, 0);
    expect(m.count).toBe(1);
    expect(m.nx).toBeCloseTo(0, 12);
    expect(m.ny).toBeCloseTo(1, 12);
    expect(m.points[0].separation).toBeCloseTo(-0.1, 12);

    const onCorner = shapeOf({ position: { x: 1.3, y: 1.4 }, shapes: [circle(0.6)] });
    collidePolygonCircle(m, square, onCorner, 0);
    expect(m.count).toBe(1);
    expect(m.nx).toBeCloseTo(0.6, 12);
    expect(m.ny).toBeCloseTo(0.8, 12);
    expect(m.points[0].separation).toBeCloseTo(-0.1, 12);

    const inside = shapeOf({ position: { x: 0.2, y: 0.6 }, shapes: [circle(0.1)] });
    collidePolygonCircle(m, square, inside, 0);
    expect(m.count).toBe(1);
    expect(m.ny).toBeCloseTo(1, 12);
    expect(m.points[0].separation).toBeCloseTo(-0.5, 12);
  });

  it('matches a brute-force distance for random polygon vs circle pairs', () => {
    const rnd = new Random(31);
    const m = new Manifold();
    for (let i = 0; i < 5000; i++) {
      const poly = randomPolygon(rnd, 0, 0);
      const cx = rnd.range(-2.5, 2.5);
      const cy = rnd.range(-2.5, 2.5);
      const radius = rnd.range(0.1, 0.8);
      const disc = shapeOf({ position: { x: cx, y: cy }, shapes: [circle(radius)] });
      // Brute force: distance from the centre to every edge segment.
      let distance = Infinity;
      for (let k = 0; k < poly.count; k++) {
        const ax = poly.worldVertices[2 * k];
        const ay = poly.worldVertices[2 * k + 1];
        const n = (k + 1) % poly.count;
        const ex = poly.worldVertices[2 * n] - ax;
        const ey = poly.worldVertices[2 * n + 1] - ay;
        const t = Math.max(0, Math.min(1, ((cx - ax) * ex + (cy - ay) * ey) / (ex * ex + ey * ey)));
        distance = Math.min(distance, Math.hypot(cx - ax - t * ex, cy - ay - t * ey));
      }
      if (poly.containsPoint(cx, cy)) distance = -distance;
      collidePolygonCircle(m, poly, disc, 10);
      expect(m.count).toBe(1);
      expect(m.points[0].separation).toBeCloseTo(distance - radius, 9);
    }
  });
});
