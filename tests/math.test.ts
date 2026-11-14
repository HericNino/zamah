import { describe, expect, it } from 'vitest';
import { atan2, box, circle, convexHull, cos, polygon, Random, Shape, sin, StateHash } from '../src/engine/index.ts';
import { atan } from '../src/engine/math/trig.ts';

describe('deterministic trigonometry', () => {
  it('matches Math.sin and Math.cos to 1e-13 over a wide range', () => {
    let worst = 0;
    for (let i = -20000; i <= 20000; i++) {
      const a = i * 0.00173;
      worst = Math.max(worst, Math.abs(sin(a) - Math.sin(a)), Math.abs(cos(a) - Math.cos(a)));
    }
    expect(worst).toBeLessThan(1e-13);
  });

  it('matches Math.atan and Math.atan2 to 1e-13', () => {
    let worst = 0;
    for (let i = -3000; i <= 3000; i++) {
      const v = i * 0.0137;
      worst = Math.max(worst, Math.abs(atan(v) - Math.atan(v)));
      worst = Math.max(worst, Math.abs(atan(v * v * v) - Math.atan(v * v * v)));
    }
    for (let i = 0; i < 720; i++) {
      const a = (i / 720) * 2 * Math.PI - Math.PI + 1e-3;
      worst = Math.max(worst, Math.abs(atan2(Math.sin(a), Math.cos(a)) - a));
    }
    expect(worst).toBeLessThan(1e-13);
  });

  it('handles the axes and the origin', () => {
    expect(atan2(0, 0)).toBe(0);
    expect(atan2(1, 0)).toBeCloseTo(Math.PI / 2, 15);
    expect(atan2(-1, 0)).toBeCloseTo(-Math.PI / 2, 15);
    expect(atan2(0, -1)).toBeCloseTo(Math.PI, 15);
    expect(sin(0)).toBe(0);
    expect(cos(0)).toBe(1);
  });
});

describe('convex hull', () => {
  it('orders points counter-clockwise and drops interior and collinear ones', () => {
    const hull = convexHull([
      { x: 1, y: 1 },
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 1, y: 0 }, // collinear
      { x: 2, y: 2 },
      { x: 0, y: 2 },
      { x: 1, y: 0.5 }, // interior
    ]);
    expect(hull).toHaveLength(4);
    let area = 0;
    for (let i = 0; i < hull.length; i++) {
      const a = hull[i];
      const b = hull[(i + 1) % hull.length];
      area += a.x * b.y - a.y * b.x;
    }
    expect(area / 2).toBeCloseTo(4, 12);
  });

  it('gives the same hull whatever the input order', () => {
    const rnd = new Random(3);
    const points = Array.from({ length: 6 }, () => ({ x: rnd.range(-1, 1), y: rnd.range(-1, 1) }));
    const a = convexHull(points);
    const b = convexHull([...points].reverse());
    expect(b).toEqual(a);
  });

  it('rejects degenerate polygons', () => {
    expect(() => new Shape(polygon([{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }]))).toThrow();
    expect(() => new Shape(circle(0))).toThrow();
  });
});

describe('mass properties', () => {
  it('computes the mass and inertia of a box analytically', () => {
    const shape = new Shape(box(1.5, 0.5, { density: 2 }));
    const data = shape.computeMass();
    const mass = 2 * 3 * 1;
    expect(data.mass).toBeCloseTo(mass, 12);
    expect(data.centerX).toBeCloseTo(0, 12);
    expect(data.inertia).toBeCloseTo((mass * (3 * 3 + 1 * 1)) / 12, 12);
  });

  it('applies the parallel axis theorem to an off-centre box', () => {
    const shape = new Shape(box(0.5, 0.5, { center: { x: 3, y: -1 } }));
    const data = shape.computeMass();
    expect(data.centerX).toBeCloseTo(3, 12);
    expect(data.centerY).toBeCloseTo(-1, 12);
    expect(data.inertia).toBeCloseTo(1 / 6 + 1 * (9 + 1), 12);
  });

  it('computes the mass and inertia of a disc', () => {
    const data = new Shape(circle(2, { density: 3 })).computeMass();
    const mass = 3 * Math.PI * 4;
    expect(data.mass).toBeCloseTo(mass, 12);
    expect(data.inertia).toBeCloseTo(0.5 * mass * 4, 12);
  });
});

describe('random numbers and hashing', () => {
  it('produces the same sequence for the same seed, in [0, 1)', () => {
    const a = new Random(42);
    const b = new Random(42);
    const c = new Random(43);
    let differs = false;
    for (let i = 0; i < 1000; i++) {
      const value = a.next();
      expect(value).toBe(b.next());
      expect(value).toBeGreaterThanOrEqual(0);
      expect(value).toBeLessThan(1);
      if (value !== c.next()) differs = true;
    }
    expect(differs).toBe(true);
  });

  it('changes the digest when a single bit of the input changes', () => {
    const digest = (values: number[]) => {
      const hash = new StateHash();
      for (const v of values) hash.add(v);
      return hash.digest();
    };
    const base = digest([1, 2, 3]);
    expect(digest([1, 2, 3])).toBe(base);
    expect(digest([1, 2, 3 + 4.440892098500626e-16])).not.toBe(base);
    expect(digest([2, 1, 3])).not.toBe(base);
    expect(digest([0, 2, 3])).not.toBe(digest([-0, 2, 3]));
  });
});
