import { describe, expect, it } from 'vitest';
import { box, circle, World } from '../src/engine/index.ts';
import { allFinite, ground, run } from './helpers.ts';

describe('XPBD constraints', () => {
  it('a weight on a compliant thread stretches it by m g * compliance, whatever the sub-step count', () => {
    const mass = 0.5;
    const compliance = 0.01; // metres per newton
    const expected = mass * 10 * compliance;
    for (const substeps of [4, 10, 40]) {
      const world = new World();
      const soft = world.soft;
      soft.substeps = substeps;
      soft.damping = 4; // let it come to rest quickly
      const top = soft.addParticle(0, 0, 0, 0.05, 0);
      const bottom = soft.addParticle(0, -1, mass, 0.05, 0);
      soft.addDistance(top, bottom, compliance);
      run(world, 900);
      const stretch = -soft.y[bottom] - 1;
      // This is the point of XPBD: stiffness is a physical quantity, not a side effect of
      // how many iterations happen to run.
      expect(stretch / expected).toBeCloseTo(1, 2);
    }
  });

  it('an inextensible thread keeps its length while swinging', () => {
    const world = new World();
    const soft = world.soft;
    const top = soft.addParticle(0, 0, 0, 0.05, 0);
    const bob = soft.addParticle(2, 0, 1, 0.05, 0);
    soft.addDistance(top, bob, 0);
    let worst = 0;
    for (let i = 0; i < 300; i++) {
      world.step();
      worst = Math.max(worst, Math.abs(Math.hypot(soft.x[bob], soft.y[bob]) - 2));
    }
    expect(worst).toBeLessThan(1e-3);
    expect(soft.y[bob]).toBeLessThan(0);
  });
});

describe('cloth', () => {
  it('hangs at rest, stretched by less than half a percent', () => {
    const world = new World();
    const cloth = world.soft.createCloth({ x: -3, y: 8, cols: 31, rows: 21, spacing: 0.2 });
    run(world, 600);
    const soft = world.soft;
    let fastest = 0;
    for (let i = 0; i < soft.count; i++) fastest = Math.max(fastest, Math.hypot(soft.vx[i], soft.vy[i]));
    expect(fastest).toBeLessThan(1e-3);
    const bottom = soft.y[cloth.start + 20 * 31 + 15];
    const length = 8 - bottom;
    expect(length).toBeGreaterThan(4);
    expect(length / 4).toBeLessThan(1.005);
  });

  it('drapes over a static shape instead of passing through it', () => {
    const world = new World();
    ground(world);
    world.createBody({ type: 'static', position: { x: 0, y: 3 }, shapes: [circle(1.5)] });
    world.soft.createCloth({ x: -3, y: 6, cols: 31, rows: 16, spacing: 0.2, pin: 'none' });
    run(world, 600);
    const soft = world.soft;
    for (let i = 0; i < soft.count; i++) {
      // No particle is inside the disc or below the floor.
      expect(Math.hypot(soft.x[i], soft.y[i] - 3)).toBeGreaterThan(1.5 + 0.1 - 0.02);
      expect(soft.y[i]).toBeGreaterThan(0.08);
    }
    expect(allFinite(world)).toBe(true);
  });

  it('tears when overstretched, and only then', () => {
    const world = new World();
    const soft = world.soft;
    const cloth = soft.createCloth({ x: 0, y: 5, cols: 11, rows: 11, spacing: 0.2, tearRatio: 2.5 });
    run(world, 120);
    const intact = soft.constraintCount;
    expect(intact).toBe(110 + 110 + 200 + 99 + 99);

    // Grab the bottom corner and pull it far away, the way the mouse does.
    const corner = cloth.start + 10 * 11 + 10;
    soft.invMass[corner] = 0;
    for (let i = 0; i < 120; i++) {
      soft.x[corner] += 0.08;
      soft.y[corner] -= 0.05;
      world.step();
    }
    expect(soft.constraintCount).toBeLessThan(intact);
    expect(allFinite(world)).toBe(true);
  });

  it('can be cut along a line', () => {
    const world = new World();
    const soft = world.soft;
    soft.createCloth({ x: 0, y: 5, cols: 11, rows: 11, spacing: 0.2 });
    const before = soft.constraintCount;
    // A horizontal cut just below the second row severs the sheet completely.
    const cuts = soft.cut(-1, 4.7, 3, 4.7);
    expect(cuts).toBeGreaterThanOrEqual(11);
    expect(soft.constraintCount).toBe(before - cuts);
    run(world, 240);
    // The lower part has fallen away from the pinned strip.
    let lowest = Infinity;
    for (let i = 0; i < soft.count; i++) lowest = Math.min(lowest, soft.y[i]);
    expect(lowest).toBeLessThan(-10);
  });
});

describe('soft bodies', () => {
  it('a pressurised blob keeps its area within a few percent while resting on the floor', () => {
    const world = new World();
    ground(world);
    const blob = world.soft.createBlob({ x: 0, y: 2, radius: 1 });
    const soft = world.soft;
    const area = () => {
      let sum = 0;
      for (let k = 0; k < blob.count; k++) {
        const i = blob.start + k;
        const j = blob.start + ((k + 1) % blob.count);
        sum += soft.x[i] * soft.y[j] - soft.x[j] * soft.y[i];
      }
      return sum / 2;
    };
    const rest = area();
    let smallest = rest;
    for (let i = 0; i < 600; i++) {
      world.step();
      smallest = Math.min(smallest, area());
    }
    expect(smallest / rest).toBeGreaterThan(0.9);
    expect(area() / rest).toBeGreaterThan(0.97);
    expect(area() / rest).toBeLessThan(1.03);
    // It sits on the floor, slightly flattened, not sunk into it.
    let lowest = Infinity;
    for (let k = 0; k < blob.count; k++) lowest = Math.min(lowest, soft.y[blob.start + k]);
    expect(lowest).toBeGreaterThan(0.1);
  });

  it('two blobs collide: no particle of one ever gets inside the other', () => {
    const world = new World();
    ground(world);
    const lower = world.soft.createBlob({ x: 0, y: 1.2, radius: 1 });
    const upper = world.soft.createBlob({ x: 0.1, y: 3.6, radius: 0.8 });
    const soft = world.soft;
    const inside = (px: number, py: number, g: typeof lower): boolean => {
      // Even-odd ray casting against the blob's outline.
      let hit = false;
      for (let k = 0; k < g.count; k++) {
        const i = g.start + k;
        const j = g.start + ((k + 1) % g.count);
        if (soft.y[i] > py !== soft.y[j] > py) {
          const t = (py - soft.y[i]) / (soft.y[j] - soft.y[i]);
          if (px < soft.x[i] + t * (soft.x[j] - soft.x[i])) hit = !hit;
        }
      }
      return hit;
    };
    let touched = false;
    for (let step = 0; step < 400; step++) {
      world.step();
      for (let k = 0; k < upper.count; k++) {
        expect(inside(soft.x[upper.start + k], soft.y[upper.start + k], lower)).toBe(false);
      }
      const gap = Math.min(...Array.from({ length: upper.count }, (_, k) => soft.y[upper.start + k])) - 2.2;
      if (gap < 0) touched = true;
    }
    expect(touched).toBe(true);
  });
});

describe('coupling with rigid bodies', () => {
  function hammock() {
    const world = new World();
    const soft = world.soft;
    const strip = soft.createCloth({ x: -3, y: 5, cols: 31, rows: 4, spacing: 0.2, pin: 'none', mass: 0.05 });
    for (let r = 0; r < 4; r++) {
      soft.invMass[strip.start + r * 31] = 0;
      soft.invMass[strip.start + r * 31 + 30] = 0;
    }
    return world;
  }

  it('a taut strip of cloth catches a falling ball and holds it', () => {
    const world = hammock();
    const ball = world.createBody({ position: { x: 0, y: 8 }, shapes: [circle(0.6)] });
    let highestAfterLanding = -Infinity;
    for (let i = 0; i < 900; i++) {
      world.step();
      if (i > 120) highestAfterLanding = Math.max(highestAfterLanding, ball.y);
    }
    // Held near the cloth line (y = 5), neither fallen through nor thrown back up.
    expect(ball.y).toBeGreaterThan(4.6);
    expect(highestAfterLanding).toBeLessThan(6.2);
    expect(Math.abs(ball.vy)).toBeLessThan(0.5);
  });

  it('a heavy box drags a light curtain along without being stopped by it', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    world.soft.createCloth({ x: 3, y: 2, cols: 3, rows: 21, spacing: 0.2, pin: 'none', mass: 0.01 });
    const crate = world.createBody({ position: { x: 0, y: 0 }, linearVelocity: { x: 5, y: 0 }, shapes: [box(0.5, 0.5, { density: 4 })] });
    run(world, 120);
    // 63 particles of 10 g against a 4 kg box: momentum says it keeps about 86% of its speed.
    expect(crate.vx).toBeGreaterThan(3.6);
    expect(crate.vx).toBeLessThan(5);
    const soft = world.soft;
    let moved = 0;
    for (let i = 0; i < soft.count; i++) if (soft.x[i] > 4.5) moved++;
    expect(moved).toBeGreaterThan(10);
  });
});
