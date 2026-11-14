import { describe, expect, it } from 'vitest';
import { type Body, box, circle, World } from '../src/engine/index.ts';
import { ground, maxSpeed, run } from './helpers.ts';

function stack(world: World, count: number): Body[] {
  const boxes: Body[] = [];
  for (let i = 0; i < count; i++) {
    boxes.push(world.createBody({ position: { x: 0, y: 0.5 + i }, shapes: [box(0.5, 0.5)] }));
  }
  return boxes;
}

function pyramid(world: World, base: number): Body[] {
  const boxes: Body[] = [];
  for (let row = 0; row < base; row++) {
    for (let i = 0; i < base - row; i++) {
      boxes.push(
        world.createBody({ position: { x: (i - (base - row - 1) / 2) * 1.02, y: 0.5 + row }, shapes: [box(0.5, 0.5)] }),
      );
    }
  }
  return boxes;
}

describe('stacking', () => {
  it('a stack of twelve boxes stands for 3,000 steps without sleeping to help it', () => {
    const world = new World({ sleeping: false });
    ground(world);
    const boxes = stack(world, 12);
    run(world, 3000);
    for (let i = 0; i < boxes.length; i++) {
      // Soft contacts let each box sink up to about a centimetre into the one below.
      expect(Math.abs(boxes[i].x)).toBeLessThan(0.05);
      expect(boxes[i].y).toBeGreaterThan(0.5 + i - 0.012 * (i + 1));
      expect(boxes[i].y).toBeLessThan(0.5 + i + 1e-6);
      expect(Math.abs(boxes[i].angle)).toBeLessThan(0.01);
    }
    // Not merely "has not fallen yet": it has come to a complete stop.
    expect(maxSpeed(world)).toBeLessThan(1e-4);
  });

  it('a 210-box pyramid settles to a standstill and stays there', () => {
    const world = new World({ sleeping: false });
    ground(world);
    const boxes = pyramid(world, 20);
    const start = boxes.map((b) => ({ x: b.x, y: b.y }));
    run(world, 1500);
    expect(maxSpeed(world)).toBeLessThan(1e-9);
    const settled = boxes.map((b) => ({ x: b.x, y: b.y }));
    run(world, 500);
    boxes.forEach((b, i) => {
      expect(Math.abs(b.x - start[i].x)).toBeLessThan(0.06);
      expect(Math.abs(b.y - start[i].y)).toBeLessThan(0.2);
      expect(Math.abs(b.x - settled[i].x)).toBeLessThan(1e-9);
      expect(Math.abs(b.y - settled[i].y)).toBeLessThan(1e-9);
    });
  });

  it('warm starting is what makes this work: without it the same stack sags several times deeper', () => {
    const sag = (warmStarting: boolean) => {
      const world = new World({ sleeping: false, warmStarting });
      ground(world);
      const boxes = stack(world, 12);
      run(world, 600);
      return 11.5 - boxes[11].y;
    };
    const withWarmStart = sag(true);
    const without = sag(false);
    expect(withWarmStart).toBeLessThan(0.12);
    expect(without).toBeGreaterThan(3 * withWarmStart);
  });

  it('a heavy box rests on a light one: mass ratio 40:1 by default, 100:1 with eight sub-steps', () => {
    const settle = (ratio: number, substeps: number) => {
      const world = new World({ sleeping: false, substeps });
      ground(world);
      const light = world.createBody({ position: { x: 0, y: 0.5 }, shapes: [box(0.5, 0.5, { density: 0.1 })] });
      const heavy = world.createBody({ position: { x: 0, y: 2 }, shapes: [box(1, 1, { density: 0.025 * ratio })] });
      expect(heavy.mass / light.mass).toBeCloseTo(ratio, 6);
      run(world, 1200);
      return { light, heavy, speed: maxSpeed(world) };
    };
    for (const [ratio, substeps] of [
      [40, 4],
      [100, 8],
    ]) {
      const { light, heavy, speed } = settle(ratio, substeps);
      expect(light.y).toBeGreaterThan(0.48);
      expect(heavy.y).toBeGreaterThan(1.93);
      expect(speed).toBeLessThan(1e-6);
    }
  });

  it('balls in a V-shaped trough come to rest instead of jittering forever', () => {
    const world = new World({ sleeping: false });
    world.createBody({ type: 'static', position: { x: -3, y: 2 }, angle: -0.5, shapes: [box(4, 0.3)] });
    world.createBody({ type: 'static', position: { x: 3, y: 2 }, angle: 0.5, shapes: [box(4, 0.3)] });
    for (let i = 0; i < 20; i++) {
      world.createBody({ position: { x: (i % 5) - 2, y: 5 + Math.floor(i / 5) * 1.1 }, shapes: [circle(0.4)] });
    }
    run(world, 1800);
    expect(maxSpeed(world)).toBeLessThan(0.01);
  });
});

describe('fast bodies', () => {
  it('a bullet at 100 m/s does not tunnel through a thin wall', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    world.createBody({ type: 'static', position: { x: 10, y: 0 }, shapes: [box(0.1, 5)] });
    // Moves 1.67 m per step, more than eight times the wall thickness.
    const bullet = world.createBody({ position: { x: 0.37, y: 0 }, linearVelocity: { x: 100, y: 0 }, shapes: [circle(0.1)] });
    run(world, 30);
    expect(bullet.x).toBeLessThan(9.9);
  });

  it('fast boxes thrown at a wall of boxes stay on their side of the floor', () => {
    const world = new World();
    ground(world, 2000);
    stack(world, 6);
    const thrown: Body[] = [];
    for (let i = 0; i < 5; i++) {
      thrown.push(
        world.createBody({
          position: { x: -12 - 3 * i, y: 6 + i },
          linearVelocity: { x: 60, y: -20 },
          angularVelocity: 10,
          shapes: [box(0.3, 0.3)],
        }),
      );
    }
    run(world, 600);
    for (const body of world.bodies) {
      if (body.isDynamic) expect(body.y).toBeGreaterThan(0.2);
    }
  });
});

describe('sleeping', () => {
  it('puts a resting stack to sleep and wakes it when something hits it', () => {
    const world = new World();
    ground(world);
    const boxes = stack(world, 5);
    run(world, 180);
    expect(boxes.every((b) => !b.awake)).toBe(true);
    expect(world.profile.solve).toBeLessThan(1);

    const ball = world.createBody({ position: { x: -6, y: 2.5 }, linearVelocity: { x: 15, y: 0 }, shapes: [circle(0.4, { density: 3 })] });
    let woke = false;
    for (let i = 0; i < 60; i++) {
      world.step();
      if (boxes.some((b) => b.awake)) woke = true;
    }
    expect(woke).toBe(true);
    expect(ball.awake || boxes.some((b) => Math.abs(b.x) > 0.05)).toBe(true);
    // The whole stack woke as one island, not just the box that was hit.
    expect(Math.max(...boxes.map((b) => Math.abs(b.x)))).toBeGreaterThan(0.05);
  });

  it('wakes a sleeping box when the box under it is removed', () => {
    const world = new World();
    ground(world);
    const [lower, upper] = stack(world, 2);
    run(world, 180);
    expect(upper.awake).toBe(false);
    world.destroyBody(lower);
    run(world, 120);
    expect(upper.y).toBeLessThan(0.6);
  });

  it('a sleeping body is bit-for-bit frozen', () => {
    const world = new World();
    ground(world);
    const [body] = stack(world, 1);
    run(world, 120);
    expect(body.awake).toBe(false);
    const before = world.hash();
    run(world, 500);
    expect(world.hash()).toBe(before);
  });

  it('waking by a new gravity direction works', () => {
    const world = new World();
    ground(world);
    const [body] = stack(world, 1);
    run(world, 120);
    world.setGravity(10, 0);
    run(world, 60);
    expect(body.x).toBeGreaterThan(0.5);
  });
});
