import { describe, expect, it } from 'vitest';
import { RevoluteJoint, World } from '../src/engine/index.ts';
import { sceneById, scenes } from '../src/scenes/index.ts';
import { allFinite, maxSpeed, run } from './helpers.ts';

function build(id: string): World {
  const scene = sceneById(id)!;
  const world = new World(scene.options);
  scene.build(world);
  return world;
}

/** Everything the sandbox shows is exercised here, headless, with the same code. */
describe('sandbox scenes', () => {
  it('every scene runs for ten seconds without NaN and keeps everything inside its arena', () => {
    for (const scene of scenes) {
      const world = build(scene.id);
      for (let step = 0; step < 600; step += 60) {
        run(world, scene.id === 'fluid' ? 30 : 60);
        expect(allFinite(world), scene.id).toBe(true);
      }
      for (const body of world.bodies) {
        if (!body.isDynamic) continue;
        expect(body.y, `${scene.id}: body below the floor`).toBeGreaterThan(-0.5);
        expect(Math.abs(body.x), `${scene.id}: body outside the walls`).toBeLessThan(scene.view.maxX + 8);
      }
    }
  });

  it('wrecking ball: the wall stands until the ball arrives, then comes down', () => {
    const world = build('tower');
    const bricks = world.bodies.filter((b) => b.isDynamic && b.shapes[0].kind === 'polygon');
    const start = bricks.map((b) => ({ x: b.x, y: b.y }));
    run(world, 80);
    // Fourteen rows of soft contacts settle by a few centimetres, nothing more.
    bricks.forEach((b, i) => expect(Math.hypot(b.x - start[i].x, b.y - start[i].y)).toBeLessThan(0.15));
    run(world, 220);
    const displaced = bricks.filter((b, i) => Math.hypot(b.x - start[i].x, b.y - start[i].y) > 1).length;
    expect(displaced).toBeGreaterThan(15);
  });

  it('pyramid: stands still and falls asleep', () => {
    const world = build('pyramid');
    const boxes = world.bodies.filter((b) => b.isDynamic);
    const start = boxes.map((b) => ({ x: b.x, y: b.y }));
    run(world, 300);
    boxes.forEach((b, i) => expect(Math.hypot(b.x - start[i].x, b.y - start[i].y)).toBeLessThan(0.2));
    expect(boxes.every((b) => !b.awake)).toBe(true);
  });

  it("Newton's cradle: after the first impact only the last ball moves", () => {
    const world = build('cradle');
    const balls = world.bodies.filter((b) => b.isDynamic);
    let impact = false;
    for (let i = 0; i < 200 && !impact; i++) {
      const before = balls[0].vx;
      world.step();
      impact = before > 1 && balls[0].vx < 0.05;
    }
    expect(impact).toBe(true);
    expect(balls[balls.length - 1].vx).toBeGreaterThan(4);
    for (let i = 0; i < balls.length - 1; i++) expect(Math.abs(balls[i].vx)).toBeLessThan(0.05);
  });

  it('cloth: drapes over the disc and catches what falls on it without passing through', () => {
    const world = build('cloth');
    run(world, 600);
    const soft = world.soft;
    for (let i = 0; i < soft.count; i++) {
      expect(Math.hypot(soft.x[i] + 1.5, soft.y[i] - 3.6)).toBeGreaterThan(1.1);
      expect(soft.y[i]).toBeGreaterThan(0);
    }
    // Nothing was torn by its own weight or by the falling bodies.
    expect(soft.constraintCount).toBeGreaterThan(6900);
  });

  it('dam break: liquid stays in the tank, light crates float, the heavy one sinks', () => {
    const world = build('fluid');
    const fluid = world.fluid;
    const count = fluid.count;
    const crates = world.bodies.filter((b) => b.isDynamic && b.shapes.length === 1 && b.shapes[0].kind === 'polygon');
    expect(crates).toHaveLength(4);
    run(world, 600);
    expect(fluid.count).toBe(count);
    for (let i = 0; i < count; i++) {
      expect(Math.abs(fluid.x[i])).toBeLessThan(9.01);
      expect(fluid.y[i]).toBeGreaterThan(-0.01);
    }
    for (const crate of crates) {
      if (crate.shapes[0].density > 1) expect(crate.y).toBeLessThan(0.8);
      else expect(crate.y).toBeGreaterThan(1.5);
    }
  });

  it('bridge: every hinge holds and the ragdolls stay in one piece', () => {
    const world = build('bridge');
    let worst = 0;
    for (let i = 0; i < 600; i++) {
      world.step();
      for (const joint of world.joints) {
        if (!(joint instanceof RevoluteJoint)) continue;
        const a = joint.bodyA.worldPoint(joint.localAx, joint.localAy);
        const b = joint.bodyB.worldPoint(joint.localBx, joint.localBy);
        worst = Math.max(worst, Math.hypot(a.x - b.x, a.y - b.y));
      }
    }
    expect(worst).toBeLessThan(0.05);
    expect(maxSpeed(world)).toBeLessThan(20);
  });

  it('soft bodies: blobs keep their volume and the hammock holds its load', () => {
    const world = build('blobs');
    const soft = world.soft;
    const blobs = soft.groups.filter((g) => g.kind === 'blob');
    const area = (g: (typeof blobs)[number]) => {
      let sum = 0;
      for (let k = 0; k < g.count; k++) {
        const i = g.start + k;
        const j = g.start + ((k + 1) % g.count);
        sum += soft.x[i] * soft.y[j] - soft.x[j] * soft.y[i];
      }
      return sum / 2;
    };
    const rest = blobs.map(area);
    run(world, 600);
    blobs.forEach((g, i) => {
      expect(area(g) / rest[i]).toBeGreaterThan(0.9);
      expect(area(g) / rest[i]).toBeLessThan(1.15);
    });
    // The box and the ball dropped into the hammock are still up there.
    const load = world.bodies.filter((b) => b.isDynamic);
    expect(load).toHaveLength(2);
    for (const body of load) expect(body.y).toBeGreaterThan(4);
  });
});
