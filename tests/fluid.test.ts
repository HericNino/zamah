import { describe, expect, it } from 'vitest';
import { box, circle, World } from '../src/engine/index.ts';
import { allFinite, run } from './helpers.ts';

/** A tank 8 m wide: floor at y = 0, walls at x = -4 and x = 4. */
function tank(): World {
  const world = new World();
  world.createBody({ type: 'static', position: { x: 0, y: -0.5 }, shapes: [box(6, 0.5)] });
  world.createBody({ type: 'static', position: { x: -4.5, y: 5 }, shapes: [box(0.5, 6)] });
  world.createBody({ type: 'static', position: { x: 4.5, y: 5 }, shapes: [box(0.5, 6)] });
  // Coarser than the sandbox, to keep the tests quick.
  world.fluid.configure({ spacing: 0.16 });
  return world;
}

describe('position based fluid', () => {
  it('a block placed at rest spacing starts at rest density', () => {
    const world = tank();
    const fluid = world.fluid;
    fluid.addBlock(-2, 2, 4, 4);
    // Zero gravity: the inside of a block that is already at rest density should not move.
    // (Its free surface does creep outwards a little; that is the anti-clumping term.)
    world.setGravity(0, 0);
    world.step();
    let checked = 0;
    for (let i = 0; i < fluid.count; i++) {
      if (Math.abs(fluid.x[i]) > 1 || Math.abs(fluid.y[i] - 4) > 1) continue;
      checked++;
      expect(fluid.density[i]).toBeGreaterThan(0.98);
      expect(fluid.density[i]).toBeLessThan(1.02);
      expect(Math.hypot(fluid.vx[i], fluid.vy[i])).toBeLessThan(0.02);
    }
    expect(checked).toBeGreaterThan(100);
  });

  it('survives abuse: particles poured on top of each other do not explode', () => {
    const world = tank();
    const fluid = world.fluid;
    // A lid, so "still inside the tank" is a fair thing to ask of particles moving this fast.
    world.createBody({ type: 'static', position: { x: 0, y: 11.5 }, shapes: [box(6, 0.5)] });
    // 400 particles in the same few spots, plus a blast: far outside what the solver likes.
    for (let i = 0; i < 400; i++) fluid.addParticle((i % 4) * 0.01, 2 + (i % 7) * 0.01, i % 2 ? 40 : -40, 30);
    let fastest = 0;
    for (let step = 0; step < 240; step++) {
      world.step();
      for (let i = 0; i < fluid.count; i++) fastest = Math.max(fastest, Math.hypot(fluid.vx[i], fluid.vy[i]));
    }
    expect(allFinite(world)).toBe(true);
    // Nothing moves faster than the solver's own speed limit of one spacing per sub-step.
    expect(fastest).toBeLessThanOrEqual((fluid.spacing * fluid.substeps) / world.dt + 1e-9);
    for (let i = 0; i < fluid.count; i++) {
      expect(Math.abs(fluid.x[i])).toBeLessThan(4.01);
      expect(fluid.y[i]).toBeGreaterThan(-0.01);
      expect(fluid.y[i]).toBeLessThan(11.01);
    }
  });

  it('a dam break settles into a level pool at rest density, with every particle inside the tank', () => {
    const world = tank();
    const fluid = world.fluid;
    const count = fluid.addBlock(-4, 0, 4, 5);
    expect(count).toBeGreaterThan(800);
    let peakSpeed = 0;
    for (let step = 0; step < 900; step++) {
      world.step();
      for (let i = 0; i < fluid.count; i += 7) peakSpeed = Math.max(peakSpeed, Math.hypot(fluid.vx[i], fluid.vy[i]));
    }
    expect(fluid.count).toBe(count);
    expect(allFinite(world)).toBe(true);
    // It really did collapse and flow ...
    expect(peakSpeed).toBeGreaterThan(4);

    let density = 0;
    let speed = 0;
    let top = -Infinity;
    for (let i = 0; i < count; i++) {
      // ... and nothing leaked through the walls or the floor.
      expect(Math.abs(fluid.x[i])).toBeLessThan(4 + 0.01);
      expect(fluid.y[i]).toBeGreaterThan(-0.01);
      density += fluid.density[i];
      speed += Math.hypot(fluid.vx[i], fluid.vy[i]);
      top = Math.max(top, fluid.y[i]);
    }
    expect(density / count).toBeGreaterThan(0.97);
    expect(density / count).toBeLessThan(1.03);
    expect(speed / count).toBeLessThan(0.15);
    // Incompressible: 4 m x 5 m of liquid spread over 8 m is 2.5 m deep.
    expect(top).toBeGreaterThan(2.3);
    expect(top).toBeLessThan(2.9);
  });

  it('light bodies float, heavy bodies sink', () => {
    const world = tank();
    world.fluid.addBlock(-4, 0, 8, 3);
    const light = world.createBody({ position: { x: -2, y: 5 }, shapes: [box(0.5, 0.5, { density: 0.4 })] });
    const heavy = world.createBody({ position: { x: 2, y: 5 }, shapes: [box(0.5, 0.5, { density: 3 })] });
    const cork = world.createBody({ position: { x: 0, y: 5 }, shapes: [circle(0.4, { density: 0.3 })] });
    run(world, 600);
    // Pool surface is at about y = 3.3 with the bodies in it.
    expect(heavy.y).toBeLessThan(0.75);
    expect(light.y).toBeGreaterThan(2.6);
    expect(light.y).toBeLessThan(3.8);
    expect(cork.y).toBeGreaterThan(2.6);
    expect(cork.y).toBeLessThan(3.8);
    expect(allFinite(world)).toBe(true);
  });

  it('removing particles keeps the rest intact', () => {
    const world = tank();
    const fluid = world.fluid;
    fluid.addBlock(-4, 0, 8, 2);
    const before = fluid.count;
    const removed = fluid.removeOutside(-10, -10, 0, 10);
    expect(removed).toBeGreaterThan(before / 3);
    expect(fluid.count).toBe(before - removed);
    for (let i = 0; i < fluid.count; i++) expect(fluid.x[i]).toBeLessThanOrEqual(0);
    run(world, 120);
    expect(allFinite(world)).toBe(true);
  });
});
