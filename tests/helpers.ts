import { type Body, box, type World } from '../src/engine/index.ts';

/** A wide static floor whose top surface is at y = 0. */
export function ground(world: World, halfWidth = 50): Body {
  return world.createBody({ type: 'static', position: { x: 0, y: -1 }, shapes: [box(halfWidth, 1)] });
}

export function run(world: World, steps: number): void {
  for (let i = 0; i < steps; i++) world.step();
}

/** True when every number in the dynamic state is finite. */
export function allFinite(world: World): boolean {
  for (const b of world.bodies) {
    if (!Number.isFinite(b.x + b.y + b.c + b.s + b.vx + b.vy + b.w)) return false;
  }
  const { soft, fluid } = world;
  for (let i = 0; i < soft.count; i++) {
    if (!Number.isFinite(soft.x[i] + soft.y[i] + soft.vx[i] + soft.vy[i])) return false;
  }
  for (let i = 0; i < fluid.count; i++) {
    if (!Number.isFinite(fluid.x[i] + fluid.y[i] + fluid.vx[i] + fluid.vy[i])) return false;
  }
  return true;
}

export function linearMomentum(world: World): { x: number; y: number } {
  let x = 0;
  let y = 0;
  for (const b of world.bodies) {
    x += b.mass * b.vx;
    y += b.mass * b.vy;
  }
  return { x, y };
}

/** Angular momentum of all bodies about the origin. */
export function angularMomentum(world: World): number {
  let total = 0;
  for (const b of world.bodies) total += b.inertia * b.w + b.mass * (b.x * b.vy - b.y * b.vx);
  return total;
}

export function maxSpeed(world: World): number {
  let max = 0;
  for (const b of world.bodies) max = Math.max(max, Math.hypot(b.vx, b.vy));
  return max;
}
