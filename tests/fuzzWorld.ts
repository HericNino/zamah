import { type Body, box, circle, Random, regularPolygon, settings, type ShapeDef, World } from '../src/engine/index.ts';
import { allFinite } from './helpers.ts';

const HALF = 10;

const WALL = 10;

/**
 * A closed room, 20 m on a side. The walls are 10 m thick: the engine has no continuous
 * collision detection, so what it promises is that nothing gets through a *thick* wall.
 */
function room(world: World): void {
  const outer = HALF + WALL;
  for (const [x, y, hw, hh] of [
    [0, -HALF - WALL / 2, outer, WALL / 2],
    [0, HALF + WALL / 2, outer, WALL / 2],
    [-HALF - WALL / 2, 0, WALL / 2, outer],
    [HALF + WALL / 2, 0, WALL / 2, outer],
  ]) {
    world.createBody({ type: 'static', position: { x, y }, shapes: [box(hw, hh)] });
  }
}

function randomShape(rnd: Random): ShapeDef {
  const material = { density: rnd.range(0.2, 5), friction: rnd.next(), restitution: rnd.chance(0.3) ? rnd.next() : 0 };
  const kind = rnd.int(3);
  if (kind === 0) return circle(rnd.range(0.15, 0.8), material);
  if (kind === 1) return box(rnd.range(0.1, 0.9), rnd.range(0.1, 0.9), material);
  return regularPolygon(3 + rnd.int(6), rnd.range(0.2, 0.8), material);
}

function randomBody(world: World, rnd: Random): Body {
  // Positions are fully random: bodies regularly start inside each other and inside walls.
  return world.createBody({
    position: { x: rnd.range(-HALF + 1, HALF - 1), y: rnd.range(-HALF + 1, HALF - 1) },
    angle: rnd.range(-3, 3),
    linearVelocity: { x: rnd.range(-25, 25), y: rnd.range(-25, 25) },
    angularVelocity: rnd.range(-15, 15),
    shapes: rnd.chance(0.15) ? [randomShape(rnd), box(0.6, 0.1, { center: { x: 0.5, y: 0 } })] : [randomShape(rnd)],
  });
}

/** Builds a random world and pokes it at random while it runs. Returns the final hash. */
export function fuzz(seed: number, steps: number, inspect?: (world: World, step: number) => void): string {
  const rnd = new Random(seed);
  const world = new World({ gravity: { x: rnd.range(-3, 3), y: rnd.chance(0.2) ? 0 : -10 }, substeps: 2 + rnd.int(5) });
  room(world);
  const dynamic = (): Body[] => world.bodies.filter((b) => b.isDynamic);

  const count = 5 + rnd.int(45);
  for (let i = 0; i < count; i++) randomBody(world, rnd);

  // Random joints between bodies that are near each other, hinged somewhere between them.
  const bodies = dynamic();
  for (let i = rnd.int(12); i > 0; i--) {
    const a = bodies[rnd.int(bodies.length)];
    let b: Body | null = null;
    for (const candidate of bodies) {
      if (candidate === a) continue;
      const d = Math.hypot(candidate.x - a.x, candidate.y - a.y);
      if (d < 3 && (b === null || d < Math.hypot(b.x - a.x, b.y - a.y))) b = candidate;
    }
    if (b === null) continue;
    if (rnd.chance(0.5)) {
      const t = rnd.next();
      world.createRevoluteJoint({
        bodyA: a,
        bodyB: b,
        anchor: { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) },
        enableLimit: rnd.chance(0.4),
        lowerAngle: -rnd.next(),
        upperAngle: rnd.next(),
        enableMotor: rnd.chance(0.3),
        motorSpeed: rnd.range(-8, 8),
        maxMotorTorque: rnd.range(0, 200),
      });
    } else {
      const modes = ['rod', 'rope', 'spring'] as const;
      world.createDistanceJoint({
        bodyA: a,
        bodyB: b,
        anchorA: { x: a.x, y: a.y },
        anchorB: { x: b.x, y: b.y },
        mode: modes[rnd.int(3)],
        hertz: rnd.range(0.5, 20),
        dampingRatio: rnd.next(),
      });
    }
  }

  if (rnd.chance(0.4)) {
    world.soft.createCloth({
      x: rnd.range(-8, 2),
      y: rnd.range(0, 8),
      cols: 6 + rnd.int(12),
      rows: 4 + rnd.int(10),
      spacing: rnd.range(0.15, 0.3),
      tearRatio: rnd.chance(0.5) ? rnd.range(1.5, 4) : 0,
      pin: rnd.chance(0.5) ? 'top' : 'corners',
    });
  }
  if (rnd.chance(0.3)) world.soft.createBlob({ x: rnd.range(-6, 6), y: rnd.range(-6, 6), radius: rnd.range(0.5, 1.5) });
  if (rnd.chance(0.4)) {
    world.fluid.configure({ spacing: rnd.range(0.18, 0.3), density: rnd.range(0.3, 3) });
    world.fluid.addBlock(rnd.range(-9, 0), rnd.range(-9, 0), rnd.range(2, 5), rnd.range(2, 5));
  }

  // A body may be caught part of the way into a wall, but must never come out the far side.
  const limit = HALF + WALL / 2;
  for (let step = 0; step < steps; step++) {
    // The kind of thing a user does with the mouse, at random moments.
    const roll = rnd.next();
    if (roll < 0.02) {
      const x = rnd.range(-HALF, HALF);
      const y = rnd.range(-HALF, HALF);
      // A blast: up to 60 m/s of extra speed, away from a random point.
      for (const body of dynamic()) {
        const dx = body.x - x;
        const dy = body.y - y;
        const kick = Math.min(60, 20 * Math.hypot(dx, dy)) / Math.max(Math.hypot(dx, dy), 1e-6);
        body.applyImpulse(dx * kick * body.mass, dy * kick * body.mass);
      }
    } else if (roll < 0.04) {
      const all = dynamic();
      if (all.length > 0) world.destroyBody(all[rnd.int(all.length)]);
    } else if (roll < 0.07) {
      randomBody(world, rnd);
    } else if (roll < 0.08) {
      world.setGravity(rnd.range(-10, 10), rnd.range(-10, 10));
    } else if (roll < 0.1) {
      world.soft.cut(rnd.range(-HALF, HALF), rnd.range(-HALF, HALF), rnd.range(-HALF, HALF), rnd.range(-HALF, HALF));
    } else if (roll < 0.11 && world.joints.length > 0) {
      world.destroyJoint(world.joints[rnd.int(world.joints.length)]);
    } else if (roll < 0.13 && world.fluid.count < 1200) {
      for (let k = 0; k < 20; k++) world.fluid.addParticle(rnd.range(-1, 1), rnd.range(-1, 1), rnd.range(-30, 30), rnd.range(-30, 30));
    }

    world.step();
    inspect?.(world, step);

    if (!allFinite(world)) throw new Error(`seed ${seed}: non-finite state at step ${step}`);
    for (const body of world.bodies) {
      const speed = Math.hypot(body.vx, body.vy);
      if (speed > settings.MAX_LINEAR_SPEED + 1e-6) throw new Error(`seed ${seed}: body at ${speed} m/s at step ${step}`);
      if (Math.abs(body.c * body.c + body.s * body.s - 1) > 1e-9) throw new Error(`seed ${seed}: rotation not unit length`);
      // Nothing may get out of a closed room, however hard it is thrown.
      if (Math.abs(body.x) > limit || Math.abs(body.y) > limit) {
        throw new Error(`seed ${seed}: body escaped to (${body.x}, ${body.y}) at step ${step}`);
      }
    }
  }
  world.broadphase.tree.validate();
  return world.hash();
}

