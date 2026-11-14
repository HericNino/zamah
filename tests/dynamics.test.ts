import { describe, expect, it } from 'vitest';
import { box, circle, Random, World } from '../src/engine/index.ts';
import { angularMomentum, ground, linearMomentum, run } from './helpers.ts';

describe('integration', () => {
  it('free fall follows the closed form of semi-implicit Euler exactly', () => {
    const world = new World();
    const ball = world.createBody({ position: { x: 0, y: 100 }, shapes: [circle(0.5)] });
    run(world, 60);
    // v = -g t is exact; position lags the continuous solution by g * h * t / 2.
    expect(ball.vy).toBeCloseTo(-10, 10);
    const h = world.dt / world.substeps;
    const n = 60 * world.substeps;
    expect(ball.y).toBeCloseTo(100 - (10 * h * h * n * (n + 1)) / 2, 9);
    expect(Math.abs(ball.y - 95)).toBeLessThan(0.025);
  });

  it('keeps rotations unit length and matches the analytic angle', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const body = world.createBody({ angularVelocity: 7.3, shapes: [box(1, 0.5)] });
    run(world, 600);
    expect(body.c * body.c + body.s * body.s).toBeCloseTo(1, 14);
    const expected = Math.atan2(Math.sin(73), Math.cos(73));
    expect(body.angle).toBeCloseTo(expected, 9);
  });
});

describe('conservation laws in ideal collisions', () => {
  it('conserves energy and momentum exactly in a head-on elastic collision', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const material = { restitution: 1, friction: 0 };
    const a = world.createBody({ position: { x: -2, y: 0 }, linearVelocity: { x: 3, y: 0 }, shapes: [circle(0.5, material)] });
    const b = world.createBody({ position: { x: 2, y: 0 }, linearVelocity: { x: -1, y: 0 }, shapes: [circle(0.3, material)] });
    const energy = world.kineticEnergy();
    const momentum = linearMomentum(world).x;
    const [m1, m2, u1, u2] = [a.mass, b.mass, 3, -1];
    run(world, 240);
    // Textbook result for a 1D elastic collision.
    expect(a.vx).toBeCloseTo(((m1 - m2) * u1 + 2 * m2 * u2) / (m1 + m2), 12);
    expect(b.vx).toBeCloseTo(((m2 - m1) * u2 + 2 * m1 * u1) / (m1 + m2), 12);
    expect(world.kineticEnergy()).toBeCloseTo(energy, 10);
    expect(linearMomentum(world).x).toBeCloseTo(momentum, 12);
  });

  it('conserves energy, linear and angular momentum in an oblique elastic collision', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const material = { restitution: 1, friction: 0 };
    world.createBody({ position: { x: -3, y: 0.4 }, linearVelocity: { x: 9, y: 0 }, shapes: [circle(0.5, material)] });
    world.createBody({ position: { x: 1, y: 0 }, shapes: [circle(0.7, material)] });
    const energy = world.kineticEnergy();
    const p = linearMomentum(world);
    const l = angularMomentum(world);
    run(world, 120);
    expect(world.contacts.size).toBe(0); // they hit and flew apart
    expect(world.bodies[1].vx).toBeGreaterThan(1);
    expect(Math.abs(world.bodies[1].vy)).toBeGreaterThan(0.5);
    expect(world.kineticEnergy() / energy).toBeCloseTo(1, 12);
    expect(linearMomentum(world).x).toBeCloseTo(p.x, 11);
    expect(linearMomentum(world).y).toBeCloseTo(p.y, 11);
    expect(angularMomentum(world)).toBeCloseTo(l, 11);
  });

  it('conserves energy when a disc hits the corner of a free box, setting it spinning', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const material = { restitution: 1, friction: 0 };
    world.createBody({ position: { x: -3, y: 0.9 }, linearVelocity: { x: 6, y: 0 }, shapes: [circle(0.4, material)] });
    const target = world.createBody({ position: { x: 1, y: 0 }, shapes: [box(0.8, 0.8, material)] });
    const energy = world.kineticEnergy();
    const p = linearMomentum(world);
    const l = angularMomentum(world);
    run(world, 120);
    expect(Math.abs(target.w)).toBeGreaterThan(0.1);
    expect(world.kineticEnergy() / energy).toBeCloseTo(1, 10);
    expect(linearMomentum(world).x).toBeCloseTo(p.x, 11);
    expect(angularMomentum(world)).toBeCloseTo(l, 10);
  });

  it('conserves momentum to rounding error through hundreds of messy collisions', () => {
    const rnd = new Random(1);
    const world = new World({ gravity: { x: 0, y: 0 } });
    for (let i = 0; i < 60; i++) {
      const material = { restitution: rnd.next(), friction: rnd.next() };
      world.createBody({
        position: { x: (i % 8) * 1.3 - 4.5, y: Math.floor(i / 8) * 1.3 - 4.5 },
        linearVelocity: { x: rnd.range(-5, 5), y: rnd.range(-5, 5) },
        angularVelocity: rnd.range(-4, 4),
        angle: rnd.range(0, 3),
        shapes: [rnd.chance(0.5) ? circle(rnd.range(0.2, 0.5), material) : box(rnd.range(0.2, 0.5), rnd.range(0.2, 0.5), material)],
      });
    }
    const p = linearMomentum(world);
    const l = angularMomentum(world);
    const energy = world.kineticEnergy();
    let hits = 0;
    for (let i = 0; i < 300; i++) {
      world.step();
      hits += world.activeContacts.length;
    }
    expect(hits).toBeGreaterThan(200);
    expect(linearMomentum(world).x).toBeCloseTo(p.x, 9);
    expect(linearMomentum(world).y).toBeCloseTo(p.y, 9);
    expect(angularMomentum(world)).toBeCloseTo(l, 8);
    // Inelastic, frictional contacts can only lose energy.
    expect(world.kineticEnergy()).toBeLessThan(energy);
  });

  it('never gains energy in a closed box of bouncing discs', () => {
    const rnd = new Random(2);
    // Threshold zero: by default, impacts slower than 1 m/s are deliberately inelastic.
    const world = new World({ gravity: { x: 0, y: 0 }, sleeping: false, restitutionThreshold: 0 });
    const wall = { restitution: 1, friction: 0 };
    for (const [x, y, hw, hh] of [
      [0, -6, 7, 1],
      [0, 6, 7, 1],
      [-6, 0, 1, 7],
      [6, 0, 1, 7],
    ]) {
      world.createBody({ type: 'static', position: { x, y }, shapes: [box(hw, hh, wall)] });
    }
    for (let i = 0; i < 36; i++) {
      world.createBody({
        position: { x: (i % 6) * 1.5 - 3.75, y: Math.floor(i / 6) * 1.5 - 3.75 },
        linearVelocity: { x: rnd.range(-8, 8), y: rnd.range(-8, 8) },
        shapes: [circle(rnd.range(0.2, 0.45), wall)],
      });
    }
    const start = world.kineticEnergy();
    let previous = start;
    let worstGain = 0;
    for (let i = 0; i < 900; i++) {
      world.step();
      const energy = world.kineticEnergy();
      worstGain = Math.max(worstGain, (energy - previous) / start);
      previous = energy;
    }
    // Fifteen seconds and thousands of impacts later, energy is where it started.
    expect(worstGain).toBeLessThan(1e-12);
    expect(previous / start).toBeGreaterThan(0.9999);
    expect(previous / start).toBeLessThanOrEqual(1 + 1e-12);
  });
});

describe("Newton's cradle", () => {
  function cradle(count: number, lifted: number) {
    const world = new World();
    const beam = world.createBody({ type: 'static', position: { x: 0, y: 6 }, shapes: [box(6, 0.1)] });
    const balls = [];
    const [radius, length, gap] = [0.5, 5, 0.002];
    for (let i = 0; i < count; i++) {
      const restX = (i - (count - 1) / 2) * (2 * radius + gap);
      const angle = i < lifted ? 0.6 : 0;
      const position = { x: restX - length * Math.sin(angle), y: 6 - length * Math.cos(angle) };
      const ball = world.createBody({ position, shapes: [circle(radius, { restitution: 1, friction: 0 })], allowSleep: false });
      world.createDistanceJoint({ bodyA: beam, bodyB: ball, anchorA: { x: restX, y: 6 }, anchorB: position });
      balls.push(ball);
    }
    return { world, balls };
  }

  it('hands the whole impact to the last ball and leaves the others at rest', () => {
    const { world, balls } = cradle(5, 1);
    let incoming = 0;
    for (let i = 0; i < 300; i++) {
      const before = balls[0].vx;
      world.step();
      // The step in which the first ball stops is the impact.
      if (before > 1 && balls[0].vx < 0.01 * before) {
        incoming = before;
        break;
      }
    }
    expect(incoming).toBeGreaterThan(3);
    expect(balls[4].vx / incoming).toBeCloseTo(1, 2);
    for (let i = 0; i < 4; i++) expect(Math.abs(balls[i].vx)).toBeLessThan(0.005 * incoming);
  });

  it('sends two balls out when two come in', () => {
    const { world, balls } = cradle(5, 2);
    let incoming = 0;
    for (let i = 0; i < 300; i++) {
      const before = balls[1].vx;
      world.step();
      if (before > 1 && balls[1].vx < 0.01 * before) {
        incoming = before;
        break;
      }
    }
    expect(incoming).toBeGreaterThan(3);
    expect(balls[3].vx / incoming).toBeCloseTo(1, 1);
    expect(balls[4].vx / incoming).toBeCloseTo(1, 1);
    for (let i = 0; i < 3; i++) expect(Math.abs(balls[i].vx)).toBeLessThan(0.02 * incoming);
  });

  it('keeps swinging: total energy stays within half a percent over twenty seconds', () => {
    const { world, balls } = cradle(5, 1);
    const height = (b: { y: number }) => b.y - 1;
    const total = () => world.kineticEnergy() + balls.reduce((sum, b) => sum + b.mass * 10 * height(b), 0);
    const start = total();
    let worst = 0;
    for (let i = 0; i < 1200; i++) {
      world.step();
      worst = Math.max(worst, Math.abs(total() / start - 1));
    }
    expect(worst).toBeLessThan(0.005);
  });
});

describe('restitution under gravity', () => {
  function apexes(restitution: number, steps: number): number[] {
    const world = new World();
    ground(world);
    const ball = world.createBody({ position: { x: 0, y: 5.5 }, shapes: [circle(0.5, { restitution })] });
    const result: number[] = [];
    let previous = 0;
    for (let i = 0; i < steps; i++) {
      world.step();
      if (previous > 0 && ball.vy <= 0) result.push(ball.y - 0.5);
      previous = ball.vy;
    }
    return result;
  }

  it('a perfectly elastic ball keeps returning to within 1% of its drop height', () => {
    const heights = apexes(1, 1800);
    expect(heights.length).toBeGreaterThan(12);
    for (const h of heights) {
      expect(h).toBeGreaterThan(4.95);
      expect(h).toBeLessThan(5.001);
    }
  });

  it('a half-elastic ball rebounds to a quarter of the height each time', () => {
    const heights = apexes(0.5, 600);
    expect(heights[0] / 5).toBeCloseTo(0.25, 2);
    expect(heights[1] / heights[0]).toBeCloseTo(0.25, 1);
  });

  it('a flat box dropped with restitution bounces straight up, without picking up spin', () => {
    const world = new World();
    ground(world);
    const body = world.createBody({ position: { x: 0, y: 4 }, shapes: [box(0.6, 0.4, { restitution: 0.6 })] });
    let maxSpin = 0;
    let bounced = false;
    for (let i = 0; i < 200; i++) {
      world.step();
      maxSpin = Math.max(maxSpin, Math.abs(body.w));
      if (body.vy > 2) bounced = true;
    }
    expect(bounced).toBe(true);
    // Solving the two corners one after the other would spin it at about 3 rad/s.
    expect(maxSpin).toBeLessThan(0.05);
    expect(Math.abs(body.angle)).toBeLessThan(0.01);
  });
});

describe('friction', () => {
  function incline(degrees: number, friction: number, shape: 'box' | 'disc') {
    const theta = (degrees * Math.PI) / 180;
    const world = new World({ sleeping: false });
    world.createBody({ type: 'static', angle: -theta, shapes: [box(40, 0.5, { friction })] });
    const normal = { x: Math.sin(theta), y: Math.cos(theta) };
    const body = world.createBody({
      position: { x: normal.x, y: normal.y },
      angle: -theta,
      shapes: [shape === 'box' ? box(0.5, 0.5, { friction }) : circle(0.5, { friction })],
    });
    return { world, body, theta, start: { x: body.x, y: body.y } };
  }

  it('slides down an incline with the analytic acceleration g (sin a - mu cos a)', () => {
    const { world, body, theta } = incline(30, 0.2, 'box');
    run(world, 60);
    const acceleration = 10 * (Math.sin(theta) - 0.2 * Math.cos(theta));
    expect(Math.hypot(body.vx, body.vy) / acceleration).toBeCloseTo(1, 2);
  });

  it('holds still when the friction coefficient exceeds the tangent of the slope', () => {
    // tan(20 deg) = 0.364
    const { world, body, start } = incline(20, 0.4, 'box');
    run(world, 300);
    expect(Math.hypot(body.x - start.x, body.y - start.y)).toBeLessThan(0.01);
    expect(Math.hypot(body.vx, body.vy)).toBeLessThan(1e-3);
  });

  it('slides when the coefficient is below the tangent of the slope', () => {
    const { world, body, start } = incline(20, 0.3, 'box');
    run(world, 120);
    const acceleration = 10 * (Math.sin((20 * Math.PI) / 180) - 0.3 * Math.cos((20 * Math.PI) / 180));
    expect(Math.hypot(body.x - start.x, body.y - start.y) / (0.5 * acceleration * 4)).toBeCloseTo(1, 1);
  });

  it('a disc rolls without slipping at two thirds of the sliding acceleration', () => {
    const { world, body, theta } = incline(20, 0.8, 'disc');
    run(world, 90);
    const speed = Math.hypot(body.vx, body.vy);
    // Rolling condition v = w r, and a = (2/3) g sin(theta) for a solid disc.
    expect((Math.abs(body.w) * 0.5) / speed).toBeCloseTo(1, 2);
    expect(speed / ((2 / 3) * 10 * Math.sin(theta) * 1.5)).toBeCloseTo(1, 2);
  });
});
