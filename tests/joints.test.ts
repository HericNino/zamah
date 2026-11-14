import { describe, expect, it } from 'vitest';
import { type Body, box, circle, type RevoluteJoint, World } from '../src/engine/index.ts';
import { ground, maxSpeed, run } from './helpers.ts';

function anchorBody(world: World, x = 0, y = 0): Body {
  return world.createBody({ type: 'static', position: { x, y }, shapes: [box(0.1, 0.1)] });
}

describe('distance joint', () => {
  it('a pendulum swings with the period 2 pi sqrt(L / g)', () => {
    const world = new World();
    const pivot = anchorBody(world);
    const length = 2;
    const amplitude = 0.1;
    const bob = world.createBody({
      position: { x: length * Math.sin(amplitude), y: -length * Math.cos(amplitude) },
      shapes: [circle(0.2)],
    });
    world.createDistanceJoint({ bodyA: pivot, bodyB: bob, anchorA: { x: 0, y: 0 }, anchorB: { x: bob.x, y: bob.y } });

    const crossings: number[] = [];
    let previous = bob.x;
    for (let i = 1; i <= 1800; i++) {
      world.step();
      // Interpolate the instant the bob passes the lowest point, moving left.
      if (previous > 0 && bob.x <= 0) crossings.push((i - bob.x / (bob.x - previous)) / 60);
      previous = bob.x;
    }
    const period = (crossings[crossings.length - 1] - crossings[0]) / (crossings.length - 1);
    const analytic = 2 * Math.PI * Math.sqrt(length / 10) * (1 + (amplitude * amplitude) / 16);
    expect(period / analytic).toBeCloseTo(1, 3);
    // The rod has not stretched, and the swing has lost almost nothing.
    expect(Math.hypot(bob.x, bob.y)).toBeCloseTo(length, 3);
  });

  it('a rope only resists stretching', () => {
    const world = new World();
    const pivot = anchorBody(world, 0, 5);
    const ball = world.createBody({ position: { x: 0, y: 4 }, shapes: [circle(0.2)] });
    const rope = world.createDistanceJoint({
      bodyA: pivot,
      bodyB: ball,
      anchorA: { x: 0, y: 5 },
      anchorB: { x: 0, y: 4 },
      length: 3,
      mode: 'rope',
    });
    // Slack: for the first half second the ball is in free fall.
    run(world, 30);
    expect(ball.vy).toBeCloseTo(-5, 6);
    run(world, 600);
    const stretched = Math.hypot(ball.x - 0, ball.y - 5);
    expect(stretched).toBeLessThan(rope.length + 0.01);
    expect(stretched).toBeGreaterThan(rope.length - 0.2);
  });

  it('a spring oscillates at its configured frequency', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const wall = anchorBody(world);
    const mass = world.createBody({ position: { x: 2.5, y: 0 }, shapes: [circle(0.3)], fixedRotation: true });
    world.createDistanceJoint({
      bodyA: wall,
      bodyB: mass,
      anchorA: { x: 0, y: 0 },
      anchorB: { x: 2.5, y: 0 },
      length: 2,
      mode: 'spring',
      hertz: 1.5,
      dampingRatio: 0,
    });
    const crossings: number[] = [];
    let previous = mass.x - 2;
    for (let i = 1; i <= 600; i++) {
      world.step();
      const offset = mass.x - 2;
      if (previous > 0 && offset <= 0) crossings.push((i - offset / (offset - previous)) / 60);
      previous = offset;
    }
    const period = (crossings[crossings.length - 1] - crossings[0]) / (crossings.length - 1);
    expect(1 / period).toBeCloseTo(1.5, 1);
  });
});

describe('revolute joint', () => {
  it('holds a chain of twenty links together while it whips around', () => {
    const world = new World({ sleeping: false });
    const pivot = anchorBody(world, 0, 10);
    const joints: RevoluteJoint[] = [];
    let previous = pivot;
    for (let i = 0; i < 20; i++) {
      const link = world.createBody({
        position: { x: 0.25 + i * 0.5, y: 10 },
        shapes: [box(0.25, 0.05, { density: 2 })],
        linearDamping: 0.5,
      });
      joints.push(world.createRevoluteJoint({ bodyA: previous, bodyB: link, anchor: { x: i * 0.5, y: 10 } }));
      previous = link;
    }
    const worstGap = () => {
      let worst = 0;
      for (const joint of joints) {
        const a = joint.bodyA.worldPoint(joint.localAx, joint.localAy);
        const b = joint.bodyB.worldPoint(joint.localBx, joint.localBy);
        worst = Math.max(worst, Math.hypot(a.x - b.x, a.y - b.y));
      }
      return worst;
    };
    // Released horizontally, the chain cracks like a whip: the hardest moment for the hinges.
    let worst = 0;
    for (let i = 0; i < 600; i++) {
      world.step();
      worst = Math.max(worst, worstGap());
    }
    expect(worst).toBeLessThan(0.05);
    // Once it has calmed down the hinges are tight and the chain hangs straight.
    run(world, 1800);
    expect(worstGap()).toBeLessThan(0.005);
    expect(Math.abs(previous.x)).toBeLessThan(0.3);
    expect(previous.y).toBeLessThan(0.4);
  });

  it('keeps the angle inside its limits', () => {
    const world = new World({ sleeping: false });
    const pivot = anchorBody(world, 0, 5);
    const arm = world.createBody({ position: { x: 1, y: 5 }, shapes: [box(1, 0.1)] });
    const joint = world.createRevoluteJoint({
      bodyA: pivot,
      bodyB: arm,
      anchor: { x: 0, y: 5 },
      enableLimit: true,
      lowerAngle: -0.5,
      upperAngle: 0.25,
    });
    let lowest = 0;
    for (let i = 0; i < 300; i++) {
      world.step();
      lowest = Math.min(lowest, joint.angle);
    }
    // Gravity swings the arm down until the lower limit stops it.
    expect(lowest).toBeGreaterThan(-0.52);
    expect(joint.angle).toBeCloseTo(-0.5, 2);
    expect(Math.abs(arm.w)).toBeLessThan(1e-3);
  });

  it('a motor drives the joint at the requested speed, within its torque limit', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const pivot = anchorBody(world);
    const wheel = world.createBody({ shapes: [circle(1)] });
    const joint = world.createRevoluteJoint({
      bodyA: pivot,
      bodyB: wheel,
      anchor: { x: 0, y: 0 },
      enableMotor: true,
      motorSpeed: 3,
      maxMotorTorque: 5,
    });
    run(world, 6);
    // Torque-limited at first: w = torque / inertia * t.
    expect(wheel.w).toBeCloseTo((5 / wheel.inertia) * 0.1, 6);
    run(world, 600);
    expect(wheel.w).toBeCloseTo(3, 6);
    joint.motorSpeed = -1;
    run(world, 600);
    expect(wheel.w).toBeCloseTo(-1, 6);
  });

  it('connected bodies do not collide unless asked to', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const a = world.createBody({ position: { x: 0, y: 0 }, shapes: [box(1, 0.2)] });
    const b = world.createBody({ position: { x: 0.5, y: 0 }, shapes: [box(1, 0.2)] });
    world.step();
    expect(world.contacts.size).toBe(1);
    const joint = world.createRevoluteJoint({ bodyA: a, bodyB: b, anchor: { x: 0.25, y: 0 } });
    world.step();
    expect(world.contacts.size).toBe(0);
    world.destroyJoint(joint);
    world.step();
    expect(world.contacts.size).toBe(1);
  });
});

describe('mouse joint', () => {
  it('pulls a body to the target and settles there', () => {
    const world = new World();
    ground(world);
    const body = world.createBody({ position: { x: 0, y: 0.5 }, shapes: [box(0.5, 0.5)] });
    run(world, 120);
    expect(body.awake).toBe(false);
    const joint = world.createMouseJoint({ body, target: { x: 0, y: 0.5 } });
    joint.setTarget(4, 6);
    run(world, 240);
    const anchor = joint.anchor;
    // It hangs slightly below the target: the spring has to carry the weight.
    expect(anchor.x).toBeCloseTo(4, 1);
    expect(anchor.y).toBeGreaterThan(5.9);
    expect(anchor.y).toBeLessThan(6);
    expect(maxSpeed(world)).toBeLessThan(0.05);
    expect(body.awake).toBe(true);

    world.destroyJoint(joint);
    run(world, 30);
    expect(body.vy).toBeLessThan(-1);
  });

  it('releasing a moving body throws it', () => {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const body = world.createBody({ shapes: [circle(0.5)] });
    const joint = world.createMouseJoint({ body, target: { x: 0, y: 0 } });
    for (let i = 0; i < 30; i++) {
      joint.setTarget(i * 0.2, 0);
      world.step();
    }
    world.destroyJoint(joint);
    run(world, 60);
    // Target moved at 12 m/s; the body keeps most of that after release.
    expect(body.vx).toBeGreaterThan(8);
    expect(world.joints).toHaveLength(0);
  });
});
