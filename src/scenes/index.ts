import { box, circle, cos, regularPolygon, sin, type World, type WorldOptions } from '../engine/index.ts';
import { arena, type Look, ragdoll } from './builders.ts';

export type ToolId = 'grab' | 'box' | 'ball' | 'poly' | 'water' | 'cut' | 'blast' | 'erase';

/**
 * A ready-made scene. Scenes only talk to the engine: no DOM, no randomness, no clock.
 * That is what lets the test suite and the benchmark run exactly what the sandbox shows.
 */
export interface Scene {
  id: string;
  title: string;
  /** One line shown under the title. */
  blurb: string;
  /** What to try with the mouse. */
  hint: string;
  /** The rectangle of the world the camera should fit. */
  view: View;
  /** A tighter framing for screens that are taller than they are wide. */
  narrowView?: View;
  tool: ToolId;
  options?: WorldOptions;
  build(world: World): void;
}

export interface View {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

const tint = (n: number): Look => ({ tint: n });

const tower: Scene = {
  id: 'tower',
  title: 'Wrecking ball',
  blurb: 'A brick wall, a heavy ball on a rope, and gravity.',
  hint: 'Wait for the swing, then grab the ball and do it again. Blast what is left.',
  view: { minX: -13, minY: -0.5, maxX: 13, maxY: 14 },
  narrowView: { minX: -5.5, minY: -0.5, maxX: 10.5, maxY: 14 },
  tool: 'grab',
  build(world) {
    arena(world, 16, 30);
    // A wall in running bond: every other row is shifted by half a brick.
    const rows = 14;
    const columns = 7;
    for (let row = 0; row < rows; row++) {
      const shifted = row % 2 === 1;
      for (let i = 0; i < columns + (shifted ? 1 : 0); i++) {
        const half = shifted && (i === 0 || i === columns);
        const width = half ? 0.5 : 1;
        const x = shifted ? (i === 0 ? 0.25 : i === columns ? columns - 0.25 : i) : i + 0.5;
        world.createBody({
          position: { x: 2 + x, y: 0.25 + row * 0.5 },
          shapes: [box(width / 2, 0.25, { friction: 0.7 })],
          userData: tint(row % 3),
        });
      }
    }
    // The ball starts pulled back and is let go by the first step.
    const pivot = world.createBody({ type: 'static', position: { x: -3, y: 13 }, shapes: [box(0.3, 0.3)] });
    const length = 10;
    const angle = 1.15;
    const ball = world.createBody({
      position: { x: -3 - length * sin(angle), y: 13 - length * cos(angle) },
      shapes: [circle(1, { density: 6, friction: 0.4 })],
      userData: tint(4),
    });
    world.createDistanceJoint({
      bodyA: pivot,
      bodyB: ball,
      anchorA: { x: -3, y: 13 },
      anchorB: { x: ball.x, y: ball.y },
      mode: 'rope',
    });
  },
};

const pyramid: Scene = {
  id: 'pyramid',
  title: 'Pyramid',
  blurb: '210 boxes resting on each other: the classic test of a contact solver.',
  hint: 'It should stand perfectly still. Throw balls at it, or pull a box out of the bottom row.',
  view: { minX: -13, minY: -0.5, maxX: 13, maxY: 21 },
  tool: 'ball',
  build(world) {
    arena(world, 16, 40);
    const base = 20;
    const size = 0.5;
    for (let row = 0; row < base; row++) {
      for (let i = 0; i < base - row; i++) {
        world.createBody({
          position: { x: (i - (base - row - 1) / 2) * (2 * size + 0.02), y: size + row * 2 * size },
          shapes: [box(size, size)],
          userData: tint(row % 4),
        });
      }
    }
  },
};

const cradle: Scene = {
  id: 'cradle',
  title: "Newton's cradle",
  blurb: 'Perfectly elastic balls pass momentum down the row, and energy is conserved.',
  hint: 'Lift one, two or three balls with the mouse and let go.',
  view: { minX: -7, minY: -0.5, maxX: 7, maxY: 9 },
  narrowView: { minX: -5, minY: -0.5, maxX: 5, maxY: 9 },
  tool: 'grab',
  build(world) {
    arena(world, 9, 20);
    const beam = world.createBody({ type: 'static', position: { x: 0, y: 8 }, shapes: [box(4.5, 0.15)] });
    const count = 7;
    const radius = 0.5;
    const length = 5;
    // A hair's width between the balls turns one simultaneous impact into a clean sequence.
    const gap = 0.002;
    for (let i = 0; i < count; i++) {
      const restX = (i - (count - 1) / 2) * (2 * radius + gap);
      const lifted = i === 0 ? 0.9 : 0;
      const x = restX - length * sin(lifted);
      const y = 8 - length * cos(lifted);
      const ball = world.createBody({
        position: { x, y },
        shapes: [circle(radius, { density: 2, restitution: 1, friction: 0 })],
        allowSleep: false,
        userData: tint(i === 0 ? 4 : 1),
      });
      world.createDistanceJoint({
        bodyA: beam,
        bodyB: ball,
        anchorA: { x: restX, y: 8 },
        anchorB: { x, y },
        mode: 'rod',
      });
    }
  },
};

const cloth: Scene = {
  id: 'cloth',
  title: 'Cloth',
  blurb: 'A tearable sheet of 1,230 particles and 6,999 XPBD constraints.',
  hint: 'Drag the cloth to stretch it until it rips, or switch to Cut and slice through it.',
  view: { minX: -8, minY: -0.5, maxX: 8, maxY: 12.6 },
  narrowView: { minX: -5.6, minY: -0.5, maxX: 5.6, maxY: 12.6 },
  tool: 'grab',
  build(world) {
    arena(world, 10, 20);
    world.soft.createCloth({ x: -5, y: 9.5, cols: 41, rows: 30, spacing: 0.25, pin: 5, tearRatio: 3.2, mass: 0.02 });
    // Something for the cloth to drape over and to be hit by.
    world.createBody({ type: 'static', position: { x: -1.5, y: 3.6 }, shapes: [circle(1.1)] });
    world.createBody({
      position: { x: 3, y: 13 },
      shapes: [circle(0.7, { density: 1.5 })],
      userData: tint(4),
    });
    world.createBody({
      position: { x: 1.5, y: 16 },
      angle: 0.4,
      shapes: [box(0.6, 0.6, { density: 1 })],
      userData: tint(2),
    });
  },
};

const fluid: Scene = {
  id: 'fluid',
  title: 'Dam break',
  blurb: 'A column of liquid collapses; light things float, heavy things sink, the wheel stirs.',
  hint: 'Pour more with the Water tool, drop boxes in, or grab the crates and push them under.',
  view: { minX: -9.8, minY: -0.5, maxX: 9.8, maxY: 11.5 },
  tool: 'water',
  build(world) {
    const floor = arena(world, 9, 24);
    world.fluid.configure({ spacing: 0.14, density: 1 });
    world.fluid.addBlock(-9, 0, 6.5, 9);

    // Crates of different densities riding on top of the column: 0.3 floats high, 0.75 sits
    // low in the water, 2.5 sinks straight through.
    const densities = [0.3, 0.75, 2.5, 0.3];
    for (let i = 0; i < densities.length; i++) {
      world.createBody({
        position: { x: -8 + i * 1.5, y: 9.75 },
        shapes: [box(0.5, 0.5, { density: densities[i], friction: 0.4 })],
        userData: tint(densities[i] > 1 ? 3 : i % 2),
      });
    }
    world.createBody({
      position: { x: 6.5, y: 0.6 },
      shapes: [circle(0.6, { density: 0.5 })],
      userData: tint(4),
    });

    // A motorised paddle wheel.
    const wheel = world.createBody({
      position: { x: 4.5, y: 3.4 },
      shapes: [box(1.7, 0.12, { density: 1.2 }), box(0.12, 1.7, { density: 1.2 })],
      userData: tint(2),
    });
    world.createRevoluteJoint({
      bodyA: floor,
      bodyB: wheel,
      anchor: { x: 4.5, y: 3.4 },
      enableMotor: true,
      motorSpeed: -1.6,
      maxMotorTorque: 60,
    });
  },
};

const bridge: Scene = {
  id: 'bridge',
  title: 'Bridge and ragdolls',
  blurb: 'Hinges with angle limits: a plank bridge, a swinging chain and four ragdolls.',
  hint: 'Grab a ragdoll by the head and throw it. Drop boxes on the bridge until it sags.',
  view: { minX: -12, minY: -0.5, maxX: 12, maxY: 13 },
  narrowView: { minX: -9, minY: -0.5, maxX: 9, maxY: 13 },
  tool: 'grab',
  build(world) {
    arena(world, 14, 26);
    const left = world.createBody({ type: 'static', position: { x: -10, y: 2.5 }, shapes: [box(2, 2.5)] });
    const right = world.createBody({ type: 'static', position: { x: 10, y: 2.5 }, shapes: [box(2, 2.5)] });

    // Sixteen planks, each hinged to the next, spanning the gap between the two cliffs.
    const planks = 16;
    const half = 0.5;
    let previous = left;
    for (let i = 0; i < planks; i++) {
      const plank = world.createBody({
        position: { x: -8 + half + i * 2 * half, y: 4.85 },
        shapes: [box(half, 0.12, { density: 2, friction: 0.8 })],
        userData: tint(3),
      });
      world.createRevoluteJoint({ bodyA: previous, bodyB: plank, anchor: { x: -8 + i * 2 * half, y: 4.85 } });
      previous = plank;
    }
    world.createRevoluteJoint({ bodyA: previous, bodyB: right, anchor: { x: 8, y: 4.85 } });

    // A chain with a weight on the end.
    const hook = world.createBody({ type: 'static', position: { x: 0, y: 12.5 }, shapes: [box(0.2, 0.2)] });
    previous = hook;
    for (let i = 0; i < 8; i++) {
      const link = world.createBody({
        position: { x: 0.3 + i * 0.6, y: 12.5 },
        shapes: [box(0.3, 0.07, { density: 2 })],
        userData: tint(2),
      });
      world.createRevoluteJoint({ bodyA: previous, bodyB: link, anchor: { x: i * 0.6, y: 12.5 } });
      previous = link;
    }
    const weight = world.createBody({
      position: { x: 5.2, y: 12.5 },
      shapes: [regularPolygon(6, 0.55, { density: 3 })],
      userData: tint(4),
    });
    world.createRevoluteJoint({ bodyA: previous, bodyB: weight, anchor: { x: 4.8, y: 12.5 } });

    for (let i = 0; i < 4; i++) {
      ragdoll(world, { x: -6 + i * 3.6, y: 7.6 + (i % 2) * 1.6, tint: i % 2 });
    }
  },
};

const blobs: Scene = {
  id: 'blobs',
  title: 'Soft bodies',
  blurb: 'Pressurised blobs, a hammock and a rope, all XPBD, colliding with rigid shapes.',
  hint: 'Squeeze a blob with the mouse, or drop boxes into the hammock.',
  view: { minX: -9, minY: -0.5, maxX: 9, maxY: 12 },
  narrowView: { minX: -8, minY: -0.5, maxX: 8, maxY: 12 },
  tool: 'grab',
  build(world) {
    arena(world, 11, 24);
    const soft = world.soft;
    // Pegs for the blobs to squeeze past.
    for (let row = 0; row < 2; row++) {
      for (let i = 0; i < 4 - row; i++) {
        world.createBody({
          type: 'static',
          position: { x: -7 + row * 1.6 + i * 3.2, y: 5.6 - row * 2.4 },
          shapes: [circle(0.45)],
        });
      }
    }
    const sizes = [1.1, 0.8, 1.3, 0.9, 1.0];
    for (let i = 0; i < sizes.length; i++) {
      soft.createBlob({ x: -6 + i * 2.3, y: 8.5 + (i % 2) * 2.2, radius: sizes[i], segments: 22, pressure: 1.05 });
    }
    // A hammock: a strip of cloth fixed at both ends.
    const hammock = soft.createCloth({
      x: 3.2,
      y: 6.2,
      cols: 26,
      rows: 3,
      spacing: 0.22,
      pin: 'none',
      mass: 0.05,
      compliance: 2e-5,
    });
    for (let r = 0; r < hammock.rows; r++) {
      soft.invMass[hammock.start + r * hammock.cols] = 0;
      soft.invMass[hammock.start + r * hammock.cols + hammock.cols - 1] = 0;
    }
    world.createBody({ position: { x: 6, y: 9 }, shapes: [box(0.45, 0.45)], userData: tint(2) });
    world.createBody({ position: { x: 5, y: 11 }, shapes: [circle(0.5)], userData: tint(4) });
    soft.createRope({ x0: 0.8, y0: 11.5, x1: 0.8, y1: 6.5, segments: 24, mass: 0.05 });
  },
};

export const scenes: Scene[] = [tower, pyramid, cradle, cloth, fluid, bridge, blobs];

export function sceneById(id: string): Scene | undefined {
  return scenes.find((scene) => scene.id === id);
}
