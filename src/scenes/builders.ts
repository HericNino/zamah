import { type Body, box, circle, type World } from '../engine/index.ts';

/** What the sandbox reads from `body.userData` to pick a colour. Purely cosmetic. */
export interface Look {
  /** Index into the renderer's palette. */
  tint?: number;
}

/** How thick the floor and walls of an arena are, in metres. */
const WALL = 10;

/**
 * A floor with a wall at each end, so nothing can leave the scene sideways.
 *
 * The walls are far thicker than they look. Collision detection runs once per step, so a
 * body can end a step some way inside a wall; it is then pushed out through the nearest
 * face. With a thick wall the nearest face is always the one it came in through.
 */
export function arena(world: World, halfWidth: number, wallHeight: number, floorY = 0): Body {
  const floor = world.createBody({
    type: 'static',
    position: { x: 0, y: floorY - WALL / 2 },
    shapes: [box(halfWidth + WALL, WALL / 2)],
  });
  for (const side of [-1, 1]) {
    world.createBody({
      type: 'static',
      position: { x: side * (halfWidth + WALL / 2), y: floorY + wallHeight / 2 },
      shapes: [box(WALL / 2, wallHeight / 2 + WALL / 2)],
    });
  }
  return floor;
}

export interface RagdollOptions {
  x: number;
  y: number;
  scale?: number;
  tint?: number;
}

/**
 * A ragdoll: eleven bodies held together by hinges with angle limits.
 * `x, y` is the centre of the torso.
 */
export function ragdoll(world: World, options: RagdollOptions): Body[] {
  const s = options.scale ?? 1;
  const { x, y } = options;
  const look: Look = { tint: options.tint ?? 0 };
  const material = { density: 1, friction: 0.5 };
  const parts: Body[] = [];

  const part = (px: number, py: number, hw: number, hh: number): Body => {
    const body = world.createBody({
      position: { x: x + px * s, y: y + py * s },
      shapes: [box(hw * s, hh * s, material)],
      userData: look,
    });
    parts.push(body);
    return body;
  };
  const hinge = (a: Body, b: Body, px: number, py: number, lower: number, upper: number): void => {
    world.createRevoluteJoint({
      bodyA: a,
      bodyB: b,
      anchor: { x: x + px * s, y: y + py * s },
      enableLimit: true,
      lowerAngle: lower,
      upperAngle: upper,
    });
  };

  const torso = part(0, 0, 0.25, 0.4);
  const head = world.createBody({
    position: { x, y: y + 0.62 * s },
    shapes: [circle(0.2 * s, material)],
    userData: look,
  });
  parts.push(head);
  hinge(torso, head, 0, 0.42, -0.5, 0.5);

  for (const side of [-1, 1]) {
    const upperArm = part(side * 0.47, 0.3, 0.2, 0.08);
    const lowerArm = part(side * 0.87, 0.3, 0.2, 0.07);
    hinge(torso, upperArm, side * 0.27, 0.3, -1.4, 1.4);
    hinge(upperArm, lowerArm, side * 0.67, 0.3, side > 0 ? 0 : -2.2, side > 0 ? 2.2 : 0);

    const upperLeg = part(side * 0.13, -0.65, 0.1, 0.25);
    const lowerLeg = part(side * 0.13, -1.15, 0.09, 0.25);
    hinge(torso, upperLeg, side * 0.13, -0.4, -1.2, 1.2);
    hinge(upperLeg, lowerLeg, side * 0.13, -0.9, -2.2, 0);
  }
  return parts;
}
