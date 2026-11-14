import {
  type Body,
  box,
  circle,
  DistanceJoint,
  type MouseJoint,
  Random,
  regularPolygon,
  type ShapeDef,
  type World,
} from '../engine/index.ts';
import type { Look } from '../scenes/builders.ts';
import type { ToolId } from '../scenes/index.ts';

/** Where the pointer is, in world coordinates, and how fast it is moving. */
export interface Pointer {
  x: number;
  y: number;
  /** World units per second, smoothed. */
  vx: number;
  vy: number;
  down: boolean;
}

export interface ToolInfo {
  id: ToolId;
  label: string;
  key: string;
  hint: string;
}

export const TOOLS: ToolInfo[] = [
  { id: 'grab', label: 'Grab', key: 'G', hint: 'Drag bodies, cloth and blobs. Let go while moving to throw.' },
  { id: 'box', label: 'Box', key: 'B', hint: 'Click to drop a box. Drag to throw it.' },
  { id: 'ball', label: 'Ball', key: 'C', hint: 'Click to drop a ball. Drag to throw it.' },
  { id: 'poly', label: 'Polygon', key: 'P', hint: 'Click to drop a convex polygon. Drag to throw it.' },
  { id: 'water', label: 'Water', key: 'W', hint: 'Hold to pour liquid.' },
  { id: 'cut', label: 'Cut', key: 'X', hint: 'Drag across cloth, ropes and rods to cut them.' },
  { id: 'blast', label: 'Blast', key: 'E', hint: 'Click to set off an explosion.' },
  { id: 'erase', label: 'Erase', key: 'Backspace', hint: 'Click or drag over bodies and liquid to remove them.' },
];

const MAX_FLUID_PARTICLES = 6000;
const BLAST_RADIUS = 5;

/** A short-lived visual left behind by a tool: an explosion ring or a cut line. */
export interface Effect {
  kind: 'ring' | 'slash';
  x0: number;
  y0: number;
  x1: number;
  y1: number;
  age: number;
}

/**
 * Turns pointer input into changes to the world. All tools share one object, because they
 * share state (what is held, where the drag started) and switching tools must release it.
 */
export class Tools {
  active: ToolId = 'grab';
  readonly effects: Effect[] = [];

  private world: World;
  private random = new Random(12345);
  private mouseJoint: MouseJoint | null = null;
  private heldParticle = -1;
  private heldInvMass = 0;
  private dragStartX = 0;
  private dragStartY = 0;
  private dragging = false;
  private pendingShape: ShapeDef | null = null;
  private lastX = 0;
  private lastY = 0;
  private spawnTint = 0;

  constructor(world: World) {
    this.world = world;
  }

  /** Call when the scene is rebuilt; anything held belonged to the old world. */
  setWorld(world: World): void {
    this.world = world;
    this.mouseJoint = null;
    this.heldParticle = -1;
    this.dragging = false;
    this.pendingShape = null;
    this.effects.length = 0;
  }

  select(tool: ToolId): void {
    this.release();
    this.active = tool;
  }

  /** True while the grab tool is holding something, so the caller should not pan the camera. */
  get holding(): boolean {
    return this.mouseJoint !== null || this.heldParticle >= 0;
  }

  /** Returns false when the press hit nothing the tool can use (the caller may pan instead). */
  down(p: Pointer): boolean {
    this.lastX = p.x;
    this.lastY = p.y;
    switch (this.active) {
      case 'grab':
        return this.grab(p);
      case 'box':
      case 'ball':
      case 'poly':
        this.dragStartX = p.x;
        this.dragStartY = p.y;
        this.dragging = true;
        this.pendingShape = this.nextShape();
        return true;
      case 'blast':
        this.blast(p.x, p.y);
        return true;
      case 'erase':
        this.erase(p.x, p.y);
        return true;
      case 'water':
      case 'cut':
        return true;
    }
  }

  move(p: Pointer): void {
    if (!p.down) return;
    if (this.active === 'grab') {
      this.mouseJoint?.setTarget(p.x, p.y);
    } else if (this.active === 'cut') {
      this.cut(this.lastX, this.lastY, p.x, p.y);
    } else if (this.active === 'erase') {
      this.erase(p.x, p.y);
    }
    this.lastX = p.x;
    this.lastY = p.y;
  }

  up(p: Pointer): void {
    if (this.dragging && this.pendingShape) {
      // Drag direction and length become the launch velocity.
      const vx = (p.x - this.dragStartX) * 5;
      const vy = (p.y - this.dragStartY) * 5;
      this.world.createBody({
        position: { x: this.dragStartX, y: this.dragStartY },
        linearVelocity: { x: vx, y: vy },
        shapes: [this.pendingShape],
        userData: { tint: this.spawnTint++ % 6 } satisfies Look,
      });
    }
    this.dragging = false;
    this.pendingShape = null;
    this.release(p);
  }

  /** Called before every physics step, for tools that act continuously. */
  beforeStep(p: Pointer): void {
    const world = this.world;
    if (this.heldParticle >= 0) {
      const soft = world.soft;
      const i = this.heldParticle;
      soft.x[i] = p.x;
      soft.y[i] = p.y;
      soft.vx[i] = soft.vy[i] = 0;
    }
    if (this.active === 'water' && p.down) this.pour(p);
  }

  /** The shape that will be created on release, and the aim line; for the overlay. */
  get preview(): { shape: ShapeDef; x: number; y: number } | null {
    if (!this.dragging || !this.pendingShape) return null;
    return { shape: this.pendingShape, x: this.dragStartX, y: this.dragStartY };
  }

  private nextShape(): ShapeDef {
    const r = this.random;
    if (this.active === 'ball') return circle(r.range(0.3, 0.6), { density: 2, restitution: 0.2 });
    if (this.active === 'poly') {
      return regularPolygon(3 + r.int(4), r.range(0.4, 0.7), { density: 1 });
    }
    return box(r.range(0.3, 0.6), r.range(0.3, 0.6));
  }

  private grab(p: Pointer): boolean {
    const world = this.world;
    const shape = world.shapeAt(p.x, p.y);
    if (shape && shape.body.isDynamic) {
      this.mouseJoint = world.createMouseJoint({ body: shape.body, target: { x: p.x, y: p.y }, hertz: 5 });
      return true;
    }
    const soft = world.soft;
    const i = soft.nearest(p.x, p.y, 0.6);
    if (i >= 0) {
      this.heldParticle = i;
      // Holding a particle makes it immovable; a pin stays a pin after release.
      this.heldInvMass = soft.invMass[i];
      soft.invMass[i] = 0;
      return true;
    }
    return false;
  }

  private release(p?: Pointer): void {
    if (this.mouseJoint) {
      this.world.destroyJoint(this.mouseJoint);
      this.mouseJoint = null;
    }
    if (this.heldParticle >= 0) {
      const soft = this.world.soft;
      const i = this.heldParticle;
      if (i < soft.count) {
        soft.invMass[i] = this.heldInvMass;
        if (p && this.heldInvMass > 0) {
          soft.vx[i] = p.vx;
          soft.vy[i] = p.vy;
        }
      }
      this.heldParticle = -1;
    }
  }

  private pour(p: Pointer): void {
    const world = this.world;
    const fluid = world.fluid;
    const spacing = fluid.spacing;
    // A small nozzle: a row of particles perpendicular to gravity, with a little jitter.
    for (let k = -2; k <= 2; k++) {
      if (fluid.count >= MAX_FLUID_PARTICLES) return;
      const x = p.x + k * spacing + this.random.range(-0.02, 0.02);
      const y = p.y + this.random.range(-0.02, 0.02);
      if (world.shapeAt(x, y)) continue;
      fluid.addParticle(x, y, 0.3 * p.vx, Math.min(0.3 * p.vy, 0) - 3);
    }
  }

  private cut(x0: number, y0: number, x1: number, y1: number): void {
    const world = this.world;
    if (x0 === x1 && y0 === y1) return;
    let cuts = world.soft.cut(x0, y0, x1, y1);
    for (const joint of world.joints.slice()) {
      if (!(joint instanceof DistanceJoint)) continue;
      const a = joint.anchorA;
      const b = joint.anchorB;
      if (segmentsCross(x0, y0, x1, y1, a.x, a.y, b.x, b.y)) {
        world.destroyJoint(joint);
        cuts++;
      }
    }
    this.effects.push({ kind: 'slash', x0, y0, x1, y1, age: cuts > 0 ? 0 : 0.1 });
  }

  private blast(x: number, y: number): void {
    const world = this.world;
    const radius = BLAST_RADIUS;
    for (const body of world.bodies) {
      if (!body.isDynamic) continue;
      const dx = body.x - x;
      const dy = body.y - y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > radius) continue;
      // Speed change falls off linearly with distance and does not depend on mass.
      const kick = (14 * (1 - dist / radius)) / Math.max(dist, 0.3);
      body.applyImpulse(dx * kick * body.mass, dy * kick * body.mass);
    }
    const push = (px: Float64Array, py: Float64Array, vx: Float64Array, vy: Float64Array, count: number) => {
      for (let i = 0; i < count; i++) {
        const dx = px[i] - x;
        const dy = py[i] - y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > radius) continue;
        const kick = (14 * (1 - dist / radius)) / Math.max(dist, 0.3);
        vx[i] += dx * kick;
        vy[i] += dy * kick;
      }
    };
    push(world.soft.x, world.soft.y, world.soft.vx, world.soft.vy, world.soft.count);
    push(world.fluid.x, world.fluid.y, world.fluid.vx, world.fluid.vy, world.fluid.count);
    this.effects.push({ kind: 'ring', x0: x, y0: y, x1: radius, y1: 0, age: 0 });
  }

  private erase(x: number, y: number): void {
    const world = this.world;
    const shape = world.shapeAt(x, y);
    if (shape && shape.body.isDynamic) this.destroy(shape.body);
    const fluid = world.fluid;
    const reach = 0.5;
    for (let i = fluid.count - 1; i >= 0; i--) {
      const dx = fluid.x[i] - x;
      const dy = fluid.y[i] - y;
      if (dx * dx + dy * dy < reach * reach) fluid.remove(i);
    }
  }

  private destroy(body: Body): void {
    if (this.mouseJoint?.bodyB === body) this.mouseJoint = null;
    this.world.destroyBody(body);
  }
}

function segmentsCross(
  ax: number,
  ay: number,
  bx: number,
  by: number,
  cx: number,
  cy: number,
  dx: number,
  dy: number,
): boolean {
  const rx = bx - ax;
  const ry = by - ay;
  const sx = dx - cx;
  const sy = dy - cy;
  const denom = rx * sy - ry * sx;
  if (denom === 0) return false;
  const t = ((cx - ax) * sy - (cy - ay) * sx) / denom;
  const u = ((cx - ax) * ry - (cy - ay) * rx) / denom;
  return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}
