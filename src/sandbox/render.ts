import {
  type Body,
  DistanceJoint,
  MouseJoint,
  RevoluteJoint,
  type Shape,
  STRUCTURAL,
  type World,
} from '../engine/index.ts';
import type { Look } from '../scenes/builders.ts';
import type { Camera } from './camera.ts';

export interface DebugOptions {
  contacts: boolean;
  bounds: boolean;
  joints: boolean;
  sleep: boolean;
  particles: boolean;
}

/** Fill and outline pairs for dynamic bodies; `Look.tint` indexes into this. */
const PALETTE: [string, string][] = [
  ['#f5a44a', '#ffd9a3'],
  ['#3fc7c0', '#a8f0ea'],
  ['#5b9cff', '#b9d5ff'],
  ['#c98b5a', '#efc7a3'],
  ['#ff6b5e', '#ffc0ba'],
  ['#b08cff', '#dccbff'],
];
const STATIC_FILL = '#262d3a';
const STATIC_LINE = '#4a5568';
const CLOTH_COLORS = ['#f1c27d', '#8fd3ff', '#ffa69e', '#b5e48c', '#d0b3ff'];

/** Draws the rigid bodies, joints, cloth and debug overlays with Canvas 2D. */
export class Renderer {
  readonly ctx: CanvasRenderingContext2D;
  dpr = 1;

  constructor(canvas: HTMLCanvasElement) {
    this.ctx = canvas.getContext('2d')!;
  }

  resize(width: number, height: number, dpr: number): void {
    this.dpr = dpr;
    this.ctx.canvas.width = Math.max(1, Math.round(width * dpr));
    this.ctx.canvas.height = Math.max(1, Math.round(height * dpr));
  }

  draw(world: World, camera: Camera, alpha: number, debug: DebugOptions): void {
    const ctx = this.ctx;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    camera.apply(ctx, this.dpr);
    // One CSS pixel, expressed in world units.
    const px = 1 / camera.scale;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    if (debug.bounds) this.drawBounds(world, px);
    this.drawSoft(world, alpha, px, debug);
    this.drawBodies(world, alpha, px, debug);
    this.drawJoints(world, alpha, px, debug);
    if (debug.particles) this.drawFluidPoints(world, alpha, px);
    if (debug.contacts) this.drawContacts(world, px);
  }

  private drawBodies(world: World, alpha: number, px: number, debug: DebugOptions): void {
    const ctx = this.ctx;
    for (const body of world.bodies) {
      const t = interpolate(body, alpha);
      const look = body.userData as Look | undefined;
      const dynamic = body.isDynamic;
      const [fill, line] = dynamic ? PALETTE[(look?.tint ?? 0) % PALETTE.length] : [STATIC_FILL, STATIC_LINE];
      ctx.globalAlpha = dynamic && debug.sleep && !body.awake ? 0.35 : 1;
      ctx.fillStyle = fill;
      ctx.strokeStyle = line;
      ctx.lineWidth = 1.5 * px;
      for (const shape of body.shapes) {
        ctx.beginPath();
        tracePath(ctx, shape, t);
        ctx.fill();
        ctx.stroke();
        if (shape.kind === 'circle' && dynamic) {
          // A spoke, so rotation is visible.
          const cx = t.x + t.c * shape.localX - t.s * shape.localY;
          const cy = t.y + t.s * shape.localX + t.c * shape.localY;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx + t.c * shape.radius, cy + t.s * shape.radius);
          ctx.stroke();
        }
      }
    }
    ctx.globalAlpha = 1;
  }

  private drawJoints(world: World, alpha: number, px: number, debug: DebugOptions): void {
    const ctx = this.ctx;
    for (const joint of world.joints) {
      if (joint instanceof DistanceJoint) {
        const a = anchor(joint.bodyA, joint.localAx, joint.localAy, alpha);
        const b = anchor(joint.bodyB, joint.localBx, joint.localBy, alpha);
        ctx.strokeStyle = joint.mode === 'spring' ? '#9be28b' : '#c9d1e0';
        ctx.lineWidth = (joint.mode === 'rope' ? 2.5 : 1.5) * px;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      } else if (joint instanceof MouseJoint) {
        const a = anchor(joint.bodyB, joint.localX, joint.localY, alpha);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5 * px;
        ctx.setLineDash([4 * px, 4 * px]);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(joint.targetX, joint.targetY);
        ctx.stroke();
        ctx.setLineDash([]);
      } else if (joint instanceof RevoluteJoint) {
        const a = anchor(joint.bodyB, joint.localBx, joint.localBy, alpha);
        ctx.fillStyle = debug.joints ? '#ffe066' : 'rgba(255, 255, 255, 0.55)';
        ctx.beginPath();
        ctx.arc(a.x, a.y, (debug.joints ? 3.5 : 2) * px, 0, 6.2832);
        ctx.fill();
      }
    }
  }

  private drawSoft(world: World, alpha: number, px: number, debug: DebugOptions): void {
    const soft = world.soft;
    if (soft.count === 0) return;
    const ctx = this.ctx;
    const { x, y, ox, oy } = soft;
    const X = (i: number) => ox[i] + (x[i] - ox[i]) * alpha;
    const Y = (i: number) => oy[i] + (y[i] - oy[i]) * alpha;

    // Blobs: a filled closed curve through the skin particles.
    for (const group of soft.groups) {
      if (group.kind !== 'blob') continue;
      const n = group.count;
      const start = group.start;
      const color = CLOTH_COLORS[(group.id + 2) % CLOTH_COLORS.length];
      ctx.beginPath();
      for (let k = 0; k <= n; k++) {
        const a = start + (k % n);
        const b = start + ((k + 1) % n);
        const mx = (X(a) + X(b)) / 2;
        const my = (Y(a) + Y(b)) / 2;
        if (k === 0) ctx.moveTo(mx, my);
        else ctx.quadraticCurveTo(X(a), Y(a), mx, my);
      }
      ctx.closePath();
      ctx.fillStyle = color + 'cc';
      ctx.strokeStyle = color;
      // The skin is as thick as the particles that collide, so nothing seems to float.
      ctx.lineWidth = 2 * soft.radius[start];
      ctx.stroke();
      ctx.fill();
    }

    // Cloth and ropes: the structural threads, one path per group.
    for (const group of soft.groups) {
      if (group.kind === 'blob') continue;
      ctx.beginPath();
      for (let i = 0; i < soft.constraintCount; i++) {
        if (soft.cKind[i] !== STRUCTURAL) continue;
        const a = soft.cA[i];
        if (soft.group[a] !== group.id) continue;
        const b = soft.cB[i];
        ctx.moveTo(X(a), Y(a));
        ctx.lineTo(X(b), Y(b));
      }
      ctx.strokeStyle = CLOTH_COLORS[group.id % CLOTH_COLORS.length];
      ctx.lineWidth = group.kind === 'rope' ? 2 * soft.radius[group.start] : Math.max(1.2 * px, 0.035);
      ctx.stroke();
    }

    // Pins.
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    for (let i = 0; i < soft.count; i++) {
      if (soft.invMass[i] !== 0) continue;
      ctx.moveTo(X(i) + 2.5 * px, Y(i));
      ctx.arc(X(i), Y(i), 2.5 * px, 0, 6.2832);
    }
    ctx.fill();

    if (debug.particles) {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
      ctx.lineWidth = px;
      ctx.beginPath();
      for (let i = 0; i < soft.count; i++) {
        ctx.moveTo(X(i) + soft.radius[i], Y(i));
        ctx.arc(X(i), Y(i), soft.radius[i], 0, 6.2832);
      }
      ctx.stroke();
    }
  }

  private drawFluidPoints(world: World, alpha: number, px: number): void {
    const fluid = world.fluid;
    const ctx = this.ctx;
    ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
    ctx.beginPath();
    const r = 1.5 * px;
    for (let i = 0; i < fluid.count; i++) {
      const fx = fluid.ox[i] + (fluid.x[i] - fluid.ox[i]) * alpha;
      const fy = fluid.oy[i] + (fluid.y[i] - fluid.oy[i]) * alpha;
      ctx.rect(fx - r, fy - r, 2 * r, 2 * r);
    }
    ctx.fill();
  }

  private drawBounds(world: World, px: number): void {
    const ctx = this.ctx;
    ctx.lineWidth = px;
    world.broadphase.tree.forEachNode((minX, minY, maxX, maxY, leaf) => {
      ctx.strokeStyle = leaf ? 'rgba(120, 220, 160, 0.55)' : 'rgba(120, 160, 255, 0.16)';
      ctx.strokeRect(minX, minY, maxX - minX, maxY - minY);
    });
  }

  private drawContacts(world: World, px: number): void {
    const ctx = this.ctx;
    ctx.lineWidth = 1.5 * px;
    for (const contact of world.contacts.values()) {
      const m = contact.manifold;
      for (let k = 0; k < m.count; k++) {
        const p = m.points[k];
        // Red: carrying load. Yellow: speculative, not touching yet.
        const touching = p.normalImpulse > 0;
        ctx.strokeStyle = ctx.fillStyle = touching ? '#ff5d73' : '#ffd166';
        ctx.beginPath();
        ctx.arc(p.x, p.y, 2.5 * px, 0, 6.2832);
        ctx.fill();
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + m.nx * 10 * px, p.y + m.ny * 10 * px);
        ctx.stroke();
      }
    }
  }
}

interface Pose {
  x: number;
  y: number;
  c: number;
  s: number;
}

const pose: Pose = { x: 0, y: 0, c: 1, s: 0 };

/** Blends the previous and current transform, so motion is smooth at any refresh rate. */
function interpolate(body: Body, alpha: number): Pose {
  pose.x = body.prevX + (body.x - body.prevX) * alpha;
  pose.y = body.prevY + (body.y - body.prevY) * alpha;
  const c = body.prevC + (body.c - body.prevC) * alpha;
  const s = body.prevS + (body.s - body.prevS) * alpha;
  const length = Math.sqrt(c * c + s * s) || 1;
  pose.c = c / length;
  pose.s = s / length;
  return pose;
}

function anchor(body: Body, localX: number, localY: number, alpha: number): { x: number; y: number } {
  const t = interpolate(body, alpha);
  return { x: t.x + t.c * localX - t.s * localY, y: t.y + t.s * localX + t.c * localY };
}

function tracePath(ctx: CanvasRenderingContext2D, shape: Shape, t: Pose): void {
  if (shape.kind === 'circle') {
    const cx = t.x + t.c * shape.localX - t.s * shape.localY;
    const cy = t.y + t.s * shape.localX + t.c * shape.localY;
    ctx.arc(cx, cy, shape.radius, 0, 6.2832);
    return;
  }
  const v = shape.localVertices;
  for (let i = 0; i < shape.count; i++) {
    const x = t.x + t.c * v[2 * i] - t.s * v[2 * i + 1];
    const y = t.y + t.s * v[2 * i] + t.c * v[2 * i + 1];
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.closePath();
}
