import type { SurfaceSample } from '../collision/shape.ts';
import type { StateHash } from '../math/hash.ts';
import { cos, sin, TWO_PI } from '../math/trig.ts';
import type { World } from '../world.ts';
import { BodyContacts } from './coupling.ts';
import { NeighborGrid, ShapeCandidates } from './grid.ts';

/** What a distance constraint is for; only structural ones are drawn. */
export const STRUCTURAL = 0;
export const SHEAR = 1;
export const BEND = 2;

/** Keeps the area enclosed by a closed loop of particles near a target (a 2D "volume"). */
export interface AreaConstraint {
  indices: Int32Array;
  restArea: number;
  /** Target area as a multiple of the rest area; above 1 inflates the loop. */
  pressure: number;
  compliance: number;
}

/** One object made of particles: a sheet of cloth, a rope or a blob. */
export interface SoftGroup {
  id: number;
  kind: 'cloth' | 'rope' | 'blob';
  /** Index of the first particle; the group's particles are contiguous. */
  start: number;
  count: number;
  cols: number;
  rows: number;
  area: AreaConstraint | null;
}

export interface ClothOptions {
  /** Top-left corner. */
  x: number;
  y: number;
  cols: number;
  rows: number;
  spacing: number;
  /** Mass of one particle, kg. */
  mass?: number;
  /** Inverse stiffness of the threads, m/N. Zero is inextensible. */
  compliance?: number;
  shearCompliance?: number;
  bendCompliance?: number;
  /** A thread snaps when stretched beyond this multiple of its rest length. Zero disables tearing. */
  tearRatio?: number;
  /** Which particles of the top row are fixed: all of them, the two corners, every n-th, or none. */
  pin?: 'top' | 'corners' | 'none' | number;
}

export interface RopeOptions {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
  segments: number;
  mass?: number;
  compliance?: number;
  pinStart?: boolean;
  pinEnd?: boolean;
  radius?: number;
}

export interface BlobOptions {
  x: number;
  y: number;
  radius: number;
  segments?: number;
  mass?: number;
  /** Stiffness of the skin. */
  compliance?: number;
  /** Stiffness of the enclosed area. */
  areaCompliance?: number;
  pressure?: number;
}

const sample: SurfaceSample = { distance: 0, nx: 0, ny: 0 };

function grown<A extends Float64Array | Int32Array | Uint8Array>(source: A, size: number): A {
  const next = new (source.constructor as new (n: number) => A)(size);
  next.set(source);
  return next;
}

/**
 * Cloth, ropes and soft bodies, simulated with extended position based dynamics (XPBD).
 *
 * Instead of computing forces, XPBD moves particles directly so that constraints such as
 * "these two particles are 5 cm apart" hold, and derives velocities from how far the particles
 * moved. Each constraint has a compliance (inverse stiffness) with physical units, and the
 * step is split into many small sub-steps with a single solver pass each, which converges
 * better than one big step with many passes.
 *
 * Particles collide with rigid shapes as small discs, and exchange momentum with the bodies
 * they touch (see BodyContacts), so a box can land on a sheet of cloth and weigh it down.
 */
export class SoftSystem {
  count = 0;
  x = new Float64Array(256);
  y = new Float64Array(256);
  /** Position at the start of the sub-step. */
  px = new Float64Array(256);
  py = new Float64Array(256);
  /** Position at the start of the step, for render interpolation. */
  ox = new Float64Array(256);
  oy = new Float64Array(256);
  vx = new Float64Array(256);
  vy = new Float64Array(256);
  /** Zero for pinned particles. */
  invMass = new Float64Array(256);
  radius = new Float64Array(256);
  group = new Int32Array(256);

  constraintCount = 0;
  cA = new Int32Array(512);
  cB = new Int32Array(512);
  cRest = new Float64Array(512);
  cCompliance = new Float64Array(512);
  cTear = new Float64Array(512);
  cKind = new Uint8Array(512);

  readonly groups: SoftGroup[] = [];
  readonly areas: AreaConstraint[] = [];

  substeps = 10;
  /** Velocity lost per second, as a rate; a crude stand-in for air drag. */
  damping = 0.4;
  friction = 0.5;
  gravityScale = 1;
  /** Particles of different groups push each other apart. */
  groupCollisions = true;

  private maxRadius = 0;
  private torn = false;
  private readonly candidates = new ShapeCandidates();
  private readonly grid = new NeighborGrid();
  private readonly contacts = new BodyContacts();

  // ---- construction ----------------------------------------------------------------------------

  addParticle(x: number, y: number, mass: number, radius: number, group: number): number {
    const i = this.count++;
    if (i === this.x.length) {
      const size = i * 2;
      this.x = grown(this.x, size);
      this.y = grown(this.y, size);
      this.px = grown(this.px, size);
      this.py = grown(this.py, size);
      this.ox = grown(this.ox, size);
      this.oy = grown(this.oy, size);
      this.vx = grown(this.vx, size);
      this.vy = grown(this.vy, size);
      this.invMass = grown(this.invMass, size);
      this.radius = grown(this.radius, size);
      this.group = grown(this.group, size);
    }
    this.x[i] = this.px[i] = this.ox[i] = x;
    this.y[i] = this.py[i] = this.oy[i] = y;
    this.vx[i] = this.vy[i] = 0;
    this.invMass[i] = mass > 0 ? 1 / mass : 0;
    this.radius[i] = radius;
    this.group[i] = group;
    if (radius > this.maxRadius) this.maxRadius = radius;
    return i;
  }

  /** Adds a distance constraint between two particles, at their current distance unless given. */
  addDistance(a: number, b: number, compliance = 0, kind = STRUCTURAL, tearRatio = 0, rest = -1): number {
    const i = this.constraintCount++;
    if (i === this.cA.length) {
      const size = i * 2;
      this.cA = grown(this.cA, size);
      this.cB = grown(this.cB, size);
      this.cRest = grown(this.cRest, size);
      this.cCompliance = grown(this.cCompliance, size);
      this.cTear = grown(this.cTear, size);
      this.cKind = grown(this.cKind, size);
    }
    if (rest < 0) {
      const dx = this.x[b] - this.x[a];
      const dy = this.y[b] - this.y[a];
      rest = Math.sqrt(dx * dx + dy * dy);
    }
    this.cA[i] = a;
    this.cB[i] = b;
    this.cRest[i] = rest;
    this.cCompliance[i] = compliance;
    this.cTear[i] = tearRatio;
    this.cKind[i] = kind;
    return i;
  }

  /** A rectangular sheet hanging from its top row. */
  createCloth(options: ClothOptions): SoftGroup {
    const { cols, rows, spacing } = options;
    const mass = options.mass ?? 0.02;
    const compliance = options.compliance ?? 0;
    const shear = options.shearCompliance ?? 2e-3;
    const bend = options.bendCompliance ?? 2e-2;
    const tear = options.tearRatio ?? 0;
    const pin = options.pin ?? 'top';
    const group = this.newGroup('cloth', cols, rows);
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const pinned =
          r === 0 &&
          (pin === 'top' ||
            (pin === 'corners' && (c === 0 || c === cols - 1)) ||
            (typeof pin === 'number' && (c % pin === 0 || c === cols - 1)));
        this.addParticle(options.x + c * spacing, options.y - r * spacing, pinned ? 0 : mass, 0.5 * spacing, group.id);
      }
    }
    group.count = cols * rows;
    const at = (c: number, r: number) => group.start + r * cols + c;
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        if (c + 1 < cols) this.addDistance(at(c, r), at(c + 1, r), compliance, STRUCTURAL, tear);
        if (r + 1 < rows) this.addDistance(at(c, r), at(c, r + 1), compliance, STRUCTURAL, tear);
        if (c + 1 < cols && r + 1 < rows) {
          this.addDistance(at(c, r), at(c + 1, r + 1), shear, SHEAR, tear);
          this.addDistance(at(c + 1, r), at(c, r + 1), shear, SHEAR, tear);
        }
        if (c + 2 < cols) this.addDistance(at(c, r), at(c + 2, r), bend, BEND, tear);
        if (r + 2 < rows) this.addDistance(at(c, r), at(c, r + 2), bend, BEND, tear);
      }
    }
    return group;
  }

  createRope(options: RopeOptions): SoftGroup {
    const { segments } = options;
    const mass = options.mass ?? 0.05;
    const spanX = options.x1 - options.x0;
    const spanY = options.y1 - options.y0;
    const length = Math.sqrt(spanX * spanX + spanY * spanY);
    const radius = options.radius ?? (0.5 * length) / segments;
    const group = this.newGroup('rope', segments + 1, 1);
    for (let i = 0; i <= segments; i++) {
      const t = i / segments;
      const pinned = (i === 0 && options.pinStart !== false) || (i === segments && options.pinEnd === true);
      this.addParticle(
        options.x0 + t * (options.x1 - options.x0),
        options.y0 + t * (options.y1 - options.y0),
        pinned ? 0 : mass,
        radius,
        group.id,
      );
    }
    group.count = segments + 1;
    for (let i = 0; i < segments; i++) {
      this.addDistance(group.start + i, group.start + i + 1, options.compliance ?? 0);
    }
    return group;
  }

  /** A closed, pressurised loop: a 2D balloon that squashes and springs back. */
  createBlob(options: BlobOptions): SoftGroup {
    const segments = options.segments ?? 20;
    const mass = options.mass ?? 0.05;
    const compliance = options.compliance ?? 1e-4;
    const group = this.newGroup('blob', segments, 1);
    const chord = 2 * options.radius * sin(Math.PI / segments);
    for (let i = 0; i < segments; i++) {
      const angle = (TWO_PI * i) / segments;
      this.addParticle(
        options.x + options.radius * cos(angle),
        options.y + options.radius * sin(angle),
        mass,
        0.6 * chord,
        group.id,
      );
    }
    group.count = segments;
    const indices = new Int32Array(segments);
    for (let i = 0; i < segments; i++) {
      indices[i] = group.start + i;
      this.addDistance(group.start + i, group.start + ((i + 1) % segments), compliance);
      this.addDistance(group.start + i, group.start + ((i + 2) % segments), 20 * compliance, BEND);
    }
    const area: AreaConstraint = {
      indices,
      restArea: this.loopArea(indices),
      pressure: options.pressure ?? 1,
      compliance: options.areaCompliance ?? 1e-5,
    };
    this.areas.push(area);
    group.area = area;
    return group;
  }

  private newGroup(kind: SoftGroup['kind'], cols: number, rows: number): SoftGroup {
    const group: SoftGroup = { id: this.groups.length, kind, start: this.count, count: 0, cols, rows, area: null };
    this.groups.push(group);
    return group;
  }

  // ---- interaction -----------------------------------------------------------------------------

  /** Index of the closest particle within `maxDistance`, or -1. */
  nearest(x: number, y: number, maxDistance: number): number {
    let best = -1;
    let best2 = maxDistance * maxDistance;
    for (let i = 0; i < this.count; i++) {
      const dx = this.x[i] - x;
      const dy = this.y[i] - y;
      const d2 = dx * dx + dy * dy;
      if (d2 < best2) {
        best2 = d2;
        best = i;
      }
    }
    return best;
  }

  /** Removes every constraint crossed by the segment; returns how many were cut. */
  cut(x0: number, y0: number, x1: number, y1: number): number {
    let cuts = 0;
    const rx = x1 - x0;
    const ry = y1 - y0;
    for (let i = 0; i < this.constraintCount; i++) {
      const a = this.cA[i];
      if (a < 0) continue;
      const b = this.cB[i];
      const ax = this.x[a];
      const ay = this.y[a];
      const sx = this.x[b] - ax;
      const sy = this.y[b] - ay;
      const denom = rx * sy - ry * sx;
      if (denom === 0) continue;
      const t = ((ax - x0) * sy - (ay - y0) * sx) / denom;
      const u = ((ax - x0) * ry - (ay - y0) * rx) / denom;
      if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        this.cA[i] = -1;
        cuts++;
      }
    }
    if (cuts > 0) this.compact();
    return cuts;
  }

  clear(): void {
    this.count = 0;
    this.constraintCount = 0;
    this.groups.length = 0;
    this.areas.length = 0;
    this.maxRadius = 0;
  }

  hashInto(hash: StateHash): void {
    for (let i = 0; i < this.count; i++) {
      hash.add(this.x[i]);
      hash.add(this.y[i]);
      hash.add(this.vx[i]);
      hash.add(this.vy[i]);
    }
    hash.add(this.constraintCount);
  }

  // ---- simulation ------------------------------------------------------------------------------

  step(world: World): void {
    const n = this.count;
    if (n === 0) return;
    const h = world.dt / this.substeps;
    const gx = world.gravityX * this.gravityScale;
    const gy = world.gravityY * this.gravityScale;
    const { x, y, px, py, vx, vy, invMass } = this;

    this.ox.set(x.subarray(0, n));
    this.oy.set(y.subarray(0, n));
    this.candidates.gather(world, x, y, vx, vy, n, this.maxRadius);

    const drag = 1 / (1 + h * this.damping);
    const collideGroups = this.groupCollisions && this.groups.length > 1;

    for (let step = 0; step < this.substeps; step++) {
      // Predict: gravity changes velocity, velocity changes position.
      for (let i = 0; i < n; i++) {
        px[i] = x[i];
        py[i] = y[i];
        if (invMass[i] === 0) continue;
        // Drag acts on the velocity the particle already has, not on gravity's contribution,
        // so it slows motion without making things hang as if they were lighter.
        vx[i] = vx[i] * drag + gx * h;
        vy[i] = vy[i] * drag + gy * h;
        x[i] += vx[i] * h;
        y[i] += vy[i] * h;
      }

      // Project: move particles so the constraints hold.
      this.contacts.begin(n);
      this.solveDistances(h);
      for (const area of this.areas) this.solveArea(area, h);
      this.collideShapes(h);
      if (collideGroups) this.collideGroups();

      // Derive velocities from the corrected positions.
      const invH = 1 / h;
      for (let i = 0; i < n; i++) {
        if (invMass[i] === 0) {
          vx[i] = vy[i] = 0;
          continue;
        }
        vx[i] = (x[i] - px[i]) * invH;
        vy[i] = (y[i] - py[i]) * invH;
      }
      this.contacts.resolve(x, y, vx, vy, invMass, 0, h, world.dt);
    }

    if (this.torn) this.compact();
  }

  private solveDistances(h: number): void {
    const { x, y, invMass, cA, cB, cRest, cCompliance, cTear } = this;
    const invH2 = 1 / (h * h);
    for (let i = 0; i < this.constraintCount; i++) {
      const a = cA[i];
      if (a < 0) continue;
      const b = cB[i];
      const wA = invMass[a];
      const wB = invMass[b];
      const w = wA + wB;
      if (w === 0) continue;
      const dx = x[b] - x[a];
      const dy = y[b] - y[a];
      const len = Math.sqrt(dx * dx + dy * dy);
      if (len < 1e-12) continue;
      const rest = cRest[i];
      if (cTear[i] > 0 && len > rest * cTear[i]) {
        cA[i] = -1;
        this.torn = true;
        continue;
      }
      // XPBD: the compliance term makes the stiffness independent of the sub-step size.
      // Each sub-step starts from a zero multiplier, which is the "small steps" variant.
      const lambda = -(len - rest) / (w + cCompliance[i] * invH2);
      const sx = (lambda * dx) / len;
      const sy = (lambda * dy) / len;
      x[a] -= wA * sx;
      y[a] -= wA * sy;
      x[b] += wB * sx;
      y[b] += wB * sy;
    }
  }

  private loopArea(indices: Int32Array): number {
    const { x, y } = this;
    let area = 0;
    for (let k = 0; k < indices.length; k++) {
      const i = indices[k];
      const j = indices[(k + 1) % indices.length];
      area += x[i] * y[j] - x[j] * y[i];
    }
    return 0.5 * area;
  }

  private solveArea(constraint: AreaConstraint, h: number): void {
    const { x, y, invMass } = this;
    const indices = constraint.indices;
    const count = indices.length;
    const c = this.loopArea(indices) - constraint.restArea * constraint.pressure;
    // Gradient of the area with respect to particle k is half the edge between its two
    // neighbours, rotated by 90 degrees.
    let denominator = constraint.compliance / (h * h);
    for (let k = 0; k < count; k++) {
      const prev = indices[(k + count - 1) % count];
      const next = indices[(k + 1) % count];
      const gx = 0.5 * (y[next] - y[prev]);
      const gy = 0.5 * (x[prev] - x[next]);
      denominator += invMass[indices[k]] * (gx * gx + gy * gy);
    }
    if (denominator < 1e-12) return;
    const lambda = -c / denominator;
    // Gradients are taken before any particle moves, hence the two passes.
    const scratch = this.areaScratch(count);
    for (let k = 0; k < count; k++) {
      const prev = indices[(k + count - 1) % count];
      const next = indices[(k + 1) % count];
      scratch[2 * k] = 0.5 * (y[next] - y[prev]);
      scratch[2 * k + 1] = 0.5 * (x[prev] - x[next]);
    }
    for (let k = 0; k < count; k++) {
      const i = indices[k];
      x[i] += lambda * invMass[i] * scratch[2 * k];
      y[i] += lambda * invMass[i] * scratch[2 * k + 1];
    }
  }

  private scratch = new Float64Array(64);
  private areaScratch(count: number): Float64Array {
    if (this.scratch.length < 2 * count) this.scratch = new Float64Array(4 * count);
    return this.scratch;
  }

  /** Pushes particles out of rigid shapes, with friction, and records the contact for coupling. */
  private collideShapes(h: number): void {
    const { x, y, px, py, invMass, radius } = this;
    const start = this.candidates.start;
    const shapes = this.candidates.shapes;
    const friction = this.friction;
    for (let i = 0; i < this.count; i++) {
      if (invMass[i] === 0) continue;
      const r = radius[i];
      for (let k = start[i]; k < start[i + 1]; k++) {
        const shape = shapes[k];
        shape.sample(x[i], y[i], sample);
        if (sample.distance >= r) continue;
        const depth = r - sample.distance;
        const nx = sample.nx;
        const ny = sample.ny;
        x[i] += nx * depth;
        y[i] += ny * depth;

        // Friction acts on the sliding of the particle relative to the moving surface.
        const body = shape.body;
        const slipX = x[i] - px[i] - (body.vx - body.w * (y[i] - body.y)) * h;
        const slipY = y[i] - py[i] - (body.vy + body.w * (x[i] - body.x)) * h;
        const along = slipX * nx + slipY * ny;
        const tx = slipX - along * nx;
        const ty = slipY - along * ny;
        const slip = Math.sqrt(tx * tx + ty * ty);
        if (slip > 1e-12) {
          // Static friction cancels the slide entirely; kinetic friction a limited part of it.
          const limit = friction * depth;
          const scale = slip <= limit ? 1 : limit / slip;
          x[i] -= tx * scale;
          y[i] -= ty * scale;
        }

        this.contacts.add(i, shape, nx, ny, depth);
      }
    }
  }

  /** Keeps particles of different objects from overlapping. */
  private collideGroups(): void {
    const { x, y, invMass, radius, group } = this;
    const grid = this.grid;
    grid.build(x, y, this.count, 2 * this.maxRadius);
    const start = grid.start;
    const neighbors = grid.neighbors;
    for (let i = 0; i < this.count; i++) {
      for (let k = start[i]; k < start[i + 1]; k++) {
        const j = neighbors[k];
        if (j < i || group[j] === group[i]) continue;
        const w = invMass[i] + invMass[j];
        if (w === 0) continue;
        const dx = x[j] - x[i];
        const dy = y[j] - y[i];
        const dist = Math.sqrt(dx * dx + dy * dy);
        const overlap = radius[i] + radius[j] - dist;
        if (overlap <= 0 || dist < 1e-12) continue;
        const push = overlap / (w * dist);
        x[i] -= dx * push * invMass[i];
        y[i] -= dy * push * invMass[i];
        x[j] += dx * push * invMass[j];
        y[j] += dy * push * invMass[j];
      }
    }
  }

  /** Drops constraints that were cut or torn, keeping the others in order. */
  private compact(): void {
    let write = 0;
    for (let i = 0; i < this.constraintCount; i++) {
      if (this.cA[i] < 0) continue;
      this.cA[write] = this.cA[i];
      this.cB[write] = this.cB[i];
      this.cRest[write] = this.cRest[i];
      this.cCompliance[write] = this.cCompliance[i];
      this.cTear[write] = this.cTear[i];
      this.cKind[write] = this.cKind[i];
      write++;
    }
    this.constraintCount = write;
    this.torn = false;
  }
}
