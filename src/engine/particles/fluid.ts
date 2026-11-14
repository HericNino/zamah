import type { SurfaceSample } from '../collision/shape.ts';
import type { StateHash } from '../math/hash.ts';
import type { World } from '../world.ts';
import { BodyContacts } from './coupling.ts';
import { NeighborGrid, ShapeCandidates } from './grid.ts';

export interface FluidOptions {
  /** Rest distance between neighbouring particles, metres. Sets the resolution of the liquid. */
  spacing?: number;
  /** Mass per unit area, kg/m^2. Bodies less dense than this float. */
  density?: number;
}

const sample: SurfaceSample = { distance: 0, nx: 0, ny: 0 };

function grown(source: Float64Array, size: number): Float64Array<ArrayBuffer> {
  const next = new Float64Array(size);
  next.set(source);
  return next;
}

/**
 * A liquid made of particles, simulated with position based fluids (Macklin and Mueller, 2013).
 *
 * The idea: a liquid is (nearly) incompressible, so the density around every particle should
 * equal the rest density. Density is estimated by summing a smooth kernel over the neighbours.
 * Where it is too high, the particles are moved apart along the kernel gradient, exactly as
 * XPBD moves cloth particles to satisfy a distance constraint. Because it corrects positions
 * rather than integrating pressure forces, it stays stable at time steps where classic SPH
 * would blow up.
 */
export class Fluid {
  count = 0;
  x = new Float64Array(1024);
  y = new Float64Array(1024);
  /** Position at the start of the sub-step. */
  px = new Float64Array(1024);
  py = new Float64Array(1024);
  /** Position at the start of the step, for render interpolation. */
  ox = new Float64Array(1024);
  oy = new Float64Array(1024);
  vx = new Float64Array(1024);
  vy = new Float64Array(1024);
  /** Density relative to rest density, from the last solver pass; 1 means "at rest". */
  density = new Float64Array(1024);

  spacing = 0.12;
  /** Mass per unit area. */
  massDensity = 1;
  substeps = 2;
  iterations = 3;
  /** XSPH viscosity: how strongly a particle's velocity is blended towards its neighbours'. */
  viscosity = 0.06;
  /** Strength of the short-range repulsion that keeps particles from clumping. */
  antiClump = 0.02;
  /** Softens the density constraint; larger is more compressible but calmer. */
  relaxation = 0.2;
  /** Tangential speed kept after touching a wall or body, per contact. */
  wallSlip = 0.98;

  private kernelRadius = 0;
  private restDensity = 0;
  private poly6 = 0;
  private spiky = 0;
  private clumpReference = 0;

  private lambda = new Float64Array(1024);
  private dx = new Float64Array(1024);
  private dy = new Float64Array(1024);
  private readonly grid = new NeighborGrid();
  private readonly candidates = new ShapeCandidates();
  private readonly contacts = new BodyContacts();

  constructor(options: FluidOptions = {}) {
    this.configure(options);
  }

  /** Sets the resolution and density. Call before adding particles. */
  configure(options: FluidOptions): void {
    this.spacing = options.spacing ?? this.spacing;
    this.massDensity = options.density ?? this.massDensity;
    const h = (this.kernelRadius = 2.1 * this.spacing);
    const h2 = h * h;
    // 2D normalisation constants of the poly6 kernel and of the spiky kernel's gradient.
    this.poly6 = 4 / (Math.PI * h2 * h2 * h2 * h2);
    this.spiky = -30 / (Math.PI * h2 * h2 * h);

    // Rest density: what a particle measures inside a perfect hexagonal packing at rest spacing.
    const d = this.spacing;
    const rowHeight = d * 0.8660254037844386;
    let rest = 0;
    for (let row = -4; row <= 4; row++) {
      for (let col = -4; col <= 4; col++) {
        const lx = (col + (row & 1 ? 0.5 : 0)) * d;
        const ly = row * rowHeight;
        const r2 = lx * lx + ly * ly;
        if (r2 < h2) rest += this.poly6 * (h2 - r2) * (h2 - r2) * (h2 - r2);
      }
    }
    this.restDensity = rest;
    const q2 = 0.09 * h2;
    this.clumpReference = this.poly6 * (h2 - q2) * (h2 - q2) * (h2 - q2);
  }

  get particleMass(): number {
    return this.massDensity * this.spacing * this.spacing * 0.8660254037844386;
  }

  /** Radius at which particles collide with rigid shapes. */
  get particleRadius(): number {
    return 0.5 * this.spacing;
  }

  addParticle(x: number, y: number, vx = 0, vy = 0): number {
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
      this.density = grown(this.density, size);
      this.lambda = grown(this.lambda, size);
      this.dx = grown(this.dx, size);
      this.dy = grown(this.dy, size);
    }
    this.x[i] = this.px[i] = this.ox[i] = x;
    this.y[i] = this.py[i] = this.oy[i] = y;
    this.vx[i] = vx;
    this.vy[i] = vy;
    this.density[i] = 1;
    return i;
  }

  /** Fills a rectangle with particles in a hexagonal packing at rest spacing. Returns how many were added. */
  addBlock(minX: number, minY: number, width: number, height: number): number {
    const d = this.spacing;
    const rowHeight = d * 0.8660254037844386;
    const rows = Math.floor(height / rowHeight);
    const cols = Math.floor(width / d);
    let added = 0;
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col < cols; col++) {
        if (row & 1 && col === cols - 1) continue;
        this.addParticle(minX + (col + (row & 1 ? 1 : 0.5)) * d, minY + (row + 0.5) * rowHeight);
        added++;
      }
    }
    return added;
  }

  /** Removes a particle by moving the last one into its slot. */
  remove(i: number): void {
    const last = --this.count;
    if (i === last) return;
    this.x[i] = this.x[last];
    this.y[i] = this.y[last];
    this.px[i] = this.px[last];
    this.py[i] = this.py[last];
    this.ox[i] = this.ox[last];
    this.oy[i] = this.oy[last];
    this.vx[i] = this.vx[last];
    this.vy[i] = this.vy[last];
    this.density[i] = this.density[last];
  }

  /** Removes every particle outside the box; returns how many were removed. */
  removeOutside(minX: number, minY: number, maxX: number, maxY: number): number {
    let removed = 0;
    for (let i = this.count - 1; i >= 0; i--) {
      if (this.x[i] < minX || this.x[i] > maxX || this.y[i] < minY || this.y[i] > maxY) {
        this.remove(i);
        removed++;
      }
    }
    return removed;
  }

  clear(): void {
    this.count = 0;
  }

  hashInto(hash: StateHash): void {
    for (let i = 0; i < this.count; i++) {
      hash.add(this.x[i]);
      hash.add(this.y[i]);
      hash.add(this.vx[i]);
      hash.add(this.vy[i]);
    }
  }

  step(world: World): void {
    const n = this.count;
    if (n === 0) return;
    const { x, y, px, py, vx, vy } = this;
    const h = world.dt / this.substeps;
    const invH = 1 / h;
    const gx = world.gravityX;
    const gy = world.gravityY;
    // No particle may cross more than one rest spacing per sub-step (a CFL condition).
    const maxSpeed = this.spacing * invH;

    this.reorder();
    this.ox.set(x.subarray(0, n));
    this.oy.set(y.subarray(0, n));
    this.candidates.gather(world, x, y, vx, vy, n, this.particleRadius);

    for (let step = 0; step < this.substeps; step++) {
      for (let i = 0; i < n; i++) {
        let ux = vx[i] + gx * h;
        let uy = vy[i] + gy * h;
        const speed2 = ux * ux + uy * uy;
        if (speed2 > maxSpeed * maxSpeed) {
          const scale = maxSpeed / Math.sqrt(speed2);
          ux *= scale;
          uy *= scale;
        }
        px[i] = x[i];
        py[i] = y[i];
        x[i] += ux * h;
        y[i] += uy * h;
      }
      this.contacts.begin(n);

      this.collideShapes(h);
      this.grid.build(x, y, n, this.kernelRadius);
      for (let iteration = 0; iteration < this.iterations; iteration++) {
        this.solveDensity();
        this.collideShapes(h);
      }

      for (let i = 0; i < n; i++) {
        let ux = (x[i] - px[i]) * invH;
        let uy = (y[i] - py[i]) * invH;
        const speed2 = ux * ux + uy * uy;
        if (speed2 > maxSpeed * maxSpeed) {
          const scale = maxSpeed / Math.sqrt(speed2);
          ux *= scale;
          uy *= scale;
        }
        vx[i] = ux;
        vy[i] = uy;
      }
      this.contacts.resolve(x, y, vx, vy, null, this.particleMass, h, world.dt);
      if (this.viscosity > 0) this.applyViscosity();
    }
  }

  /**
   * Rearranges the particle arrays so that particles close in space are close in memory.
   * Every pass over neighbours then touches a few cache lines instead of the whole array,
   * which is worth more than any change to the arithmetic.
   */
  private reorder(): void {
    const n = this.count;
    const order = this.grid.sort(this.x, this.y, n, this.kernelRadius);
    const scratch = this.dx;
    for (const array of [this.x, this.y, this.vx, this.vy, this.density]) {
      for (let k = 0; k < n; k++) scratch[k] = array[order[k]];
      array.set(scratch.subarray(0, n));
    }
  }

  /** One pass of the density constraint over all particles (Jacobi style: compute all, then apply). */
  private solveDensity(): void {
    const n = this.count;
    const { x, y, lambda, dx, dy, density } = this;
    const start = this.grid.start;
    const neighbors = this.grid.neighbors;
    const h = this.kernelRadius;
    const h2 = h * h;
    const poly6 = this.poly6;
    const spiky = this.spiky;
    const invRest = 1 / this.restDensity;
    // The relaxation term has the units of a squared gradient, so scale it by 1/spacing^2.
    const epsilon = this.relaxation / (this.spacing * this.spacing);

    for (let i = 0; i < n; i++) {
      const xi = x[i];
      const yi = y[i];
      let rho = poly6 * h2 * h2 * h2;
      let gradX = 0;
      let gradY = 0;
      let sumGrad2 = 0;
      for (let k = start[i]; k < start[i + 1]; k++) {
        const j = neighbors[k];
        const rx = xi - x[j];
        const ry = yi - y[j];
        const r2 = rx * rx + ry * ry;
        const diff = h2 - r2;
        rho += poly6 * diff * diff * diff;
        const r = Math.sqrt(r2);
        if (r < 1e-9) continue;
        // Gradient of the constraint with respect to neighbour j (and, summed, to i itself).
        const g = (spiky * (h - r) * (h - r) * invRest) / r;
        const gx = g * rx;
        const gy = g * ry;
        gradX += gx;
        gradY += gy;
        sumGrad2 += gx * gx + gy * gy;
      }
      const ratio = rho * invRest;
      density[i] = ratio;
      // One-sided: only compression is corrected. Letting it also pull particles together
      // would make the free surface clump.
      const c = ratio - 1;
      lambda[i] = c > 0 ? -c / (sumGrad2 + gradX * gradX + gradY * gradY + epsilon) : 0;
    }

    const maxMove = 0.3 * this.spacing;
    const antiClump = this.antiClump * this.spacing * this.spacing;
    const invReference = 1 / this.clumpReference;
    for (let i = 0; i < n; i++) {
      const xi = x[i];
      const yi = y[i];
      const li = lambda[i];
      let moveX = 0;
      let moveY = 0;
      for (let k = start[i]; k < start[i + 1]; k++) {
        const j = neighbors[k];
        const rx = xi - x[j];
        const ry = yi - y[j];
        const r2 = rx * rx + ry * ry;
        const r = Math.sqrt(r2);
        if (r < 1e-9) continue;
        // Artificial pressure: a small extra repulsion at short range (the "tensile
        // instability" fix from the paper), which keeps the particles evenly spread.
        const diff = h2 - r2;
        const w = poly6 * diff * diff * diff * invReference;
        const w2 = w * w;
        const correction = -antiClump * w2 * w2;
        const g = ((li + lambda[j] + correction) * spiky * (h - r) * (h - r) * invRest) / r;
        moveX += g * rx;
        moveY += g * ry;
      }
      // All particles are corrected at once from the same snapshot (Jacobi), so a badly
      // compressed clump can overshoot. Capping one pass's correction keeps that bounded,
      // however the particles got there: poured on top of each other, hit by a blast, crushed.
      const move2 = moveX * moveX + moveY * moveY;
      if (move2 > maxMove * maxMove) {
        const scale = maxMove / Math.sqrt(move2);
        moveX *= scale;
        moveY *= scale;
      }
      dx[i] = moveX;
      dy[i] = moveY;
    }
    for (let i = 0; i < n; i++) {
      x[i] += dx[i];
      y[i] += dy[i];
    }
  }

  /** XSPH: nudges each velocity towards the local average, which damps noise like viscosity does. */
  private applyViscosity(): void {
    const n = this.count;
    const { x, y, vx, vy, dx, dy } = this;
    const start = this.grid.start;
    const neighbors = this.grid.neighbors;
    const h2 = this.kernelRadius * this.kernelRadius;
    const scale = (this.viscosity * this.poly6) / this.restDensity;
    for (let i = 0; i < n; i++) {
      let ax = 0;
      let ay = 0;
      for (let k = start[i]; k < start[i + 1]; k++) {
        const j = neighbors[k];
        const rx = x[i] - x[j];
        const ry = y[i] - y[j];
        const diff = h2 - rx * rx - ry * ry;
        if (diff <= 0) continue;
        const w = scale * diff * diff * diff;
        ax += w * (vx[j] - vx[i]);
        ay += w * (vy[j] - vy[i]);
      }
      dx[i] = ax;
      dy[i] = ay;
    }
    for (let i = 0; i < n; i++) {
      vx[i] += dx[i];
      vy[i] += dy[i];
    }
  }

  /** Pushes particles out of rigid shapes and records the contact for coupling. */
  private collideShapes(h: number): void {
    const { x, y, px, py } = this;
    const start = this.candidates.start;
    const shapes = this.candidates.shapes;
    const radius = this.particleRadius;
    const slip = this.wallSlip;
    for (let i = 0; i < this.count; i++) {
      for (let k = start[i]; k < start[i + 1]; k++) {
        const shape = shapes[k];
        shape.sample(x[i], y[i], sample);
        if (sample.distance >= radius) continue;
        const depth = radius - sample.distance;
        const nx = sample.nx;
        const ny = sample.ny;
        let moveX = nx * depth;
        let moveY = ny * depth;

        // A little drag along the surface, relative to the surface's own motion.
        const body = shape.body;
        const slideX = x[i] + moveX - px[i] - (body.vx - body.w * (y[i] - body.y)) * h;
        const slideY = y[i] + moveY - py[i] - (body.vy + body.w * (x[i] - body.x)) * h;
        const along = slideX * nx + slideY * ny;
        moveX -= (1 - slip) * (slideX - along * nx);
        moveY -= (1 - slip) * (slideY - along * ny);

        x[i] += moveX;
        y[i] += moveY;

        this.contacts.add(i, shape, nx, ny, depth);
      }
    }
  }
}
