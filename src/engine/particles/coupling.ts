import type { Shape } from '../collision/shape.ts';

/** Fraction of a particle's penetration that a body is asked to back out of per step. */
const PUSH_FACTOR = 0.2;
/** Fastest speed, m/s, at which a body is pushed out of particles it overlaps. */
const MAX_PUSH_SPEED = 1;

/**
 * Two-way coupling between particles (cloth, liquid) and rigid bodies.
 *
 * The particle systems run after the rigid solver and see bodies as fixed obstacles: a
 * particle found inside a shape is simply moved out. This class records those corrections
 * during a sub-step and afterwards turns them into an exchange of momentum:
 *
 * - The particle must not keep the velocity it gained from being moved out ("pop-out"); it
 *   may leave the surface no faster than the surface itself moves.
 * - The body receives the opposite of the momentum the particle really gained, which is how
 *   cloth tension and liquid pressure reach it. That impulse is capped at what stops the body
 *   moving into the particle, plus a gentle push out of the overlap. Particles are light and
 *   their constraints stiff, so without the cap this explicit hand-over overshoots and throws
 *   bodies around; with it, cloth and liquid can halt and carry a body but never launch it.
 */
export class BodyContacts {
  count = 0;
  private particle = new Int32Array(256);
  private shapes: Shape[] = [];
  private nx = new Float64Array(256);
  private ny = new Float64Array(256);
  private depth = new Float64Array(256);
  /** Per particle: index of its most recent record in this sub-step, or -1. */
  private last = new Int32Array(256);

  begin(particleCount: number): void {
    this.count = 0;
    if (this.last.length < particleCount) this.last = new Int32Array(particleCount * 2);
    this.last.fill(-1, 0, particleCount);
  }

  /** Records that particle `i` was moved `depth` along the normal to leave `shape`. */
  add(i: number, shape: Shape, nx: number, ny: number, depth: number): void {
    const previous = this.last[i];
    if (previous >= 0 && this.shapes[previous] === shape) {
      // Same particle against the same shape again (a later solver pass): merge.
      this.depth[previous] += depth;
      this.nx[previous] = nx;
      this.ny[previous] = ny;
      return;
    }
    const k = this.count++;
    if (k === this.particle.length) {
      const size = k * 2;
      const grow = <A extends Float64Array | Int32Array>(a: A): A => {
        const next = new (a.constructor as new (n: number) => A)(size);
        next.set(a);
        return next;
      };
      this.particle = grow(this.particle);
      this.nx = grow(this.nx);
      this.ny = grow(this.ny);
      this.depth = grow(this.depth);
    }
    this.particle[k] = i;
    this.shapes[k] = shape;
    this.nx[k] = nx;
    this.ny[k] = ny;
    this.depth[k] = depth;
    this.last[i] = k;
  }

  /**
   * Call once per sub-step, after velocities were derived from positions.
   * `invMass` gives per-particle inverse masses; when null every particle weighs `mass`.
   */
  resolve(
    x: Float64Array,
    y: Float64Array,
    vx: Float64Array,
    vy: Float64Array,
    invMass: Float64Array | null,
    mass: number,
    h: number,
    dt: number,
  ): void {
    const invH = 1 / h;
    for (let k = 0; k < this.count; k++) {
      const i = this.particle[k];
      const nx = this.nx[k];
      const ny = this.ny[k];
      const depth = this.depth[k];
      const body = this.shapes[k].body;

      const rx = x[i] - body.x;
      const ry = y[i] - body.y;
      const awake = body.awake;
      // Normal velocity of the body's surface at the particle (zero for static or sleeping bodies).
      const surface = awake ? (body.vx - body.w * ry) * nx + (body.vy + body.w * rx) * ny : 0;

      const vn = vx[i] * nx + vy[i] * ny;
      // What the particle's normal velocity would have been had the shape not been there.
      const before = vn - depth * invH;
      let after = vn;
      if (vn > surface) {
        after = Math.max(surface, before);
        vx[i] += (after - vn) * nx;
        vy[i] += (after - vn) * ny;
      }

      if (!body.isDynamic) continue;
      const particleMass = invMass === null ? mass : 1 / invMass[i];
      const gained = particleMass * (after - before);
      if (gained <= 0) continue;
      // The body should end up moving into the particle no faster than this.
      const target = after - Math.min((PUSH_FACTOR * depth) / dt, MAX_PUSH_SPEED);
      const excess = surface - target;
      if (excess <= 0) continue;
      const rn = rx * ny - ry * nx;
      const impulse = Math.min(gained, excess / (body.invMass + body.invInertia * rn * rn));
      if (awake) {
        body.vx -= body.invMass * impulse * nx;
        body.vy -= body.invMass * impulse * ny;
        body.w -= body.invInertia * rn * impulse;
      } else {
        // A sleeping body only keeps score; a strong enough total wakes it next step.
        body.queueImpulse(-impulse * nx, -impulse * ny, x[i], y[i]);
      }
    }
  }
}
