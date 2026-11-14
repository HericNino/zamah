import { LINEAR_SLOP, MAX_IMPACT_PASSES } from '../settings.ts';
import type { ManifoldPoint } from '../collision/manifold.ts';
import type { Contact } from './contact.ts';
import type { StepContext } from './joint.ts';

/**
 * Contact constraints, solved with sequential impulses.
 *
 * Each contact point contributes two constraints: the bodies may not approach along the normal
 * (non-penetration), and they may not slide along the tangent harder than friction allows.
 * Every constraint is solved on its own by applying the impulse that fixes its velocity error;
 * sweeping over all of them repeatedly converges on a solution of the whole system
 * (projected Gauss-Seidel). The impulses are accumulated and clamped as totals, which lets an
 * iteration take back part of what an earlier one applied.
 */

/** Computes the per-step constants of every contact point. */
export function prepareContacts(contacts: Contact[], context: StepContext): void {
  for (let i = 0; i < contacts.length; i++) {
    const contact = contacts[i];
    const A = contact.bodyA;
    const B = contact.bodyB;
    const m = contact.manifold;
    const nx = m.nx;
    const ny = m.ny;
    const mA = A.invMass;
    const mB = B.invMass;
    const iA = A.invInertia;
    const iB = B.invInertia;
    contact.softness = mA === 0 || mB === 0 ? context.staticSoftness : context.contactSoftness;

    for (let k = 0; k < m.count; k++) {
      const p = m.points[k];
      const rAx = p.x - A.x;
      const rAy = p.y - A.y;
      const rBx = p.x - B.x;
      const rBy = p.y - B.y;
      // Remember the point in each body's own frame, so it can be followed as they move.
      p.localAx = A.c * rAx + A.s * rAy;
      p.localAy = -A.s * rAx + A.c * rAy;
      p.localBx = B.c * rBx + B.s * rBy;
      p.localBy = -B.s * rBx + B.c * rBy;
      p.gap = p.separation;
      setLeverArms(p, rAx, rAy, rBx, rBy, nx, ny, mA, mB, iA, iB);

      if (!context.warmStarting) {
        p.normalImpulse = 0;
        p.tangentImpulse = 0;
      }
    }
  }
}

/**
 * Brings every contact point up to date with where the bodies are now. Called after each
 * position update, so the solver always works on the current gap.
 *
 * Both bodies get their impulse at one shared world point (the midpoint of the two anchors).
 * Equal and opposite impulses at the same point cannot create angular momentum, which the
 * simpler "keep the lever arms from the start of the step" approach does.
 *
 * Two circles are cheap to collide exactly, so for them the normal is recomputed as well;
 * a glancing hit between fast balls then bounces along the true line of centres instead of
 * the one from up to a full step earlier.
 */
export function refreshContacts(contacts: Contact[]): void {
  for (let i = 0; i < contacts.length; i++) {
    const contact = contacts[i];
    const A = contact.bodyA;
    const B = contact.bodyB;
    const m = contact.manifold;
    const mA = A.invMass;
    const mB = B.invMass;
    const iA = A.invInertia;
    const iB = B.invInertia;

    if (contact.shapeB.kind === 'circle') {
      // The second shape is a circle (the first is a circle or a polygon).
      const b = contact.shapeB;
      const bx = B.x + B.c * b.localX - B.s * b.localY;
      const by = B.y + B.s * b.localX + B.c * b.localY;
      const p = m.points[0];
      let px: number;
      let py: number;
      if (contact.shapeA.kind === 'circle') {
        const a = contact.shapeA;
        const ax = A.x + A.c * a.localX - A.s * a.localY;
        const ay = A.y + A.s * a.localX + A.c * a.localY;
        const dx = bx - ax;
        const dy = by - ay;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > 1e-12) {
          m.nx = dx / dist;
          m.ny = dy / dist;
        }
        p.gap = dist - a.radius - b.radius;
        const offset = a.radius + 0.5 * p.gap;
        px = ax + m.nx * offset;
        py = ay + m.ny * offset;
      } else {
        // On the polygon the contact is a material point; on the circle it is always the
        // rim point facing the polygon, however much the circle has spun since. Letting it
        // rotate with the circle would act as rolling resistance.
        const pAx = A.x + A.c * p.localAx - A.s * p.localAy;
        const pAy = A.y + A.s * p.localAx + A.c * p.localAy;
        p.gap = (bx - pAx) * m.nx + (by - pAy) * m.ny - b.radius + 0.5 * p.separation;
        const reach = b.radius + 0.5 * p.gap;
        px = bx - m.nx * reach;
        py = by - m.ny * reach;
      }
      setLeverArms(p, px - A.x, py - A.y, px - B.x, py - B.y, m.nx, m.ny, mA, mB, iA, iB);
      continue;
    }

    const nx = m.nx;
    const ny = m.ny;
    for (let k = 0; k < m.count; k++) {
      const p = m.points[k];
      const pAx = A.x + A.c * p.localAx - A.s * p.localAy;
      const pAy = A.y + A.s * p.localAx + A.c * p.localAy;
      const pBx = B.x + B.c * p.localBx - B.s * p.localBy;
      const pBy = B.y + B.s * p.localBx + B.c * p.localBy;
      p.gap = (pBx - pAx) * nx + (pBy - pAy) * ny + p.separation;
      const midX = 0.5 * (pAx + pBx);
      const midY = 0.5 * (pAy + pBy);
      setLeverArms(p, midX - A.x, midY - A.y, midX - B.x, midY - B.y, nx, ny, mA, mB, iA, iB);
    }
  }
}

/** Stores the lever arms of a contact point and the effective masses that follow from them. */
function setLeverArms(
  p: ManifoldPoint,
  rAx: number,
  rAy: number,
  rBx: number,
  rBy: number,
  nx: number,
  ny: number,
  mA: number,
  mB: number,
  iA: number,
  iB: number,
): void {
  p.rAx = rAx;
  p.rAy = rAy;
  p.rBx = rBx;
  p.rBy = rBy;
  // How much the contact point's relative velocity changes per unit of impulse along the
  // normal: linear response of both bodies plus what their rotation adds at this lever arm.
  const rnA = rAx * ny - rAy * nx;
  const rnB = rBx * ny - rBy * nx;
  const kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
  p.normalMass = kNormal > 0 ? 1 / kNormal : 0;
  // Same along the tangent, which is the normal rotated by -90 degrees: (ny, -nx).
  const rtA = -rAx * nx - rAy * ny;
  const rtB = -rBx * nx - rBy * ny;
  const kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
  p.tangentMass = kTangent > 0 ? 1 / kTangent : 0;
}

/** Starts the sub-step from last step's answer instead of from zero. */
export function warmStartContacts(contacts: Contact[]): void {
  for (let i = 0; i < contacts.length; i++) {
    const contact = contacts[i];
    const A = contact.bodyA;
    const B = contact.bodyB;
    const m = contact.manifold;
    const nx = m.nx;
    const ny = m.ny;
    // Sum the impulses of the manifold's points first, then touch each body once.
    let px = 0;
    let py = 0;
    let torqueA = 0;
    let torqueB = 0;
    for (let k = 0; k < m.count; k++) {
      const p = m.points[k];
      const ix = nx * p.normalImpulse + ny * p.tangentImpulse;
      const iy = ny * p.normalImpulse - nx * p.tangentImpulse;
      px += ix;
      py += iy;
      torqueA += p.rAx * iy - p.rAy * ix;
      torqueB += p.rBx * iy - p.rBy * ix;
    }
    A.vx -= A.invMass * px;
    A.vy -= A.invMass * py;
    A.w -= A.invInertia * torqueA;
    B.vx += B.invMass * px;
    B.vy += B.invMass * py;
    B.w += B.invInertia * torqueB;
  }
}

/**
 * One Gauss-Seidel sweep. With `useBias` the normal constraint also pushes overlapping shapes
 * apart (softly); the relax pass afterwards runs without it, which removes the velocity that
 * the push introduced.
 */
export function solveContacts(contacts: Contact[], context: StepContext, useBias: boolean): void {
  const invH = context.invH;
  const pushVelocity = context.pushVelocity;
  for (let i = 0; i < contacts.length; i++) {
    const contact = contacts[i];
    const A = contact.bodyA;
    const B = contact.bodyB;
    const m = contact.manifold;
    const nx = m.nx;
    const ny = m.ny;
    const mA = A.invMass;
    const mB = B.invMass;
    const iA = A.invInertia;
    const iB = B.invInertia;
    const friction = contact.friction;
    const soft = contact.softness;
    const count = m.count;

    // This is the hottest loop of the engine. The six velocity components live in locals
    // while the contact is solved and are written back once, instead of being read from and
    // stored to the body objects for every impulse.
    let vAx = A.vx;
    let vAy = A.vy;
    let wA = A.w;
    let vBx = B.vx;
    let vBy = B.vy;
    let wB = B.w;

    for (let k = 0; k < count; k++) {
      const p = m.points[k];
      const separation = p.gap;

      let bias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (separation > 0) {
        // Speculative: not touching yet. Allow exactly enough approach speed to close the gap.
        bias = separation * invH;
      } else if (useBias) {
        // Overlap up to the slop is left alone; only what exceeds it is pushed out.
        bias = Math.max(soft.biasRate * Math.min(separation + LINEAR_SLOP, 0), -pushVelocity);
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }

      const vn = (vBx - wB * p.rBy - vAx + wA * p.rAy) * nx + (vBy + wB * p.rBx - vAy - wA * p.rAx) * ny;
      const old = p.normalImpulse;
      // Contacts can only push, never pull: the accumulated impulse stays non-negative.
      const total = Math.max(old - p.normalMass * massScale * (vn + bias) - impulseScale * old, 0);
      p.normalImpulse = total;
      const impulse = total - old;
      const px = nx * impulse;
      const py = ny * impulse;
      vAx -= mA * px;
      vAy -= mA * py;
      wA -= iA * (p.rAx * py - p.rAy * px);
      vBx += mB * px;
      vBy += mB * py;
      wB += iB * (p.rBx * py - p.rBy * px);
    }

    // Friction second, so its limit (friction coefficient times normal impulse) is based on
    // the normal impulse this very pass just computed, not a stale one.
    for (let k = 0; k < count; k++) {
      const p = m.points[k];
      const vt = (vBx - wB * p.rBy - vAx + wA * p.rAy) * ny - (vBy + wB * p.rBx - vAy - wA * p.rAx) * nx;
      const max = friction * p.normalImpulse;
      const old = p.tangentImpulse;
      const total = Math.max(-max, Math.min(max, old - p.tangentMass * vt));
      p.tangentImpulse = total;
      const impulse = total - old;
      const px = ny * impulse;
      const py = -nx * impulse;
      vAx -= mA * px;
      vAy -= mA * py;
      wA -= iA * (p.rAx * py - p.rAy * px);
      vBx += mB * px;
      vBy += mB * py;
      wB += iB * (p.rBx * py - p.rBy * px);
    }

    A.vx = vAx;
    A.vy = vAy;
    A.w = wA;
    B.vx = vBx;
    B.vy = vBy;
    B.w = wB;
  }
}

/**
 * The impact pass: restitution, handled as a sequence of instantaneous collisions.
 *
 * Bounce is usually bolted onto the contact constraint as a target velocity fixed at the start
 * of the step. That cannot pass an impact down a row of touching balls within one step, so a
 * Newton's cradle turns to mush. Here, before the regular solve of each sub-step, every
 * bouncy contact that is about to be hit is resolved as its own two-body collision, in order,
 * and the sweep repeats so a hit can travel along a chain.
 *
 * Each impulse is a one-dimensional collision with restitution <= 1 (or friction that only
 * slows sliding), so this pass can remove kinetic energy but never add any.
 */
export function solveImpacts(contacts: Contact[], context: StepContext): void {
  for (let pass = 0; pass < MAX_IMPACT_PASSES; pass++) {
    let any = false;
    for (let i = 0; i < contacts.length; i++) {
      if (solveImpact(contacts[i], context)) any = true;
    }
    if (!any) return;
  }
}

function solveImpact(contact: Contact, context: StepContext): boolean {
  const A = contact.bodyA;
  const B = contact.bodyB;
  const m = contact.manifold;
  const nx = m.nx;
  const ny = m.ny;
  const h = context.h;
  const threshold = context.restitutionThreshold;
  const e = contact.restitution;

  // Which points are closing fast enough to bounce, and will actually touch this sub-step?
  let vn0 = 0;
  let vn1 = 0;
  let hit0 = false;
  let hit1 = false;
  for (let k = 0; k < m.count; k++) {
    const p = m.points[k];
    const vn =
      (B.vx - B.w * p.rBy - A.vx + A.w * p.rAy) * nx + (B.vy + B.w * p.rBx - A.vy - A.w * p.rAx) * ny;
    if (vn >= -threshold) continue;
    const separation = p.gap;
    // The slop makes this fire one sub-step early rather than a rounding error too late: once
    // the gap has closed, the regular solver would treat the hit as a plain inelastic contact.
    if (separation + vn * h >= LINEAR_SLOP) continue;
    if (k === 0) {
      hit0 = true;
      vn0 = vn;
    } else {
      hit1 = true;
      vn1 = vn;
    }
  }
  if (!hit0 && !hit1) return false;

  const p0 = m.points[0];
  const p1 = m.points[1];
  let j0 = 0;
  let j1 = 0;

  if (hit0 && hit1) {
    // Both corners land together: solve the two normal impulses as one 2x2 problem, so a box
    // dropped flat bounces flat instead of being spun by whichever corner was visited first.
    const mA = A.invMass;
    const mB = B.invMass;
    const iA = A.invInertia;
    const iB = B.invInertia;
    const rn0A = p0.rAx * ny - p0.rAy * nx;
    const rn0B = p0.rBx * ny - p0.rBy * nx;
    const rn1A = p1.rAx * ny - p1.rAy * nx;
    const rn1B = p1.rBx * ny - p1.rBy * nx;
    const k00 = mA + mB + iA * rn0A * rn0A + iB * rn0B * rn0B;
    const k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
    const k01 = mA + mB + iA * rn0A * rn1A + iB * rn0B * rn1B;
    // Wanted change in normal velocity at each point: from vn to -e * vn.
    const b0 = -(1 + e) * vn0;
    const b1 = -(1 + e) * vn1;
    const det = k00 * k11 - k01 * k01;
    let solved = false;
    if (k00 * k00 < 1000 * det) {
      const x0 = (k11 * b0 - k01 * b1) / det;
      const x1 = (k00 * b1 - k01 * b0) / det;
      if (x0 >= 0 && x1 >= 0) {
        j0 = x0;
        j1 = x1;
        solved = true;
      }
    }
    if (!solved) {
      // One point alone already makes the other separate fast enough (or the system is
      // degenerate): fall back to the single-point collision at the faster point.
      if (vn0 <= vn1) j0 = b0 / k00;
      else j1 = b1 / k11;
    }
  } else if (hit0) {
    j0 = -(1 + e) * vn0 * p0.normalMass;
  } else {
    j1 = -(1 + e) * vn1 * p1.normalMass;
  }

  for (let k = 0; k < m.count; k++) {
    const p = m.points[k];
    const j = k === 0 ? j0 : j1;
    if (j <= 0) continue;
    const px = nx * j;
    const py = ny * j;
    A.vx -= A.invMass * px;
    A.vy -= A.invMass * py;
    A.w -= A.invInertia * (p.rAx * py - p.rAy * px);
    B.vx += B.invMass * px;
    B.vy += B.invMass * py;
    B.w += B.invInertia * (p.rBx * py - p.rBy * px);
  }

  // Friction during the impact, limited by the normal impulse just applied (Coulomb).
  const friction = contact.friction;
  if (friction > 0) {
    for (let k = 0; k < m.count; k++) {
      const p = m.points[k];
      const j = k === 0 ? j0 : j1;
      if (j <= 0) continue;
      const vt =
        (B.vx - B.w * p.rBy - A.vx + A.w * p.rAy) * ny - (B.vy + B.w * p.rBx - A.vy - A.w * p.rAx) * nx;
      const max = friction * j;
      const impulse = Math.max(-max, Math.min(max, -p.tangentMass * vt));
      const px = ny * impulse;
      const py = -nx * impulse;
      A.vx -= A.invMass * px;
      A.vy -= A.invMass * py;
      A.w -= A.invInertia * (p.rAx * py - p.rAy * px);
      B.vx += B.invMass * px;
      B.vy += B.invMass * py;
      B.w += B.invInertia * (p.rBx * py - p.rBy * px);
    }
  }
  return true;
}
