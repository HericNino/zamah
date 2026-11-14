import { LINEAR_SLOP } from '../settings.ts';
import type { Shape } from './shape.ts';

/**
 * One point of a contact manifold, plus the solver state that belongs to it.
 * Keeping both in one object lets accumulated impulses survive from step to step.
 */
export class ManifoldPoint {
  /** World position, halfway between the two surfaces. */
  x = 0;
  y = 0;
  /** Distance between the surfaces along the normal; negative when overlapping. */
  separation = 0;
  /** Identifies which features (edges, vertices) made this point, to match it across steps. */
  id = 0;

  /** Accumulated impulses, reused next step as the solver's starting guess (warm starting). */
  normalImpulse = 0;
  tangentImpulse = 0;

  // --- solver scratch, filled in by prepareContacts and refreshContacts ---
  /** Current distance between the surfaces, re-measured after every sub-step. */
  gap = 0;
  /** The point where impulses are applied, relative to each body's centre, in world axes. */
  rAx = 0;
  rAy = 0;
  rBx = 0;
  rBy = 0;
  /** The same anchors in each body's local frame, to re-measure separation after sub-steps. */
  localAx = 0;
  localAy = 0;
  localBx = 0;
  localBy = 0;
  normalMass = 0;
  tangentMass = 0;
}

/** Up to two contact points sharing one normal, which points from shape A to shape B. */
export class Manifold {
  nx = 0;
  ny = 0;
  count = 0;
  readonly points: [ManifoldPoint, ManifoldPoint] = [new ManifoldPoint(), new ManifoldPoint()];
}

// ---- circle vs circle ---------------------------------------------------------------------------

export function collideCircles(m: Manifold, a: Shape, b: Shape, margin: number): void {
  m.count = 0;
  const dx = b.worldX - a.worldX;
  const dy = b.worldY - a.worldY;
  const dist = Math.sqrt(dx * dx + dy * dy);
  const separation = dist - a.radius - b.radius;
  if (separation > margin) return;
  if (dist > 1e-12) {
    m.nx = dx / dist;
    m.ny = dy / dist;
  } else {
    m.nx = 0;
    m.ny = 1;
  }
  const p = m.points[0];
  const offset = a.radius + 0.5 * separation;
  p.x = a.worldX + m.nx * offset;
  p.y = a.worldY + m.ny * offset;
  p.separation = separation;
  p.id = 0;
  m.count = 1;
}

// ---- polygon vs circle --------------------------------------------------------------------------

export function collidePolygonCircle(m: Manifold, poly: Shape, circle: Shape, margin: number): void {
  m.count = 0;
  const v = poly.worldVertices;
  const n = poly.worldNormals;
  const cx = circle.worldX;
  const cy = circle.worldY;
  const radius = circle.radius;

  // The face the circle centre is furthest outside of.
  let best = -Infinity;
  let edge = 0;
  for (let i = 0; i < poly.count; i++) {
    const sep = n[2 * i] * (cx - v[2 * i]) + n[2 * i + 1] * (cy - v[2 * i + 1]);
    if (sep > best) {
      best = sep;
      edge = i;
    }
  }
  if (best - radius > margin) return;

  const next = (edge + 1) % poly.count;
  const ax = v[2 * edge];
  const ay = v[2 * edge + 1];
  const bx = v[2 * next];
  const by = v[2 * next + 1];

  // Closest point on the polygon: a corner of that face, or the centre's projection onto it.
  let px: number;
  let py: number;
  let dist = best;
  let nx = n[2 * edge];
  let ny = n[2 * edge + 1];
  let corner = -1;
  if (best > 1e-9) {
    if ((cx - ax) * (bx - ax) + (cy - ay) * (by - ay) < 0) corner = edge;
    else if ((cx - bx) * (ax - bx) + (cy - by) * (ay - by) < 0) corner = next;
  }
  if (corner >= 0) {
    px = v[2 * corner];
    py = v[2 * corner + 1];
    const dx = cx - px;
    const dy = cy - py;
    dist = Math.sqrt(dx * dx + dy * dy);
    if (dist - radius > margin) return;
    nx = dx / dist;
    ny = dy / dist;
  } else {
    px = cx - nx * best;
    py = cy - ny * best;
  }

  const separation = dist - radius;
  m.nx = nx;
  m.ny = ny;
  const p = m.points[0];
  p.x = px + nx * 0.5 * separation;
  p.y = py + ny * 0.5 * separation;
  p.separation = separation;
  p.id = corner >= 0 ? 1 + corner : 64 + edge;
  m.count = 1;
}

// ---- polygon vs polygon -------------------------------------------------------------------------

let foundEdge = 0;

/**
 * Separating axis test over the face normals of `a`: for each face, how far is the deepest
 * vertex of `b` in front of it? Returns the largest such separation and stores its face in
 * `foundEdge`. A positive result means that face's normal is a separating axis.
 */
function maxSeparation(a: Shape, b: Shape): number {
  const av = a.worldVertices;
  const an = a.worldNormals;
  const bv = b.worldVertices;
  let best = -Infinity;
  for (let i = 0; i < a.count; i++) {
    const nx = an[2 * i];
    const ny = an[2 * i + 1];
    const vx = av[2 * i];
    const vy = av[2 * i + 1];
    let deepest = Infinity;
    for (let j = 0; j < b.count; j++) {
      const sep = nx * (bv[2 * j] - vx) + ny * (bv[2 * j + 1] - vy);
      if (sep < deepest) deepest = sep;
    }
    if (deepest > best) {
      best = deepest;
      foundEdge = i;
    }
  }
  return best;
}

/**
 * SAT plus reference-face clipping.
 *
 * 1. Find the axis of least penetration on each polygon. If either is separated by more than
 *    the margin, there is no contact.
 * 2. The face with the smaller penetration becomes the reference face. On the other polygon,
 *    the face most opposed to it is the incident face.
 * 3. Clip the incident face to the side planes of the reference face. What survives, and lies
 *    within the margin of the reference face, are the (up to two) contact points.
 */
export function collidePolygons(m: Manifold, a: Shape, b: Shape, margin: number): void {
  m.count = 0;
  const sepA = maxSeparation(a, b);
  const edgeA = foundEdge;
  if (sepA > margin) return;
  const sepB = maxSeparation(b, a);
  const edgeB = foundEdge;
  if (sepB > margin) return;

  // Prefer A unless B is clearly better, so the choice does not flip-flop between steps.
  const flip = sepB > sepA + 0.1 * LINEAR_SLOP;
  const ref = flip ? b : a;
  const inc = flip ? a : b;
  const refEdge = flip ? edgeB : edgeA;

  const rv = ref.worldVertices;
  const rn = ref.worldNormals;
  const iv = inc.worldVertices;
  const inn = inc.worldNormals;

  const nx = rn[2 * refEdge];
  const ny = rn[2 * refEdge + 1];
  const refNext = (refEdge + 1) % ref.count;
  const v1x = rv[2 * refEdge];
  const v1y = rv[2 * refEdge + 1];
  const v2x = rv[2 * refNext];
  const v2y = rv[2 * refNext + 1];

  // Incident face: the one whose normal points most against the reference normal.
  let incEdge = 0;
  let minDot = Infinity;
  for (let j = 0; j < inc.count; j++) {
    const d = nx * inn[2 * j] + ny * inn[2 * j + 1];
    if (d < minDot) {
      minDot = d;
      incEdge = j;
    }
  }
  const incNext = (incEdge + 1) % inc.count;
  let p1x = iv[2 * incEdge];
  let p1y = iv[2 * incEdge + 1];
  let p2x = iv[2 * incNext];
  let p2y = iv[2 * incNext + 1];

  // Clip against the two side planes of the reference face, in its tangent coordinate.
  const tx = -ny;
  const ty = nx;
  const upper = tx * (v2x - v1x) + ty * (v2y - v1y);
  let d1 = tx * (p1x - v1x) + ty * (p1y - v1y);
  let d2 = tx * (p2x - v1x) + ty * (p2y - v1y);
  if ((d1 < 0 && d2 < 0) || (d1 > upper && d2 > upper)) return;
  if (d1 < 0 || d2 < 0) {
    const t = d1 / (d1 - d2);
    const qx = p1x + t * (p2x - p1x);
    const qy = p1y + t * (p2y - p1y);
    if (d1 < 0) {
      p1x = qx;
      p1y = qy;
      d1 = 0;
    } else {
      p2x = qx;
      p2y = qy;
      d2 = 0;
    }
  }
  if (d1 > upper || d2 > upper) {
    const t = (upper - d1) / (d2 - d1);
    const qx = p1x + t * (p2x - p1x);
    const qy = p1y + t * (p2y - p1y);
    if (d1 > upper) {
      p1x = qx;
      p1y = qy;
    } else {
      p2x = qx;
      p2y = qy;
    }
  }

  const s1 = nx * (p1x - v1x) + ny * (p1y - v1y);
  const s2 = nx * (p2x - v1x) + ny * (p2y - v1y);
  const baseId = (flip ? 1 : 0) | (refEdge << 1) | (incEdge << 5);

  let count = 0;
  if (s1 <= margin) {
    const p = m.points[count++];
    p.x = p1x - 0.5 * s1 * nx;
    p.y = p1y - 0.5 * s1 * ny;
    p.separation = s1;
    p.id = baseId;
  }
  if (s2 <= margin) {
    const p = m.points[count++];
    p.x = p2x - 0.5 * s2 * nx;
    p.y = p2y - 0.5 * s2 * ny;
    p.separation = s2;
    p.id = baseId | (1 << 9);
  }
  m.count = count;
  // The manifold normal always points from A to B.
  m.nx = flip ? -nx : nx;
  m.ny = flip ? -ny : ny;
}
