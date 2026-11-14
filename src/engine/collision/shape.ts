import type { Body } from '../dynamics/body.ts';
import { cos, sin, TWO_PI } from '../math/trig.ts';
import type { Vec2 } from '../math/vec2.ts';
import { MAX_POLYGON_VERTICES } from '../settings.ts';

/** Surface and mass properties shared by every shape kind. */
export interface Material {
  /** Mass per unit area, kg/m^2. Zero makes the shape weightless. */
  density?: number;
  friction?: number;
  /** 0 = no bounce, 1 = perfectly elastic. */
  restitution?: number;
  /** Collision filtering: two shapes collide when each one's mask contains the other's category. */
  category?: number;
  mask?: number;
}

export interface CircleDef extends Material {
  kind: 'circle';
  radius: number;
  center?: Vec2;
}

export interface PolygonDef extends Material {
  kind: 'polygon';
  vertices: Vec2[];
}

export type ShapeDef = CircleDef | PolygonDef;

export function circle(radius: number, options: Material & { center?: Vec2 } = {}): CircleDef {
  return { kind: 'circle', radius, ...options };
}

/** An axis-aligned or rotated box given by its half extents, in body-local coordinates. */
export function box(
  halfWidth: number,
  halfHeight: number,
  options: Material & { center?: Vec2; angle?: number } = {},
): PolygonDef {
  const { center, angle, ...material } = options;
  const cx = center?.x ?? 0;
  const cy = center?.y ?? 0;
  const c = cos(angle ?? 0);
  const s = sin(angle ?? 0);
  const corners: [number, number][] = [
    [-halfWidth, -halfHeight],
    [halfWidth, -halfHeight],
    [halfWidth, halfHeight],
    [-halfWidth, halfHeight],
  ];
  return {
    kind: 'polygon',
    vertices: corners.map(([x, y]) => ({ x: cx + c * x - s * y, y: cy + s * x + c * y })),
    ...material,
  };
}

/** A convex polygon. The points may come in any order; their convex hull is used. */
export function polygon(vertices: Vec2[], options: Material = {}): PolygonDef {
  return { kind: 'polygon', vertices, ...options };
}

export function regularPolygon(sides: number, radius: number, options: Material = {}): PolygonDef {
  const vertices: Vec2[] = [];
  for (let i = 0; i < sides; i++) {
    const angle = (TWO_PI * i) / sides;
    vertices.push({ x: radius * cos(angle), y: radius * sin(angle) });
  }
  return { kind: 'polygon', vertices, ...options };
}

/**
 * Convex hull by Andrew's monotone chain, counter-clockwise, without collinear points.
 * Sorting makes the result independent of the input order.
 */
export function convexHull(points: Vec2[]): Vec2[] {
  const sorted = points.map((p) => ({ x: p.x, y: p.y })).sort((a, b) => a.x - b.x || a.y - b.y);
  if (sorted.length < 3) return sorted;
  const turn = (o: Vec2, a: Vec2, b: Vec2) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const hull: Vec2[] = [];
  for (const p of sorted) {
    while (hull.length >= 2 && turn(hull[hull.length - 2], hull[hull.length - 1], p) <= 1e-12) hull.pop();
    hull.push(p);
  }
  const lowerSize = hull.length + 1;
  for (let i = sorted.length - 2; i >= 0; i--) {
    const p = sorted[i];
    while (hull.length >= lowerSize && turn(hull[hull.length - 2], hull[hull.length - 1], p) <= 1e-12) {
      hull.pop();
    }
    hull.push(p);
  }
  hull.pop();
  return hull;
}

export interface MassData {
  mass: number;
  /** Centre of mass in body-local coordinates. */
  centerX: number;
  centerY: number;
  /** Rotational inertia about the body-local origin. */
  inertia: number;
}

/** Result of a point-versus-shape distance query. */
export interface SurfaceSample {
  /** Signed distance to the surface; negative inside the shape. */
  distance: number;
  /** Outward surface normal at the closest point. */
  nx: number;
  ny: number;
}

let nextShapeId = 1;

/**
 * A convex collision shape attached to a body: a circle or a convex polygon.
 *
 * Geometry is stored twice. The local copy is fixed in the body's frame. The world copy is a
 * cache refreshed once per step, so the narrowphase can work in world space without
 * transforming vertices for every pair it tests.
 */
export class Shape {
  readonly id: number;
  readonly kind: 'circle' | 'polygon';
  body!: Body;

  density: number;
  friction: number;
  restitution: number;
  category: number;
  mask: number;

  /** Circle radius; zero for polygons. */
  readonly radius: number;
  /** Circle centre, or polygon centroid, in body-local coordinates. */
  localX = 0;
  localY = 0;

  /** Polygon vertex count; zero for circles. */
  readonly count: number;
  /** Interleaved x,y pairs, counter-clockwise. */
  readonly localVertices: Float64Array;
  readonly localNormals: Float64Array;

  worldX = 0;
  worldY = 0;
  readonly worldVertices: Float64Array;
  readonly worldNormals: Float64Array;

  /** Tight world-space bounding box. */
  minX = 0;
  minY = 0;
  maxX = 0;
  maxY = 0;

  /** Largest distance from the body's centre of mass to any point of the shape. */
  boundRadius = 0;

  /** Handle into the broadphase tree, or -1 while detached. */
  proxy = -1;

  constructor(def: ShapeDef) {
    this.id = nextShapeId++;
    this.kind = def.kind;
    this.density = def.density ?? 1;
    this.friction = def.friction ?? 0.6;
    this.restitution = def.restitution ?? 0;
    this.category = def.category ?? 1;
    this.mask = def.mask ?? 0xffff;

    if (def.kind === 'circle') {
      if (!(def.radius > 0)) throw new Error('A circle needs a positive radius.');
      this.radius = def.radius;
      this.localX = def.center?.x ?? 0;
      this.localY = def.center?.y ?? 0;
      this.count = 0;
      this.localVertices = this.localNormals = this.worldVertices = this.worldNormals = new Float64Array(0);
    } else {
      const hull = convexHull(def.vertices);
      if (hull.length < 3) throw new Error('A polygon needs at least three non-collinear points.');
      if (hull.length > MAX_POLYGON_VERTICES) {
        throw new Error(`A polygon may have at most ${MAX_POLYGON_VERTICES} vertices.`);
      }
      this.radius = 0;
      this.count = hull.length;
      this.localVertices = new Float64Array(2 * this.count);
      this.localNormals = new Float64Array(2 * this.count);
      this.worldVertices = new Float64Array(2 * this.count);
      this.worldNormals = new Float64Array(2 * this.count);
      for (let i = 0; i < this.count; i++) {
        this.localVertices[2 * i] = hull[i].x;
        this.localVertices[2 * i + 1] = hull[i].y;
      }
      for (let i = 0; i < this.count; i++) {
        const j = (i + 1) % this.count;
        const ex = this.localVertices[2 * j] - this.localVertices[2 * i];
        const ey = this.localVertices[2 * j + 1] - this.localVertices[2 * i + 1];
        const len = Math.sqrt(ex * ex + ey * ey);
        // Outward normal of a counter-clockwise edge is the edge rotated by -90 degrees.
        this.localNormals[2 * i] = ey / len;
        this.localNormals[2 * i + 1] = -ex / len;
      }
      const mass = this.computeMass();
      this.localX = mass.centerX;
      this.localY = mass.centerY;
    }
  }

  /** Mass, centre of mass and inertia about the local origin, from the shape's density. */
  computeMass(): MassData {
    if (this.kind === 'circle') {
      const mass = this.density * Math.PI * this.radius * this.radius;
      const offset2 = this.localX * this.localX + this.localY * this.localY;
      return {
        mass,
        centerX: this.localX,
        centerY: this.localY,
        inertia: mass * (0.5 * this.radius * this.radius + offset2),
      };
    }
    // Sum signed triangles (origin, v[i], v[i+1]).
    const v = this.localVertices;
    let area = 0;
    let cx = 0;
    let cy = 0;
    let inertia = 0;
    for (let i = 0; i < this.count; i++) {
      const j = (i + 1) % this.count;
      const ax = v[2 * i];
      const ay = v[2 * i + 1];
      const bx = v[2 * j];
      const by = v[2 * j + 1];
      const d = ax * by - ay * bx;
      area += 0.5 * d;
      cx += (d * (ax + bx)) / 6;
      cy += (d * (ay + by)) / 6;
      inertia += (d * (ax * ax + ax * bx + bx * bx + ay * ay + ay * by + by * by)) / 12;
    }
    return {
      mass: this.density * area,
      centerX: cx / area,
      centerY: cy / area,
      inertia: this.density * inertia,
    };
  }

  /** Shifts the local geometry; used once, to move the body origin onto the centre of mass. */
  translateLocal(dx: number, dy: number): void {
    this.localX += dx;
    this.localY += dy;
    for (let i = 0; i < this.count; i++) {
      this.localVertices[2 * i] += dx;
      this.localVertices[2 * i + 1] += dy;
    }
  }

  computeBoundRadius(): void {
    if (this.kind === 'circle') {
      this.boundRadius = Math.sqrt(this.localX * this.localX + this.localY * this.localY) + this.radius;
      return;
    }
    let max2 = 0;
    for (let i = 0; i < this.count; i++) {
      const x = this.localVertices[2 * i];
      const y = this.localVertices[2 * i + 1];
      max2 = Math.max(max2, x * x + y * y);
    }
    this.boundRadius = Math.sqrt(max2);
  }

  /** Refreshes the world-space cache and bounding box from the body's transform. */
  sync(): void {
    const body = this.body;
    const c = body.c;
    const s = body.s;
    this.worldX = body.x + c * this.localX - s * this.localY;
    this.worldY = body.y + s * this.localX + c * this.localY;
    if (this.kind === 'circle') {
      this.minX = this.worldX - this.radius;
      this.minY = this.worldY - this.radius;
      this.maxX = this.worldX + this.radius;
      this.maxY = this.worldY + this.radius;
      return;
    }
    const lv = this.localVertices;
    const ln = this.localNormals;
    const wv = this.worldVertices;
    const wn = this.worldNormals;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (let i = 0; i < this.count; i++) {
      const lx = lv[2 * i];
      const ly = lv[2 * i + 1];
      const x = body.x + c * lx - s * ly;
      const y = body.y + s * lx + c * ly;
      wv[2 * i] = x;
      wv[2 * i + 1] = y;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      const nx = ln[2 * i];
      const ny = ln[2 * i + 1];
      wn[2 * i] = c * nx - s * ny;
      wn[2 * i + 1] = s * nx + c * ny;
    }
    this.minX = minX;
    this.minY = minY;
    this.maxX = maxX;
    this.maxY = maxY;
  }

  containsPoint(x: number, y: number): boolean {
    if (this.kind === 'circle') {
      const dx = x - this.worldX;
      const dy = y - this.worldY;
      return dx * dx + dy * dy <= this.radius * this.radius;
    }
    const v = this.worldVertices;
    const n = this.worldNormals;
    for (let i = 0; i < this.count; i++) {
      if (n[2 * i] * (x - v[2 * i]) + n[2 * i + 1] * (y - v[2 * i + 1]) > 0) return false;
    }
    return true;
  }

  /** Signed distance from a world point to the surface, with the outward normal there. */
  sample(x: number, y: number, out: SurfaceSample): void {
    if (this.kind === 'circle') {
      const dx = x - this.worldX;
      const dy = y - this.worldY;
      const dist = Math.sqrt(dx * dx + dy * dy);
      out.distance = dist - this.radius;
      if (dist > 1e-12) {
        out.nx = dx / dist;
        out.ny = dy / dist;
      } else {
        out.nx = 0;
        out.ny = 1;
      }
      return;
    }
    const v = this.worldVertices;
    const n = this.worldNormals;
    let best = -Infinity;
    let edge = 0;
    for (let i = 0; i < this.count; i++) {
      const sep = n[2 * i] * (x - v[2 * i]) + n[2 * i + 1] * (y - v[2 * i + 1]);
      if (sep > best) {
        best = sep;
        edge = i;
      }
    }
    out.distance = best;
    out.nx = n[2 * edge];
    out.ny = n[2 * edge + 1];
    if (best <= 0) return;
    // Outside: the closest feature is this edge or one of its end points.
    const next = (edge + 1) % this.count;
    const ax = v[2 * edge];
    const ay = v[2 * edge + 1];
    const bx = v[2 * next];
    const by = v[2 * next + 1];
    let cornerX = 0;
    let cornerY = 0;
    if ((x - ax) * (bx - ax) + (y - ay) * (by - ay) < 0) {
      cornerX = ax;
      cornerY = ay;
    } else if ((x - bx) * (ax - bx) + (y - by) * (ay - by) < 0) {
      cornerX = bx;
      cornerY = by;
    } else {
      return;
    }
    const dx = x - cornerX;
    const dy = y - cornerY;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > 1e-12) {
      out.distance = dist;
      out.nx = dx / dist;
      out.ny = dy / dist;
    }
  }
}
