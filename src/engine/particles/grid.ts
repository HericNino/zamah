import type { DynamicTree } from '../collision/dynamicTree.ts';
import type { Shape } from '../collision/shape.ts';
import type { World } from '../world.ts';

/**
 * Finds all pairs of particles closer than a given radius.
 *
 * Particles are binned into a uniform grid whose cells are as large as the search radius, so
 * the neighbours of a particle can only be in its own cell or the eight around it. The grid is
 * unbounded: cell coordinates are hashed into a fixed-size table, and the table is filled with
 * a counting sort, which needs no per-cell allocations and visits memory in order.
 *
 * The result is a compressed adjacency list: the neighbours of particle i are
 * `neighbors[start[i] .. start[i + 1]]`.
 */
export class NeighborGrid {
  start = new Int32Array(1);
  neighbors = new Int32Array(1024);

  private cellStart = new Int32Array(1);
  private order = new Int32Array(0);
  private cellOf = new Int32Array(0);
  private sortedX = new Float64Array(0);
  private sortedY = new Float64Array(0);
  private rangeFrom = new Int32Array(9);
  private rangeTo = new Int32Array(9);
  private mask = 0;

  /**
   * Sorts the particles into buckets. Afterwards `order[k]` is the index of the k-th particle
   * in bucket order, which is also (roughly) spatial order.
   */
  sort(x: Float64Array, y: Float64Array, count: number, radius: number): Int32Array {
    // Table size: a power of two at least twice the particle count keeps collisions rare.
    let tableSize = 64;
    while (tableSize < 2 * count) tableSize *= 2;
    if (this.cellStart.length !== tableSize + 1) this.cellStart = new Int32Array(tableSize + 1);
    else this.cellStart.fill(0);
    if (this.order.length < count) {
      this.order = new Int32Array(count * 2);
      this.cellOf = new Int32Array(count * 2);
      this.sortedX = new Float64Array(count * 2);
      this.sortedY = new Float64Array(count * 2);
    }
    const mask = (this.mask = tableSize - 1);
    const cellStart = this.cellStart;
    const order = this.order;
    const cellOf = this.cellOf;
    const inv = 1 / radius;

    // Counting sort of particles by hashed cell.
    for (let i = 0; i < count; i++) {
      const cell = hashCell(Math.floor(x[i] * inv), Math.floor(y[i] * inv)) & mask;
      cellOf[i] = cell;
      cellStart[cell + 1]++;
    }
    for (let c = 0; c < tableSize; c++) cellStart[c + 1] += cellStart[c];
    // Fill backwards so each bucket ends up in ascending particle order.
    for (let i = count - 1; i >= 0; i--) {
      // Temporarily use the bucket's end as a write cursor moving down.
      order[--cellStart[cellOf[i] + 1]] = i;
    }
    // The cursors now sit at each bucket's start, i.e. cellStart[c + 1] == start of bucket c.
    // Shift back so cellStart[c] is the start of bucket c again.
    for (let c = 0; c < tableSize; c++) cellStart[c] = cellStart[c + 1];
    cellStart[tableSize] = count;
    return order;
  }

  build(x: Float64Array, y: Float64Array, count: number, radius: number): void {
    const order = this.sort(x, y, count, radius);
    if (this.start.length < count + 1) this.start = new Int32Array(2 * count + 1);
    const mask = this.mask;
    const cellStart = this.cellStart;
    const inv = 1 / radius;

    // Positions copied into bucket order, so scanning a bucket reads memory front to back
    // instead of hopping around the particle arrays.
    const sortedX = this.sortedX;
    const sortedY = this.sortedY;
    for (let k = 0; k < count; k++) {
      sortedX[k] = x[order[k]];
      sortedY[k] = y[order[k]];
    }

    const radius2 = radius * radius;
    const start = this.start;
    let neighbors = this.neighbors;
    let total = 0;
    // Start and end of each distinct bucket around the current cell.
    const from = this.rangeFrom;
    const to = this.rangeTo;
    let ranges = 0;
    let lastCx = NaN;
    let lastCy = NaN;
    for (let i = 0; i < count; i++) {
      start[i] = total;
      const xi = x[i];
      const yi = y[i];
      const cx = Math.floor(xi * inv);
      const cy = Math.floor(yi * inv);
      // Neighbouring particles are usually in the same cell (the liquid keeps its arrays
      // spatially sorted), and then the nine buckets to scan are the same as last time.
      if (cx !== lastCx || cy !== lastCy) {
        lastCx = cx;
        lastCy = cy;
        ranges = 0;
        for (let oy = -1; oy <= 1; oy++) {
          for (let ox = -1; ox <= 1; ox++) {
            const bucket = hashCell(cx + ox, cy + oy) & mask;
            const begin = cellStart[bucket];
            const end = cellStart[bucket + 1];
            if (begin === end) continue;
            // Two of the nine cells may hash to the same bucket; scanning it twice would
            // report the same neighbour twice.
            let seen = false;
            for (let r = 0; r < ranges; r++) {
              if (from[r] === begin) {
                seen = true;
                break;
              }
            }
            if (seen) continue;
            from[ranges] = begin;
            to[ranges++] = end;
          }
        }
      }
      if (total + 256 > neighbors.length) {
        const grown = new Int32Array(neighbors.length * 2);
        grown.set(neighbors);
        this.neighbors = neighbors = grown;
      }
      for (let r = 0; r < ranges; r++) {
        const end = to[r];
        for (let k = from[r]; k < end; k++) {
          const dx = sortedX[k] - xi;
          const dy = sortedY[k] - yi;
          if (dx * dx + dy * dy >= radius2) continue;
          const j = order[k];
          if (j === i) continue;
          if (total === neighbors.length) {
            const grown = new Int32Array(neighbors.length * 2);
            grown.set(neighbors);
            this.neighbors = neighbors = grown;
          }
          neighbors[total++] = j;
        }
      }
    }
    start[count] = total;
  }
}

function hashCell(ix: number, iy: number): number {
  return Math.imul(ix, 73856093) ^ Math.imul(iy, 19349663);
}

/**
 * For every particle, the rigid shapes it could touch during the coming step. Gathered once
 * per step from the broadphase tree; the exact tests then run against this short list in
 * every sub-step.
 */
export class ShapeCandidates {
  start = new Int32Array(1);
  shapes: Shape[] = [];

  private tree: DynamicTree<Shape> | null = null;
  // One persistent callback, so gathering does not allocate a closure per particle.
  private collect = (id: number): void => {
    this.shapes.push(this.tree!.getData(id));
  };

  gather(
    world: World,
    x: Float64Array,
    y: Float64Array,
    vx: Float64Array,
    vy: Float64Array,
    count: number,
    radius: number,
  ): void {
    if (this.start.length < count + 1) this.start = new Int32Array(2 * count + 1);
    const start = this.start;
    const shapes = this.shapes;
    shapes.length = 0;
    if (world.bodies.length === 0) {
      start.fill(0, 0, count + 1);
      return;
    }
    const dt = world.dt;
    const tree = (this.tree = world.broadphase.tree);
    for (let i = 0; i < count; i++) {
      start[i] = shapes.length;
      // Reach: the particle's size plus how far it can travel in one step, with some slack.
      const reachX = 2 * radius + Math.abs(vx[i]) * dt;
      const reachY = 2 * radius + Math.abs(vy[i]) * dt + 5 * dt * dt;
      tree.query(x[i] - reachX, y[i] - reachY, x[i] + reachX, y[i] + reachY, this.collect);
    }
    start[count] = shapes.length;
  }
}
