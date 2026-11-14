import { AABB_MARGIN } from '../settings.ts';
import { DynamicTree } from './dynamicTree.ts';
import type { Shape } from './shape.ts';

/**
 * The broadphase answers one question cheaply: which pairs of shapes *might* touch?
 *
 * Each shape gets a "fat" box in the tree: its tight bounding box inflated by a margin and
 * stretched along its direction of travel. A shape only has to be re-inserted when it leaves
 * its fat box, so resting or slow bodies cost nothing here. Only shapes that were inserted or
 * re-inserted since the last step query the tree for new neighbours.
 */
export class BroadPhase {
  readonly tree = new DynamicTree<Shape>();

  /** Proxies inserted or moved since the last updatePairs, in the order it happened. */
  private moved: number[] = [];
  private movedFlag = new Uint8Array(64);

  createProxy(shape: Shape): void {
    shape.proxy = this.tree.createProxy(
      shape.minX - AABB_MARGIN,
      shape.minY - AABB_MARGIN,
      shape.maxX + AABB_MARGIN,
      shape.maxY + AABB_MARGIN,
      shape,
    );
    this.markMoved(shape.proxy);
  }

  destroyProxy(shape: Shape): void {
    if (shape.proxy < 0) return;
    this.movedFlag[shape.proxy] = 0;
    this.tree.destroyProxy(shape.proxy);
    shape.proxy = -1;
  }

  /**
   * Call before and after a shape moves. `dx, dy` is the displacement expected over the next
   * step. The shape is re-inserted when the box it will sweep through no longer fits inside
   * its fat box, so a body that was just given a large velocity is paired with what lies
   * ahead of it before it gets there, not after it has passed through.
   */
  moveProxy(shape: Shape, dx: number, dy: number): void {
    const id = shape.proxy;
    const sweptMinX = dx < 0 ? shape.minX + dx : shape.minX;
    const sweptMinY = dy < 0 ? shape.minY + dy : shape.minY;
    const sweptMaxX = dx > 0 ? shape.maxX + dx : shape.maxX;
    const sweptMaxY = dy > 0 ? shape.maxY + dy : shape.maxY;
    if (this.tree.contains(id, sweptMinX, sweptMinY, sweptMaxX, sweptMaxY)) return;
    // Stretch by twice the displacement, so steady motion re-inserts only every other step.
    this.tree.moveProxy(
      id,
      sweptMinX - AABB_MARGIN + (dx < 0 ? dx : 0),
      sweptMinY - AABB_MARGIN + (dy < 0 ? dy : 0),
      sweptMaxX + AABB_MARGIN + (dx > 0 ? dx : 0),
      sweptMaxY + AABB_MARGIN + (dy > 0 ? dy : 0),
    );
    this.markMoved(id);
  }

  /** Forces a shape to look for new neighbours on the next update, e.g. after a filter change. */
  touch(shape: Shape): void {
    if (shape.proxy >= 0) this.markMoved(shape.proxy);
  }

  /** Reports every new candidate pair exactly once. Pairs already known are reported again; the caller dedupes. */
  updatePairs(addPair: (a: Shape, b: Shape) => void): void {
    const tree = this.tree;
    const moved = this.moved;
    const flag = this.movedFlag;
    for (let i = 0; i < moved.length; i++) {
      const id = moved[i];
      if (!flag[id]) continue; // destroyed after it moved
      tree.query(tree.minX(id), tree.minY(id), tree.maxX(id), tree.maxY(id), (other) => {
        if (other === id) return;
        // When both proxies moved, the pair would be seen from both sides; keep one.
        if (flag[other] && other < id) return;
        addPair(tree.getData(id), tree.getData(other));
      });
    }
    for (let i = 0; i < moved.length; i++) flag[moved[i]] = 0;
    moved.length = 0;
  }

  /** True while the fat boxes of two shapes overlap. */
  overlaps(a: Shape, b: Shape): boolean {
    return this.tree.overlaps(a.proxy, b.proxy);
  }

  private markMoved(id: number): void {
    if (id >= this.movedFlag.length) {
      const grown = new Uint8Array(Math.max(id + 1, this.movedFlag.length * 2));
      grown.set(this.movedFlag);
      this.movedFlag = grown;
    }
    if (this.movedFlag[id]) return;
    this.movedFlag[id] = 1;
    this.moved.push(id);
  }
}
