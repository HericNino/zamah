/**
 * A dynamic bounding volume hierarchy: a balanced binary tree of axis-aligned boxes.
 *
 * Leaves hold the (inflated) boxes of individual proxies; every internal node holds the union
 * of its two children. Queries descend only into nodes whose box overlaps the query, so finding
 * the neighbours of one proxy costs O(log n) instead of O(n).
 *
 * Insertion picks the sibling that grows the tree's total perimeter the least (a surface area
 * heuristic), and AVL-style rotations on the way back up keep the height logarithmic.
 * Nodes live in typed arrays and are recycled through a free list, so the tree itself never
 * allocates while the simulation runs.
 */

const NULL = -1;

export class DynamicTree<T> {
  root = NULL;

  private capacity = 64;
  private nodeCount = 0;
  private freeList = 0;

  /** Four numbers per node: minX, minY, maxX, maxY. */
  private boxes = new Float64Array(this.capacity * 4);
  /** Parent index, or the next free node while the node sits on the free list. */
  private parent = new Int32Array(this.capacity);
  private child1 = new Int32Array(this.capacity);
  private child2 = new Int32Array(this.capacity);
  /** 0 for leaves, -1 for free nodes. */
  private height = new Int32Array(this.capacity);
  private data: (T | undefined)[] = new Array(this.capacity);

  private stack = new Int32Array(256);

  constructor() {
    this.linkFreeList(0);
  }

  get size(): number {
    return this.nodeCount;
  }

  get treeHeight(): number {
    return this.root === NULL ? 0 : this.height[this.root];
  }

  createProxy(minX: number, minY: number, maxX: number, maxY: number, data: T): number {
    const id = this.allocateNode();
    this.setBox(id, minX, minY, maxX, maxY);
    this.data[id] = data;
    this.height[id] = 0;
    this.insertLeaf(id);
    return id;
  }

  destroyProxy(id: number): void {
    this.removeLeaf(id);
    this.freeNode(id);
  }

  /** Replaces a leaf's box and re-inserts it at the best place for its new position. */
  moveProxy(id: number, minX: number, minY: number, maxX: number, maxY: number): void {
    this.removeLeaf(id);
    this.setBox(id, minX, minY, maxX, maxY);
    this.insertLeaf(id);
  }

  getData(id: number): T {
    return this.data[id] as T;
  }

  minX(id: number): number {
    return this.boxes[4 * id];
  }
  minY(id: number): number {
    return this.boxes[4 * id + 1];
  }
  maxX(id: number): number {
    return this.boxes[4 * id + 2];
  }
  maxY(id: number): number {
    return this.boxes[4 * id + 3];
  }

  /** True when the stored box of `id` fully contains the given box. */
  contains(id: number, minX: number, minY: number, maxX: number, maxY: number): boolean {
    const b = this.boxes;
    const o = 4 * id;
    return b[o] <= minX && b[o + 1] <= minY && b[o + 2] >= maxX && b[o + 3] >= maxY;
  }

  overlaps(a: number, b: number): boolean {
    const box = this.boxes;
    const i = 4 * a;
    const j = 4 * b;
    return box[i] <= box[j + 2] && box[j] <= box[i + 2] && box[i + 1] <= box[j + 3] && box[j + 1] <= box[i + 3];
  }

  /**
   * Calls `visit` for every leaf whose box overlaps the query box.
   * Returning false from `visit` stops the query early.
   */
  query(minX: number, minY: number, maxX: number, maxY: number, visit: (id: number) => boolean | void): void {
    if (this.root === NULL) return;
    const b = this.boxes;
    let stack = this.stack;
    let top = 0;
    stack[top++] = this.root;
    while (top > 0) {
      const node = stack[--top];
      const o = 4 * node;
      if (b[o] > maxX || b[o + 2] < minX || b[o + 1] > maxY || b[o + 3] < minY) continue;
      if (this.height[node] === 0) {
        if (visit(node) === false) return;
      } else {
        if (top + 2 > stack.length) {
          const grown = new Int32Array(stack.length * 2);
          grown.set(stack);
          this.stack = stack = grown;
        }
        stack[top++] = this.child1[node];
        stack[top++] = this.child2[node];
      }
    }
  }

  /** Visits every node, internal ones included; used by the debug renderer. */
  forEachNode(visit: (minX: number, minY: number, maxX: number, maxY: number, isLeaf: boolean) => void): void {
    if (this.root === NULL) return;
    const pending = [this.root];
    while (pending.length > 0) {
      const node = pending.pop()!;
      const o = 4 * node;
      const leaf = this.height[node] === 0;
      visit(this.boxes[o], this.boxes[o + 1], this.boxes[o + 2], this.boxes[o + 3], leaf);
      if (!leaf) pending.push(this.child1[node], this.child2[node]);
    }
  }

  /** Checks every structural invariant; throws on the first violation. For tests. */
  validate(): void {
    if (this.root === NULL) return;
    if (this.parent[this.root] !== NULL) throw new Error('root has a parent');
    let leaves = 0;
    const check = (node: number): number => {
      if (this.height[node] === 0) {
        leaves++;
        return 0;
      }
      const c1 = this.child1[node];
      const c2 = this.child2[node];
      if (this.parent[c1] !== node || this.parent[c2] !== node) throw new Error('broken parent link');
      const h = 1 + Math.max(check(c1), check(c2));
      if (this.height[node] !== h) throw new Error('wrong cached height');
      const o = 4 * node;
      const a = 4 * c1;
      const b = 4 * c2;
      const box = this.boxes;
      if (
        box[o] !== Math.min(box[a], box[b]) ||
        box[o + 1] !== Math.min(box[a + 1], box[b + 1]) ||
        box[o + 2] !== Math.max(box[a + 2], box[b + 2]) ||
        box[o + 3] !== Math.max(box[a + 3], box[b + 3])
      ) {
        throw new Error('node box is not the union of its children');
      }
      return h;
    };
    check(this.root);
    if (2 * leaves - 1 !== this.nodeCount) throw new Error('node count does not match leaf count');
  }

  // ---- internals -----------------------------------------------------------------------------

  private linkFreeList(from: number): void {
    for (let i = from; i < this.capacity; i++) {
      this.parent[i] = i + 1 < this.capacity ? i + 1 : NULL;
      this.height[i] = -1;
    }
    this.freeList = from;
  }

  private allocateNode(): number {
    if (this.freeList === NULL) {
      const old = this.capacity;
      this.capacity *= 2;
      const grow = <A extends Float64Array | Int32Array>(source: A, size: number): A => {
        const next = new (source.constructor as new (n: number) => A)(size);
        next.set(source);
        return next;
      };
      this.boxes = grow(this.boxes, this.capacity * 4);
      this.parent = grow(this.parent, this.capacity);
      this.child1 = grow(this.child1, this.capacity);
      this.child2 = grow(this.child2, this.capacity);
      this.height = grow(this.height, this.capacity);
      this.data.length = this.capacity;
      this.linkFreeList(old);
    }
    const id = this.freeList;
    this.freeList = this.parent[id];
    this.parent[id] = NULL;
    this.child1[id] = NULL;
    this.child2[id] = NULL;
    this.height[id] = 0;
    this.nodeCount++;
    return id;
  }

  private freeNode(id: number): void {
    this.parent[id] = this.freeList;
    this.height[id] = -1;
    this.data[id] = undefined;
    this.freeList = id;
    this.nodeCount--;
  }

  private setBox(id: number, minX: number, minY: number, maxX: number, maxY: number): void {
    const o = 4 * id;
    this.boxes[o] = minX;
    this.boxes[o + 1] = minY;
    this.boxes[o + 2] = maxX;
    this.boxes[o + 3] = maxY;
  }

  private setUnion(target: number, a: number, b: number): void {
    const box = this.boxes;
    const o = 4 * target;
    const i = 4 * a;
    const j = 4 * b;
    box[o] = Math.min(box[i], box[j]);
    box[o + 1] = Math.min(box[i + 1], box[j + 1]);
    box[o + 2] = Math.max(box[i + 2], box[j + 2]);
    box[o + 3] = Math.max(box[i + 3], box[j + 3]);
  }

  private perimeter(id: number): number {
    const o = 4 * id;
    return 2 * (this.boxes[o + 2] - this.boxes[o] + this.boxes[o + 3] - this.boxes[o + 1]);
  }

  private unionPerimeter(a: number, b: number): number {
    const box = this.boxes;
    const i = 4 * a;
    const j = 4 * b;
    const w = Math.max(box[i + 2], box[j + 2]) - Math.min(box[i], box[j]);
    const h = Math.max(box[i + 3], box[j + 3]) - Math.min(box[i + 1], box[j + 1]);
    return 2 * (w + h);
  }

  private insertLeaf(leaf: number): void {
    if (this.root === NULL) {
      this.root = leaf;
      this.parent[leaf] = NULL;
      return;
    }

    // Walk down, at each node choosing between "stop here" and descending into a child,
    // whichever adds the least perimeter to the tree.
    let index = this.root;
    while (this.height[index] > 0) {
      const c1 = this.child1[index];
      const c2 = this.child2[index];
      const area = this.perimeter(index);
      const combined = this.unionPerimeter(index, leaf);
      const cost = 2 * combined;
      const inheritance = 2 * (combined - area);
      const cost1 =
        inheritance + this.unionPerimeter(c1, leaf) - (this.height[c1] === 0 ? 0 : this.perimeter(c1));
      const cost2 =
        inheritance + this.unionPerimeter(c2, leaf) - (this.height[c2] === 0 ? 0 : this.perimeter(c2));
      if (cost < cost1 && cost < cost2) break;
      index = cost1 < cost2 ? c1 : c2;
    }

    const sibling = index;
    const oldParent = this.parent[sibling];
    const newParent = this.allocateNode();
    this.parent[newParent] = oldParent;
    this.setUnion(newParent, sibling, leaf);
    this.height[newParent] = this.height[sibling] + 1;
    if (oldParent !== NULL) {
      if (this.child1[oldParent] === sibling) this.child1[oldParent] = newParent;
      else this.child2[oldParent] = newParent;
    } else {
      this.root = newParent;
    }
    this.child1[newParent] = sibling;
    this.child2[newParent] = leaf;
    this.parent[sibling] = newParent;
    this.parent[leaf] = newParent;

    this.refit(this.parent[leaf]);
  }

  private removeLeaf(leaf: number): void {
    if (leaf === this.root) {
      this.root = NULL;
      return;
    }
    const parent = this.parent[leaf];
    const grand = this.parent[parent];
    const sibling = this.child1[parent] === leaf ? this.child2[parent] : this.child1[parent];
    if (grand !== NULL) {
      if (this.child1[grand] === parent) this.child1[grand] = sibling;
      else this.child2[grand] = sibling;
      this.parent[sibling] = grand;
      this.freeNode(parent);
      this.refit(grand);
    } else {
      this.root = sibling;
      this.parent[sibling] = NULL;
      this.freeNode(parent);
    }
  }

  /** Walks to the root, rebalancing and recomputing heights and boxes. */
  private refit(start: number): void {
    let index = start;
    while (index !== NULL) {
      index = this.balance(index);
      const c1 = this.child1[index];
      const c2 = this.child2[index];
      this.height[index] = 1 + Math.max(this.height[c1], this.height[c2]);
      this.setUnion(index, c1, c2);
      index = this.parent[index];
    }
  }

  /** Rotates node `a` if its children differ in height by more than one. Returns the new subtree root. */
  private balance(a: number): number {
    if (this.height[a] < 2) return a;
    const b = this.child1[a];
    const c = this.child2[a];
    const difference = this.height[c] - this.height[b];
    if (difference > 1) return this.rotateUp(a, c, b, false);
    if (difference < -1) return this.rotateUp(a, b, c, true);
    return a;
  }

  /**
   * Promotes `up` (the taller child of `a`) into a's place. The taller grandchild stays under
   * `up`; the shorter one moves under `a`, next to `other`.
   */
  private rotateUp(a: number, up: number, other: number, upIsChild1: boolean): number {
    const f = this.child1[up];
    const g = this.child2[up];

    this.child1[up] = a;
    this.parent[up] = this.parent[a];
    this.parent[a] = up;

    const top = this.parent[up];
    if (top !== NULL) {
      if (this.child1[top] === a) this.child1[top] = up;
      else this.child2[top] = up;
    } else {
      this.root = up;
    }

    const keep = this.height[f] > this.height[g] ? f : g;
    const give = keep === f ? g : f;
    this.child2[up] = keep;
    if (upIsChild1) this.child1[a] = give;
    else this.child2[a] = give;
    this.parent[give] = a;

    this.setUnion(a, other, give);
    this.setUnion(up, a, keep);
    this.height[a] = 1 + Math.max(this.height[other], this.height[give]);
    this.height[up] = 1 + Math.max(this.height[a], this.height[keep]);
    return up;
  }
}
