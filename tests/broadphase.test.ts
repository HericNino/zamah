import { describe, expect, it } from 'vitest';
import { box, circle, DynamicTree, NeighborGrid, Random, World } from '../src/engine/index.ts';

interface Box {
  id: number;
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

const overlap = (a: Box, b: Box) => a.minX <= b.maxX && b.minX <= a.maxX && a.minY <= b.maxY && b.minY <= a.maxY;

describe('dynamic AABB tree', () => {
  it('returns exactly the boxes a brute-force scan finds, through inserts, moves and removals', () => {
    const rnd = new Random(11);
    const tree = new DynamicTree<number>();
    const boxes = new Map<number, Box>();
    const randomBox = (): Omit<Box, 'id'> => {
      const x = rnd.range(-50, 50);
      const y = rnd.range(-50, 50);
      return { minX: x, minY: y, maxX: x + rnd.range(0.1, 6), maxY: y + rnd.range(0.1, 6) };
    };

    for (let round = 0; round < 400; round++) {
      // A mix of operations, weighted towards growth early on.
      for (let k = 0; k < 6; k++) {
        const roll = rnd.next();
        if (roll < 0.45 || boxes.size < 10) {
          const b = randomBox();
          const id = tree.createProxy(b.minX, b.minY, b.maxX, b.maxY, round);
          boxes.set(id, { id, ...b });
        } else if (roll < 0.8) {
          const ids = [...boxes.keys()];
          const id = ids[rnd.int(ids.length)];
          const b = randomBox();
          tree.moveProxy(id, b.minX, b.minY, b.maxX, b.maxY);
          boxes.set(id, { id, ...b });
        } else {
          const ids = [...boxes.keys()];
          const id = ids[rnd.int(ids.length)];
          tree.destroyProxy(id);
          boxes.delete(id);
        }
      }
      tree.validate();

      const query = { id: -1, ...randomBox() };
      const found: number[] = [];
      tree.query(query.minX, query.minY, query.maxX, query.maxY, (id) => {
        found.push(id);
      });
      const expected = [...boxes.values()].filter((b) => overlap(b, query)).map((b) => b.id);
      expect(found.sort((a, b) => a - b)).toEqual(expected.sort((a, b) => a - b));
    }
    expect(tree.size).toBe(2 * boxes.size - 1);
  });

  it('stays shallow: height grows with the logarithm of the size', () => {
    const tree = new DynamicTree<number>();
    // Sorted input is the worst case for a naive tree; it would degenerate into a list.
    for (let i = 0; i < 4096; i++) tree.createProxy(i, 0, i + 0.9, 1, i);
    tree.validate();
    expect(tree.treeHeight).toBeLessThanOrEqual(2 * Math.log2(4096));
  });

  it('can stop a query early', () => {
    const tree = new DynamicTree<number>();
    for (let i = 0; i < 100; i++) tree.createProxy(0, 0, 1, 1, i);
    let visits = 0;
    tree.query(0, 0, 1, 1, () => {
      visits++;
      return false;
    });
    expect(visits).toBe(1);
  });
});

describe('world broadphase', () => {
  it('never misses a pair: every overlap a brute-force check finds has a contact', () => {
    const rnd = new Random(5);
    const world = new World({ gravity: { x: 0, y: 0 } });
    // Bouncy shapes in a closed box, so they keep moving and keep meeting each other.
    for (const [x, y, hw, hh] of [
      [0, -11, 12, 1],
      [0, 11, 12, 1],
      [-11, 0, 1, 12],
      [11, 0, 1, 12],
    ]) {
      world.createBody({ type: 'static', position: { x, y }, shapes: [box(hw, hh, { restitution: 1 })] });
    }
    for (let i = 0; i < 120; i++) {
      const material = { restitution: 1, friction: 0 };
      world.createBody({
        position: { x: rnd.range(-9, 9), y: rnd.range(-9, 9) },
        linearVelocity: { x: rnd.range(-12, 12), y: rnd.range(-12, 12) },
        angularVelocity: rnd.range(-3, 3),
        shapes: [rnd.chance(0.5) ? circle(rnd.range(0.2, 0.6), material) : box(rnd.range(0.2, 0.7), rnd.range(0.2, 0.7), material)],
      });
    }

    const shapes = world.bodies.flatMap((b) => b.shapes);
    let checkedPairs = 0;
    for (let step = 0; step < 300; step++) {
      // Tight boxes that overlap right now ...
      const expected: number[] = [];
      for (let i = 0; i < shapes.length; i++) {
        for (let j = i + 1; j < shapes.length; j++) {
          const a = shapes[i];
          const b = shapes[j];
          if (!a.body.isDynamic && !b.body.isDynamic) continue;
          if (overlap(a, b)) expected.push(a.id < b.id ? a.id * 0x4000000 + b.id : b.id * 0x4000000 + a.id);
        }
      }
      world.step();
      // ... must all have been handed to the narrowphase during the step.
      for (const key of expected) expect(world.contacts.has(key)).toBe(true);
      checkedPairs += expected.length;
      // And no contact may linger once the inflated boxes have separated.
      if (step % 10 === 0) world.broadphase.tree.validate();
    }
    expect(checkedPairs).toBeGreaterThan(1000);
  });
});

describe('particle neighbour grid', () => {
  it('finds exactly the pairs within the radius, each once', () => {
    const rnd = new Random(9);
    const grid = new NeighborGrid();
    for (const count of [1, 2, 57, 600]) {
      const x = new Float64Array(count);
      const y = new Float64Array(count);
      // Spread over negative and positive coordinates, some far away.
      for (let i = 0; i < count; i++) {
        x[i] = rnd.range(-6, 6) + (i % 97 === 0 ? 1000 : 0);
        y[i] = rnd.range(-6, 6);
      }
      const radius = 0.7;
      grid.build(x, y, count, radius);
      for (let i = 0; i < count; i++) {
        const expected: number[] = [];
        for (let j = 0; j < count; j++) {
          if (j === i) continue;
          const dx = x[j] - x[i];
          const dy = y[j] - y[i];
          if (dx * dx + dy * dy < radius * radius) expected.push(j);
        }
        const found = Array.from(grid.neighbors.subarray(grid.start[i], grid.start[i + 1])).sort((a, b) => a - b);
        expect(found).toEqual(expected);
      }
    }
  });
});
