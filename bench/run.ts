/**
 * Headless benchmark: `npm run bench`.
 *
 * Runs the engine without a browser and reports how long one step takes. Each case is warmed
 * up first (so the JIT has compiled the hot loops), then timed step by step; the median is
 * the headline number because it ignores the occasional garbage-collection pause.
 */
import { cpus } from 'node:os';
import { box, circle, Random, World } from '../src/engine/index.ts';
import { sceneById } from '../src/scenes/index.ts';
import { arena } from '../src/scenes/builders.ts';

interface Case {
  name: string;
  /** What is being stressed. */
  note: string;
  build(): World;
  warmup: number;
  steps: number;
}

function pyramid(base: number): World {
  const world = new World({ sleeping: false });
  arena(world, base, 2 * base);
  for (let row = 0; row < base; row++) {
    for (let i = 0; i < base - row; i++) {
      world.createBody({ position: { x: (i - (base - row - 1) / 2) * 1.02, y: 0.5 + row }, shapes: [box(0.5, 0.5)] });
    }
  }
  return world;
}

function pile(count: number): World {
  const rnd = new Random(7);
  const world = new World({ sleeping: false });
  arena(world, 30, 80);
  for (let i = 0; i < count; i++) {
    world.createBody({
      position: { x: rnd.range(-28, 28), y: rnd.range(1, 70) },
      angle: rnd.range(0, 6),
      shapes: [rnd.chance(0.5) ? circle(rnd.range(0.2, 0.5)) : box(rnd.range(0.2, 0.5), rnd.range(0.2, 0.5))],
    });
  }
  return world;
}

function cloth(cols: number, rows: number): World {
  const world = new World();
  arena(world, 20, 40);
  world.soft.createCloth({ x: -cols * 0.1, y: rows * 0.2 + 2, cols, rows, spacing: 0.2 });
  world.createBody({ type: 'static', position: { x: 0, y: rows * 0.1 }, shapes: [circle(1.5)] });
  return world;
}

function liquid(width: number, height: number): World {
  const world = new World();
  arena(world, 12, 40);
  world.fluid.configure({ spacing: 0.14 });
  world.fluid.addBlock(-12, 0, width, height);
  return world;
}

const scene = (id: string) => (): World => {
  const world = new World(sceneById(id)!.options);
  sceneById(id)!.build(world);
  world.sleeping = false;
  return world;
};

const cases: Case[] = [
  { name: 'Pyramid, 210 boxes', note: 'resting contacts', build: () => pyramid(20), warmup: 120, steps: 600 },
  { name: 'Pyramid, 1,275 boxes', note: 'resting contacts', build: () => pyramid(50), warmup: 60, steps: 240 },
  { name: 'Pile, 2,000 mixed bodies', note: 'broadphase and impacts', build: () => pile(2000), warmup: 60, steps: 300 },
  { name: 'Bridge and ragdolls scene', note: 'joints', build: scene('bridge'), warmup: 60, steps: 600 },
  { name: 'Cloth, 60 x 40', note: '2,400 particles, 13,902 constraints', build: () => cloth(60, 40), warmup: 60, steps: 300 },
  { name: 'Liquid, 2,000 particles', note: 'dam break', build: () => liquid(5, 6.9), warmup: 60, steps: 300 },
  { name: 'Liquid, 5,000 particles', note: 'dam break', build: () => liquid(8, 10.7), warmup: 60, steps: 300 },
  { name: 'Dam break scene', note: 'liquid coupled to rigid bodies', build: scene('fluid'), warmup: 60, steps: 300 },
];

const percentile = (sorted: number[], p: number): number => sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))];

console.log(`Zamah benchmark: ${cpus()[0].model}, Node ${process.version}, ${process.platform} ${process.arch}`);
console.log('One step is 1/60 s of simulated time; the real-time budget is 16.67 ms.\n');
console.log('| Case | Size | Median step | 95th percentile | Steps per second |');
console.log('|---|---|---:|---:|---:|');

for (const item of cases) {
  const world = item.build();
  for (let i = 0; i < item.warmup; i++) world.step();
  const times: number[] = [];
  for (let i = 0; i < item.steps; i++) {
    const start = performance.now();
    world.step();
    times.push(performance.now() - start);
  }
  times.sort((a, b) => a - b);
  const median = percentile(times, 0.5);
  const size = [
    world.bodies.length > 3 ? `${world.bodies.filter((b) => b.isDynamic).length} bodies` : '',
    world.contacts.size > 0 ? `${world.contacts.size} contacts` : '',
    world.joints.length > 0 ? `${world.joints.length} joints` : '',
    world.soft.count > 0 ? `${world.soft.count} cloth particles` : '',
    world.fluid.count > 0 ? `${world.fluid.count} liquid particles` : '',
  ]
    .filter(Boolean)
    .join(', ');
  console.log(
    `| ${item.name} | ${size} | ${median.toFixed(2)} ms | ${percentile(times, 0.95).toFixed(2)} ms | ${Math.round(1000 / median).toLocaleString('en-US')} |`,
  );
}
