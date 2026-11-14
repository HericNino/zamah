/**
 * Regenerates tests/golden.json: the state hash of every scene after a fixed number of steps.
 *
 * The determinism test compares against this file. Because the engine uses nothing but
 * correctly rounded floating point operations, the hashes must come out the same on every
 * machine and JavaScript engine; if a run on another platform disagrees, that is a bug.
 *
 * Run `npm run golden` after any change that is *meant* to alter the simulation.
 */
import { writeFileSync } from 'node:fs';
import { World } from '../src/engine/index.ts';
import { scenes } from '../src/scenes/index.ts';

export const GOLDEN_STEPS = 240;

export function sceneHashes(): Record<string, string> {
  const hashes: Record<string, string> = {};
  for (const scene of scenes) {
    const world = new World(scene.options);
    scene.build(world);
    for (let i = 0; i < GOLDEN_STEPS; i++) world.step();
    hashes[scene.id] = world.hash();
  }
  return hashes;
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const hashes = sceneHashes();
  const target = new URL('../tests/golden.json', import.meta.url);
  writeFileSync(target, `${JSON.stringify({ steps: GOLDEN_STEPS, hashes }, null, 2)}\n`);
  for (const [id, hash] of Object.entries(hashes)) console.log(id.padEnd(10), hash);
  console.log(`wrote ${target.pathname}`);
}
