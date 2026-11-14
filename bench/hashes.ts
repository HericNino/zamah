/**
 * Prints the state hash of every scene after a fixed number of steps, using nothing but the
 * language itself, so the same bundled file runs in any JavaScript engine's shell.
 * See bench/cross-engine.ts.
 */
import { World } from '../src/engine/index.ts';
import { scenes } from '../src/scenes/index.ts';

const lines: string[] = [];
for (const scene of scenes) {
  const world = new World(scene.options);
  scene.build(world);
  for (let i = 0; i < 240; i++) world.step();
  lines.push(`${scene.id} ${world.hash()}`);
}
const host = globalThis as unknown as { print?: (text: string) => void; console: { log(text: string): void } };
(host.print ?? host.console.log)(lines.join('\n'));
