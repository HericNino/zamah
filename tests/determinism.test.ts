import { readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { GOLDEN_STEPS, sceneHashes } from '../bench/golden.ts';
import { World } from '../src/engine/index.ts';
import { scenes } from '../src/scenes/index.ts';
import golden from './golden.json' with { type: 'json' };

function sourceFiles(directory: string): string[] {
  return readdirSync(directory, { withFileTypes: true }).flatMap((entry) => {
    const path = join(directory, entry.name);
    if (entry.isDirectory()) return sourceFiles(path);
    return entry.name.endsWith('.ts') ? [path] : [];
  });
}

describe('determinism', () => {
  it('two runs of every scene agree bit for bit at every single step', () => {
    for (const scene of scenes) {
      const a = new World(scene.options);
      const b = new World(scene.options);
      scene.build(a);
      // Building other worlds in between must not matter: no hidden shared state.
      scene.build(new World(scene.options));
      scene.build(b);
      for (let step = 0; step < 120; step++) {
        a.step();
        b.step();
        if (a.hash() !== b.hash()) throw new Error(`${scene.id} diverged at step ${step}`);
      }
    }
  });

  it('the hash sees a one-bit change, and the simulation amplifies a nanometre into a different outcome', () => {
    const scene = scenes[0];
    const a = new World();
    const b = new World();
    scene.build(a);
    scene.build(b);
    const last = (world: World) => world.bodies[world.bodies.length - 1];
    last(b).x += Math.abs(last(b).x) * Number.EPSILON;
    expect(a.hash()).not.toBe(b.hash());

    // Sensitivity to initial conditions: the reason "almost deterministic" is worthless.
    const c = new World();
    scene.build(c);
    last(c).x += 1e-9;
    for (let i = 0; i < 300; i++) {
      a.step();
      c.step();
    }
    expect(a.hash()).not.toBe(c.hash());
    let largest = 0;
    for (let i = 0; i < a.bodies.length; i++) {
      largest = Math.max(largest, Math.hypot(a.bodies[i].x - c.bodies[i].x, a.bodies[i].y - c.bodies[i].y));
    }
    expect(largest).toBeGreaterThan(1e-6);
  });

  it('matches the recorded hashes, which were produced on a different machine', () => {
    // On the author's machine this is an arm64 Mac; in CI it is x64 Linux. Passing in both
    // places is the evidence that results do not depend on the CPU or the operating system.
    expect(golden.steps).toBe(GOLDEN_STEPS);
    expect(sceneHashes()).toEqual(golden.hashes);
  });

  it('the simulation never calls a Math function whose result may vary between engines', () => {
    // sin, cos, pow and friends are allowed to differ in the last bit from one JavaScript
    // engine to the next. The engine and the scenes use src/engine/math/trig.ts instead.
    const forbidden =
      /Math\.(sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|asinh|acosh|atanh|exp|expm1|log|log2|log10|log1p|pow|hypot|cbrt|random)\b|\*\*/;
    const offenders: string[] = [];
    for (const directory of ['src/engine', 'src/scenes']) {
      for (const file of sourceFiles(directory)) {
        readFileSync(file, 'utf8')
          .split('\n')
          .forEach((line, index) => {
            // Look at code only: drop line comments and the lines of block comments.
            const code = /^\s*(\/\*|\*)/.test(line) ? '' : line.replace(/\/\*.*?\*\//g, '').replace(/\/\/.*$/, '');
            if (forbidden.test(code)) offenders.push(`${file}:${index + 1}: ${line.trim()}`);
          });
      }
    }
    expect(offenders).toEqual([]);
  });
});
