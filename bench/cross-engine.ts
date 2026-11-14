/**
 * Cross-engine determinism check: `npm run cross-engine`.
 *
 * Bundles the engine into one plain script, runs it in every JavaScript engine found on this
 * machine, and compares the scene hashes with tests/golden.json. On macOS that means V8 (Node)
 * and JavaScriptCore (the engine inside Safari), which ships with the system.
 */
import { execFileSync } from 'node:child_process';
import { existsSync, mkdtempSync, readFileSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('..', import.meta.url));
const directory = mkdtempSync(join(tmpdir(), 'zamah-'));
const bundle = join(directory, 'hashes.js');

execFileSync(join(root, 'node_modules/.bin/rolldown'), ['bench/hashes.ts', '--file', bundle, '--format', 'iife'], {
  cwd: root,
  stdio: 'pipe',
});
// Bare engine shells have no `performance`; the engine only uses it for profiling.
writeFileSync(
  bundle,
  `if (typeof performance === 'undefined') globalThis.performance = { now: () => 0 };\n${readFileSync(bundle, 'utf8')}`,
);

const golden = JSON.parse(readFileSync(join(root, 'tests/golden.json'), 'utf8')) as { hashes: Record<string, string> };
const expected = Object.entries(golden.hashes)
  .map(([id, hash]) => `${id} ${hash}`)
  .join('\n');

const engines: [string, string][] = [
  ['V8 (Node)', process.execPath],
  ['JavaScriptCore (Safari)', '/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc'],
  ['SpiderMonkey (Firefox)', '/usr/local/bin/js'],
  ['SpiderMonkey (Firefox)', '/opt/homebrew/bin/js'],
];

let failed = false;
let ran = 0;
for (const [name, binary] of engines) {
  if (!existsSync(binary)) continue;
  const output = execFileSync(binary, [bundle], { encoding: 'utf8' }).trim();
  const ok = output === expected;
  ran++;
  failed ||= !ok;
  console.log(`${ok ? 'identical' : 'DIFFERENT'}  ${name}`);
  if (!ok) console.log(output);
}
console.log(`${ran} engine(s) checked against tests/golden.json`);
process.exit(failed ? 1 : 0);
