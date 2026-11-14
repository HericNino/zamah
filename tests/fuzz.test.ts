import { describe, expect, it } from 'vitest';
import { fuzz } from './fuzzWorld.ts';

describe('fuzzing', () => {
  it('80 random worlds, poked at random, never produce NaN, runaway speeds or escapes', () => {
    for (let seed = 1; seed <= 80; seed++) fuzz(seed, 120);
  });

  it('replaying a seed reproduces the run exactly, random pokes included', () => {
    for (const seed of [3, 17, 42]) expect(fuzz(seed, 120)).toBe(fuzz(seed, 120));
  });

  it('long runs stay sane too', () => {
    for (const seed of [1001, 1002]) fuzz(seed, 900);
  });
});
