/**
 * Seeded pseudo-random numbers (mulberry32). Scenes, tests and the fuzzer never touch
 * Math.random, so every run with the same seed builds exactly the same world.
 */
export class Random {
  private state: number;

  constructor(seed = 1) {
    this.state = seed >>> 0;
  }

  /** Uniform in [0, 1). */
  next(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    let t = this.state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  /** Uniform in [min, max). */
  range(min: number, max: number): number {
    return min + (max - min) * this.next();
  }

  /** Uniform integer in [0, count). */
  int(count: number): number {
    return Math.floor(this.next() * count);
  }

  chance(probability: number): boolean {
    return this.next() < probability;
  }
}
