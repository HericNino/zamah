const float = new Float64Array(1);
const words = new Uint32Array(float.buffer);

/**
 * Hashes a stream of doubles by their exact bit patterns (two independent 32-bit mixes, so
 * the digest is 64 bits). Used to compare simulation states: equal digests mean the states
 * are identical to the last bit, not merely close.
 */
export class StateHash {
  private a = 0x811c9dc5;
  private b = 0x9e3779b9;

  add(value: number): void {
    float[0] = value;
    const lo = words[0];
    const hi = words[1];
    // FNV-1a over the two words.
    this.a = Math.imul(this.a ^ lo, 16777619);
    this.a = Math.imul(this.a ^ hi, 16777619);
    // A murmur-style mix with different constants and a rotation.
    let b = this.b ^ Math.imul(hi, 0xcc9e2d51);
    b = (b << 13) | (b >>> 19);
    b = Math.imul(b ^ lo, 0x1b873593) + 0xe6546b64;
    this.b = b | 0;
  }

  digest(): string {
    const hex = (n: number) => (n >>> 0).toString(16).padStart(8, '0');
    return hex(this.a) + hex(this.b);
  }
}
