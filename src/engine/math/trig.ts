/**
 * Deterministic trigonometry.
 *
 * The ECMAScript spec lets engines return slightly different results for Math.sin, Math.cos,
 * Math.atan2 and friends, so a simulation that calls them is only reproducible on one engine.
 * Addition, subtraction, multiplication, division and Math.sqrt, on the other hand, are
 * correctly rounded by IEEE 754 everywhere. Everything below is built from those alone, which
 * is what lets the same scene produce bit-identical state in Node, Chrome, Firefox and Safari.
 *
 * Accuracy is about 1e-14, far below anything the simulation can notice.
 */

export const PI = 3.141592653589793;
export const TWO_PI = 6.283185307179586;
export const HALF_PI = 1.5707963267948966;
const QUARTER_PI = 0.7853981633974483;

/** Taylor series of sin on [-pi/4, pi/4], evaluated with Horner's rule in x^2. */
function sinPoly(x: number): number {
  const z = x * x;
  return (
    x *
    (1 -
      (z / 6) *
        (1 - (z / 20) * (1 - (z / 42) * (1 - (z / 72) * (1 - (z / 110) * (1 - (z / 156) * (1 - z / 210)))))))
  );
}

/** Taylor series of cos on [-pi/4, pi/4]. */
function cosPoly(x: number): number {
  const z = x * x;
  return (
    1 -
    (z / 2) *
      (1 -
        (z / 12) *
          (1 - (z / 30) * (1 - (z / 56) * (1 - (z / 90) * (1 - (z / 132) * (1 - (z / 182) * (1 - z / 240)))))))
  );
}

export function sin(angle: number): number {
  // Reduce to [-pi, pi], fold to [-pi/2, pi/2], then pick the series that is accurate there.
  let x = angle - TWO_PI * Math.round(angle / TWO_PI);
  if (x > HALF_PI) x = PI - x;
  else if (x < -HALF_PI) x = -PI - x;
  if (x > QUARTER_PI) return cosPoly(HALF_PI - x);
  if (x < -QUARTER_PI) return -cosPoly(HALF_PI + x);
  return sinPoly(x);
}

export function cos(angle: number): number {
  return sin(angle + HALF_PI);
}

export function atan(value: number): number {
  const negative = value < 0;
  let a = negative ? -value : value;
  const inverted = a > 1;
  if (inverted) a = 1 / a;
  // Two half-angle reductions: atan(a) = 2 * atan(a / (1 + sqrt(1 + a^2))).
  a = a / (1 + Math.sqrt(1 + a * a));
  a = a / (1 + Math.sqrt(1 + a * a));
  // Now a <= tan(pi/16), where the Taylor series converges quickly.
  const z = a * a;
  let result =
    4 *
    a *
    (1 -
      z *
        (1 / 3 -
          z * (1 / 5 - z * (1 / 7 - z * (1 / 9 - z * (1 / 11 - z * (1 / 13 - z * (1 / 15 - z / 17))))))));
  if (inverted) result = HALF_PI - result;
  return negative ? -result : result;
}

export function atan2(y: number, x: number): number {
  if (x > 0) return atan(y / x);
  if (x < 0) return y >= 0 ? atan(y / x) + PI : atan(y / x) - PI;
  return y > 0 ? HALF_PI : y < 0 ? -HALF_PI : 0;
}
