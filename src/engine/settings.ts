/**
 * Tuning constants shared by the whole engine. Units are metres, kilograms, seconds.
 * The values assume objects between roughly 0.1 m and 10 m, like most 2D engines.
 */

/** Penetration the solver tolerates on purpose, so resting manifolds do not flicker. */
export const LINEAR_SLOP = 0.005;

/** Contacts are created this far *before* shapes touch (speculative contacts). */
export const SPECULATIVE_DISTANCE = 4 * LINEAR_SLOP;

/** Broadphase boxes are inflated by this much, so slow bodies rarely need a tree update. */
export const AABB_MARGIN = 0.1;

/** Upper bound on convex polygon size; keeps the O(n*m) SAT loops small. */
export const MAX_POLYGON_VERTICES = 8;

/** A body below both speeds for TIME_TO_SLEEP seconds may fall asleep with its island. */
export const SLEEP_LINEAR_SPEED = 0.05;
export const SLEEP_ANGULAR_SPEED = 0.035;
export const TIME_TO_SLEEP = 0.5;

/** How many times one sub-step may hand an elastic impact down a chain of touching bodies. */
export const MAX_IMPACT_PASSES = 8;

/** Safety clamps; nothing sensible moves faster than this. */
export const MAX_LINEAR_SPEED = 200;
export const MAX_ROTATION_PER_SUBSTEP = 0.25 * Math.PI;
