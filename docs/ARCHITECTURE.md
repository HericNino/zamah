# Architecture

This document explains how Zamah works and, more importantly, why it is built the way it is. Each section states a decision, the reason for it, and what was given up. Several sections also record an approach that was tried first and failed, because the failure is usually the best explanation of the final design.

Units are metres, kilograms and seconds. The y axis points up.

## The layers

```
math        vectors, deterministic sin/cos/atan2, seeded random numbers, state hash
   ^
collision   shapes -> dynamic AABB tree -> broadphase pairs -> contact manifolds
   ^
dynamics    bodies, contacts, joints, the sequential-impulse solver
   ^
particles   XPBD cloth and soft bodies, position based fluid, coupling to rigid bodies
   ^
world       one fixed time step that runs all of the above in order
   ^
scenes      seven scenes, written against the engine API only
   ^
sandbox     camera, renderers, tools, UI (the only code that touches the DOM)
```

Each layer imports only from the ones below it. The engine (`src/engine`) has no DOM access, no clock except for profiling, and no `Math.random`, which is what allows the tests, the benchmark and the sandbox to run the very same scenes.

## One step

`World.step()` advances time by exactly `dt` (1/60 s by default):

```
1. broadphase      re-insert shapes that left their fat box; find new candidate pairs
2. narrowphase     recompute the contact manifold of every pair with an awake body
3. islands         union-find over touching contacts and joints; wake whole islands
4. solve           repeated `substeps` times (default 4), with h = dt / substeps:
     a. integrate velocities (gravity, forces, damping)
     b. impact pass             elastic collisions, resolved one by one
     c. warm start              re-apply last step's impulses
     d. solve joints, contacts  with position correction ("bias")
     e. integrate positions     then re-measure every contact gap
     f. impact pass again       claims impacts that are now imminent
     g. relax joints, contacts  same solve without position correction
5. finish          refresh shape caches, put resting islands to sleep
6. cloth           XPBD sub-steps, collide with shapes, exchange momentum
7. liquid          position based fluid sub-steps, collide, exchange momentum
```

## Decisions

### Fixed time step, with an accumulator in the front end

**Decision.** The engine only ever takes steps of `dt`. The sandbox keeps an accumulator: each animation frame adds the real elapsed time, runs as many whole steps as fit (at most four per frame), and draws the state interpolated between the last two steps by the leftover fraction.

**Why.** A solver tuned for one step size behaves differently at another: stiffness, damping and stability all depend on it. Tying the step to the display would make a pyramid that stands at 60 Hz wobble at 120 Hz and explode at 30 Hz, and no test would be reproducible. With the accumulator the physics is identical on every display; a 120 Hz screen simply shows two interpolated frames per step.

**Cost.** One step of latency from interpolation, and when the machine cannot keep up the simulation slows down rather than skipping time (the four-step cap prevents the "spiral of death", where each slow frame demands more steps from the next).

### Sub-steps instead of solver iterations

**Decision.** One step is split into four sub-steps. Each sub-step does a single pass over the constraints, twice (once with position correction, once without).

**Why.** The traditional scheme takes one large step and iterates the solver eight or ten times. Running the same number of passes spread over smaller time steps converges much better for stiff problems such as tall stacks and long chains, because errors grow with the square of the step size and each small step starts from a nearly valid state. Collision detection, the expensive part, still runs once per step: sub-steps re-measure existing contacts from the bodies' current positions instead of recomputing manifolds.

**Cost.** Positions integrate four times per step, and contact normals are up to one step old (see "Re-measuring contacts" below).

### Sequential impulses with accumulated, clamped totals

**Decision.** Constraints are solved one at a time by applying the impulse that fixes that constraint's velocity error, sweeping over all of them (projected Gauss-Seidel). Each constraint remembers its *total* impulse, and it is the total that is clamped: a contact's total normal impulse must stay non-negative (it can push, not pull); friction's total must stay within ±μ times the normal impulse.

**Why.** Solving the whole system at once would need a linear complementarity solver; solving constraints one by one needs ten lines per constraint and no matrices. Clamping totals rather than increments is the detail that makes it correct: a later sweep is allowed to *take back* impulse that an earlier sweep applied too generously, which a per-increment clamp would forbid.

### Warm starting

**Decision.** Contact points carry identifiers of the geometric features that created them. When the same feature pair shows up next step, the point inherits last step's impulses, and the solver starts from those instead of from zero.

**Why.** A resting stack needs nearly the same impulses every step. Starting from last step's answer means the few passes available only have to correct small changes. A test measures the difference: without warm starting a 12-box stack sags more than three times deeper. The sandbox has a checkbox to see it.

**Cost.** Contact persistence: manifolds must be matched across steps, which is why the narrowphase produces stable feature ids rather than just points.

### Soft constraints and the relax pass

**Decision.** Position error (overlap, joint drift) is corrected by a "soft constraint": the constraint behaves like a critically damped spring of a chosen frequency (30 Hz for contacts between moving bodies, 60 Hz against static ones and for joints). After positions are integrated, the same constraints are solved once more without any position correction.

**Why.** The classic fix for overlap, Baumgarte stabilisation, adds a velocity proportional to the penetration. That velocity is real: it stays in the body and shows up as bounce and jitter, i.e. it adds energy. The soft formulation folds an implicit spring and damper into the impulse formula (three coefficients: a bias rate, a mass scale and an impulse scale), so the correction is smooth and bounded. The relax pass then removes whatever velocity the correction introduced, so the positions keep the fix and the velocities do not keep the push.

**Cost.** Contacts are not rigid. Bodies rest a few millimetres inside each other, more under load, and very heavy things on very light things compress visibly (see "Mass ratios").

### Speculative contacts instead of continuous collision detection

**Decision.** A contact is created when two shapes are still apart, as long as the gap is smaller than a margin of 2 cm plus the distance they can close in one step. While the gap is positive the constraint is "the closing speed may not exceed gap / h": the bodies are allowed to arrive, not to pass.

**Why.** Without this a fast body is simply found on the far side of a thin one. The textbook answer is a time-of-impact solver, which is a project of its own. Speculative contacts reuse the existing contact constraint and cost almost nothing; a 100 m/s bullet stops at a 20 cm wall in the tests.

**Cost.** It only works for pairs that the broadphase already reported, and collision detection runs once per step. A body that is hit mid-step and redirected towards something that was not a candidate can still end the step inside it. That is why arena walls are 10 m thick: a body caught inside is pushed out through the nearest face, and with a thick wall the nearest face is the one it came through. The fuzzer found the first version of this weakness: the fat box was only stretched when a shape moved, so a body given a large velocity *between* steps was not paired with what lay ahead. The broadphase now tests the box the shape is about to sweep.

### The impact pass: restitution as discrete collisions

**Decision.** Bounce is not part of the contact constraint. Before the constraints are solved (and again before the relax pass), every contact with restitution that is closing faster than 1 m/s and will touch within the sub-step is resolved as an instantaneous collision: the impulse that turns the approach speed *v* into −*e·v*. The sweep repeats up to eight times so that a hit can travel along a chain.

**Why.** The usual approach stores the approach speed at the start of the step and makes the contact aim for −*e* times that. In a Newton's cradle the second ball is at rest at the start of the step, so its contact with the third aims for zero, and the impact is absorbed instead of passed on. Treating collisions as a sequence of two-body events is what really happens (the scene leaves a 2 mm gap between balls so the sequence is well defined), and it has a property worth having: every impulse is a one-dimensional collision with *e* ≤ 1, whose energy change is (1+*e*)(*e*−1)/2 · *v*²/*k* ≤ 0. The pass can dissipate energy but cannot create it, and the test suite checks exactly that.

**What failed first.** With the impact pass only before the main solve, the first ball of the cradle kept a quarter of its speed: the relax pass looks one sub-step ahead through speculative contacts and quietly absorbed impacts that were about to happen. Hence the second impact pass. A second issue was caught on paper before it was written: resolving the two corners of a flat-landing box one after the other would leave it spinning (about 3 rad/s for a 1 m box landing at 5 m/s), so two simultaneous points are solved together as a 2×2 system.

**Cost.** Bounces under 1 m/s are inelastic (so things can come to rest), and the two-point case is an approximation for anything but symmetric landings.

### Re-measuring contacts every sub-step

**Decision.** After every position update each contact point is refreshed: its gap is re-measured from where the bodies are now, the impulse for both bodies is applied at one shared world point, and the effective masses are recomputed for the new lever arms. For two circles the normal is recomputed too; for a circle against a polygon the contact stays on the circle's rim point facing the polygon.

**Why.** This design came out of three failing tests.

- *Angular momentum drifted by 3%* in a box of colliding shapes. The lever arms were frozen at the start of the step while the bodies moved, so the two equal and opposite impulses acted at slightly different points and formed a couple. Applying both at one shared point makes that impossible; the test now holds to rounding error.
- *A rolling disc lost 7% of its speed.* The contact point was a material point of the disc, so it rotated away from the true contact and the normal force acquired a torque that opposed rolling. On a circle the contact is always the rim point facing the other shape, however much the circle has turned.
- *Glancing hits between fast balls bounced in the wrong direction*, because the normal was the line of centres from up to a full step earlier. Circle against circle is a square root, so it is simply done exactly every sub-step.

### Friction after the normal constraint

**Decision.** Within a pass, each contact's normal constraints are solved before its friction constraints.

**Why.** The friction limit is μ times the normal impulse. With friction first, the limit came from the previous pass, and during the pass with position correction a settling body's normal impulse is briefly near zero. A box on an incline slid several percent too fast and one that should have stuck crept downhill. The incline test (analytic acceleration *g*(sin θ − μ cos θ)) found it; the measured speed now matches to four digits.

### Separating axis test with reference-face clipping

**Decision.** For two convex polygons: find, on each, the face whose outward normal separates the shapes the most (or penetrates the least). The polygon with the better face supplies the reference face; on the other, the face most opposed to it is the incident face. Clip the incident face to the side planes of the reference face; the surviving end points that lie within the margin are the contact points (at most two).

**Why.** Two convex shapes are apart exactly when some face normal separates them, so the search over face normals is both the overlap test and the source of the contact normal. Clipping yields two points for face-to-face contact, which a stack needs to resist tipping, and the points come with stable identifiers (reference face, incident face, which end) for warm starting.

**Cost.** O(n·m) per pair, fine for at most eight vertices. The reference face is kept unless the other is better by a small tolerance, so the choice does not flip between steps.

**How it is tested.** Against an independent overlap test (do edges cross, or is one polygon inside the other) on 20,000 random pairs, and by checking that the reported normal and depth really are a separating translation.

### Dynamic bounding-box tree

**Decision.** The broadphase is a binary tree of axis-aligned boxes. Leaves are shapes with "fat" boxes (inflated by 10 cm and stretched along the direction of travel); internal nodes hold the union of their children. Insertion descends towards the sibling that increases total perimeter the least; rotations on the way back up keep the tree balanced. Nodes live in typed arrays and are recycled through a free list.

**Why.** Testing all pairs is O(n²): 2,000 bodies would mean two million box tests per step. The tree answers "what overlaps this box" in O(log n), and thanks to the fat boxes a body that rests or moves slowly is not touched at all. A uniform grid would be simpler but needs a cell size that suits every object and wastes memory on sparse scenes; sweep-and-prune degrades when many things line up on one axis. The same tree serves mouse picking and the particle systems' queries.

**Cost.** About 370 lines, the most intricate data structure in the project. Its invariants (parent links, cached heights, boxes being unions) are checked after every operation in the tests.

### Islands and sleeping by union-find

**Decision.** Every step, bodies connected by touching contacts or joints are merged into islands with a union-find. An island falls asleep when all of its bodies have been below 5 cm/s and 2°/s for half a second, and wakes as a whole when any awake body touches it.

**Why.** A resting pyramid should cost nothing, and it should be exactly still rather than jitter in the fifth decimal. Sleeping per body would not work: one box in a stack would sleep while its neighbour was still settling. Rebuilding the islands from scratch every step with union-find is a few lines and nearly linear, and avoids the bookkeeping of persistent island graphs.

**Cost.** A slow-moving body can be put to sleep before it has finished moving (a box creeping at 3 cm/s stops). Particle systems cannot wake a body directly; a sleeping body only keeps score of the pushes it would have received and wakes when the total is large enough.

### Rotation as a unit vector, and no `Math.sin`

**Decision.** A body's orientation is stored as (cos θ, sin θ) and rotated incrementally each sub-step. All trigonometry in the engine and the scenes comes from `math/trig.ts`, which builds sine, cosine and arctangent from polynomials using only `+ - * /` and `sqrt`.

**Why.** IEEE 754 requires those five operations to be correctly rounded, so they give the same bits on every CPU and in every JavaScript engine. It does not cover `sin`, `cos`, `pow` or `exp`, and the ECMAScript standard explicitly leaves their accuracy to the implementation. A rigid-body simulation is chaotic: a test shows that moving one body by a nanometre changes the outcome of the wrecking-ball scene within five seconds. So "almost the same" numbers mean a different simulation. Avoiding the unspecified functions makes the whole engine reproducible across engines, which was verified by running every scene in V8 and in JavaScriptCore and comparing hashes of the full state.

**Cost.** The polynomials are accurate to about 1e-14 rather than the last bit, which nothing in a simulation can notice. A test scans the engine source and fails if one of the forbidden functions appears.

### Objects for bodies, typed arrays for particles

**Decision.** Bodies, shapes and contacts are ordinary classes. Particles (cloth, liquid) and the tree's nodes are structures of typed arrays. The solver's inner loops use scalar locals instead of vector objects.

**Why.** A few hundred bodies with varied shapes and joints are easier to read and debug as objects, and JavaScript engines optimise objects of a fixed shape well. Thousands of identical particles are different: arrays of numbers are several times faster to sweep and create no garbage. Allocating a small vector for every arithmetic operation would produce millions of short-lived objects per second, so hot loops spell the components out. In the contact solver, the hottest loop, the six velocity components of a contact's two bodies are held in locals and written back once, which made it about 20% faster.

### XPBD with small steps for cloth and soft bodies

**Decision.** Cloth, ropes and blobs are particles joined by distance constraints (and, for blobs, an area constraint). Each step is ten sub-steps of: predict positions, project each constraint once, collide, derive velocities from the change in position.

**Why.** Position based dynamics moves particles directly onto the constraint, which cannot overshoot, so it is unconditionally stable: the right property for something a user will grab and yank. Plain PBD has a flaw, though: stiffness depends on the iteration count and the step size. XPBD adds a compliance term α/h² to each projection, which turns stiffness into a physical quantity (metres per newton). A test hangs a weight on a compliant thread and checks that it stretches by *mg*α with 4, 10 and 40 sub-steps. Many small steps with one projection each converge better than one step with many projections, for the same reason as in the rigid solver.

**Cost.** No self-collision. Tearing is simply removing a constraint when it stretches beyond a ratio.

### Position based fluid

**Decision.** The liquid is particles with a density constraint: the density around each particle, estimated with a smoothing kernel over its neighbours, may not exceed the rest density. Violations are corrected by moving particles along the kernel gradient, all at once from the same snapshot (a Jacobi iteration), three times per sub-step, two sub-steps per step. A small extra repulsion at short range keeps particles from clumping, and XSPH viscosity blends each velocity towards its neighbours'.

**Why.** Classic SPH computes pressure forces from density and integrates them, which needs very small time steps to stay stable. The position based form corrects positions directly and tolerates a 1/120 s step. The constraint is one-sided (only compression is corrected): letting it also pull particles together makes the free surface clump. The rest density is not a magic constant but measured at start-up from an ideal hexagonal packing at the chosen spacing, so a block created at rest spacing starts exactly at rest.

**What failed first.** Jacobi iteration can overshoot when many particles are crushed together, and with unlucky settings a clump flung particles to 1e39. Each pass's correction is now capped at 0.3 spacings and speeds at one spacing per sub-step, so however the particles were abused (poured onto each other, blasted) the state stays bounded.

**Neighbour search.** Particles are binned into a hashed uniform grid by counting sort; the neighbours of a particle are in its cell and the eight around it. The arrays are re-sorted into grid order every step so that neighbours in space are neighbours in memory. It is checked against a brute-force search.

### Two-way coupling, with a cap

**Decision.** The particle systems run after the rigid solver and treat bodies as fixed obstacles: a particle inside a shape is moved out. Afterwards the momentum the particle really gained is taken from the body, but never more than what stops the body moving into the particle plus a gentle push out of the overlap (20% of the depth per step, at most 1 m/s).

**Why.** The first version handed the body the full reaction, *m·Δx/h*. A ball dropped on a taut cloth then bounced forever: an inextensible XPBD constraint restores any stretch within one sub-step, so its "force" is limited only by the step size, and feeding that back one step late is an explicit integration of a nearly infinitely stiff spring. A proper fix is a unified solver in which bodies and particles are solved together. The cap is the pragmatic alternative: cloth tension and liquid pressure can halt a body and carry its weight, but can never throw it. Light curtains still give way to heavy bodies correctly, because there the particle's own momentum is the smaller quantity.

**Cost.** Trampolines are weak, and buoyancy needs liquid under the body.

### Rendering: Canvas 2D, plus WebGL2 for the liquid, with a fallback

**Decision.** Bodies, joints, cloth and debug overlays are drawn with Canvas 2D. The liquid is drawn on a separate WebGL2 canvas underneath: each particle is splatted as a soft blob into a half-resolution texture (additive blending), and a second pass thresholds the summed field into a surface, tinted by depth and speed. If WebGL2 is unavailable or the context is lost, a Canvas 2D renderer draws the particles as discs instead.

**Why.** A few hundred polygons are trivial for Canvas 2D and the code stays short. Three thousand discs do not look like water; a thresholded density field does, and that is a two-pass shader. WebGPU was the original idea, but it would add nothing here: the simulation deliberately stays on the CPU (determinism, testability in Node), and drawing a few thousand sprites is far below what WebGL2 handles, while WebGL2 runs in every current browser. The fallback exists because "every current browser" still includes machines with the GPU blocklisted.

## Testing strategy

Unit tests of a physics engine tend to assert that code does what the code does. These tests try to pin the engine to something outside itself:

- **Analytic results**: free fall, incline, rolling, pendulum, spring, elastic collision formulas.
- **Conservation laws**: momentum always, energy where the physics says so.
- **Differential tests**: the tree against a linear scan, the narrowphase against a different algorithm, the neighbour grid against brute force.
- **Properties**: a reported contact normal and depth must separate the shapes; energy must never increase in an elastic box.
- **Determinism**: run twice and compare hashes; compare with hashes recorded on another architecture; scan the source for forbidden functions.
- **Fuzzing**: random worlds, random abuse, invariants checked every step.
- **Scenes**: every sandbox scene runs headless with scene-specific expectations.

Long tests are sized to finish in a few seconds each, so they do not time out on a loaded machine.

## Questions worth being able to answer

1. Why does the engine use a fixed time step, and what exactly happens on a 120 Hz display? What is the "spiral of death" and how is it avoided?
2. What is a sequential impulse solver? Why is the *accumulated* impulse clamped rather than each increment?
3. What does warm starting do, what does it require from the narrowphase, and how would you demonstrate its effect?
4. What is wrong with Baumgarte stabilisation? What are the three coefficients of a soft constraint, and what does the relax pass undo?
5. Why four sub-steps with one pass each instead of one step with eight passes?
6. How do speculative contacts prevent tunnelling, and in which situation do they fail? Why are the arena walls 10 m thick?
7. Why does a Newton's cradle fail in a typical impulse solver? Why can the impact pass never add energy? Why does it run twice per sub-step?
8. Walk through SAT and clipping for two boxes. What are the reference and incident faces? Why does a contact point need an identifier?
9. How does the bounding-box tree choose where to insert a leaf, and how does it stay balanced? What is a fat box for?
10. How are islands built, and why must a stack fall asleep as a whole?
11. Which floating point operations are deterministic across machines and which are not? How was cross-engine determinism verified, and what would break it?
12. Semi-implicit Euler: what does it conserve, and why does an elastic ball's rebound height alternate between two values?
13. What does XPBD add to PBD? What are the units of compliance, and which test shows that stiffness no longer depends on the sub-step count?
14. How does position based fluid enforce incompressibility? Why Jacobi rather than Gauss-Seidel there, and why does each correction have to be capped?
15. Why did a ball on a taut cloth bounce forever with the naive coupling, and what does the cap trade away?
16. Why is friction solved after the normal constraint? Which test caught the difference?
17. Why did angular momentum drift before both bodies received their impulse at a shared point?
18. Why do iterative solvers struggle with large mass ratios, and what are the measured limits here?
19. Where does the engine use objects and where typed arrays, and why? Why are there no vector objects in the solver loops?
20. How does the neighbour search work, and why does sorting particles by cell make everything else faster?
21. How is the liquid surface drawn? Why WebGL2 and not WebGPU, and what happens without either?
22. What kinds of tests are there, and which bugs did each kind find?
23. What would you build next? (A time-of-impact pass for fast thin bodies; a block solver for two-point manifolds; solving bodies and particles in one loop; moving the solver to WebAssembly with SIMD.)
