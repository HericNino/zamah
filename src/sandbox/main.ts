import { World } from '../engine/index.ts';
import { type Scene, sceneById, scenes, type ToolId } from '../scenes/index.ts';
import { Camera } from './camera.ts';
import { createFluidRenderer, type FluidRenderer } from './fluidRenderer.ts';
import { type DebugOptions, Renderer } from './render.ts';
import { type Pointer, TOOLS, Tools } from './tools.ts';

declare const __REPO_URL__: string;

const $ = <T extends HTMLElement>(selector: string): T => document.querySelector<T>(selector)!;

const stage = $('#stage');
const view = $<HTMLCanvasElement>('#view');
const panel = $('#panel');

const camera = new Camera();
const renderer = new Renderer(view);
const params = new URLSearchParams(location.search);

let fluidRenderer: FluidRenderer = createFluidRenderer(stage, params.get('fluid') === '2d', onContextLost);
function onContextLost(): void {
  // The GPU took the context away; carry on with plain canvas drawing.
  fluidRenderer = createFluidRenderer(stage, true, onContextLost);
  resize();
}

// ---- state --------------------------------------------------------------------------------------

let scene: Scene = sceneById(location.hash.slice(1)) ?? scenes[0];
let world = new World();
const tools = new Tools(world);
const pointer: Pointer = { x: 0, y: 0, vx: 0, vy: 0, down: false };
let pointerScreenX = 0;
let pointerScreenY = 0;
let pointerMovedAt = 0;
let panning = false;

let paused = false;
let stepRequested = false;
let accumulator = 0;
let lastFrame = performance.now();
const debug: DebugOptions = { contacts: false, bounds: false, joints: false, sleep: false, particles: false };
const settings = { substeps: 4, warmStarting: true, sleeping: true, gravity: true, slow: false };

/** Smoothed timings in milliseconds, so the numbers are readable. */
const timing = { frame: 16.7, step: 0, broadphase: 0, narrowphase: 0, solve: 0, soft: 0, fluid: 0, render: 0 };

// ---- scene management ---------------------------------------------------------------------------

/** Phones in portrait get a tighter framing of the part of the scene that matters. */
function viewFor(target: Scene): Scene['view'] {
  return camera.width < camera.height && target.narrowView ? target.narrowView : target.view;
}

function applySettings(): void {
  world.substeps = settings.substeps;
  world.warmStarting = settings.warmStarting;
  world.sleeping = settings.sleeping;
  if (!settings.sleeping) for (const body of world.bodies) body.wake();
  world.setGravity(0, settings.gravity ? -10 : 0);
}

function loadScene(next: Scene): void {
  scene = next;
  world = new World(scene.options);
  scene.build(world);
  applySettings();
  tools.setWorld(world);
  selectTool(scene.tool);
  accumulator = 0;
  camera.fit(viewFor(scene));
  fittedScale = camera.scale;
  $('#scene-title').textContent = scene.title;
  $('#scene-blurb').textContent = scene.blurb;
  $('#scene-hint').textContent = scene.hint;
  for (const button of document.querySelectorAll<HTMLButtonElement>('#scenes button')) {
    const active = button.dataset.scene === scene.id;
    button.setAttribute('aria-pressed', String(active));
    if (active) button.scrollIntoView({ block: 'nearest', inline: 'nearest' });
  }
  history.replaceState(null, '', `#${scene.id}`);
}

function selectTool(id: ToolId): void {
  tools.select(id);
  for (const button of document.querySelectorAll<HTMLButtonElement>('#tools button')) {
    button.setAttribute('aria-pressed', String(button.dataset.tool === id));
  }
}

// ---- layout -------------------------------------------------------------------------------------

function resize(): void {
  const rect = stage.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, rect.width);
  const height = Math.max(1, rect.height);
  const first = camera.width <= 1;
  const oldScale = camera.scale;
  camera.resize(width, height);
  renderer.resize(width, height, dpr);
  fluidRenderer.resize(width, height, dpr);
  // Refit on the first layout and whenever the user has not zoomed away from the fitted view.
  if (first || Math.abs(oldScale - fittedScale) < 1e-6) {
    camera.fit(viewFor(scene));
    fittedScale = camera.scale;
  }
}
let fittedScale = -1;

new ResizeObserver(resize).observe(stage);

// ---- toolbar and buttons ------------------------------------------------------------------------

const ICONS: Record<ToolId, string> = {
  grab: '<path d="M8 13V5.5a1.5 1.5 0 0 1 3 0V11m0-6.5a1.5 1.5 0 0 1 3 0V11m0-5a1.5 1.5 0 0 1 3 0v8a6 6 0 0 1-6 6h-1a6 6 0 0 1-5-2.7L3.6 14a1.5 1.5 0 0 1 2.5-1.6L8 15"/>',
  box: '<rect x="4" y="4" width="16" height="16" rx="2"/>',
  ball: '<circle cx="12" cy="12" r="8.5"/>',
  poly: '<path d="M12 3l8.5 6.2-3.2 10H6.7L3.5 9.2z"/>',
  water: '<path d="M12 3c3.5 4.5 6 7.6 6 11a6 6 0 0 1-12 0c0-3.4 2.5-6.5 6-11z"/>',
  cut: '<circle cx="6" cy="6.5" r="2.5"/><circle cx="6" cy="17.5" r="2.5"/><path d="M8.2 8l11.8 9M8.2 16l11.8-9"/>',
  blast: '<path d="M12 2l2.2 6 5.8-2.5-3 5.5 5 3.5-6 .5.5 6.5-4.5-4.5-4.5 4.5.5-6.5-6-.5 5-3.5-3-5.5 5.8 2.5z"/>',
  erase: '<path d="M4 15.5 13.5 6a2 2 0 0 1 2.8 0l3.2 3.2a2 2 0 0 1 0 2.8L12 19.5H7.5z"/><path d="M9 11l5.5 5.5M12 19.5h8.5"/>',
};

const toolsElement = $('#tools');
for (const tool of TOOLS) {
  const button = document.createElement('button');
  button.type = 'button';
  button.dataset.tool = tool.id;
  button.title = `${tool.hint} (${tool.key})`;
  button.setAttribute('aria-label', tool.label);
  button.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${ICONS[tool.id]}</svg><span>${tool.label}</span>`;
  button.addEventListener('click', () => selectTool(tool.id));
  toolsElement.append(button);
}

const scenesElement = $('#scenes');
for (const item of scenes) {
  const button = document.createElement('button');
  button.type = 'button';
  button.dataset.scene = item.id;
  button.textContent = item.title;
  button.addEventListener('click', () => loadScene(item));
  scenesElement.append(button);
}

const pauseButton = $<HTMLButtonElement>('#pause');
function setPaused(value: boolean): void {
  paused = value;
  pauseButton.textContent = paused ? 'Play' : 'Pause';
  pauseButton.setAttribute('aria-pressed', String(paused));
}
pauseButton.addEventListener('click', () => setPaused(!paused));
$('#step').addEventListener('click', () => {
  setPaused(true);
  stepRequested = true;
});
$('#reset').addEventListener('click', () => loadScene(scene));

const panelToggle = $<HTMLButtonElement>('#panel-toggle');
function setPanel(open: boolean): void {
  panel.hidden = !open;
  panelToggle.setAttribute('aria-expanded', String(open));
}
panelToggle.addEventListener('click', () => setPanel(panel.hasAttribute('hidden')));

for (const input of document.querySelectorAll<HTMLInputElement>('[data-debug]')) {
  input.addEventListener('change', () => {
    debug[input.dataset.debug as keyof DebugOptions] = input.checked;
  });
}

const substepsInput = $<HTMLInputElement>('#substeps');
substepsInput.addEventListener('input', () => {
  settings.substeps = Number(substepsInput.value);
  $('#substeps-value').textContent = substepsInput.value;
  applySettings();
});
const bindToggle = (selector: string, key: 'warmStarting' | 'sleeping' | 'gravity' | 'slow'): void => {
  const input = $<HTMLInputElement>(selector);
  input.addEventListener('change', () => {
    settings[key] = input.checked;
    applySettings();
  });
};
bindToggle('#warm', 'warmStarting');
bindToggle('#sleeping', 'sleeping');
bindToggle('#gravity', 'gravity');
bindToggle('#slow', 'slow');

if (__REPO_URL__) {
  const repo = $<HTMLAnchorElement>('#repo');
  repo.href = __REPO_URL__;
  repo.hidden = false;
}

// ---- pointer input ------------------------------------------------------------------------------

function updatePointer(event: PointerEvent): void {
  const rect = view.getBoundingClientRect();
  const sx = event.clientX - rect.left;
  const sy = event.clientY - rect.top;
  const wx = camera.toWorldX(sx);
  const wy = camera.toWorldY(sy);
  const now = performance.now();
  const dt = Math.max(1, now - pointerMovedAt) / 1000;
  if (pointerMovedAt > 0 && dt < 0.2) {
    pointer.vx += 0.5 * ((wx - pointer.x) / dt - pointer.vx);
    pointer.vy += 0.5 * ((wy - pointer.y) / dt - pointer.vy);
  } else {
    pointer.vx = pointer.vy = 0;
  }
  pointerMovedAt = now;
  pointerScreenX = sx;
  pointerScreenY = sy;
  pointer.x = wx;
  pointer.y = wy;
}

view.addEventListener('pointerdown', (event) => {
  try {
    // Keep receiving the drag when the pointer leaves the canvas.
    view.setPointerCapture(event.pointerId);
  } catch {
    // Synthetic events have no capturable pointer; the drag still works inside the canvas.
  }
  pointerMovedAt = 0;
  updatePointer(event);
  if (event.button !== 0 || !tools.down(pointer)) {
    // Right or middle button, or the grab tool on empty space: pan the camera.
    panning = true;
    view.style.cursor = 'grabbing';
    return;
  }
  pointer.down = true;
});

view.addEventListener('pointermove', (event) => {
  if (panning) {
    const rect = view.getBoundingClientRect();
    camera.x -= (event.clientX - rect.left - pointerScreenX) / camera.scale;
    camera.y += (event.clientY - rect.top - pointerScreenY) / camera.scale;
    fittedScale = -1;
    pointerScreenX = event.clientX - rect.left;
    pointerScreenY = event.clientY - rect.top;
    return;
  }
  updatePointer(event);
  tools.move(pointer);
});

const endPointer = (event: PointerEvent): void => {
  if (panning) {
    panning = false;
    view.style.cursor = '';
    return;
  }
  if (!pointer.down) return;
  updatePointer(event);
  tools.up(pointer);
  pointer.down = false;
};
view.addEventListener('pointerup', endPointer);
view.addEventListener('pointercancel', endPointer);
view.addEventListener('contextmenu', (event) => event.preventDefault());

view.addEventListener(
  'wheel',
  (event) => {
    event.preventDefault();
    const rect = view.getBoundingClientRect();
    camera.zoomAt(event.clientX - rect.left, event.clientY - rect.top, Math.exp(-event.deltaY * 0.0015));
    fittedScale = -1;
  },
  { passive: false },
);

window.addEventListener('keydown', (event) => {
  if (event.metaKey || event.ctrlKey || event.altKey) return;
  if (event.target instanceof HTMLInputElement && event.target.type !== 'checkbox' && event.target.type !== 'range') {
    return;
  }
  const key = event.key;
  if (key === ' ') {
    event.preventDefault();
    setPaused(!paused);
  } else if (key === '.' || key === 'ArrowRight') {
    setPaused(true);
    stepRequested = true;
  } else if (key === 'r' || key === 'R') {
    loadScene(scene);
  } else if (key === 'd' || key === 'D') {
    setPanel(panel.hasAttribute('hidden'));
  } else if (key >= '1' && key <= '9') {
    const target = scenes[Number(key) - 1];
    if (target) loadScene(target);
  } else {
    const tool = TOOLS.find((t) => t.key.toLowerCase() === key.toLowerCase());
    if (tool) selectTool(tool.id);
  }
});

window.addEventListener('hashchange', () => {
  const target = sceneById(location.hash.slice(1));
  if (target && target !== scene) loadScene(target);
});

// ---- stats panel --------------------------------------------------------------------------------

interface Row {
  value: HTMLElement;
  bar: HTMLElement | null;
}

function makeRows(host: HTMLElement, labels: string[], bars: boolean): Row[] {
  return labels.map((label) => {
    const key = document.createElement('span');
    key.className = 'k';
    key.textContent = label;
    const value = document.createElement('span');
    value.className = label === 'State hash' ? 'v wide' : 'v';
    host.append(key, value);
    let bar: HTMLElement | null = null;
    if (bars) {
      const track = document.createElement('div');
      track.className = 'bar-row';
      bar = document.createElement('i');
      track.append(bar);
      host.append(track);
    }
    return { value, bar };
  });
}

const timingRows = makeRows(
  $('#timing'),
  ['Frames per second', 'Physics step', 'Broadphase', 'Narrowphase', 'Solver', 'Cloth', 'Liquid', 'Render'],
  true,
);
const countRows = makeRows(
  $('#counts'),
  ['Bodies (awake)', 'Contacts (touching)', 'Joints', 'Cloth particles', 'Liquid particles', 'Tree height', 'Liquid renderer', 'Step', 'State hash'],
  false,
);

function updatePanel(): void {
  if (panel.hidden) return;
  const ms = (value: number) => `${value.toFixed(2)} ms`;
  const budget = 1000 * world.dt;
  const values = [timing.step, timing.broadphase, timing.narrowphase, timing.solve, timing.soft, timing.fluid, timing.render];
  timingRows[0].value.textContent = (1000 / timing.frame).toFixed(0);
  timingRows[0].bar!.style.width = `${Math.min(100, (100 * 1000) / timing.frame / 120)}%`;
  values.forEach((value, i) => {
    timingRows[i + 1].value.textContent = ms(value);
    timingRows[i + 1].bar!.style.width = `${Math.min(100, (100 * value) / budget)}%`;
  });

  let awake = 0;
  for (const body of world.bodies) if (body.awake) awake++;
  const text = [
    `${world.bodies.length} (${awake})`,
    `${world.contacts.size} (${world.activeContacts.length})`,
    `${world.joints.length}`,
    `${world.soft.count}`,
    `${world.fluid.count}`,
    `${world.broadphase.tree.treeHeight}`,
    fluidRenderer.kind === 'webgl2' ? 'WebGL2' : 'Canvas 2D',
    `${world.stepCount}`,
    world.hash(),
  ];
  text.forEach((value, i) => (countRows[i].value.textContent = value));
}
setInterval(updatePanel, 250);

// ---- main loop ----------------------------------------------------------------------------------

/** Never run more than this many physics steps per frame; beyond it, time slows down instead. */
const MAX_STEPS_PER_FRAME = 4;

function stepWorld(): void {
  // Keep the pointer's world position current even if only the camera moved.
  pointer.x = camera.toWorldX(pointerScreenX);
  pointer.y = camera.toWorldY(pointerScreenY);
  tools.beforeStep(pointer);
  world.step();

  const p = world.profile;
  const blend = 0.1;
  timing.step += blend * (p.total - timing.step);
  timing.broadphase += blend * (p.broadphase - timing.broadphase);
  timing.narrowphase += blend * (p.narrowphase - timing.narrowphase);
  timing.solve += blend * (p.solve - timing.solve);
  timing.soft += blend * (p.soft - timing.soft);
  timing.fluid += blend * (p.fluid - timing.fluid);

  // Whatever falls out of the scene is gone for good.
  for (let i = world.bodies.length - 1; i >= 0; i--) {
    const body = world.bodies[i];
    if (body.isDynamic && (body.y < -40 || Math.abs(body.x) > 200)) world.destroyBody(body);
  }
  if (world.fluid.count > 0 && world.stepCount % 30 === 0) world.fluid.removeOutside(-200, -40, 200, 400);
}

function frame(now: number): void {
  const elapsed = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  timing.frame += 0.1 * (elapsed * 1000 - timing.frame);
  if (performance.now() - pointerMovedAt > 120) pointer.vx = pointer.vy = 0;

  // Fixed time step: physics always advances in steps of world.dt, however fast the display
  // refreshes. Leftover time stays in the accumulator and is used to interpolate the drawing.
  let alpha = 1;
  if (paused) {
    if (stepRequested) stepWorld();
    stepRequested = false;
    accumulator = 0;
  } else {
    accumulator += elapsed * (settings.slow ? 0.2 : 1);
    let steps = 0;
    while (accumulator >= world.dt && steps < MAX_STEPS_PER_FRAME) {
      stepWorld();
      accumulator -= world.dt;
      steps++;
    }
    if (accumulator >= world.dt) accumulator = 0;
    alpha = accumulator / world.dt;
  }

  render(alpha, elapsed);
  requestAnimationFrame(frame);
}

function render(alpha: number, elapsed: number): void {
  const renderStart = performance.now();
  fluidRenderer.draw(world.fluid, camera, alpha);
  renderer.draw(world, camera, alpha, debug);
  drawOverlay(elapsed);
  timing.render += 0.1 * (performance.now() - renderStart - timing.render);
}

/** Tool feedback: the shape about to be thrown, cut lines, explosion rings. */
function drawOverlay(elapsed: number): void {
  const ctx = renderer.ctx;
  const px = 1 / camera.scale;
  const preview = tools.preview;
  if (preview) {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.12)';
    ctx.lineWidth = 1.5 * px;
    ctx.beginPath();
    const shape = preview.shape;
    if (shape.kind === 'circle') {
      ctx.arc(preview.x, preview.y, shape.radius, 0, 6.2832);
    } else {
      shape.vertices.forEach((v, i) => {
        if (i === 0) ctx.moveTo(preview.x + v.x, preview.y + v.y);
        else ctx.lineTo(preview.x + v.x, preview.y + v.y);
      });
      ctx.closePath();
    }
    ctx.fill();
    ctx.stroke();
    ctx.setLineDash([5 * px, 5 * px]);
    ctx.beginPath();
    ctx.moveTo(preview.x, preview.y);
    ctx.lineTo(pointer.x, pointer.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  for (let i = tools.effects.length - 1; i >= 0; i--) {
    const effect = tools.effects[i];
    effect.age += elapsed;
    const life = effect.kind === 'ring' ? 0.35 : 0.25;
    const t = effect.age / life;
    if (t >= 1) {
      tools.effects.splice(i, 1);
      continue;
    }
    ctx.globalAlpha = 1 - t;
    if (effect.kind === 'ring') {
      ctx.strokeStyle = '#ffd08a';
      ctx.lineWidth = 3 * px;
      ctx.beginPath();
      ctx.arc(effect.x0, effect.y0, effect.x1 * (0.15 + 0.85 * t), 0, 6.2832);
      ctx.stroke();
    } else {
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2 * px;
      ctx.beginPath();
      ctx.moveTo(effect.x0, effect.y0);
      ctx.lineTo(effect.x1, effect.y1);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }
}

// A console hook: `zamah.world` to inspect the engine, `zamah.advance(600)` to fast-forward.
// The automated browser checks use it too, since a background tab gets almost no frames.
Object.assign(window, {
  zamah: {
    get world() {
      return world;
    },
    get camera() {
      return camera;
    },
    get fluidRenderer() {
      return fluidRenderer.kind;
    },
    load(id: string): void {
      const target = sceneById(id);
      if (target) loadScene(target);
    },
    advance(steps = 1): void {
      for (let i = 0; i < steps; i++) stepWorld();
      render(1, 0);
      updatePanel();
    },
  },
});

resize();
loadScene(scene);
fittedScale = camera.scale;
if (params.has('debug')) setPanel(true);
requestAnimationFrame((now) => {
  lastFrame = now;
  frame(now);
});
