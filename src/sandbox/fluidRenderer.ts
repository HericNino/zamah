import type { Fluid } from '../engine/index.ts';
import type { Camera } from './camera.ts';

/**
 * Draws the liquid. There are two implementations behind one interface:
 *
 * - WebGL2: every particle is splatted as a soft blob into an off-screen texture, and a second
 *   pass turns the summed field into a surface by thresholding it ("metaballs"). This is what
 *   makes 3,000 discs read as one body of water.
 * - Canvas 2D: plain translucent discs. Used when WebGL2 is unavailable, when the context is
 *   lost, or when the page is opened with `?fluid=2d`.
 *
 * The simulation itself always runs on the CPU, so both paths show exactly the same physics.
 */
export interface FluidRenderer {
  readonly kind: 'webgl2' | 'canvas2d';
  resize(width: number, height: number, dpr: number): void;
  draw(fluid: Fluid, camera: Camera, alpha: number): void;
}

const SPLAT_VERTEX = `#version 300 es
in vec3 a_particle; // x, y in world space, speed in 0..1
uniform vec2 u_center;
uniform vec2 u_scale;
uniform float u_size;
out float v_speed;
void main() {
  gl_Position = vec4((a_particle.xy - u_center) * u_scale, 0.0, 1.0);
  gl_PointSize = u_size;
  v_speed = a_particle.z;
}`;

const SPLAT_FRAGMENT = `#version 300 es
precision mediump float;
in float v_speed;
out vec4 color;
void main() {
  vec2 d = gl_PointCoord * 2.0 - 1.0;
  float r2 = dot(d, d);
  if (r2 > 1.0) discard;
  float w = (1.0 - r2) * (1.0 - r2);
  // Red accumulates density, green accumulates density-weighted speed.
  color = vec4(0.5 * w, 0.5 * w * v_speed, 0.0, 1.0);
}`;

const RESOLVE_VERTEX = `#version 300 es
out vec2 v_uv;
void main() {
  // One oversized triangle covering the whole screen.
  vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
  v_uv = p;
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}`;

const RESOLVE_FRAGMENT = `#version 300 es
precision mediump float;
in vec2 v_uv;
uniform sampler2D u_field;
out vec4 color;
void main() {
  vec4 field = texture(u_field, v_uv);
  float density = field.r;
  float inside = smoothstep(0.26, 0.34, density);
  if (inside <= 0.0) discard;
  float speed = clamp(field.g / max(field.r, 0.001), 0.0, 1.0);
  // A lighter rim near the surface, darker in the body of the liquid.
  float rim = 1.0 - smoothstep(0.34, 0.8, density);
  vec3 deep = vec3(0.07, 0.36, 0.78);
  vec3 shallow = vec3(0.30, 0.72, 0.98);
  vec3 foam = vec3(0.88, 0.97, 1.0);
  vec3 rgb = mix(deep, shallow, rim * 0.85);
  rgb = mix(rgb, foam, smoothstep(0.7, 1.0, speed) * 0.5);
  float alpha = inside * 0.88;
  color = vec4(rgb * alpha, alpha);
}`;

function compile(gl: WebGL2RenderingContext, vertex: string, fragment: string): WebGLProgram {
  const program = gl.createProgram();
  for (const [type, source] of [
    [gl.VERTEX_SHADER, vertex],
    [gl.FRAGMENT_SHADER, fragment],
  ] as const) {
    const shader = gl.createShader(type)!;
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(shader) ?? 'shader compile failed');
    }
    gl.attachShader(program, shader);
  }
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program) ?? 'program link failed');
  }
  return program;
}

class WebGLFluidRenderer implements FluidRenderer {
  readonly kind = 'webgl2';
  private readonly gl: WebGL2RenderingContext;
  private readonly splat: WebGLProgram;
  private readonly resolve: WebGLProgram;
  private readonly buffer: WebGLBuffer;
  private readonly vao: WebGLVertexArrayObject;
  private readonly emptyVao: WebGLVertexArrayObject;
  private readonly framebuffer: WebGLFramebuffer;
  private readonly texture: WebGLTexture;
  private data = new Float32Array(3 * 1024);
  private fieldWidth = 1;
  private fieldHeight = 1;
  private dpr = 1;
  private readonly maxPointSize: number;

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl;
    this.splat = compile(gl, SPLAT_VERTEX, SPLAT_FRAGMENT);
    this.resolve = compile(gl, RESOLVE_VERTEX, RESOLVE_FRAGMENT);
    this.buffer = gl.createBuffer();
    this.vao = gl.createVertexArray();
    this.emptyVao = gl.createVertexArray();
    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    const location = gl.getAttribLocation(this.splat, 'a_particle');
    gl.enableVertexAttribArray(location);
    gl.vertexAttribPointer(location, 3, gl.FLOAT, false, 0, 0);
    gl.bindVertexArray(null);

    this.texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    this.framebuffer = gl.createFramebuffer();
    this.maxPointSize = (gl.getParameter(gl.ALIASED_POINT_SIZE_RANGE) as Float32Array)[1];
  }

  resize(width: number, height: number, dpr: number): void {
    const gl = this.gl;
    this.dpr = dpr;
    const canvas = gl.canvas as HTMLCanvasElement;
    canvas.width = Math.max(1, Math.round(width * dpr));
    canvas.height = Math.max(1, Math.round(height * dpr));
    // The field is blurry by nature, so half resolution is plenty and four times cheaper.
    this.fieldWidth = Math.max(1, canvas.width >> 1);
    this.fieldHeight = Math.max(1, canvas.height >> 1);
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, this.fieldWidth, this.fieldHeight, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.framebuffer);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.texture, 0);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  draw(fluid: Fluid, camera: Camera, alpha: number): void {
    const gl = this.gl;
    const canvas = gl.canvas as HTMLCanvasElement;
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    const count = fluid.count;
    if (count === 0) return;

    if (this.data.length < 3 * count) this.data = new Float32Array(3 * count * 2);
    const data = this.data;
    const { x, y, ox, oy, vx, vy } = fluid;
    for (let i = 0; i < count; i++) {
      data[3 * i] = ox[i] + (x[i] - ox[i]) * alpha;
      data[3 * i + 1] = oy[i] + (y[i] - oy[i]) * alpha;
      data[3 * i + 2] = Math.min(1, Math.sqrt(vx[i] * vx[i] + vy[i] * vy[i]) / 12);
    }

    // Pass 1: accumulate the density field.
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.framebuffer);
    gl.viewport(0, 0, this.fieldWidth, this.fieldHeight);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.ONE, gl.ONE);
    gl.useProgram(this.splat);
    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, data.subarray(0, 3 * count), gl.DYNAMIC_DRAW);
    gl.uniform2f(gl.getUniformLocation(this.splat, 'u_center'), camera.x, camera.y);
    gl.uniform2f(
      gl.getUniformLocation(this.splat, 'u_scale'),
      (2 * camera.scale) / camera.width,
      (2 * camera.scale) / camera.height,
    );
    const size = Math.min(this.maxPointSize, 3.3 * fluid.spacing * camera.scale * this.dpr * 0.5);
    gl.uniform1f(gl.getUniformLocation(this.splat, 'u_size'), size);
    gl.drawArrays(gl.POINTS, 0, count);

    // Pass 2: threshold the field into a surface.
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    gl.useProgram(this.resolve);
    gl.bindVertexArray(this.emptyVao);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.uniform1i(gl.getUniformLocation(this.resolve, 'u_field'), 0);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.bindVertexArray(null);
  }
}

class CanvasFluidRenderer implements FluidRenderer {
  readonly kind = 'canvas2d';
  private readonly ctx: CanvasRenderingContext2D;
  private dpr = 1;

  constructor(canvas: HTMLCanvasElement) {
    this.ctx = canvas.getContext('2d')!;
  }

  resize(width: number, height: number, dpr: number): void {
    this.dpr = dpr;
    this.ctx.canvas.width = Math.max(1, Math.round(width * dpr));
    this.ctx.canvas.height = Math.max(1, Math.round(height * dpr));
  }

  draw(fluid: Fluid, camera: Camera, alpha: number): void {
    const ctx = this.ctx;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    const count = fluid.count;
    if (count === 0) return;
    camera.apply(ctx, this.dpr);
    const { x, y, ox, oy, vx, vy } = fluid;
    const radius = 0.62 * fluid.spacing;
    // Three passes, slow to fast, so each needs only one path and one fill.
    const colors = ['rgba(38, 122, 222, 0.8)', 'rgba(84, 176, 246, 0.8)', 'rgba(205, 238, 255, 0.85)'];
    for (let band = 0; band < 3; band++) {
      ctx.beginPath();
      for (let i = 0; i < count; i++) {
        const speed2 = vx[i] * vx[i] + vy[i] * vy[i];
        const b = speed2 < 6 ? 0 : speed2 < 30 ? 1 : 2;
        if (b !== band) continue;
        const px = ox[i] + (x[i] - ox[i]) * alpha;
        const py = oy[i] + (y[i] - oy[i]) * alpha;
        ctx.moveTo(px + radius, py);
        ctx.arc(px, py, radius, 0, 6.2832);
      }
      ctx.fillStyle = colors[band];
      ctx.fill();
    }
  }
}

/**
 * Creates a canvas inside `host` (behind everything else) and the best renderer for it.
 * A canvas can only ever have one kind of context, so a failed WebGL2 attempt throws its
 * canvas away and starts over with a fresh one for Canvas 2D. When the WebGL2 context is lost
 * later, `onLost` is called so the caller can do the same.
 */
export function createFluidRenderer(host: HTMLElement, force2d: boolean, onLost: () => void): FluidRenderer {
  const mount = (): HTMLCanvasElement => {
    host.querySelector('canvas.fluid')?.remove();
    const canvas = document.createElement('canvas');
    canvas.className = 'fluid';
    canvas.setAttribute('aria-hidden', 'true');
    host.prepend(canvas);
    return canvas;
  };
  if (!force2d) {
    const canvas = mount();
    try {
      const gl = canvas.getContext('webgl2', { alpha: true, premultipliedAlpha: true, antialias: false });
      if (gl) {
        const renderer = new WebGLFluidRenderer(gl);
        canvas.addEventListener('webglcontextlost', (event) => {
          event.preventDefault();
          onLost();
        });
        return renderer;
      }
    } catch (error) {
      console.warn('WebGL2 fluid renderer unavailable, using Canvas 2D instead.', error);
    }
  }
  return new CanvasFluidRenderer(mount());
}
