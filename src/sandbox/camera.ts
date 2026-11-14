/** Maps between world coordinates (metres, y up) and screen coordinates (CSS pixels, y down). */
export class Camera {
  /** World point at the centre of the screen. */
  x = 0;
  y = 0;
  /** CSS pixels per metre. */
  scale = 40;
  width = 1;
  height = 1;

  resize(width: number, height: number): void {
    this.width = width;
    this.height = height;
  }

  /** Centres the camera on a world rectangle and zooms so that all of it is visible. */
  fit(view: { minX: number; minY: number; maxX: number; maxY: number }, padding = 12): void {
    const w = view.maxX - view.minX;
    const h = view.maxY - view.minY;
    this.scale = Math.max(4, Math.min((this.width - 2 * padding) / w, (this.height - 2 * padding) / h));
    this.x = (view.minX + view.maxX) / 2;
    // Keep the floor near the bottom when the screen is taller than the scene.
    const visibleHeight = this.height / this.scale;
    this.y = Math.max((view.minY + view.maxY) / 2, view.minY + visibleHeight / 2 - padding / this.scale);
  }

  toWorldX(screenX: number): number {
    return this.x + (screenX - this.width / 2) / this.scale;
  }

  toWorldY(screenY: number): number {
    return this.y - (screenY - this.height / 2) / this.scale;
  }

  /** Zooms by `factor` while keeping the world point under the cursor fixed. */
  zoomAt(screenX: number, screenY: number, factor: number): void {
    const wx = this.toWorldX(screenX);
    const wy = this.toWorldY(screenY);
    this.scale = Math.max(6, Math.min(400, this.scale * factor));
    this.x = wx - (screenX - this.width / 2) / this.scale;
    this.y = wy + (screenY - this.height / 2) / this.scale;
  }

  /** Sets a canvas transform so that drawing commands can use world coordinates directly. */
  apply(ctx: CanvasRenderingContext2D, dpr: number): void {
    const s = this.scale * dpr;
    ctx.setTransform(s, 0, 0, -s, dpr * (this.width / 2) - this.x * s, dpr * (this.height / 2) + this.y * s);
  }
}
