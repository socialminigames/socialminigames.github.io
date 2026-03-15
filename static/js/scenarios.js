/**
 * SMGLib Scenario Definitions — Doorway, Hallway, Intersection
 * Ported from SMGLib XML configs and run_simulation.py
 *
 * Coordinate system: world units (matching SMGLib's 64x64 grid).
 * For the canvas we normalise to a visible viewport.
 */

var SCENARIOS = {
  /* ─── Doorway ───────────────────────────────────────────── */
  doorway: {
    name: 'Doorway',
    desc: 'Bottleneck through a narrow gap in a vertical wall.',
    // Obstacles are arrays of {x, y, w, h} rectangles
    obstacles: [
      { x: 30, y: 0, w: 1, h: 30 },   // upper wall
      { x: 30, y: 34, w: 1, h: 30 },   // lower wall
    ],
    gapWidth: 4.0,
    // Viewport bounds (what the camera shows)
    view: { xMin: 5, xMax: 58, yMin: 5, yMax: 58 },
    // Default agent setups (start → goal)
    presets: {
      2: [
        { start: { x: 15, y: 31 }, goal: { x: 45, y: 31 }, color: '#3298dc' },
        { start: { x: 45, y: 33 }, goal: { x: 15, y: 33 }, color: '#ff3860' },
      ],
      3: [
        { start: { x: 15, y: 30 }, goal: { x: 45, y: 30 }, color: '#3298dc' },
        { start: { x: 45, y: 34 }, goal: { x: 15, y: 34 }, color: '#ff3860' },
        { start: { x: 15, y: 34 }, goal: { x: 45, y: 34 }, color: '#48c774' },
      ],
      4: [
        { start: { x: 15, y: 30 }, goal: { x: 45, y: 30 }, color: '#3298dc' },
        { start: { x: 45, y: 34 }, goal: { x: 15, y: 34 }, color: '#ff3860' },
        { start: { x: 15, y: 34 }, goal: { x: 45, y: 34 }, color: '#48c774' },
        { start: { x: 45, y: 30 }, goal: { x: 15, y: 30 }, color: '#ffdd57' },
      ],
    },
  },

  /* ─── Hallway ───────────────────────────────────────────── */
  hallway: {
    name: 'Hallway',
    desc: 'Two-way corridor bounded by parallel walls.',
    obstacles: [
      { x: 0, y: 31, w: 64, h: 1 },   // top wall
      { x: 0, y: 36, w: 64, h: 1 },   // bottom wall
    ],
    gapWidth: 4.0,
    view: { xMin: 5, xMax: 58, yMin: 26, yMax: 42 },
    presets: {
      2: [
        { start: { x: 12, y: 33.5 }, goal: { x: 52, y: 33.5 }, color: '#3298dc' },
        { start: { x: 52, y: 33.5 }, goal: { x: 12, y: 33.5 }, color: '#ff3860' },
      ],
      3: [
        { start: { x: 12, y: 33 }, goal: { x: 52, y: 33 }, color: '#3298dc' },
        { start: { x: 52, y: 34 }, goal: { x: 12, y: 34 }, color: '#ff3860' },
        { start: { x: 12, y: 35 }, goal: { x: 52, y: 35 }, color: '#48c774' },
      ],
      4: [
        { start: { x: 12, y: 33 }, goal: { x: 52, y: 33 }, color: '#3298dc' },
        { start: { x: 52, y: 34 }, goal: { x: 12, y: 34 }, color: '#ff3860' },
        { start: { x: 12, y: 35 }, goal: { x: 52, y: 35 }, color: '#48c774' },
        { start: { x: 52, y: 33 }, goal: { x: 12, y: 33 }, color: '#ffdd57' },
      ],
    },
  },

  /* ─── Intersection ──────────────────────────────────────── */
  intersection: {
    name: 'Intersection',
    desc: 'Four corridors meeting at a central open area.',
    obstacles: [
      // Top-left block
      { x: 0, y: 0, w: 26, h: 26 },
      // Top-right block
      { x: 38, y: 0, w: 26, h: 26 },
      // Bottom-left block
      { x: 0, y: 38, w: 26, h: 26 },
      // Bottom-right block
      { x: 38, y: 38, w: 26, h: 26 },
    ],
    gapWidth: 12.0,
    view: { xMin: 5, xMax: 58, yMin: 5, yMax: 58 },
    presets: {
      2: [
        { start: { x: 30, y: 10 }, goal: { x: 30, y: 54 }, color: '#3298dc' },
        { start: { x: 10, y: 34 }, goal: { x: 54, y: 34 }, color: '#ff3860' },
      ],
      3: [
        { start: { x: 30, y: 10 }, goal: { x: 30, y: 54 }, color: '#3298dc' },
        { start: { x: 10, y: 34 }, goal: { x: 54, y: 34 }, color: '#ff3860' },
        { start: { x: 54, y: 30 }, goal: { x: 10, y: 30 }, color: '#48c774' },
      ],
      4: [
        { start: { x: 29, y: 10 }, goal: { x: 29, y: 54 }, color: '#3298dc' },
        { start: { x: 10, y: 35 }, goal: { x: 54, y: 35 }, color: '#ff3860' },
        { start: { x: 35, y: 54 }, goal: { x: 35, y: 10 }, color: '#48c774' },
        { start: { x: 54, y: 29 }, goal: { x: 10, y: 29 }, color: '#ffdd57' },
      ],
    },
  },
};
