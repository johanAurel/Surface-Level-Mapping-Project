const assert = require("node:assert/strict");
const test = require("node:test");

const Figure3D = require("../figure3d.js");

const OPTIONS = {
  width: 800,
  height: 500,
  xLen: 10,
  yLen: 10,
  zMid: 0,
  zScale: 2,
  rotationX: -0.4,
  rotationY: 0.6,
  zoom: 1
};

test("sample reduces points evenly without duplicates", () => {
  const points = Array.from({ length: 10 }, (_, index) => ({ id: index }));
  assert.deepEqual(Figure3D.sample(points, 4).map((point) => point.id), [0, 3, 6, 9]);
});

test("project scales with zoom", () => {
  const point = { x_m: 7, y_m: 3, depth_mm: 0.5 };
  const base = Figure3D.project(point, OPTIONS);
  const zoomed = Figure3D.project(point, { ...OPTIONS, zoom: 2 });
  assert.ok(Math.abs(zoomed.x - OPTIONS.width / 2) > Math.abs(base.x - OPTIONS.width / 2));
});

test("far points are reduced by perspective", () => {
  const near = Figure3D.project({ x_m: 5, y_m: 1, depth_mm: 0 }, OPTIONS);
  const far = Figure3D.project({ x_m: 5, y_m: 9, depth_mm: 0 }, OPTIONS);
  assert.ok(far.perspective < near.perspective);
});

test("prepare sorts far points before near points", () => {
  const points = [
    { id: "near", x_m: 5, y_m: 1, depth_mm: 0 },
    { id: "far", x_m: 5, y_m: 9, depth_mm: 0 }
  ];
  const prepared = Figure3D.prepare(points, { ...OPTIONS, limit: 10 });
  assert.equal(prepared[0].point.id, "far");
});
