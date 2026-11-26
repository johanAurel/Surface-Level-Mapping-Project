(function (root) {
  function clamp(value, low, high) {
    return Math.max(low, Math.min(high, value));
  }

  function sample(points, limit) {
    const source = Array.isArray(points) ? points : [];
    const cap = Math.max(1, Math.floor(limit || 1));
    if (source.length <= cap) return source.slice();
    if (cap === 1) return [source[source.length - 1]];
    return Array.from({ length: cap }, (_, index) =>
      source[Math.round((index * (source.length - 1)) / (cap - 1))]);
  }

  function project(point, options) {
    const width = Number(options.width) || 800;
    const height = Number(options.height) || 500;
    const xLen = Number(options.xLen) || 1;
    const yLen = Number(options.yLen) || 1;
    const zoom = Number(options.zoom) || 1;
    const rotationX = Number(options.rotationX) || 0;
    const rotationY = Number(options.rotationY) || 0;
    const zMid = Number(options.zMid) || 0;
    const zScale = Number(options.zScale) || 1;
    const x = Number(point.x_m) - xLen / 2;
    const y = Number(point.y_m) - yLen / 2;
    const z = (Number(point.depth_mm) - zMid) * zScale;

    const cosYaw = Math.cos(rotationY);
    const sinYaw = Math.sin(rotationY);
    const yawX = x * cosYaw - y * sinYaw;
    const yawY = x * sinYaw + y * cosYaw;

    const cosPitch = Math.cos(rotationX);
    const sinPitch = Math.sin(rotationX);
    const sceneY = yawY * cosPitch - z * sinPitch;
    const sceneZ = yawY * sinPitch + z * cosPitch;

    const distance = Math.max(xLen, yLen) * 2.4;
    const perspective = clamp(distance / Math.max(distance + sceneY, distance * 0.35), 0.5, 2.2);
    const scale = Math.min(width / (xLen * 1.55), height / (Math.max(xLen, yLen) * 1.25)) * zoom;

    return {
      x: width / 2 + yawX * scale * perspective,
      y: height * 0.62 - sceneZ * scale * perspective,
      depth: sceneY,
      perspective
    };
  }

  function prepare(points, options) {
    const limit = Number(options.limit) || 2500;
    return sample(points, limit)
      .map((point) => ({ point, ...project(point, options) }))
      .sort((left, right) => right.depth - left.depth);
  }

  const api = { sample, project, prepare };
  root.Figure3D = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})(globalThis);
