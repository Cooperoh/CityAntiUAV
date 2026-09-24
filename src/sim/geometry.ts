import { DRONE_RADIUS, HORIZONTAL_BOUNDARY_MARGIN, WORLD_BOUNDS, type Building, type Vec3 } from './types';

export const cloneVec = (v: Vec3): Vec3 => ({ x: v.x, y: v.y, z: v.z });
export const add = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
export const subtract = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
export const scale = (a: Vec3, s: number): Vec3 => ({ x: a.x * s, y: a.y * s, z: a.z * s });
export const magnitude = (v: Vec3): number => Math.hypot(v.x, v.y, v.z);
export const distance = (a: Vec3, b: Vec3): number => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
export const limit = (v: Vec3, max: number): Vec3 => {
  const length = magnitude(v);
  return length > max && length > 0 ? scale(v, max / length) : cloneVec(v);
};
export const clampToBounds = (p: Vec3): Vec3 => ({
  x: Math.max(WORLD_BOUNDS.minX + HORIZONTAL_BOUNDARY_MARGIN, Math.min(WORLD_BOUNDS.maxX - HORIZONTAL_BOUNDARY_MARGIN, p.x)),
  y: Math.max(WORLD_BOUNDS.minY + HORIZONTAL_BOUNDARY_MARGIN, Math.min(WORLD_BOUNDS.maxY - HORIZONTAL_BOUNDARY_MARGIN, p.y)),
  z: Math.max(WORLD_BOUNDS.minZ, Math.min(WORLD_BOUNDS.maxZ - DRONE_RADIUS, p.z)),
});
export function insideBounds(p: Vec3): boolean {
  const q = clampToBounds(p);
  return Math.abs(q.x - p.x) + Math.abs(q.y - p.y) + Math.abs(q.z - p.z) < 1e-7;
}

/** A conservative swept-sphere test, using an expanded building AABB. */
function intersectsBuilding(a: Vec3, b: Vec3, box: Building, radius: number): boolean {
  const min = [box.x - box.width / 2 - radius, box.y - box.depth / 2 - radius, -radius];
  const max = [box.x + box.width / 2 + radius, box.y + box.depth / 2 + radius, box.height + radius];
  const start = [a.x, a.y, a.z];
  const end = [b.x, b.y, b.z];
  let near = 0;
  let far = 1;
  for (let axis = 0; axis < 3; axis++) {
    const delta = end[axis]! - start[axis]!;
    if (Math.abs(delta) < 1e-10) {
      if (start[axis]! < min[axis]! || start[axis]! > max[axis]!) return false;
    } else {
      let t0 = (min[axis]! - start[axis]!) / delta;
      let t1 = (max[axis]! - start[axis]!) / delta;
      if (t0 > t1) [t0, t1] = [t1, t0];
      near = Math.max(near, t0);
      far = Math.min(far, t1);
      if (near > far) return false;
    }
  }
  return far >= 0 && near <= 1;
}

export function segmentClear(a: Vec3, b: Vec3, buildings: Building[], radius = DRONE_RADIUS): boolean {
  if (!insideBounds(a) || !insideBounds(b)) return false;
  const minX = Math.min(a.x, b.x) - radius;
  const maxX = Math.max(a.x, b.x) + radius;
  const minY = Math.min(a.y, b.y) - radius;
  const maxY = Math.max(a.y, b.y) + radius;
  const minZ = Math.min(a.z, b.z) - radius;
  for (const building of buildings) {
    if (minZ > building.height || minX > building.x + building.width / 2 || maxX < building.x - building.width / 2 ||
      minY > building.y + building.depth / 2 || maxY < building.y - building.depth / 2) continue;
    if (intersectsBuilding(a, b, building, radius)) return false;
  }
  return true;
}

export function isFree(p: Vec3, buildings: Building[], radius = DRONE_RADIUS): boolean {
  return segmentClear(p, p, buildings, radius);
}

/** Constant net acceleration over one step; the speed cap also limits effective acceleration. */
export function integrateMotion(position: Vec3, velocity: Vec3, acceleration: Vec3, dt: number, maxSpeed: number, maxAccel: number) {
  const requested = limit(acceleration, maxAccel);
  const nextVelocity = limit(add(velocity, scale(requested, dt)), maxSpeed);
  const appliedAcceleration = dt > 0 ? scale(subtract(nextVelocity, velocity), 1 / dt) : { x: 0, y: 0, z: 0 };
  return {
    position: add(position, scale(add(velocity, nextVelocity), dt / 2)),
    velocity: nextVelocity,
    acceleration: appliedAcceleration,
  };
}
