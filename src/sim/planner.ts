import { DRONE_RADIUS, type Building, type Vec3 } from './types';
import { cloneVec, distance, isFree, segmentClear } from './geometry';

const NX = 31;
const NY = 31;
const NZ = 10;
const COUNT = NX * NY * NZ;
const CLEARANCE = DRONE_RADIUS + 2;
const toId = (x: number, y: number, z: number) => x + NX * (y + NY * z);
const pointOf = (id: number): Vec3 => ({ x: -290 + (id % NX) * 19.333333333333332, y: -290 + (Math.floor(id / NX) % NY) * 19.333333333333332, z: 8 + Math.floor(id / (NX * NY)) * 15 });

class MinHeap {
  items: { id: number; score: number }[] = [];
  push(id: number, score: number) {
    const item = { id, score };
    let index = this.items.length;
    this.items.push(item);
    while (index > 0) {
      const parent = (index - 1) >> 1;
      if (this.items[parent]!.score <= score) break;
      this.items[index] = this.items[parent]!;
      index = parent;
    }
    this.items[index] = item;
  }
  pop() {
    const result = this.items[0]!;
    const tail = this.items.pop()!;
    if (this.items.length) {
      let index = 0;
      while (index * 2 + 1 < this.items.length) {
        let child = index * 2 + 1;
        if (child + 1 < this.items.length && this.items[child + 1]!.score < this.items[child]!.score) child++;
        if (this.items[child]!.score >= tail.score) break;
        this.items[index] = this.items[child]!;
        index = child;
      }
      this.items[index] = tail;
    }
    return result.id;
  }
}

/** Six-connected 3D A*, followed by line-of-sight path simplification. */
export class GridPlanner {
  private free = new Uint8Array(COUNT);
  private points = Array.from({ length: COUNT }, (_, id) => pointOf(id));
  private edges = new Map<number, boolean>();
  constructor(private buildings: Building[]) {
    for (let i = 0; i < COUNT; i++) this.free[i] = isFree(this.points[i]!, buildings, CLEARANCE) ? 1 : 0;
  }

  private closestNode(position: Vec3): number {
    const cx = Math.max(0, Math.min(NX - 1, Math.round((position.x + 290) / (580 / 30))));
    const cy = Math.max(0, Math.min(NY - 1, Math.round((position.y + 290) / (580 / 30))));
    const cz = Math.max(0, Math.min(NZ - 1, Math.round((position.z - 8) / 15)));
    for (let radius = 1; radius <= 4; radius++) {
      let best = -1;
      let bestDistance = Infinity;
      for (let z = Math.max(0, cz - radius); z <= Math.min(NZ - 1, cz + radius); z++) {
        for (let y = Math.max(0, cy - radius); y <= Math.min(NY - 1, cy + radius); y++) {
          for (let x = Math.max(0, cx - radius); x <= Math.min(NX - 1, cx + radius); x++) {
            const id = toId(x, y, z);
            if (!this.free[id]) continue;
            const d = distance(position, this.points[id]!);
            if (d < bestDistance && segmentClear(position, this.points[id]!, this.buildings, CLEARANCE)) {
              best = id;
              bestDistance = d;
            }
          }
        }
      }
      if (best !== -1) return best;
    }
    return -1;
  }

  plan(start: Vec3, goal: Vec3): Vec3[] {
    if (segmentClear(start, goal, this.buildings, CLEARANCE)) return [cloneVec(goal)];
    const first = this.closestNode(start);
    const last = this.closestNode(goal);
    if (first < 0 || last < 0) return [];
    const costs = new Float64Array(COUNT).fill(Infinity);
    const parents = new Int32Array(COUNT).fill(-1);
    const closed = new Uint8Array(COUNT);
    const queue = new MinHeap();
    costs[first] = 0;
    queue.push(first, distance(this.points[first]!, this.points[last]!));
    while (queue.items.length) {
      const current = queue.pop();
      if (closed[current]) continue;
      if (current === last) {
        const nodes: Vec3[] = [cloneVec(goal)];
        for (let id = last; id !== -1; id = parents[id]!) nodes.push(cloneVec(this.points[id]!));
        nodes.push(cloneVec(start));
        nodes.reverse();
        const simplified: Vec3[] = [];
        let at = 0;
        while (at < nodes.length - 1) {
          let next = nodes.length - 1;
          while (next > at + 1 && !segmentClear(nodes[at]!, nodes[next]!, this.buildings, CLEARANCE)) next--;
          simplified.push(nodes[next]!);
          at = next;
        }
        return simplified;
      }
      closed[current] = 1;
      const x = current % NX;
      const y = Math.floor(current / NX) % NY;
      const z = Math.floor(current / (NX * NY));
      const neighbors = [x > 0 ? current - 1 : -1, x < NX - 1 ? current + 1 : -1, y > 0 ? current - NX : -1,
        y < NY - 1 ? current + NX : -1, z > 0 ? current - NX * NY : -1, z < NZ - 1 ? current + NX * NY : -1];
      for (const next of neighbors) {
        if (next < 0 || !this.free[next] || closed[next]) continue;
        const key = Math.min(current, next) * COUNT + Math.max(current, next);
        let clear = this.edges.get(key);
        if (clear === undefined) {
          clear = segmentClear(this.points[current]!, this.points[next]!, this.buildings, CLEARANCE);
          this.edges.set(key, clear);
        }
        if (!clear) continue;
        const cost = costs[current]! + distance(this.points[current]!, this.points[next]!);
        if (cost < costs[next]!) {
          costs[next] = cost;
          parents[next] = current;
          queue.push(next, cost + distance(this.points[next]!, this.points[last]!));
        }
      }
    }
    return [];
  }
}
