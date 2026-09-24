/** Run with: pnpm exec tsx scripts/diagnose-bounds.ts --seeds=42,7,2026 */
import { Simulation } from '../src/sim/simulation';
import {
  DEFAULT_CONFIG, DRONE_RADIUS, FIXED_DT, HORIZONTAL_BOUNDARY_MARGIN, WORLD_BOUNDS,
  type Team, type Vec3,
} from '../src/sim/types';

const seedsArg = process.argv.slice(2).find(value => value.startsWith('--seeds='));
const seeds = (seedsArg?.slice(8) ?? '42,7,2026').split(',').map(Number);
if (seeds.some(seed => !Number.isInteger(seed))) throw new Error('Seeds must be comma-separated integers');
const EPSILON = 1e-6;
const SECONDS_LIMIT = 600;
type Source = 'position' | 'path' | 'trail';
interface Statistics {
  team: Team; source: Source; points: number;
  maxAbsX: number; maxAbsY: number; minZ: number; maxZ: number;
  minHorizontalCenterMargin: number; minHorizontalSphereMargin: number; minHorizontalEnvelopeMargin: number;
  outsideWorld: number; outsideCenterEnvelope: number;
  closest: { id: string; time: number; point: Vec3 } | null;
}

function createStatistics(team: Team, source: Source): Statistics {
  return { team, source, points: 0, maxAbsX: 0, maxAbsY: 0, minZ: Infinity, maxZ: -Infinity,
    minHorizontalCenterMargin: Infinity, minHorizontalSphereMargin: Infinity, minHorizontalEnvelopeMargin: Infinity,
    outsideWorld: 0, outsideCenterEnvelope: 0, closest: null };
}

function record(stats: Statistics, point: Vec3, id: string, time: number): void {
  if (![point.x, point.y, point.z].every(Number.isFinite)) throw new Error(`${id} has a non-finite ${stats.source}`);
  stats.points++;
  stats.maxAbsX = Math.max(stats.maxAbsX, Math.abs(point.x));
  stats.maxAbsY = Math.max(stats.maxAbsY, Math.abs(point.y));
  stats.minZ = Math.min(stats.minZ, point.z);
  stats.maxZ = Math.max(stats.maxZ, point.z);
  const horizontalMargin = Math.min(point.x - WORLD_BOUNDS.minX, WORLD_BOUNDS.maxX - point.x,
    point.y - WORLD_BOUNDS.minY, WORLD_BOUNDS.maxY - point.y);
  if (horizontalMargin < stats.minHorizontalCenterMargin) {
    stats.minHorizontalCenterMargin = horizontalMargin;
    stats.minHorizontalSphereMargin = horizontalMargin - DRONE_RADIUS;
    stats.minHorizontalEnvelopeMargin = horizontalMargin - HORIZONTAL_BOUNDARY_MARGIN;
    stats.closest = { id, time, point: { ...point } };
  }
  if (horizontalMargin < -EPSILON || point.z < WORLD_BOUNDS.minZ - EPSILON || point.z > WORLD_BOUNDS.maxZ + EPSILON) stats.outsideWorld++;
  if (horizontalMargin < HORIZONTAL_BOUNDARY_MARGIN - EPSILON || point.z < WORLD_BOUNDS.minZ - EPSILON || point.z > WORLD_BOUNDS.maxZ - DRONE_RADIUS + EPSILON) stats.outsideCenterEnvelope++;
}

console.log(JSON.stringify({ world: WORLD_BOUNDS, radius: DRONE_RADIUS, horizontalBoundaryMargin: HORIZONTAL_BOUNDARY_MARGIN, epsilonMeters: EPSILON, sampleHz: 1 / FIXED_DT, maxSimulationSeconds: SECONDS_LIMIT }));
for (const seed of seeds) {
  const simulation = new Simulation({ ...DEFAULT_CONFIG, seed });
  const statistics = (['red', 'blue'] as const).flatMap(team => (['position', 'path', 'trail'] as const).map(source => createStatistics(team, source)));
  const lookup = new Map(statistics.map(stats => [`${stats.team}/${stats.source}`, stats]));
  let state = simulation.snapshot();
  let steps = 0;
  while (true) {
    for (const drone of state.drones) {
      record(lookup.get(`${drone.team}/position`)!, drone.position, drone.id, state.time);
      for (const point of drone.path) record(lookup.get(`${drone.team}/path`)!, point, drone.id, state.time);
      for (const point of drone.trail) record(lookup.get(`${drone.team}/trail`)!, point, drone.id, state.time);
    }
    if (state.finished || steps >= SECONDS_LIMIT / FIXED_DT) break;
    simulation.step(FIXED_DT);
    steps++;
    state = simulation.snapshot();
  }
  console.log(JSON.stringify({ seed, steps, simulatedSeconds: state.time, captured: state.captured, collisions: state.collisions, finished: state.finished, statistics }));
  if (!state.finished || statistics.some(stats => stats.outsideWorld > 0 || stats.outsideCenterEnvelope > 0)) process.exitCode = 1;
}
