import { Simulation } from '../src/sim/simulation';
import { DEFAULT_CONFIG, DRONE_RADIUS, FIXED_DT, HORIZONTAL_BOUNDARY_MARGIN, WORLD_BOUNDS, type SimulationSnapshot, type Vec3 } from '../src/sim/types';

const args = process.argv.slice(2);
function option(name: string, fallback: string): string {
  const value = args.find(arg => arg.startsWith(`--${name}=`));
  return value ? value.slice(name.length + 3) : fallback;
}
const seeds = option('seeds', '42,7,2026,1,99').split(',').map(Number);
const seconds = Number(option('seconds', '600'));
if (seeds.some(seed => !Number.isInteger(seed)) || !Number.isFinite(seconds) || seconds <= 0) {
  throw new Error('Usage: pnpm validate:sim --seeds=42,7,2026 --seconds=600');
}

function illegalState(state: SimulationSnapshot): string | null {
  const withinEnvelope = (point: Vec3) => [point.x, point.y, point.z].every(Number.isFinite)
    && point.x >= WORLD_BOUNDS.minX + HORIZONTAL_BOUNDARY_MARGIN - 1e-6
    && point.x <= WORLD_BOUNDS.maxX - HORIZONTAL_BOUNDARY_MARGIN + 1e-6
    && point.y >= WORLD_BOUNDS.minY + HORIZONTAL_BOUNDARY_MARGIN - 1e-6
    && point.y <= WORLD_BOUNDS.maxY - HORIZONTAL_BOUNDARY_MARGIN + 1e-6
    && point.z >= WORLD_BOUNDS.minZ - 1e-6
    && point.z <= WORLD_BOUNDS.maxZ - DRONE_RADIUS + 1e-6;
  for (const d of state.drones) {
    const { x, y, z } = d.position;
    if (![x, y, z, d.velocity.x, d.velocity.y, d.velocity.z, d.acceleration.x, d.acceleration.y, d.acceleration.z].every(Number.isFinite)) return `${d.id}: non-finite state`;
    if (!withinEnvelope(d.position)) return `${d.id}: position outside flight envelope`;
    if (!d.path.every(withinEnvelope)) return `${d.id}: path outside flight envelope`;
    if (!d.trail.every(withinEnvelope)) return `${d.id}: trail outside flight envelope`;
    const maxSpeed = d.team === 'red' ? DEFAULT_CONFIG.redMaxSpeed : DEFAULT_CONFIG.blueMaxSpeed;
    const maxAccel = d.team === 'red' ? DEFAULT_CONFIG.redMaxAccel : DEFAULT_CONFIG.blueMaxAccel;
    if (Math.hypot(d.velocity.x, d.velocity.y, d.velocity.z) > maxSpeed + 1e-5) return `${d.id}: speed limit exceeded`;
    if (Math.hypot(d.acceleration.x, d.acceleration.y, d.acceleration.z) > maxAccel + 1e-5) return `${d.id}: acceleration limit exceeded`;
    const r = DRONE_RADIUS - 1e-5;
    if (state.buildings.some(b => x > b.x - b.width / 2 - r && x < b.x + b.width / 2 + r && y > b.y - b.depth / 2 - r && y < b.y + b.depth / 2 + r && z > -r && z < b.height + r)) return `${d.id}: overlaps building`;
  }
  return null;
}

console.log(`City pursuit validation | fixed dt=${FIXED_DT} s | max ${seconds} s per seed`);
const rows: Record<string, number | string>[] = [];
let failed = false;
for (const seed of seeds) {
  const started = performance.now();
  const sim = new Simulation({ ...DEFAULT_CONFIG, seed });
  let state = sim.snapshot();
  let invalid = illegalState(state);
  let steps = 0;
  while (!state.finished && steps < seconds / FIXED_DT && !invalid) {
    sim.step(FIXED_DT);
    steps++;
    // Integrate at 60 Hz; independently inspect at 10 Hz to keep validation economical.
    if (steps % 6 === 0) {
      state = sim.snapshot();
      invalid = illegalState(state);
    }
  }
  state = sim.snapshot();
  const milliseconds = performance.now() - started;
  const captures = state.events.filter(event => event.kind === 'capture').length;
  const passed = state.finished && state.captured === DEFAULT_CONFIG.blueCount && captures === DEFAULT_CONFIG.blueCount && state.collisions === 0 && !invalid;
  failed ||= !passed;
  rows.push({ seed, result: passed ? 'PASS' : 'FAIL', captured: `${state.captured}/${DEFAULT_CONFIG.blueCount}`, collisions: state.collisions, simSeconds: Number(state.time.toFixed(2)), wallSeconds: Number((milliseconds / 1000).toFixed(2)), steps, reason: invalid ?? (passed ? '' : 'incomplete capture or collision') });
  console.log(JSON.stringify(rows.at(-1)));
}
console.table(rows);
if (failed) process.exitCode = 1;
