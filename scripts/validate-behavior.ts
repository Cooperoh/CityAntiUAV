import { mkdir, writeFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { DEFAULT_CONFIG, FIXED_DT, WORLD_BOUNDS, type Drone, type SimulationSnapshot, type WorldConfig } from '../src/sim/types';

const args = process.argv.slice(2);
const option = (name: string, fallback: string) => args.find(arg => arg.startsWith(`--${name}=`))?.slice(name.length + 3) ?? fallback;
const enginePath = resolve(option('engine', 'src/sim/simulation.ts'));
const { Simulation } = await import(pathToFileURL(enginePath).href) as {
  Simulation: new (config: WorldConfig) => { step(dt?: number): void; snapshot(): SimulationSnapshot };
};
const seeds = option('seeds', '42,7,2026,1,99').split(',').map(Number);
const seconds = Number(option('seconds', '600'));
const patrolSeconds = Number(option('patrol-seconds', '240'));
if (seeds.some(seed => !Number.isInteger(seed)) || !Number.isFinite(seconds) || seconds <= 0 || !Number.isFinite(patrolSeconds) || patrolSeconds <= 0) {
  throw new Error('Usage: tsx scripts/validate-behavior.ts [--engine=src/sim/simulation.ts] [--output=output/behavior.json] [--seeds=42,7,2026,1,99] [--seconds=600] [--patrol-seconds=240]');
}

const edgeDistance = (d: Drone) => Math.min(d.position.x - WORLD_BOUNDS.minX, WORLD_BOUNDS.maxX - d.position.x, d.position.y - WORLD_BOUNDS.minY, WORLD_BOUNDS.maxY - d.position.y);
// A cell is one 64 m city block; ground projection makes altitude irrelevant to coverage.
const blockOf = (d: Drone) => `${Math.floor((d.position.x + 256) / 64)},${Math.floor((d.position.y + 256) / 64)}`;
const rounded = (value: number) => Number(value.toFixed(4));

function run(seed: number, redCount: number, limit: number) {
  const config = { ...DEFAULT_CONFIG, seed, redCount };
  const sim = new Simulation(config);
  let state = sim.snapshot();
  let previousTime = state.time;
  let previous = new Map(state.drones.filter(d => d.team === 'blue').map(d => [d.id, d]));
  let activeBlueSeconds = 0;
  let edgeBlueSeconds = 0;
  let transitions = 0;
  const transitionExamples: { id: string; time: number; from: string; to: string; missionPreserved: boolean }[] = [];
  let maxIdleSeconds = 0;
  const behaviorSeconds: Record<string, number> = { patrol: 0, evade: 0, recover: 0, unreported: 0 };
  const coverage = new Map<string, Set<string>>();
  const idle = new Map<string, number>();
  for (const drone of previous.values()) coverage.set(drone.id, new Set([blockOf(drone)]));
  for (let step = 0; step < Math.ceil(limit / FIXED_DT) && !state.finished; step++) {
    sim.step(FIXED_DT);
    if (step % 6 !== 5) continue;
    state = sim.snapshot();
    const elapsed = state.time - previousTime;
    for (const drone of state.drones.filter(d => d.team === 'blue')) {
      const before = previous.get(drone.id)!;
      if (!before.active) continue;
      // Left-sample integration at 10 Hz includes the final partial interval before capture.
      activeBlueSeconds += elapsed;
      if (edgeDistance(before) < 40) edgeBlueSeconds += elapsed;
      const behavior = before.behavior ?? 'unreported';
      behaviorSeconds[behavior] = (behaviorSeconds[behavior] ?? 0) + elapsed;
      if (drone.active && before.behavior !== drone.behavior) {
        transitions++;
        if (transitionExamples.length < 20) transitionExamples.push({
          id: drone.id, time: rounded(drone.behaviorSince), from: before.behavior, to: drone.behavior,
          missionPreserved: JSON.stringify(before.missionTarget) === JSON.stringify(drone.missionTarget),
        });
      }
      coverage.get(drone.id)!.add(blockOf(drone));
      const displacement = Math.hypot(drone.position.x - before.position.x, drone.position.y - before.position.y, drone.position.z - before.position.z);
      const stationary = displacement < 0.02 ? (idle.get(drone.id) ?? 0) + elapsed : 0;
      idle.set(drone.id, stationary);
      maxIdleSeconds = Math.max(maxIdleSeconds, stationary);
    }
    previous = new Map(state.drones.filter(d => d.team === 'blue').map(d => [d.id, d]));
    previousTime = state.time;
  }
  state = sim.snapshot();
  const waypointsByDrone = Object.fromEntries(state.drones.filter(d => d.team === 'blue').map(d => [d.id, d.waypointsReached ?? 0]));
  const blocksByDrone = Object.fromEntries([...coverage].map(([id, cells]) => [id, cells.size]));
  return {
    seed, redCount, captured: state.captured, finished: state.finished, collisions: state.collisions,
    simSeconds: rounded(state.time), activeBlueSeconds: rounded(activeBlueSeconds), edgeBlueSeconds: rounded(edgeBlueSeconds),
    edgeRatio: rounded(edgeBlueSeconds / activeBlueSeconds),
    behaviorRatios: Object.fromEntries(Object.entries(behaviorSeconds).map(([key, value]) => [key, rounded(value / activeBlueSeconds)])),
    transitions, transitionsPerBlueMinute: rounded(transitions / (activeBlueSeconds / 60)), transitionExamples,
    totalWaypointsReached: Object.values(waypointsByDrone).reduce((sum, count) => sum + count, 0), waypointsByDrone,
    minBlocksVisited: Math.min(...Object.values(blocksByDrone)), blocksByDrone, maxIdleSeconds: rounded(maxIdleSeconds),
  };
}

const pursuit = [];
for (const seed of seeds) {
  const row = run(seed, DEFAULT_CONFIG.redCount, seconds);
  pursuit.push(row);
  console.log(JSON.stringify({ scenario: 'pursuit', ...row }));
}
const patrol = run(seeds[0]!, 0, patrolSeconds);
console.log(JSON.stringify({ scenario: 'patrol', ...patrol }));
const totals = pursuit.reduce((sum, row) => ({ activeBlueSeconds: sum.activeBlueSeconds + row.activeBlueSeconds, edgeBlueSeconds: sum.edgeBlueSeconds + row.edgeBlueSeconds }), { activeBlueSeconds: 0, edgeBlueSeconds: 0 });
const report = {
  engine: enginePath,
  methodology: '60 Hz integration, 10 Hz left-sample inspection; edge = ground distance to world side boundary < 40 m; ratio denominator = summed surviving blue-drone time. Coverage = distinct 64 m grid cells in ground projection. Transitions exclude capture. Baseline unreported means no behavior fields existed.',
  pursuit,
  aggregate: { activeBlueSeconds: rounded(totals.activeBlueSeconds), edgeBlueSeconds: rounded(totals.edgeBlueSeconds), edgeRatio: rounded(totals.edgeBlueSeconds / totals.activeBlueSeconds) },
  patrol,
};
console.log(JSON.stringify({ aggregate: report.aggregate }));
const output = option('output', '');
if (output) {
  const outputPath = resolve(output);
  await mkdir(dirname(outputPath), { recursive: true });
  await writeFile(outputPath, `${JSON.stringify(report, null, 2)}\n`, 'utf8');
}
// Baseline can run intentionally without missions, but still must preserve physical validity and complete pursuit.
const missionEnabled = patrol.behaviorRatios.unreported === 0;
const pursuitFailed = pursuit.some(row => !row.finished || row.captured !== DEFAULT_CONFIG.blueCount || row.collisions !== 0 || (missionEnabled && row.maxIdleSeconds > 15));
const patrolFailed = patrol.collisions !== 0 || (missionEnabled && (
  Object.values(patrol.waypointsByDrone).some(count => count < 2) || patrol.minBlocksVisited < 5 ||
  patrol.edgeRatio >= 0.1 || patrol.maxIdleSeconds > 15
));
if (pursuitFailed || patrolFailed) {
  console.error(JSON.stringify({ validationFailed: true, pursuitFailed, patrolFailed }));
  process.exitCode = 1;
}
