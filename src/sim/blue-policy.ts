import { DRONE_RADIUS, HORIZONTAL_BOUNDARY_MARGIN, WORLD_BOUNDS, type DroneBehavior, type Observation, type Vec3 } from './types';
import { add, clampToBounds, cloneVec, distance, isFree, magnitude, scale, segmentClear, subtract } from './geometry';
import type { GridPlanner } from './planner';

/** Distances in metres and durations in simulated seconds. These are policy, not sensor limits. */
export const BLUE_POLICY = {
  threatEnterDistance: 80,
  threatExitDistance: 120,
  safeHoldSeconds: 2,
  minimumEvadeSeconds: 3,
  recoverySeconds: 3,
  waypointRadius: 7,
  missionMinDistance: 112,
  missionMaxDistance: 290,
  missionAltitudes: [24, 32, 40, 48] as readonly number[],
  streetCoordinates: [-192, -128, -64, 0, 64, 128, 192] as readonly number[],
  evadeReplanSeconds: 1,
  patrolReplanSeconds: 4,
  stallCheckSeconds: 5,
  stallDistance: 3,
  stalledChecksBeforeNewMission: 2,
  safetySaturationDistance: 125,
  boundarySoftMargin: 60,
  boundaryPenalty: 115,
  escapeRadii: [48, 76] as readonly number[],
  escapeAltitudeChanges: [-14, 0, 14] as readonly number[],
  escapeBearings: 16,
  minimumEscapeTravel: 20,
  minimumEscapeAltitude: 18,
  maximumEscapeAltitude: 70,
  preferredAltitude: 48,
  predictionSeconds: 1.25,
  planningClearance: DRONE_RADIUS + 2,
  pathCandidateCount: 24,
  missionRevisitRadius: 70,
  missionRevisitPenalty: 0.5,
  safetyWeight: 0.85,
  progressLimit: 45,
  progressWeight: 0.24,
  headingWeight: 8,
  clearPathBonus: 10,
  altitudePenalty: 0.8,
  pathSafetySampleInterval: 16,
  safetyLossPenalty: 1.2,
  pathLengthPenalty: 0.08,
  highPathAltitude: 65,
  highPathPenalty: 0.9,
} as const;

type BlueBehavior = Extract<DroneBehavior, 'patrol' | 'evade' | 'recover'>;
export interface BluePolicyState {
  behavior: BlueBehavior;
  behaviorSince: number;
  missionTarget: Vec3 | null;
  waypointsReached: number;
  safeSince: number | null;
  nextPlan: number;
  nextProgressCheck: number;
  lastProgressPosition: Vec3 | null;
  previousMission: Vec3 | null;
  randomState: number;
  stalledChecks: number;
}
export interface BlueDecision {
  behavior: BlueBehavior;
  behaviorSince: number;
  missionTarget: Vec3 | null;
  waypointsReached: number;
  /** Undefined preserves the existing route and its already consumed waypoints. */
  path?: Vec3[];
}

export function createBluePolicyState(seed: number, id: string): BluePolicyState {
  let hash = seed ^ 0x74c15;
  for (const character of id) hash = Math.imul(hash ^ character.charCodeAt(0), 16777619);
  return { behavior: 'patrol', behaviorSince: 0, missionTarget: null, waypointsReached: 0,
    safeSince: null, nextPlan: 0, nextProgressCheck: 0, lastProgressPosition: null,
    previousMission: null, randomState: hash >>> 0, stalledChecks: 0 };
}

function random(state: BluePolicyState): number {
  state.randomState = (state.randomState + 0x6d2b79f5) >>> 0;
  let value = state.randomState;
  value = Math.imul(value ^ (value >>> 15), value | 1);
  value ^= value + Math.imul(value ^ (value >>> 7), value | 61);
  return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
}

function setBehavior(state: BluePolicyState, behavior: BlueBehavior, time: number): void {
  if (state.behavior === behavior) return;
  state.behavior = behavior;
  state.behaviorSince = time;
  state.nextPlan = time;
  state.safeSince = null;
}

/** Uses local seeded randomness, never a mutable global RNG shared with other drones. */
function chooseMission(observation: Observation, state: BluePolicyState, planner: GridPlanner): Vec3[] {
  const { position } = observation.self;
  const candidates: { point: Vec3; score: number }[] = [];
  const altitude = BLUE_POLICY.missionAltitudes[Math.floor(random(state) * BLUE_POLICY.missionAltitudes.length)]!;
  for (const x of BLUE_POLICY.streetCoordinates) for (const y of BLUE_POLICY.streetCoordinates) {
    const point = { x, y, z: altitude };
    const length = distance(position, point);
    if (length < BLUE_POLICY.missionMinDistance || length > BLUE_POLICY.missionMaxDistance ||
      !isFree(point, observation.buildings, DRONE_RADIUS + 2)) continue;
    // Distributed street intersections, with a weak anti-backtracking bias. No preferred map centre.
    const revisitPenalty = state.previousMission && distance(point, state.previousMission) < BLUE_POLICY.missionRevisitRadius ? BLUE_POLICY.missionRevisitPenalty : 0;
    candidates.push({ point, score: random(state) - revisitPenalty });
  }
  candidates.sort((a, b) => b.score - a.score);
  for (const candidate of candidates) {
    const path = planner.plan(position, candidate.point);
    if (!path.length) continue;
    state.missionTarget = cloneVec(candidate.point);
    return path;
  }
  // No reachable goal is different from arriving at one: wait and retry without credit.
  state.missionTarget = null;
  return [];
}

function escapeRoute(observation: Observation, state: BluePolicyState, planner: GridPlanner): Vec3[] {
  const { self, buildings } = observation;
  const threats = observation.drones.filter((d) => d.active && d.team === 'red');
  const predicted = threats.map((d) => add(d.position, scale(d.velocity, BLUE_POLICY.predictionSeconds)));
  const candidates: { point: Vec3; score: number }[] = [];
  const clearance = DRONE_RADIUS + 2;
  for (let bearing = 0; bearing < BLUE_POLICY.escapeBearings; bearing++) for (const radius of BLUE_POLICY.escapeRadii) {
    const angle = bearing * 2 * Math.PI / BLUE_POLICY.escapeBearings;
    for (const dz of BLUE_POLICY.escapeAltitudeChanges) {
      const point = clampToBounds({ x: self.position.x + Math.cos(angle) * radius,
        y: self.position.y + Math.sin(angle) * radius, z: Math.max(BLUE_POLICY.minimumEscapeAltitude, Math.min(BLUE_POLICY.maximumEscapeAltitude, self.position.z + dz)) });
      if (distance(point, self.position) < BLUE_POLICY.minimumEscapeTravel || !isFree(point, buildings, clearance)) continue;
      const separation = Math.min(...predicted.map((p) => distance(point, p)));
      const room = Math.min(point.x - WORLD_BOUNDS.minX, WORLD_BOUNDS.maxX - point.x,
        point.y - WORLD_BOUNDS.minY, WORLD_BOUNDS.maxY - point.y) - HORIZONTAL_BOUNDARY_MARGIN;
      const edgeCost = BLUE_POLICY.boundaryPenalty * Math.pow(Math.max(0, 1 - room / BLUE_POLICY.boundarySoftMargin), 2);
      const offset = subtract(point, self.position);
      const speed = magnitude(self.velocity);
      const forward = speed > 0.5 ? (offset.x * self.velocity.x + offset.y * self.velocity.y + offset.z * self.velocity.z) / (magnitude(offset) * speed) : 0;
      const progress = state.missionTarget ? distance(self.position, state.missionTarget) - distance(point, state.missionTarget) : 0;
      const clearBonus = segmentClear(self.position, point, buildings, clearance) ? BLUE_POLICY.clearPathBonus : 0;
      // Safety stops earning reward once sufficient: don't endlessly maximize distance from red.
      const score = Math.min(BLUE_POLICY.safetySaturationDistance, separation) * BLUE_POLICY.safetyWeight - edgeCost +
        Math.max(-BLUE_POLICY.progressLimit, Math.min(BLUE_POLICY.progressLimit, progress)) * BLUE_POLICY.progressWeight +
        forward * BLUE_POLICY.headingWeight + clearBonus - Math.max(0, point.z - BLUE_POLICY.preferredAltitude) * BLUE_POLICY.altitudePenalty;
      candidates.push({ point, score });
    }
  }
  candidates.sort((a, b) => b.score - a.score);
  let bestPath: Vec3[] = [];
  let bestScore = -Infinity;
  const currentSafety = Math.min(...predicted.map((p) => distance(self.position, p)));
  for (const candidate of candidates.slice(0, BLUE_POLICY.pathCandidateCount)) {
    const path = planner.plan(self.position, candidate.point);
    if (!path.length) continue;
    // A valid destination is insufficient; the first route segment must be traversable.
    if (!segmentClear(self.position, path[0]!, buildings, BLUE_POLICY.planningClearance)) continue;
    let last = self.position;
    let length = 0;
    let leastSafety = currentSafety;
    let highest = self.position.z;
    for (const point of path) {
      const segmentLength = distance(last, point);
      const samples = Math.max(1, Math.ceil(segmentLength / BLUE_POLICY.pathSafetySampleInterval));
      for (let sample = 1; sample <= samples; sample++) {
        const p = add(last, scale(subtract(point, last), sample / samples));
        leastSafety = Math.min(leastSafety, ...predicted.map((red) => distance(p, red)));
      }
      length += segmentLength;
      highest = Math.max(highest, point.z);
      last = point;
    }
    const score = candidate.score - Math.max(0, currentSafety - leastSafety) * BLUE_POLICY.safetyLossPenalty - length * BLUE_POLICY.pathLengthPenalty - Math.max(0, highest - BLUE_POLICY.highPathAltitude) * BLUE_POLICY.highPathPenalty;
    if (score > bestScore) { bestScore = score; bestPath = path; }
  }
  return bestPath;
}

/** Stateful decisions depend only on the observed world and this drone's own remembered mission. */
export function updateBluePolicy(observation: Observation, state: BluePolicyState, planner: GridPlanner): BlueDecision {
  const { timestamp, self } = observation;
  const nearest = Math.min(Infinity, ...observation.drones.filter((d) => d.active && d.team === 'red').map((d) => distance(self.position, d.position)));
  if (nearest <= BLUE_POLICY.threatEnterDistance) {
    setBehavior(state, 'evade', timestamp);
    state.safeSince = null;
  } else if (state.behavior === 'evade') {
    if (nearest >= BLUE_POLICY.threatExitDistance) state.safeSince ??= timestamp;
    else state.safeSince = null;
    if (state.safeSince !== null && timestamp - state.safeSince >= BLUE_POLICY.safeHoldSeconds &&
      timestamp - state.behaviorSince >= BLUE_POLICY.minimumEvadeSeconds) setBehavior(state, 'recover', timestamp);
  } else if (state.behavior === 'recover' && timestamp - state.behaviorSince >= BLUE_POLICY.recoverySeconds) {
    setBehavior(state, 'patrol', timestamp);
  }
  let path: Vec3[] | undefined;
  if (state.behavior !== 'evade' && state.missionTarget && distance(self.position, state.missionTarget) <= BLUE_POLICY.waypointRadius) {
    state.previousMission = cloneVec(state.missionTarget);
    state.waypointsReached++;
    state.missionTarget = null;
    state.nextPlan = timestamp;
  }
  if (state.missionTarget && !isFree(state.missionTarget, observation.buildings, BLUE_POLICY.planningClearance)) {
    state.missionTarget = null;
    state.nextPlan = timestamp;
  }
  if (!state.missionTarget && timestamp + 1e-9 >= state.nextPlan) {
    path = chooseMission(observation, state, planner);
    state.nextPlan = timestamp;
  }
  if (timestamp >= state.nextProgressCheck) {
    if (state.lastProgressPosition && distance(self.position, state.lastProgressPosition) < BLUE_POLICY.stallDistance) {
      state.nextPlan = timestamp;
      state.stalledChecks++;
      // Reconsider an unreachable mission only after sustained lack of progress.
      // A new destination never counts as a completed waypoint.
      if (state.behavior !== 'evade' && state.stalledChecks >= BLUE_POLICY.stalledChecksBeforeNewMission) {
        state.previousMission = state.missionTarget ? cloneVec(state.missionTarget) : null;
        path = chooseMission(observation, state, planner);
        state.stalledChecks = 0;
      }
    } else state.stalledChecks = 0;
    state.lastProgressPosition = cloneVec(self.position);
    state.nextProgressCheck = timestamp + BLUE_POLICY.stallCheckSeconds;
  }
  if (timestamp + 1e-9 >= state.nextPlan) {
    path = state.behavior === 'evade' ? escapeRoute(observation, state, planner) : state.missionTarget ? planner.plan(self.position, state.missionTarget) : [];
    state.nextPlan = timestamp + (state.behavior === 'evade' ? BLUE_POLICY.evadeReplanSeconds : BLUE_POLICY.patrolReplanSeconds);
  }
  return { behavior: state.behavior, behaviorSince: state.behaviorSince, missionTarget: state.missionTarget ? cloneVec(state.missionTarget) : null,
    waypointsReached: state.waypointsReached, ...(path ? { path } : {}) };
}
