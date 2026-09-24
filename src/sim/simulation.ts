import { DRONE_RADIUS, FIXED_DT, type Building, type Drone, type Observation, type SensorModel, type SimEvent, type SimulationSnapshot, type Vec3, type WorldConfig } from './types';
import { add, clampToBounds, cloneVec, distance, integrateMotion, isFree, limit, magnitude, scale, segmentClear, subtract } from './geometry';
import { GridPlanner } from './planner';
import { createBluePolicyState, updateBluePolicy, type BluePolicyState } from './blue-policy';

const ZERO: Vec3 = { x: 0, y: 0, z: 0 };
const BRAKING_CLEARANCE = DRONE_RADIUS + 2 + 1e-5;
const cloneDrone = (d: Drone, includeHistory = true): Drone => ({ ...d, position: cloneVec(d.position), velocity: cloneVec(d.velocity), acceleration: cloneVec(d.acceleration), missionTarget: d.missionTarget ? cloneVec(d.missionTarget) : null, path: includeHistory ? d.path.map(cloneVec) : [], trail: includeHistory ? d.trail.map(cloneVec) : [] });

/** Only this adapter sees truth. Policies below consume Observation exclusively. */
export class FullKnowledgeSensor implements SensorModel {
  observe(selfId: string, world: SimulationSnapshot): Observation {
    const self = world.drones.find((drone) => drone.id === selfId);
    if (!self) throw new Error(`Unknown drone ${selfId}`);
    return { timestamp: world.time, self, drones: world.drones, buildings: world.buildings };
  }
}

function randomGenerator(seed: number) {
  let state = seed >>> 0;
  return () => {
    state += 0x6d2b79f5;
    let value = state;
    value = Math.imul(value ^ (value >>> 15), value | 1);
    value ^= value + Math.imul(value ^ (value >>> 7), value | 61);
    return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
  };
}

export function generateCity(seed: number): Building[] {
  const random = randomGenerator(seed);
  const result: Building[] = [];
  for (let row = 0; row < 8; row++) {
    for (let col = 0; col < 8; col++) {
      result.push({ id: `block-${row}-${col}`, x: -224 + col * 64, y: -224 + row * 64,
        width: 30 + random() * 16, depth: 30 + random() * 16,
        height: 14 + Math.pow(random(), 1.25) * 94, style: Math.floor(random() * 4) });
    }
  }
  return result;
}

interface PilotState { nextPlan: number; nextTrail: number; mapSignature: string; planner: GridPlanner | null; blue: BluePolicyState | null; }

/** A deterministic, renderer-independent sandbox. Time advances only through step(). */
export class Simulation {
  private time = 0;
  private buildings: Building[];
  private drones: Drone[] = [];
  private events: SimEvent[] = [];
  private captured = 0;
  private collisions = 0;
  private done = false;
  private config: WorldConfig;
  private pilots = new Map<string, PilotState>();
  private plannerCache = new Map<string, GridPlanner>();

  constructor(config: WorldConfig, private sensor: SensorModel = new FullKnowledgeSensor()) {
    this.config = { ...config };
    this.buildings = generateCity(config.seed);
    const random = randomGenerator(config.seed ^ 0x4d32);
    for (const team of ['red', 'blue'] as const) {
      const count = Math.max(0, Math.floor(team === 'red' ? config.redCount : config.blueCount));
      for (let index = 0; index < count; index++) {
        const position = { x: team === 'red' ? -256 + (index % 2) * 64 : 128 + (index % 3) * 64,
          y: -192 + (index % 7) * 64, z: 22 + random() * 15 + Math.floor(index / 7) * 6 };
        const id = `${team === 'red' ? 'R' : 'B'}${String(index + 1).padStart(2, '0')}`;
        this.drones.push({ id, team, position, velocity: cloneVec(ZERO), acceleration: cloneVec(ZERO), active: true, targetId: null, path: [], trail: [cloneVec(position)], behavior: team === 'red' ? 'pursuit' : 'patrol', behaviorSince: 0, missionTarget: null, waypointsReached: 0 });
        this.pilots.set(id, { nextPlan: 0, nextTrail: 0, mapSignature: '', planner: null, blue: team === 'blue' ? createBluePolicyState(config.seed, id) : null });
      }
    }
    this.events.push({ time: 0, kind: 'info', message: '完全感知已就绪 · 64 栋建筑 · 三维质点运动' });
    this.done = config.blueCount === 0;
  }

  snapshot(): SimulationSnapshot {
    return { time: this.time, seed: this.config.seed, buildings: this.buildings.map((b) => ({ ...b })),
      drones: this.drones.map((d) => cloneDrone(d)), events: this.events.map((e) => ({ ...e, ...(e.position ? { position: cloneVec(e.position) } : {}) })),
      captured: this.captured, collisions: this.collisions, finished: this.done };
  }

  private sensorWorld(): SimulationSnapshot {
    return { time: this.time, seed: this.config.seed, buildings: this.buildings.map((b) => ({ ...b })),
      drones: this.drones.map((d) => cloneDrone(d, false)), events: [], captured: this.captured, collisions: this.collisions, finished: this.done };
  }

  private plannerFor(observation: Observation, pilot: PilotState): GridPlanner {
    const signature = observation.buildings.map((b) => `${b.id},${b.x},${b.y},${b.width},${b.depth},${b.height}`).join(';');
    if (signature !== pilot.mapSignature || !pilot.planner) {
      let planner = this.plannerCache.get(signature);
      if (!planner) {
        planner = new GridPlanner(observation.buildings);
        this.plannerCache.set(signature, planner);
        if (this.plannerCache.size > 12) this.plannerCache.delete(this.plannerCache.keys().next().value!);
      }
      pilot.mapSignature = signature;
      pilot.planner = planner;
    }
    return pilot.planner;
  }

  private selectTarget(observation: Observation): Drone | null {
    const enemies = observation.drones.filter((d) => d.active && d.team === 'blue');
    if (!enemies.length) return null;
    const allies = observation.drones.filter((d) => d.active && d.team === 'red');
    if (!allies.some((d) => d.id === observation.self.id)) allies.push(observation.self);
    const pairs = allies.flatMap((red) => enemies.map((blue) => ({ red, blue, cost: distance(red.position, blue.position) * (red.targetId === blue.id ? 0.85 : 1) })));
    pairs.sort((a, b) => a.cost - b.cost || a.red.id.localeCompare(b.red.id) || a.blue.id.localeCompare(b.blue.id));
    const assignedRed = new Set<string>();
    const assignedBlue = new Set<string>();
    for (const pair of pairs) {
      if (assignedRed.has(pair.red.id) || assignedBlue.has(pair.blue.id)) continue;
      if (pair.red.id === observation.self.id) return pair.blue;
      assignedRed.add(pair.red.id);
      assignedBlue.add(pair.blue.id);
    }
    return enemies.reduce((best, next) => distance(observation.self.position, next.position) < distance(observation.self.position, best.position) ? next : best);
  }

  private replan(observation: Observation, pilot: PilotState, drone: Drone) {
    const self = observation.self;
    const planner = this.plannerFor(observation, pilot);
    if (self.team === 'red') {
      const target = this.selectTarget(observation);
      drone.targetId = target?.id ?? null;
      if (!target) { drone.path = []; return; }
      const lead = Math.min(1.15, distance(self.position, target.position) / Math.max(1, this.config.redMaxSpeed) * 0.35);
      let destination = clampToBounds(add(target.position, scale(target.velocity, lead)));
      if (!segmentClear(target.position, destination, observation.buildings, DRONE_RADIUS + 2)) destination = target.position;
      drone.path = planner.plan(self.position, destination);
      if (!drone.path.length) drone.path = planner.plan(self.position, target.position);
      pilot.nextPlan = observation.timestamp + 0.7;

    }
  }

  private commandedAcceleration(observation: Observation, drone: Drone, dt: number): Vec3 {
    const self = observation.self;
    const maxSpeed = self.team === 'red' ? this.config.redMaxSpeed : this.config.blueMaxSpeed;
    const maxAccel = self.team === 'red' ? this.config.redMaxAccel : this.config.blueMaxAccel;
    // The tiny numerical reserve is not a wall: a point rounding onto its face
    // must still be allowed to move along/away from it while respecting A* clearance.
    const clearance = isFree(self.position, observation.buildings, BRAKING_CLEARANCE) ? BRAKING_CLEARANCE : DRONE_RADIUS + 2;
    // Being near a corner does not make the following segment traversable. Removing
    // this waypoint early used to aim through the expanded wall and deadlock braking.
    while (drone.path.length > 1 && distance(self.position, drone.path[0]!) < 4 &&
      segmentClear(self.position, drone.path[1]!, observation.buildings, clearance)) drone.path.shift();
    // Only skip waypoints where the observer knows the whole shortcut is clear.
    for (let index = drone.path.length - 1; index > 0; index--) {
      if (segmentClear(self.position, drone.path[index]!, observation.buildings, clearance)) {
        drone.path.splice(0, index);
        break;
      }
    }
    const destination = drone.path[0];
    let desired = cloneVec(ZERO);
    if (destination) {
      const offset = subtract(destination, self.position);
      const d = magnitude(offset);
      // An intermediate corner may need its last few centimetres before the next
      // segment becomes visible. Only the final destination has an arrival deadzone.
      const intermediate = drone.path.length > 1;
      const arrivalDeadzone = intermediate ? 0 : 0.7;
      const speed = Math.min(maxSpeed, Math.sqrt(Math.max(0, 2 * maxAccel * (d - arrivalDeadzone))));
      if (d > (intermediate ? 1e-6 : 0.1)) desired = scale(offset, speed / d);
    }
    const request = limit(scale(subtract(desired, self.velocity), 1 / 0.42), maxAccel);
    const next = integrateMotion(self.position, self.velocity, request, dt, maxSpeed, maxAccel);
    const stoppingDistance = magnitude(next.velocity) / (2 * Math.max(0.1, maxAccel));
    const stop = add(next.position, scale(next.velocity, stoppingDistance + 0.08));
    // Preserve a free braking corridor. This turns cornering into a bounded-acceleration maneuver.
    if (segmentClear(self.position, next.position, observation.buildings, clearance) &&
      segmentClear(next.position, stop, observation.buildings, clearance)) return request;
    const speed = magnitude(self.velocity);
    return speed > 1e-8 ? scale(self.velocity, -Math.min(maxAccel, speed / dt) / speed) : cloneVec(ZERO);
  }

  step(dt = FIXED_DT): void {
    if (this.done || !Number.isFinite(dt) || dt <= 0) return;
    // Bound integration granularity even when callers advance a large wall-clock interval.
    if (dt > FIXED_DT + 1e-10) {
      let remaining = dt;
      while (remaining > 1e-10 && !this.done) { const h = Math.min(FIXED_DT, remaining); this.step(h); remaining -= h; }
      return;
    }
    const observations = new Map<string, Observation>();
    for (const drone of this.drones) if (drone.active) observations.set(drone.id, this.sensor.observe(drone.id, this.sensorWorld()));
    for (const drone of this.drones) {
      if (!drone.active) continue;
      const observation = observations.get(drone.id)!;
      const pilot = this.pilots.get(drone.id)!;
      if (pilot.blue) {
        const decision = updateBluePolicy(observation, pilot.blue, this.plannerFor(observation, pilot));
        drone.behavior = decision.behavior;
        drone.behaviorSince = decision.behaviorSince;
        drone.missionTarget = decision.missionTarget;
        drone.waypointsReached = decision.waypointsReached;
        if (decision.path) drone.path = decision.path;
      } else if (this.time + 1e-9 >= pilot.nextPlan) this.replan(observation, pilot, drone);
      const command = this.commandedAcceleration(observation, drone, dt);
      const maxSpeed = drone.team === 'red' ? this.config.redMaxSpeed : this.config.blueMaxSpeed;
      const maxAccel = drone.team === 'red' ? this.config.redMaxAccel : this.config.blueMaxAccel;
      const motion = integrateMotion(drone.position, drone.velocity, command, dt, maxSpeed, maxAccel);
      // The world enforces collision physics even when a future sensor cannot see an obstacle.
      if (segmentClear(drone.position, motion.position, this.buildings)) {
        drone.position = motion.position;
        drone.velocity = motion.velocity;
        drone.acceleration = motion.acceleration;
      } else {
        this.collisions++;
        drone.velocity = cloneVec(ZERO);
        drone.acceleration = cloneVec(ZERO);
        drone.path = [];
        pilot.nextPlan = this.time;
        if (pilot.blue) pilot.blue.nextPlan = this.time;
      }
      if (this.time >= pilot.nextTrail) {
        drone.trail.push(cloneVec(drone.position));
        if (drone.trail.length > 180) drone.trail.shift();
        pilot.nextTrail = this.time + 0.22;
      }
    }
    this.time += dt;
    for (const blue of this.drones) {
      if (blue.team !== 'blue' || !blue.active) continue;
      const red = this.drones.find((d) => d.team === 'red' && d.active && distance(d.position, blue.position) <= this.config.captureRadius && segmentClear(d.position, blue.position, this.buildings, 0));
      if (red) {
        blue.active = false;
        blue.behavior = 'captured';
        blue.behaviorSince = this.time;
        blue.velocity = cloneVec(ZERO);
        blue.acceleration = cloneVec(ZERO);
        blue.path = [];
        this.captured++;
        this.events.push({ time: this.time, kind: 'capture', message: `${red.id} 捕获 ${blue.id}`, position: cloneVec(blue.position) });
        for (const hunter of this.drones) if (hunter.targetId === blue.id) this.pilots.get(hunter.id)!.nextPlan = this.time;
      }
    }
    this.done = this.drones.every((d) => d.team !== 'blue' || !d.active);
    if (this.done) this.events.push({ time: this.time, kind: 'info', message: '对抗结束 · 蓝方已全部被捕获' });
  }
}
