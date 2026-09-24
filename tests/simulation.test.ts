import { describe, expect, it } from 'vitest';
import { integrateMotion, segmentClear } from '../src/sim/geometry';
import { GridPlanner } from '../src/sim/planner';
import { Simulation } from '../src/sim/simulation';
import {
  DEFAULT_CONFIG, DRONE_RADIUS, FIXED_DT, HORIZONTAL_BOUNDARY_MARGIN, WORLD_BOUNDS,
  type Building, type SensorModel, type SimulationSnapshot, type Vec3,
} from '../src/sim/types';

const magnitude = (p: Vec3) => Math.hypot(p.x, p.y, p.z);
const separation = (a: Vec3, b: Vec3) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
const withinFlightEnvelope = (point: Vec3) => [point.x, point.y, point.z].every(Number.isFinite)
  && point.x >= WORLD_BOUNDS.minX + HORIZONTAL_BOUNDARY_MARGIN - 1e-6
  && point.x <= WORLD_BOUNDS.maxX - HORIZONTAL_BOUNDARY_MARGIN + 1e-6
  && point.y >= WORLD_BOUNDS.minY + HORIZONTAL_BOUNDARY_MARGIN - 1e-6
  && point.y <= WORLD_BOUNDS.maxY - HORIZONTAL_BOUNDARY_MARGIN + 1e-6
  && point.z >= WORLD_BOUNDS.minZ - 1e-6
  && point.z <= WORLD_BOUNDS.maxZ - DRONE_RADIUS + 1e-6;

// Deliberately independent of the simulator's geometry helper.
function insideExpandedBuilding(p: Vec3, b: Building): boolean {
  const r = DRONE_RADIUS - 1e-5;
  return p.x > b.x - b.width / 2 - r && p.x < b.x + b.width / 2 + r
    && p.y > b.y - b.depth / 2 - r && p.y < b.y + b.depth / 2 + r
    && p.z < b.height + r && p.z > -r;
}

function assertLegalState(state: SimulationSnapshot): void {
  for (const drone of state.drones) {
    const values = [drone.position, drone.velocity, drone.acceleration].flatMap(p => [p.x, p.y, p.z]);
    expect(values.every(Number.isFinite), `${drone.id} contains a non-finite state`).toBe(true);
    expect(withinFlightEnvelope(drone.position), `${drone.id} position exceeds the flight envelope`).toBe(true);
    expect(drone.path.every(withinFlightEnvelope), `${drone.id} path exceeds the flight envelope`).toBe(true);
    expect(drone.trail.every(withinFlightEnvelope), `${drone.id} trail exceeds the flight envelope`).toBe(true);
    const maxSpeed = drone.team === 'red' ? DEFAULT_CONFIG.redMaxSpeed : DEFAULT_CONFIG.blueMaxSpeed;
    const maxAccel = drone.team === 'red' ? DEFAULT_CONFIG.redMaxAccel : DEFAULT_CONFIG.blueMaxAccel;
    expect(magnitude(drone.velocity)).toBeLessThanOrEqual(maxSpeed + 1e-6);
    expect(magnitude(drone.acceleration)).toBeLessThanOrEqual(maxAccel + 1e-6);
    expect(state.buildings.some(b => insideExpandedBuilding(drone.position, b)), `${drone.id} overlaps a building`).toBe(false);
  }
}

function runUntilFinished(seed: number, seconds = 600): SimulationSnapshot {
  const sim = new Simulation({ ...DEFAULT_CONFIG, seed });
  let state = sim.snapshot();
  for (let i = 0; i < seconds / FIXED_DT && !state.finished; i++) {
    sim.step(FIXED_DT);
    // Snapshot copies are intentionally sampled, independent of numerical integration.
    if (i % 30 === 29) state = sim.snapshot();
  }
  return sim.snapshot();
}

describe('point-mass motion and continuous geometry', () => {
  it('uses the three acceleration axes with the expected constant-acceleration displacement', () => {
    const next = integrateMotion(
      { x: 1, y: 2, z: 20 }, { x: 2, y: -3, z: 1 }, { x: 4, y: 2, z: -2 }, 0.25, 100, 100,
    );
    expect(next.velocity).toEqual({ x: 3, y: -2.5, z: 0.5 });
    expect(next.position).toEqual({ x: 1.625, y: 1.3125, z: 20.1875 });
  });

  it('limits vector magnitudes instead of applying three independent axis limits', () => {
    const next = integrateMotion(
      { x: 0, y: 0, z: 20 }, { x: 6, y: 6, z: 5 }, { x: 30, y: 40, z: 0 }, 0.1, 10, 5,
    );
    expect(magnitude(next.velocity)).toBeLessThanOrEqual(10 + 1e-9);
    expect(magnitude(next.acceleration)).toBeLessThanOrEqual(5 + 1e-9);
    const accelerating = integrateMotion(
      { x: 0, y: 0, z: 20 }, { x: 0, y: 0, z: 0 }, { x: 30, y: 40, z: 0 }, 0.1, 10, 5,
    );
    expect(magnitude(accelerating.acceleration)).toBeCloseTo(5, 9);
  });

  it('rejects a segment through a building even when both endpoints are outside', () => {
    const obstacle: Building = { id: 'wall', x: 0, y: 0, width: 10, depth: 20, height: 40, style: 0 };
    expect(segmentClear({ x: -20, y: 0, z: 20 }, { x: 20, y: 0, z: 20 }, [obstacle])).toBe(false);
    expect(segmentClear({ x: -20, y: 0, z: 43 }, { x: 20, y: 0, z: 43 }, [obstacle])).toBe(true);
    expect(segmentClear({ x: -20, y: 11, z: 20 }, { x: 20, y: 11, z: 20 }, [obstacle])).toBe(false);
  });

  it('connects boundary-adjacent starting points through the city without planning outside the flight envelope', () => {
    const buildings = new Simulation({ ...DEFAULT_CONFIG }).snapshot().buildings;
    const planner = new GridPlanner(buildings);
    const start = { x: WORLD_BOUNDS.maxX - HORIZONTAL_BOUNDARY_MARGIN, y: WORLD_BOUNDS.minY + HORIZONTAL_BOUNDARY_MARGIN, z: 20 };
    const goal = { x: WORLD_BOUNDS.minX + HORIZONTAL_BOUNDARY_MARGIN, y: WORLD_BOUNDS.maxY - HORIZONTAL_BOUNDARY_MARGIN, z: 20 };
    expect(segmentClear(start, goal, buildings)).toBe(false);
    const path = planner.plan(start, goal);
    expect(path.length).toBeGreaterThan(1);
    expect(path.every(withinFlightEnvelope)).toBe(true);
    expect(path.at(-1)).toEqual(goal);
    let previous = start;
    for (const point of path) {
      expect(segmentClear(previous, point, buildings)).toBe(true);
      previous = point;
    }
  });
});

describe('simulation contracts', () => {
  it('reproduces the same world and state for the same seed and time steps', () => {
    const a = new Simulation({ ...DEFAULT_CONFIG, seed: 7 });
    const b = new Simulation({ ...DEFAULT_CONFIG, seed: 7 });
    expect(a.snapshot()).toEqual(b.snapshot());
    for (let i = 0; i < 240; i++) { a.step(); b.step(); }
    expect(a.snapshot()).toEqual(b.snapshot());
    expect(new Simulation({ ...DEFAULT_CONFIG, seed: 8 }).snapshot().buildings).not.toEqual(a.snapshot().buildings);
  });

  it('returns snapshots that cannot mutate the live world', () => {
    const sim = new Simulation({ ...DEFAULT_CONFIG });
    sim.step();
    const before = sim.snapshot();
    const external = sim.snapshot();
    external.drones[0]!.position.x = 1e9;
    const blue = external.drones.find(d => d.team === 'blue')!;
    expect(blue.missionTarget).not.toBeNull();
    blue.missionTarget!.x = 1e9;
    external.buildings[0]!.height = 1e9;
    external.events.push({ time: 0, kind: 'info', message: 'external mutation' });
    expect(sim.snapshot()).toEqual(before);
  });

  it('moves blue drones, keeps finite bounded states, and respects speed/acceleration limits', () => {
    const sim = new Simulation({ ...DEFAULT_CONFIG });
    const first = sim.snapshot();
    let previous = first;
    const moved = new Set<string>();
    assertLegalState(first);
    for (let i = 0; i < 60 * 30; i++) {
      sim.step();
      if (i % 6 !== 5) continue;
      const state = sim.snapshot();
      assertLegalState(state);
      for (const drone of state.drones.filter(d => d.active)) {
        const before = previous.drones.find(d => d.id === drone.id)!;
        const maxAccel = drone.team === 'red' ? DEFAULT_CONFIG.redMaxAccel : DEFAULT_CONFIG.blueMaxAccel;
        expect(separation(drone.velocity, before.velocity), `${drone.id} changed velocity faster than its acceleration limit`)
          .toBeLessThanOrEqual(maxAccel * (state.time - previous.time) + 1e-5);
      }
      for (const drone of state.drones.filter(d => d.team === 'blue')) {
        const initial = first.drones.find(d => d.id === drone.id)!;
        if (separation(drone.position, initial.position) > 1) moved.add(drone.id);
      }
      previous = state;
    }
    expect(moved.size).toBe(DEFAULT_CONFIG.blueCount);
    expect(sim.snapshot().collisions).toBe(0);
  });

  it('accepts a replacement sensor and does not select targets absent from its observations', () => {
    const calls = new Set<string>();
    const sensor: SensorModel = {
      observe(selfId, world) {
        calls.add(selfId);
        const self = world.drones.find(d => d.id === selfId)!;
        return { timestamp: world.time, self, drones: [], buildings: world.buildings };
      },
    };
    const sim = new Simulation({ ...DEFAULT_CONFIG }, sensor);
    for (let i = 0; i < 180; i++) sim.step();
    const state = sim.snapshot();
    expect(calls.size).toBe(DEFAULT_CONFIG.redCount + DEFAULT_CONFIG.blueCount);
    expect(state.drones.filter(d => d.team === 'red').every(d => d.targetId === null)).toBe(true);
    expect(state.captured).toBe(0);
  });

  it('brakes near the shared horizontal buffer without snapping velocities or clipping positions', () => {
    // Drive a real red point mass toward a stationary observed target on the west limit.
    // This controlled route keeps the braking regression independent of the blue policy.
    const sensor: SensorModel = {
      observe(selfId, world) {
        const self = world.drones.find(d => d.id === selfId)!;
        const blue = world.drones.find(d => d.team === 'blue')!;
        const target = { ...blue, position: { x: WORLD_BOUNDS.minX + HORIZONTAL_BOUNDARY_MARGIN, y: -192, z: self.position.z }, velocity: { x: 0, y: 0, z: 0 } };
        return { timestamp: world.time, self, drones: self.team === 'red' ? [self, target] : [], buildings: world.buildings };
      },
    };
    const sim = new Simulation({ ...DEFAULT_CONFIG, redCount: 1, blueCount: 1, captureRadius: 0 }, sensor);
    let previous = sim.snapshot();
    let witnessedEdgeBraking = false;
    for (let i = 0; i < 60 * 12; i++) {
      sim.step();
      const current = sim.snapshot();
      assertLegalState(current);
      for (const drone of current.drones.filter(d => d.active)) {
        const before = previous.drones.find(d => d.id === drone.id)!;
        const maxAccel = drone.team === 'red' ? DEFAULT_CONFIG.redMaxAccel : DEFAULT_CONFIG.blueMaxAccel;
        expect(separation(drone.velocity, before.velocity)).toBeLessThanOrEqual(maxAccel * FIXED_DT + 1e-6);
        // An active point mass must still integrate its velocity continuously near the edge.
        const expected = {
          x: before.position.x + (before.velocity.x + drone.velocity.x) * FIXED_DT / 2,
          y: before.position.y + (before.velocity.y + drone.velocity.y) * FIXED_DT / 2,
          z: before.position.z + (before.velocity.z + drone.velocity.z) * FIXED_DT / 2,
        };
        expect(separation(drone.position, expected)).toBeLessThan(1e-6);
        if (drone.team !== 'red') continue;
        const nearX = Math.abs(drone.position.x) >= Math.abs(drone.position.y);
        const outwardVelocity = nearX ? Math.sign(drone.position.x) * drone.velocity.x : Math.sign(drone.position.y) * drone.velocity.y;
        const centerMargin = Math.min(WORLD_BOUNDS.maxX - Math.abs(drone.position.x), WORLD_BOUNDS.maxY - Math.abs(drone.position.y));
        if (centerMargin < HORIZONTAL_BOUNDARY_MARGIN + 3 && outwardVelocity < 0.5) witnessedEdgeBraking = true;
      }
      previous = current;
    }
    expect(witnessedEdgeBraking).toBe(true);
    expect(previous.collisions).toBe(0);
  });

  it.each([42, 7, 2026])('actually captures every blue drone for default settings, seed %s', (seed) => {
    const state = runUntilFinished(seed);
    expect(state.finished, `seed ${seed} did not finish by 600 simulated seconds`).toBe(true);
    expect(state.captured).toBe(DEFAULT_CONFIG.blueCount);
    expect(state.drones.filter(d => d.team === 'blue' && d.active)).toHaveLength(0);
    expect(state.events.filter(event => event.kind === 'capture')).toHaveLength(DEFAULT_CONFIG.blueCount);
    expect(state.collisions).toBe(0);
    assertLegalState(state);
  }, 120_000);

  it('only captures blue drones through actual proximity without a building in between', () => {
    const sim = new Simulation({ ...DEFAULT_CONFIG });
    let previous = sim.snapshot();
    let witnessed = 0;
    for (let i = 0; i < 60 * 600 && !previous.finished; i++) {
      sim.step();
      const current = sim.snapshot();
      for (const drone of current.drones.filter(d => d.team === 'blue')) {
        if (!drone.active && previous.drones.find(d => d.id === drone.id)!.active) {
          const captor = current.drones.find(red => red.team === 'red' && red.active
            && separation(red.position, drone.position) <= DEFAULT_CONFIG.captureRadius + 1e-5
            && segmentClear(red.position, drone.position, current.buildings, 0));
          expect(captor, `${drone.id} was removed without a nearby, unobstructed red drone`).toBeDefined();
          witnessed++;
        }
      }
      previous = current;
    }
    expect(witnessed).toBe(DEFAULT_CONFIG.blueCount);
  }, 120_000);

  it('freezes time and state after the last capture', () => {
    const sim = new Simulation({ ...DEFAULT_CONFIG, redCount: 3, blueCount: 1, seed: 42 });
    for (let i = 0; i < 60 * 600; i++) {
      sim.step();
      if (i % 60 === 59 && sim.snapshot().finished) break;
    }
    const end = sim.snapshot();
    expect(end.finished).toBe(true);
    for (let i = 0; i < 120; i++) sim.step();
    expect(sim.snapshot()).toEqual(end);
  }, 120_000);
});
