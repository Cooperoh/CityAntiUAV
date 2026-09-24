import { expect, it } from 'vitest';
import { distance, integrateMotion, magnitude, segmentClear } from '../src/sim/geometry';
import { GridPlanner } from '../src/sim/planner';
import { Simulation } from '../src/sim/simulation';
import { DEFAULT_CONFIG, FIXED_DT, type Building, type Drone, type Observation, type Vec3 } from '../src/sim/types';

it('continues through a nearby intermediate corner instead of stopping before line of sight opens', () => {
  const building: Building = { id: 'tight-corner', x: 0, y: 0, width: 30.5, depth: 30.5, height: 60, style: 0 };
  const sim = new Simulation({ ...DEFAULT_CONFIG, redCount: 0, blueCount: 1 });
  const drone = sim.snapshot().drones[0]!;
  drone.position = { x: -19.3333333333, y: 18.8, z: 23 };
  const goal = { x: 40, y: 19.3333333333, z: 23 };
  drone.path = new GridPlanner([building]).plan(drone.position, goal);
  expect(drone.path).toHaveLength(2);
  expect(distance(drone.position, drone.path[0]!)).toBeLessThan(0.7);
  expect(segmentClear(drone.position, drone.path[1]!, [building], 4)).toBe(false);

  // Exercise the production controller and the production integrator with the real
  // A* route, without exposing a public API for arbitrary world-state mutation.
  const controller = sim as unknown as {
    commandedAcceleration(observation: Observation, drone: Drone, dt: number): Vec3;
  };
  for (let step = 0; step < 30 / FIXED_DT; step++) {
    const previous = { ...drone.position };
    const observation: Observation = { timestamp: step * FIXED_DT, self: drone, drones: [], buildings: [building] };
    const acceleration = controller.commandedAcceleration(observation, drone, FIXED_DT);
    const next = integrateMotion(drone.position, drone.velocity, acceleration, FIXED_DT,
      DEFAULT_CONFIG.blueMaxSpeed, DEFAULT_CONFIG.blueMaxAccel);
    expect(magnitude(next.velocity)).toBeLessThanOrEqual(DEFAULT_CONFIG.blueMaxSpeed + 1e-6);
    expect(magnitude(next.acceleration)).toBeLessThanOrEqual(DEFAULT_CONFIG.blueMaxAccel + 1e-6);
    expect(segmentClear(previous, next.position, [building])).toBe(true);
    Object.assign(drone, next);
  }
  expect(distance(drone.position, goal)).toBeLessThan(1);
  expect(drone.path).toHaveLength(1);
});
