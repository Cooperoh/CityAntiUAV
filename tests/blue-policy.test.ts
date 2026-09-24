import { describe, expect, it, vi } from 'vitest';
import { BLUE_POLICY, createBluePolicyState, updateBluePolicy } from '../src/sim/blue-policy';
import { GridPlanner } from '../src/sim/planner';
import { Simulation } from '../src/sim/simulation';
import { DEFAULT_CONFIG, type Drone, type Observation } from '../src/sim/types';

function observer(timestamp: number, enemyDistance = Infinity): Observation {
  const self: Drone = {
    id: 'B01', team: 'blue', position: { x: 0, y: 0, z: 32 }, velocity: { x: 0, y: 0, z: 0 },
    acceleration: { x: 0, y: 0, z: 0 }, active: true, targetId: null, path: [], trail: [],
    behavior: 'patrol', behaviorSince: 0, missionTarget: null, waypointsReached: 0,
  };
  const red: Drone = { ...self, id: 'R01', team: 'red', position: { ...self.position, x: enemyDistance }, behavior: 'pursuit' };
  return { timestamp, self, drones: Number.isFinite(enemyDistance) ? [self, red] : [self], buildings: [] };
}

describe('blue mission behavior', () => {
  it('retains a stable mission while threats are distant, independent of periodic replanning', () => {
    const state = createBluePolicyState(42, 'B01');
    const planner = new GridPlanner([]);
    const initial = updateBluePolicy(observer(0, 200), state, planner);
    expect(initial.behavior).toBe('patrol');
    expect(initial.missionTarget).not.toBeNull();
    expect(initial.path?.at(-1)).toEqual(initial.missionTarget);
    for (const time of [0.1, 1, 2, 4, 8]) {
      const decision = updateBluePolicy(observer(time, 200), state, planner);
      expect(decision.behavior).toBe('patrol');
      expect(decision.missionTarget).toEqual(initial.missionTarget);
      expect(decision.waypointsReached).toBe(0);
    }
    const independent = updateBluePolicy(observer(0, 200), createBluePolicyState(42, 'B02'), planner);
    expect(independent.missionTarget).not.toEqual(initial.missionTarget);
  });

  it('evades a nearby threat and resumes the original mission only after sustained safety', () => {
    const state = createBluePolicyState(42, 'B01');
    const planner = new GridPlanner([]);
    const initial = updateBluePolicy(observer(0, 200), state, planner);
    expect(updateBluePolicy(observer(1, 70), state, planner).behavior).toBe('evade');
    // Inside the 80–120 m band the current response is retained.
    expect(updateBluePolicy(observer(4, 100), state, planner).behavior).toBe('evade');
    expect(updateBluePolicy(observer(4.1, 150), state, planner).behavior).toBe('evade');
    expect(updateBluePolicy(observer(5.9, 150), state, planner).behavior).toBe('evade');
    // A renewed threat interrupts the safety dwell timer.
    expect(updateBluePolicy(observer(6, 100), state, planner).behavior).toBe('evade');
    expect(updateBluePolicy(observer(6.1, 150), state, planner).behavior).toBe('evade');
    const recovery = updateBluePolicy(observer(8.2, 150), state, planner);
    expect(recovery.behavior).toBe('recover');
    expect(recovery.missionTarget).toEqual(initial.missionTarget);
    expect(recovery.path?.at(-1)).toEqual(initial.missionTarget);
    expect(updateBluePolicy(observer(8.3, 100), state, planner).behavior).toBe('recover');
    const progressing = observer(8.2 + BLUE_POLICY.recoverySeconds + 0.1, 100);
    progressing.self.position.x = 5;
    const patrol = updateBluePolicy(progressing, state, planner);
    expect(patrol.behavior).toBe('patrol');
    expect(patrol.missionTarget).toEqual(initial.missionTarget);
  });

  it('does not switch straight back after a brief threat disappears', () => {
    const state = createBluePolicyState(42, 'B01');
    const planner = new GridPlanner([]);
    expect(updateBluePolicy(observer(0, 70), state, planner).behavior).toBe('evade');
    expect(updateBluePolicy(observer(0.1), state, planner).behavior).toBe('evade');
    expect(updateBluePolicy(observer(2.2), state, planner).behavior).toBe('evade');
    expect(updateBluePolicy(observer(3.1), state, planner).behavior).toBe('recover');
    expect(updateBluePolicy(observer(3.2, 70), state, planner).behavior).toBe('evade');
  });

  it('counts arrival and chooses a new task, while returned targets cannot mutate policy memory', () => {
    const state = createBluePolicyState(42, 'B01');
    const planner = new GridPlanner([]);
    const initial = updateBluePolicy(observer(0), state, planner);
    const goal = { ...initial.missionTarget! };
    initial.missionTarget!.x = 1e9;
    expect(updateBluePolicy(observer(0.1), state, planner).missionTarget).toEqual(goal);
    const atGoal = observer(10);
    atGoal.self.position = goal;
    const next = updateBluePolicy(atGoal, state, planner);
    expect(next.waypointsReached).toBe(1);
    expect(next.missionTarget).not.toEqual(goal);
    expect(next.path?.at(-1)).toEqual(next.missionTarget);
  });

  it('does not count failed route planning as completed missions', () => {
    const state = createBluePolicyState(42, 'B01');
    const planner = new GridPlanner([]);
    const failedPlanner = vi.spyOn(planner, 'plan').mockReturnValue([]);
    for (const time of [0, 1, 5, 10]) {
      const decision = updateBluePolicy(observer(time), state, planner);
      expect(decision.missionTarget).toBeNull();
      expect(decision.waypointsReached).toBe(0);
      expect(decision.path ?? []).toEqual([]);
    }
    failedPlanner.mockRestore();
    const resumed = updateBluePolicy(observer(10 + BLUE_POLICY.patrolReplanSeconds + 0.1), state, planner);
    expect(resumed.missionTarget).not.toBeNull();
    expect(resumed.waypointsReached).toBe(0);
    expect(resumed.path?.at(-1)).toEqual(resumed.missionTarget);
  });

  it('visits multiple city blocks and completes repeatable missions for 240 seconds without red drones', () => {
    const config = { ...DEFAULT_CONFIG, redCount: 0, blueCount: 6 };
    const simulation = new Simulation(config);
    const repeated = new Simulation(config);
    const cells = new Map<string, Set<string>>();
    const idleSeconds = new Map<string, number>();
    let previous = simulation.snapshot();
    let edgeSamples = 0;
    let activeSamples = 0;
    for (let second = 0; second < 240; second++) {
      simulation.step(1);
      repeated.step(1);
      const state = simulation.snapshot();
      expect(state.collisions).toBe(0);
      for (const drone of state.drones) {
        expect(drone.behavior).toBe('patrol');
        expect(drone.active).toBe(true);
        activeSamples++;
        const before = previous.drones.find(d => d.id === drone.id)!;
        const moved = Math.hypot(drone.position.x - before.position.x, drone.position.y - before.position.y, drone.position.z - before.position.z);
        const idle = moved < 0.2 ? (idleSeconds.get(drone.id) ?? 0) + 1 : 0;
        idleSeconds.set(drone.id, idle);
        expect(idle, `${drone.id} stopped progressing for more than 15 seconds`).toBeLessThanOrEqual(15);
        if (Math.min(300 - Math.abs(drone.position.x), 300 - Math.abs(drone.position.y)) < 40) edgeSamples++;
        const visited = cells.get(drone.id) ?? new Set<string>();
        visited.add(`${Math.floor((drone.position.x + 256) / 64)},${Math.floor((drone.position.y + 256) / 64)}`);
        cells.set(drone.id, visited);
      }
      previous = state;
    }
    const final = simulation.snapshot();
    expect(final).toEqual(repeated.snapshot());
    expect(final.finished).toBe(false);
    expect(final.captured).toBe(0);
    expect(edgeSamples / activeSamples).toBeLessThan(0.1);
    for (const drone of final.drones) {
      expect(drone.waypointsReached, `${drone.id} did not complete repeated tasks`).toBeGreaterThanOrEqual(2);
      expect(cells.get(drone.id)!.size, `${drone.id} stayed in too few city blocks`).toBeGreaterThanOrEqual(5);
    }
  }, 30_000);
});
