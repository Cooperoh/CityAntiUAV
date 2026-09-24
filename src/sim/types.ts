export interface Vec3 {
  x: number;
  y: number;
  z: number;
}
export type Team = "red" | "blue";
export type DroneBehavior = "pursuit" | "patrol" | "evade" | "recover" | "captured";
export interface Building {
  id: string;
  x: number;
  y: number;
  width: number;
  depth: number;
  height: number;
  style: number;
}
export interface Drone {
  id: string;
  team: Team;
  position: Vec3;
  velocity: Vec3;
  acceleration: Vec3;
  active: boolean;
  targetId: string | null;
  path: Vec3[];
  trail: Vec3[];
  behavior: DroneBehavior;
  behaviorSince: number;
  missionTarget: Vec3 | null;
  waypointsReached: number;
}
export interface SimEvent {
  time: number;
  kind: "capture" | "info";
  message: string;
  position?: Vec3;
}
export interface WorldConfig {
  seed: number;
  redCount: number;
  blueCount: number;
  redMaxSpeed: number;
  blueMaxSpeed: number;
  redMaxAccel: number;
  blueMaxAccel: number;
  captureRadius: number;
}
export interface SimulationSnapshot {
  time: number;
  seed: number;
  drones: Drone[];
  buildings: Building[];
  events: SimEvent[];
  captured: number;
  collisions: number;
  finished: boolean;
}
export interface Observation {
  timestamp: number;
  self: Drone;
  drones: Drone[];
  buildings: Building[];
}
export interface SensorModel {
  observe(selfId: string, world: SimulationSnapshot): Observation;
}
export const DEFAULT_CONFIG: WorldConfig = {
  seed: 42,
  redCount: 3,
  blueCount: 6,
  redMaxSpeed: 18,
  blueMaxSpeed: 12,
  redMaxAccel: 8,
  blueMaxAccel: 5,
  captureRadius: 6,
};
export const WORLD_BOUNDS = {
  minX: -300,
  maxX: 300,
  minY: -300,
  maxY: 300,
  minZ: 6,
  maxZ: 150,
};
export const FIXED_DT = 1 / 60;
export const DRONE_RADIUS = 2;
// Both teams leave room at the horizontal map edge for turns and enlarged display models.
// This flight buffer is independent of the physical collision radius.
export const HORIZONTAL_BOUNDARY_MARGIN = 12;
