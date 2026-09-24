import { Simulation } from "./sim/simulation";
import { FIXED_DT, type WorldConfig } from "./sim/types";

type Command =
  | { type: "reset"; config: WorldConfig }
  | { type: "running"; value: boolean }
  | { type: "speed"; value: number }
  | { type: "step" };
let simulation: Simulation | null = null;
let running = false;
let speed = 1;
let accumulator = 0;
let previous = performance.now();
let lastPublish = 0;
function publish() {
  if (!simulation) return;
  const snapshot = simulation.snapshot();
  if (snapshot.finished) running = false;
  self.postMessage({ type: "snapshot", snapshot, running });
  lastPublish = performance.now();
}
function fail(error: unknown) {
  running = false;
  self.postMessage({
    type: "error",
    message: error instanceof Error ? error.message : String(error),
  });
}
self.onmessage = (event: MessageEvent<Command>) => {
  try {
    const command = event.data;
    if (command.type === "reset") {
      running = false;
      simulation = new Simulation(command.config);
      accumulator = 0;
      previous = performance.now();
      publish();
    } else if (command.type === "running") {
      running =
        command.value && !!simulation && !simulation.snapshot().finished;
      accumulator = 0;
      previous = performance.now();
      publish();
    } else if (command.type === "speed") {
      speed = [1, 2, 4, 8].includes(command.value) ? command.value : 1;
    } else if (command.type === "step" && simulation && !running) {
      simulation.step(FIXED_DT);
      publish();
    }
  } catch (error) {
    fail(error);
  }
};
setInterval(() => {
  const now = performance.now();
  const elapsed = Math.min((now - previous) / 1000, 0.1);
  previous = now;
  if (!running || !simulation) return;
  try {
    accumulator += elapsed * speed;
    let steps = 0;
    while (accumulator >= FIXED_DT && steps < 60) {
      simulation.step(FIXED_DT);
      accumulator -= FIXED_DT;
      steps++;
    }
    if (now - lastPublish >= 80) publish();
  } catch (error) {
    fail(error);
  }
}, 16);
