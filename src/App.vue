<script setup lang="ts">
import {
  computed,
  onBeforeUnmount,
  onMounted,
  reactive,
  ref,
  watch,
} from "vue";
import { CityRenderer } from "./render/CityRenderer";
import {
  DEFAULT_CONFIG,
  WORLD_BOUNDS,
  type Drone,
  type DroneBehavior,
  type SimulationSnapshot,
  type WorldConfig,
} from "./sim/types";

const canvas = ref<HTMLDivElement>();
const snapshot = ref<SimulationSnapshot | null>(null);
const running = ref(false);
const speed = ref(1);
const config = reactive<WorldConfig>({ ...DEFAULT_CONFIG });
const applied = ref<WorldConfig>({ ...DEFAULT_CONFIG });
const selectedId = ref<string | null>("B01");
const camera = ref<"orbit" | "top" | "follow">("orbit");
const options = reactive({
  showTrails: true,
  showPaths: true,
  showLabels: true,
});
const error = ref("");
const help = ref(false);
const waiting = ref(true);
let worker: Worker | undefined;
let renderer: CityRenderer | undefined;
const dirty = computed(
  () => JSON.stringify(config) !== JSON.stringify(applied.value),
);
const selected = computed(() =>
  snapshot.value?.drones.find((d) => d.id === selectedId.value),
);
const boundaryClearance = computed(() => {
  const p = selected.value?.position;
  return p
    ? Math.min(
        p.x - WORLD_BOUNDS.minX,
        WORLD_BOUNDS.maxX - p.x,
        p.y - WORLD_BOUNDS.minY,
        WORLD_BOUNDS.maxY - p.y,
      )
    : 0;
});
const reds = computed(
  () => snapshot.value?.drones.filter((d) => d.team === "red") ?? [],
);
const blues = computed(
  () => snapshot.value?.drones.filter((d) => d.team === "blue") ?? [],
);
const remaining = computed(() => blues.value.filter((d) => d.active).length);
const behaviorNames: Record<DroneBehavior, string> = {
  pursuit: "追击目标",
  patrol: "任务巡航",
  evade: "临时规避",
  recover: "恢复任务",
  captured: "已捕获",
};
const blueStates = [
  { value: "patrol", label: "巡航" },
  { value: "evade", label: "规避" },
  { value: "recover", label: "恢复" },
] as const;
const blueStateCounts = computed(() =>
  Object.fromEntries(
    blueStates.map(({ value }) => [
      value,
      blues.value.filter((drone) => drone.active && drone.behavior === value)
        .length,
    ]),
  ),
);
const missionDistance = computed(() => {
  const drone = selected.value;
  return drone?.missionTarget
    ? Math.hypot(
        drone.position.x - drone.missionTarget.x,
        drone.position.y - drone.missionTarget.y,
        drone.position.z - drone.missionTarget.z,
      )
    : null;
});
const phase = computed(() =>
  waiting.value
    ? "正在生成场景"
    : snapshot.value?.finished
      ? "对抗结束"
      : running.value
        ? "仿真运行中"
        : (snapshot.value?.time ?? 0) > 0
          ? "仿真已暂停"
          : "场景就绪",
);
const events = computed(() =>
  [...(snapshot.value?.events ?? [])].reverse().slice(0, 12),
);
const magnitude = (v?: { x: number; y: number; z: number }) =>
  v ? Math.hypot(v.x, v.y, v.z) : 0;
const timeText = (time: number) => {
  const ticks = Math.round(time * 100);
  return `${Math.floor(ticks / 6000)
    .toString()
    .padStart(2, "0")}:${((ticks % 6000) / 100).toFixed(2).padStart(5, "0")}`;
};
function choose(drone: Drone) {
  selectedId.value = drone.id;
  renderer?.selectDrone(drone.id);
  if (camera.value === "follow") renderer?.setCamera("follow", drone.id);
}
function setCamera(mode: "orbit" | "top" | "follow") {
  camera.value = mode;
  renderer?.setCamera(mode, selectedId.value ?? undefined);
}
function toggleRun() {
  if (snapshot.value?.finished || waiting.value) return;
  worker?.postMessage({ type: "running", value: !running.value });
}
function normalizeConfig() {
  const clamp = (n: number, low: number, high: number, fallback: number) =>
    Number.isFinite(n) ? Math.max(low, Math.min(high, n)) : fallback;
  config.seed = Math.round(clamp(Number(config.seed), 1, 999999, 42));
  config.redCount = Math.round(clamp(Number(config.redCount), 0, 8, 3));
  config.blueCount = Math.round(clamp(Number(config.blueCount), 1, 12, 6));
  config.redMaxSpeed = clamp(Number(config.redMaxSpeed), 4, 30, 18);
  config.blueMaxSpeed = clamp(Number(config.blueMaxSpeed), 4, 30, 12);
  config.redMaxAccel = clamp(Number(config.redMaxAccel), 2, 16, 8);
  config.blueMaxAccel = clamp(Number(config.blueMaxAccel), 2, 16, 5);
  config.captureRadius = clamp(Number(config.captureRadius), 3, 12, 6);
}
function reset(apply = false) {
  if (apply) {
    normalizeConfig();
    applied.value = { ...config };
  }
  running.value = false;
  waiting.value = true;
  worker?.postMessage({ type: "reset", config: { ...applied.value } });
}
function newCity() {
  config.seed = (Number(config.seed) || 42) + 1;
  reset(true);
}
function step() {
  worker?.postMessage({ type: "step" });
}
function keydown(event: KeyboardEvent) {
  if (event.code === "Escape") help.value = false;
  const target = event.target as HTMLElement;
  if (target.closest("input,select,textarea,button") || help.value) return;
  if (event.code === "Space") {
    event.preventDefault();
    toggleRun();
  }
}
watch(options, () => renderer?.setOptions({ ...options }));
watch(speed, (value) =>
  worker?.postMessage({ type: "speed", value: Number(value) }),
);
onMounted(() => {
  try {
    renderer = new CityRenderer(canvas.value!);
    renderer.setOptions({ ...options });
    renderer.onSelect = (id) => {
      const drone = snapshot.value?.drones.find((d) => d.id === id);
      if (drone) choose(drone);
    };
    worker = new Worker(new URL("./simulation.worker.ts", import.meta.url), {
      type: "module",
    });
    worker.onmessage = (event) => {
      if (event.data.type === "error") {
        error.value = event.data.message;
        running.value = false;
        waiting.value = false;
        return;
      }
      const next: SimulationSnapshot = event.data.snapshot;
      snapshot.value = next;
      running.value = event.data.running;
      waiting.value = false;
      if (!next.drones.some((d) => d.id === selectedId.value))
        selectedId.value = next.drones[0]?.id ?? null;
      renderer?.update(next);
      renderer?.selectDrone(selectedId.value);
    };
    worker.onerror = (event) => {
      error.value = event.message || "仿真线程加载失败，请刷新页面。";
      waiting.value = false;
    };
    reset();
  } catch (cause) {
    error.value = `三维场景初始化失败：${cause instanceof Error ? cause.message : cause}`;
    waiting.value = false;
  }
  window.addEventListener("keydown", keydown);
});
onBeforeUnmount(() => {
  worker?.terminate();
  renderer?.dispose();
  window.removeEventListener("keydown", keydown);
});
</script>

<template>
  <div class="app-shell">
    <header class="topbar">
      <a class="brand" href="./" aria-label="City Pursuit 首页"
        ><span class="brand-symbol">⌘</span
        ><span
          >CITY<span class="brand-light">PURSUIT</span
          ><small>城市无人机追逃实验室</small></span
        ></a
      >
      <div class="header-middle">
        <span class="header-tag">SANDBOX / 01</span
        ><span class="header-divider"></span><span>城市全域感知</span>
      </div>
      <div class="header-actions">
        <span class="version">LOCAL SIMULATION <b>v0.1</b></span
        ><button
          class="icon-button help-button"
          title="使用说明"
          aria-label="使用说明"
          @click="help = true"
        >
          ?
        </button>
      </div>
    </header>

    <main class="workspace">
      <aside class="sidebar config-panel">
        <div class="panel-heading">
          <div>
            <span class="eyebrow">ENVIRONMENT</span>
            <h2>场景配置</h2>
          </div>
          <span class="tiny-index">01</span>
        </div>
        <section class="scene-summary">
          <div class="scene-preview" aria-hidden="true">
            <span
              v-for="n in 14"
              :key="n"
              :style="{ height: `${18 + ((n * 19) % 52)}px` }"
            ></span
            ><i></i>
          </div>
          <div class="scene-name">
            <strong>都市街区</strong><span>PROCEDURAL CITY</span>
          </div>
          <div class="scene-dimensions">
            <span>600 × 600 <small>m</small></span
            ><span>高度 ≤ 150 <small>m</small></span>
          </div>
          <div class="seed-row">
            <label for="seed">场景种子</label
            ><input
              id="seed"
              v-model.number="config.seed"
              type="number"
              min="1"
              max="999999"
            /><button
              class="icon-button"
              title="生成下一个种子并重置"
              aria-label="更换城市"
              @click="newCity"
            >
              ↻
            </button>
          </div>
        </section>
        <section class="config-section">
          <div class="section-title">
            <h3>阵营与性能</h3>
            <span>m/s · m/s²</span>
          </div>
          <div class="team-settings red-settings">
            <div class="team-heading">
              <span class="team-dot red"></span><strong>红方 · 追击者</strong
              ><span class="team-role">PURSUER</span>
            </div>
            <label class="setting-row"
              ><span>无人机数量</span
              ><input
                v-model.number="config.redCount"
                aria-label="红方无人机数量"
                type="number"
                min="0"
                max="8"
            /></label>
            <label class="setting-row"
              ><span>最大速度</span
              ><input
                v-model.number="config.redMaxSpeed"
                aria-label="红方最大速度"
                type="number"
                min="4"
                max="30"
            /></label>
            <label class="setting-row"
              ><span>最大加速度</span
              ><input
                v-model.number="config.redMaxAccel"
                aria-label="红方最大加速度"
                type="number"
                min="2"
                max="16"
            /></label>
          </div>
          <div class="team-settings blue-settings">
            <div class="team-heading">
              <span class="team-dot blue"></span><strong>蓝方 · 逃逸者</strong
              ><span class="team-role">EVADER</span>
            </div>
            <label class="setting-row"
              ><span>无人机数量</span
              ><input
                v-model.number="config.blueCount"
                aria-label="蓝方无人机数量"
                type="number"
                min="1"
                max="12"
            /></label>
            <label class="setting-row"
              ><span>最大速度</span
              ><input
                v-model.number="config.blueMaxSpeed"
                aria-label="蓝方最大速度"
                type="number"
                min="4"
                max="30"
            /></label>
            <label class="setting-row"
              ><span>最大加速度</span
              ><input
                v-model.number="config.blueMaxAccel"
                aria-label="蓝方最大加速度"
                type="number"
                min="2"
                max="16"
            /></label>
          </div>
          <label class="setting-row capture-setting"
            ><span>捕获半径 <small>m</small></span
            ><input
              v-model.number="config.captureRadius"
              aria-label="捕获半径"
              type="number"
              min="3"
              max="12"
          /></label>
          <p
            v-if="
              config.redCount > 0 &&
              (config.redMaxSpeed <= config.blueMaxSpeed ||
                config.redMaxAccel <= config.blueMaxAccel)
            "
            class="config-warning"
          >
            当前红方性能不占优，可能无法完成全部捕获。
          </p>
          <button
            class="apply-button"
            :disabled="!dirty || waiting"
            @click="reset(true)"
          >
            {{ dirty ? "应用参数并重置" : "参数已应用"
            }}<span>{{ dirty ? "↗" : "✓" }}</span>
          </button>
        </section>
        <div class="blue-policy-note">
          <span class="eyebrow">BLUE BEHAVIOR</span>
          <strong>目标巡航 → 近敌规避 → 恢复任务</strong>
          <p>各自前往航点，到达后再选下一站。</p>
        </div>
        <section class="config-section display-section">
          <div class="section-title">
            <h3>显示图层</h3>
            <span>LAYERS</span>
          </div>
          <label class="toggle-row"
            ><span>飞行轨迹</span
            ><input v-model="options.showTrails" type="checkbox" role="switch"
          /></label>
          <label class="toggle-row"
            ><span>规划路径</span
            ><input v-model="options.showPaths" type="checkbox" role="switch"
          /></label>
          <label class="toggle-row"
            ><span>无人机编号</span
            ><input v-model="options.showLabels" type="checkbox" role="switch"
          /></label>
        </section>
        <div class="perception-note">
          <span class="status-dot"></span>
          <div>
            <strong>完全感知模式</strong>
            <p>全场景地图 · 全部无人机状态</p>
          </div>
          <span class="lock-icon">◎</span>
        </div>
      </aside>

      <section class="viewport" aria-label="三维城市仿真">
        <div ref="canvas" class="canvas-host"></div>
        <div class="viewport-header">
          <div class="view-title">
            <span class="eyebrow">LIVE VIEW</span>
            <h1>城市追逃场</h1>
            <p>
              {{ snapshot?.buildings.length ?? "—" }} 栋建筑<span> / </span
              >有界三维空域
            </p>
          </div>
          <div
            class="view-status"
            :class="{
              'is-running': running,
              'is-finished': snapshot?.finished,
            }"
          >
            <span class="status-dot"></span>{{ phase }}
          </div>
        </div>
        <div class="camera-controls" role="group" aria-label="相机视角">
          <button
            :class="{ active: camera === 'orbit' }"
            @click="setCamera('orbit')"
          >
            ◈ <span>自由视角</span></button
          ><button
            :class="{ active: camera === 'top' }"
            title="正交俯视：高度不会造成地图边界视差"
            @click="setCamera('top')"
          >
            ⊞ <span>俯视</span></button
          ><button
            :class="{ active: camera === 'follow' }"
            :disabled="!selected"
            @click="setCamera('follow')"
          >
            ⌖ <span>跟随</span>
          </button>
        </div>
        <div v-if="waiting && !error" class="loading-notice">
          <span class="spinner"></span>正在构建城市与飞行网络…
        </div>
        <div v-if="error" class="error-notice" role="alert">
          <strong>场景未能正常运行</strong>
          <p>{{ error }}</p>
          <p>请使用支持 WebGL 的浏览器，并检查硬件加速设置。</p>
        </div>
        <div v-if="snapshot?.finished" class="finish-card">
          <span class="finish-check">✓</span>
          <div>
            <span class="eyebrow">SIMULATION COMPLETE</span>
            <h2>蓝方已全部捕获</h2>
            <p>
              {{ snapshot.captured }} 次捕获 · 用时
              {{ timeText(snapshot.time) }}
            </p>
          </div>
          <button @click="reset()">重新演练 ↻</button>
        </div>
        <div class="viewport-footer">
          <div class="map-legend">
            <span><i class="team-dot red"></i>追击者</span
            ><span><i class="team-dot blue"></i>逃逸者</span
            ><span
              v-if="
                selected?.team === 'blue' &&
                selected.active &&
                selected.missionTarget &&
                options.showPaths
              "
              ><i class="goal-symbol">◇</i>任务目标</span
            >
            <span class="north-indicator">↑ N</span>
          </div>
          <div class="mouse-hint">拖动旋转 · 滚轮缩放 · 右键平移</div>
        </div>
        <div class="transport">
          <button
            class="play-button"
            :disabled="waiting || !!error || snapshot?.finished"
            @click="toggleRun"
          >
            <span>{{ snapshot?.finished ? "✓" : running ? "Ⅱ" : "▶" }}</span
            >{{
              snapshot?.finished
                ? "对抗结束"
                : running
                  ? "暂停仿真"
                  : (snapshot?.time ?? 0) > 0
                    ? "继续仿真"
                    : "开始仿真"
            }}
          </button>
          <button
            class="transport-button"
            :disabled="running || waiting || !!error || snapshot?.finished"
            title="推进一个物理步（1/60 秒）"
            @click="step"
          >
            ▹|<span>单步</span>
          </button>
          <button
            class="transport-button"
            :disabled="waiting || !!error"
            @click="reset()"
          >
            ↺<span>重置</span>
          </button>
          <span class="transport-divider"></span>
          <div class="clock">
            <span>仿真时间 · 分:秒</span>
            <strong>{{ timeText(snapshot?.time ?? 0) }}</strong>
          </div>
          <div class="speed-control">
            <label for="speed">倍速</label
            ><select id="speed" v-model.number="speed">
              <option :value="1">1×</option>
              <option :value="2">2×</option>
              <option :value="4">4×</option>
              <option :value="8">8×</option>
            </select>
          </div>
        </div>
      </section>

      <aside class="sidebar monitor-panel">
        <div class="panel-heading">
          <div>
            <span class="eyebrow">MISSION MONITOR</span>
            <h2>态势监控</h2>
          </div>
          <span class="tiny-index">02</span>
        </div>
        <div class="metrics">
          <div>
            <span>红方在线</span
            ><strong class="red-text">{{
              reds.length.toString().padStart(2, "0")
            }}</strong>
          </div>
          <div>
            <span>蓝方存活</span
            ><strong class="blue-text"
              >{{ remaining.toString().padStart(2, "0")
              }}<small>/ {{ blues.length }}</small></strong
            >
          </div>
        </div>
        <div class="capture-progress">
          <div>
            <span>捕获进度</span
            ><strong>{{ snapshot?.captured ?? 0 }} / {{ blues.length }}</strong>
          </div>
          <div class="progress-track">
            <i
              :style="{
                width: `${blues.length ? ((snapshot?.captured ?? 0) / blues.length) * 100 : 0}%`,
              }"
            ></i>
          </div>
        </div>
        <section class="fleet-section">
          <div class="section-title">
            <h3>无人机列表</h3>
            <span>选择以查看</span>
          </div>
          <div class="fleet-grid">
            <button
              v-for="drone in snapshot?.drones ?? []"
              :key="drone.id"
              :class="[
                'drone-chip',
                drone.team,
                { selected: drone.id === selectedId, captured: !drone.active },
              ]"
              :aria-pressed="drone.id === selectedId"
              :title="`${drone.id} · ${behaviorNames[drone.behavior]}`"
              @click="choose(drone)"
            >
              <span class="team-dot" :class="drone.team"></span>{{ drone.id
              }}<span v-if="!drone.active">×</span>
            </button>
          </div>
          <div class="behavior-counts" aria-label="蓝方行为统计">
            <span
              v-for="state in blueStates"
              :key="state.value"
              :class="state.value"
              >{{ state.label }} <b>{{ blueStateCounts[state.value] }}</b></span
            >
          </div>
        </section>
        <section v-if="selected" class="telemetry-section">
          <div class="section-title">
            <h3>
              <span class="team-dot" :class="selected.team"></span
              >{{ selected.id }} <span class="telemetry-name">飞行状态</span>
            </h3>
            <span :class="selected.active ? `${selected.team}-text` : ''">{{
              selected.active ? "● 在线" : "已捕获"
            }}</span>
          </div>
          <div class="drone-intent">
            <span>{{ selected.team === "red" ? "追踪目标" : "行为状态" }}</span
            ><strong :class="['behavior-label', selected.behavior]">{{
              !selected.active
                ? "任务结束"
                : selected.team === "red"
                  ? (selected.targetId ?? "搜索目标")
                  : behaviorNames[selected.behavior]
            }}</strong>
          </div>
          <div v-if="selected.team === 'blue'" class="mission-panel">
            <div class="mission-heading">
              <span><i class="goal-symbol">◇</i> 任务目标</span
              ><small>已到达 {{ selected.waypointsReached }} 个</small>
            </div>
            <template v-if="selected.missionTarget && selected.active">
              <div class="mission-coordinates">
                <span
                  >X <b>{{ selected.missionTarget.x.toFixed(0) }}</b></span
                ><span
                  >Y <b>{{ selected.missionTarget.y.toFixed(0) }}</b></span
                ><span
                  >Z <b>{{ selected.missionTarget.z.toFixed(0) }}</b></span
                ><small>m</small>
              </div>
              <p v-if="selected.behavior === 'evade'">临时绕行，保留原任务点</p>
              <p v-else-if="selected.behavior === 'recover'">
                威胁已减轻，正在恢复任务
              </p>
              <p v-else>距目标直线 {{ missionDistance?.toFixed(0) }} m</p>
            </template>
            <p v-else>
              {{ selected.active ? ((snapshot?.time ?? 0) === 0 ? "开始后分配航点" : "正在选择任务点") : "本机任务已结束" }}
            </p>
          </div>
          <div class="telemetry-values">
            <div>
              <span>速度</span
              ><strong
                >{{ magnitude(selected.velocity).toFixed(1)
                }}<small>m/s</small></strong
              >
            </div>
            <div>
              <span>高度</span
              ><strong
                >{{ selected.position.z.toFixed(1) }}<small>m</small></strong
              >
            </div>
          </div>
          <div class="vector-table">
            <div>
              <span>POSITION / m</span><span>X</span><span>Y</span
              ><span>Z</span>
            </div>
            <div>
              <span>位置</span><b>{{ selected.position.x.toFixed(1) }}</b
              ><b>{{ selected.position.y.toFixed(1) }}</b
              ><b>{{ selected.position.z.toFixed(1) }}</b>
            </div>
            <div>
              <span>速度</span><b>{{ selected.velocity.x.toFixed(1) }}</b
              ><b>{{ selected.velocity.y.toFixed(1) }}</b
              ><b>{{ selected.velocity.z.toFixed(1) }}</b>
            </div>
            <div>
              <span>加速度</span><b>{{ selected.acceleration.x.toFixed(1) }}</b
              ><b>{{ selected.acceleration.y.toFixed(1) }}</b
              ><b>{{ selected.acceleration.z.toFixed(1) }}</b>
            </div>
          </div>
          <div
            class="boundary-status"
            title="无人机中心到最近空域侧面的水平距离"
          >
            <span>距侧边界</span
            ><strong>{{ boundaryClearance.toFixed(1) }} m</strong>
          </div>
        </section>
        <section class="event-section">
          <div class="section-title">
            <h3>事件记录</h3>
            <span class="event-count">{{ snapshot?.events.length ?? 0 }}</span>
          </div>
          <div class="event-list">
            <div
              v-for="(event, i) in events"
              :key="`${event.time}-${i}`"
              class="event-item"
            >
              <span
                class="event-mark"
                :class="{ capture: event.kind === 'capture' }"
              ></span>
              <div>
                <time>{{ timeText(event.time) }}</time>
                <p>{{ event.message }}</p>
              </div>
            </div>
            <div v-if="!events.length" class="empty-events">
              <span>◷</span>
              <p>等待仿真开始</p>
              <small>捕获事件将在这里记录</small>
            </div>
          </div>
        </section>
        <div class="monitor-footer">
          <span>运动碰撞拦截</span
          ><strong>{{ snapshot?.collisions ?? 0 }}</strong
          ><span title="固定物理时间步">60 Hz</span>
        </div>
      </aside>
    </main>
    <footer class="statusbar">
      <span
        ><i class="status-dot"></i>LOCAL ENGINE<span class="status-separator"
          >/</span
        >质点运动模型</span
      ><span
        >XYZ 三轴 · Z 轴向上<span class="status-separator">/</span>SEED
        {{ applied.seed }}</span
      ><span class="keyboard-hint"><kbd>SPACE</kbd> 开始 / 暂停</span>
    </footer>
    <div v-if="help" class="modal-backdrop" @click.self="help = false">
      <section
        class="help-modal"
        role="dialog"
        aria-modal="true"
        aria-labelledby="help-title"
      >
        <button
          class="modal-close icon-button"
          aria-label="关闭说明"
          @click="help = false"
        >
          ×</button
        ><span class="eyebrow">QUICK GUIDE</span>
        <h2 id="help-title">欢迎来到城市追逃实验室</h2>
        <p>
          红方追击；蓝方平时执行自己的航点任务，遇到近距离威胁时规避，安全后继续任务。双方保持完全感知。
        </p>
        <ol>
          <li>点击「开始仿真」，或按空格启动。</li>
          <li>拖动观察城市；点击无人机或右侧编号查看三轴运动状态。</li>
          <li>
            选中蓝机可查看巡航／规避／恢复状态，金色菱形标记它的任务目标。将红方数量设为
            0 可单独观察巡航。
          </li>
          <li>
            「俯视」使用正交投影，便于核对边界；「跟随」观察选中无人机。三维线框表示可飞行空域。
          </li>
          <li>修改参数后点击「应用参数并重置」。同一种子可重复相同场景。</li>
        </ol>
        <p class="help-rule">
          捕获条件：距离进入捕获半径，且中间没有建筑阻挡。路径线是规划参考，飞行仍受速度与加速度限制。模型适度放大以便观察，实际安全半径为
          2 m。
        </p>
        <button class="play-button" @click="help = false">开始探索 ↗</button>
      </section>
    </div>
  </div>
</template>
