import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { WORLD_BOUNDS } from '../sim/types';
import type { Building, Drone, SimulationSnapshot, Vec3 } from '../sim/types';

type CameraMode = 'orbit' | 'top' | 'follow';
interface DisplayOptions { showTrails: boolean; showPaths: boolean; showLabels: boolean }
interface DroneVisual {
  group: THREE.Group;
  body: THREE.Group;
  rotors: THREE.Group[];
  label: THREE.Sprite;
  groundRing: THREE.Mesh;
  tether: THREE.Line;
  trail: THREE.Line;
  path: THREE.Line;
  selection: THREE.Mesh;
  from: THREE.Vector3;
  target: THREE.Vector3;
  velocity: THREE.Vector3;
  active: boolean;
  team: 'red' | 'blue';
}
interface MissionTargetVisual {
  group: THREE.Group;
  diamond: THREE.Mesh;
  label: THREE.Sprite;
  groundRing: THREE.Mesh;
  tether: THREE.Line;
}

const RED = 0xf16b58;
const BLUE = 0x15abc8;
const MISSION_GOLD = 0xd69b32;
const toThree = (v: Vec3) => new THREE.Vector3(v.x, v.z, -v.y);

/** Presentation only: the simulation is the sole owner of all physical state. */
export class CityRenderer {
  onSelect?: (id: string) => void;
  private readonly scene = new THREE.Scene();
  private readonly perspectiveCamera = new THREE.PerspectiveCamera(43, 1, 1, 2500);
  private readonly topCamera = new THREE.OrthographicCamera(-400, 400, 400, -400, 1, 2500);
  private camera: THREE.PerspectiveCamera | THREE.OrthographicCamera = this.perspectiveCamera;
  private readonly renderer: THREE.WebGLRenderer;
  private controls: OrbitControls;
  private readonly world = new THREE.Group();
  private readonly droneLayer = new THREE.Group();
  private readonly visuals = new Map<string, DroneVisual>();
  private readonly missionTargets = new Map<string, THREE.Vector3>();
  private missionTargetVisual: MissionTargetVisual | null = null;
  private readonly resizeObserver: ResizeObserver;
  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private readonly pointerStart = new THREE.Vector2();
  private readonly clock = new THREE.Clock();
  private animationId = 0;
  private citySeed: number | null = null;
  private selectedId: string | null = null;
  private followId: string | null = null;
  private mode: CameraMode = 'orbit';
  private options: DisplayOptions = { showTrails: true, showPaths: false, showLabels: true };
  private lastTime = -1;
  private arrivalTime = 0;
  private interpolationMs = 100;
  private lastArrival = 0;
  private disposed = false;
  private readonly pulses: { mesh: THREE.Mesh; age: number }[] = [];
  private readonly cameraObstacles: THREE.Box3[] = [];
  private capturedCount = 0;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.12;
    this.renderer.domElement.setAttribute('aria-label', '三维城市红蓝无人机追逃场景，可拖动旋转、滚轮缩放、点击无人机选择');
    this.renderer.domElement.style.display = 'block';
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.renderer.domElement.style.touchAction = 'none';
    container.appendChild(this.renderer.domElement);
    this.scene.background = new THREE.Color(0xd7dedc);
    this.scene.fog = new THREE.Fog(0xd7dedc, 1000, 1850);
    this.scene.add(this.world, this.droneLayer);

    const ambient = new THREE.HemisphereLight(0xf4fbff, 0x8f9987, 2.35);
    this.scene.add(ambient);
    const sun = new THREE.DirectionalLight(0xffeed5, 3.2);
    sun.position.set(-250, 520, 300);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -460;
    sun.shadow.camera.right = 460;
    sun.shadow.camera.top = 460;
    sun.shadow.camera.bottom = -460;
    sun.shadow.camera.near = 50;
    sun.shadow.camera.far = 1100;
    sun.shadow.normalBias = 1.1;
    sun.shadow.bias = -0.0002;
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xc3dcf0, 0.7);
    fill.position.set(200, 100, -200);
    this.scene.add(fill);

    this.camera.position.set(495, 500, 580);
    this.topCamera.up.set(0, 0, -1);
    this.controls = this.createControls('orbit');
    this.controls.target.set(0, 25, 0);
    this.controls.update();
    this.renderer.domElement.addEventListener('pointerdown', this.handlePointerDown);
    this.renderer.domElement.addEventListener('pointerup', this.handlePointerUp);
    this.resizeObserver = new ResizeObserver(this.resize);
    this.resizeObserver.observe(container);
    this.resize();
    this.animate();
  }

  update(snapshot: SimulationSnapshot): void {
    const reset = this.citySeed !== snapshot.seed || snapshot.time < this.lastTime;
    if (this.citySeed !== snapshot.seed) {
      this.clearGroup(this.world);
      this.buildCity(snapshot.buildings, snapshot.seed);
      this.citySeed = snapshot.seed;
    }
    if (reset) {
      for (const visual of this.visuals.values()) this.removeVisual(visual);
      this.visuals.clear();
      this.capturedCount = 0;
      this.lastTime = -1;
      for (const pulse of this.pulses) this.disposeObject(pulse.mesh);
      this.pulses.length = 0;
      this.clearMissionTarget();
    }
    // Cache only presentation coordinates, never retain or mutate simulation objects.
    this.missionTargets.clear();
    for (const drone of snapshot.drones) {
      if (drone.team === 'blue' && drone.active && drone.missionTarget) {
        this.missionTargets.set(drone.id, toThree(drone.missionTarget));
      }
    }
    // A fresh configuration can also arrive while both snapshots are at t=0.
    const changed = snapshot.time !== this.lastTime || snapshot.time === 0;
    if (changed) {
      const now = performance.now();
      this.interpolationMs = this.lastArrival ? THREE.MathUtils.clamp(now - this.lastArrival, 30, 150) : 100;
      this.lastArrival = now;
      this.arrivalTime = now;
    }
    const ids = new Set(snapshot.drones.map(drone => drone.id));
    for (const [id, visual] of this.visuals) {
      if (!ids.has(id)) { this.removeVisual(visual); this.visuals.delete(id); }
    }
    for (const drone of snapshot.drones) {
      let visual = this.visuals.get(drone.id);
      if (!visual) {
        visual = this.createDrone(drone);
        this.visuals.set(drone.id, visual);
      }
      visual.active = drone.active;
      visual.group.visible = drone.active;
      visual.groundRing.visible = drone.active;
      visual.tether.visible = drone.active;
      visual.trail.visible = this.options.showTrails && drone.trail.length > 1;
      visual.path.visible = this.options.showPaths && drone.active && drone.path.length > 1;
      visual.label.visible = this.options.showLabels;
      visual.selection.visible = this.selectedId === drone.id;
      if (changed) {
        visual.from.copy(visual.group.position);
        visual.target.copy(toThree(drone.position));
        visual.velocity.copy(toThree(drone.velocity));
        if (reset || snapshot.time === 0) visual.from.copy(visual.target);
        if (!drone.active) visual.group.position.copy(visual.target);
        this.setLine(visual.trail, drone.trail.slice(-600).map(toThree));
        this.setLine(visual.path, [toThree(drone.position), ...drone.path.map(toThree)]);
      }
    }
    if (snapshot.captured > this.capturedCount && !reset) {
      const recent = snapshot.events.filter(event => event.kind === 'capture').slice(-(snapshot.captured - this.capturedCount));
      for (const event of recent) if (event.position) this.addPulse(toThree(event.position));
    }
    this.capturedCount = snapshot.captured;
    this.lastTime = snapshot.time;
    this.syncMissionTarget();
  }

  setOptions(options: DisplayOptions): void {
    this.options = { ...this.options, ...options };
    for (const visual of this.visuals.values()) {
      visual.label.visible = this.options.showLabels;
      visual.trail.visible = this.options.showTrails;
      visual.path.visible = this.options.showPaths && visual.active;
    }
    this.syncMissionTarget();
  }

  setCamera(mode: CameraMode, droneId?: string): void {
    // OrbitControls caches the camera-up transform. Recreate it when switching
    // projection/up axes, rather than replacing only its object reference.
    this.controls.dispose();
    this.mode = mode;
    this.followId = droneId ?? this.selectedId;
    this.camera = mode === 'top' ? this.topCamera : this.perspectiveCamera;
    this.controls = this.createControls(mode);
    if (mode === 'top') {
      const x = (WORLD_BOUNDS.minX + WORLD_BOUNDS.maxX) / 2;
      const z = -(WORLD_BOUNDS.minY + WORLD_BOUNDS.maxY) / 2;
      this.controls.target.set(x, 0, z);
      this.topCamera.position.set(x, WORLD_BOUNDS.maxZ + 750, z);
      this.topCamera.zoom = 1;
      this.topCamera.updateProjectionMatrix();
    } else if (mode === 'orbit') {
      this.controls.target.set(0, 25, 0);
      this.camera.position.set(495, 500, 580);
    } else if (this.followId) {
      const visual = this.visuals.get(this.followId);
      if (visual) {
        this.controls.target.copy(visual.group.position);
        const preferred = visual.group.position.clone().add(new THREE.Vector3(95, 80, 120));
        this.camera.position.copy(this.visibleFollowPosition(visual.group.position, preferred));
      }
    }
    this.controls.update();
  }

  private createControls(mode: CameraMode): OrbitControls {
    const controls = new OrbitControls(this.camera, this.renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.075;
    controls.enableRotate = mode !== 'top';
    controls.screenSpacePanning = true;
    controls.minDistance = 35;
    controls.maxDistance = 1400;
    controls.minZoom = 0.45;
    controls.maxZoom = 6;
    controls.maxPolarAngle = mode === 'top' ? Math.PI : Math.PI / 2 - 0.04;
    controls.minPolarAngle = mode === 'top' ? 0 : 0.005;
    controls.mouseButtons = { LEFT: mode === 'top' ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
    controls.touches = { ONE: mode === 'top' ? THREE.TOUCH.PAN : THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };
    return controls;
  }

  selectDrone(id: string | null): void {
    this.selectedId = id;
    if (this.mode === 'follow') this.followId = id;
    for (const [droneId, visual] of this.visuals) {
      visual.selection.visible = id === droneId;
      const mat = visual.path.material as THREE.LineBasicMaterial;
      mat.opacity = id === droneId ? 0.9 : 0.4;
      visual.label.scale.set(id === droneId ? 23 : 19, id === droneId ? 8 : 6.6, 1);
    }
    this.syncMissionTarget();
  }

  dispose(): void {
    this.disposed = true;
    cancelAnimationFrame(this.animationId);
    this.resizeObserver.disconnect();
    this.renderer.domElement.removeEventListener('pointerdown', this.handlePointerDown);
    this.renderer.domElement.removeEventListener('pointerup', this.handlePointerUp);
    this.controls.dispose();
    this.clearMissionTarget();
    this.missionTargets.clear();
    this.disposeObject(this.scene);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private readonly resize = (): void => {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.perspectiveCamera.aspect = width / height;
    this.perspectiveCamera.updateProjectionMatrix();
    // Fit the complete footprint while reserving screen space for the top and
    // bottom UI overlays. Orthographic projection makes x/y independent of z.
    const worldWidth = WORLD_BOUNDS.maxX - WORLD_BOUNDS.minX;
    const worldDepth = WORLD_BOUNDS.maxY - WORLD_BOUNDS.minY;
    const availableWidth = Math.max(width - 48, width * 0.5);
    const availableHeight = Math.max(height - 160, height * 0.5);
    const pixelsPerMeter = Math.min(availableWidth / (worldWidth + 40), availableHeight / (worldDepth + 65));
    const halfWidth = width / pixelsPerMeter / 2;
    const halfHeight = height / pixelsPerMeter / 2;
    const verticalOffset = 16 / pixelsPerMeter;
    this.topCamera.left = -halfWidth;
    this.topCamera.right = halfWidth;
    this.topCamera.top = halfHeight + verticalOffset;
    this.topCamera.bottom = -halfHeight + verticalOffset;
    this.topCamera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  };

  private readonly animate = (): void => {
    if (this.disposed) return;
    this.animationId = requestAnimationFrame(this.animate);
    const dt = Math.min(this.clock.getDelta(), 0.1);
    const blend = THREE.MathUtils.clamp((performance.now() - this.arrivalTime) / this.interpolationMs, 0, 1);
    for (const visual of this.visuals.values()) {
      if (!visual.active) continue;
      visual.group.position.lerpVectors(visual.from, visual.target, blend);
      const speed = visual.velocity.length();
      if (speed > 0.5) {
        const heading = Math.atan2(visual.velocity.x, visual.velocity.z);
        const delta = Math.atan2(Math.sin(heading - visual.body.rotation.y), Math.cos(heading - visual.body.rotation.y));
        visual.body.rotation.y += delta * Math.min(1, dt * 6);
        visual.body.rotation.x = THREE.MathUtils.lerp(visual.body.rotation.x, Math.min(speed / 110, 0.15), dt * 5);
      }
      for (let index = 0; index < visual.rotors.length; index++) visual.rotors[index].rotation.y += dt * 40 * (index % 2 ? 1 : -1);
      visual.groundRing.position.set(visual.group.position.x, 0.24, visual.group.position.z);
      this.setLine(visual.tether, [new THREE.Vector3(visual.group.position.x, 0.4, visual.group.position.z), visual.group.position]);
    }
    this.controls.update();
    if (this.mode === 'follow' && this.followId) {
      const visual = this.visuals.get(this.followId);
      if (visual) {
        const delta = visual.group.position.clone().sub(this.controls.target).multiplyScalar(Math.min(dt * 6, 1));
        this.controls.target.add(delta);
        this.camera.position.add(delta);
        // Check the actual body, not just its label: a street-level drone can be
        // hidden by tall buildings even when the ordinary orbit view looks fine.
        if (!this.followViewClear(visual.group.position, this.camera.position)) {
          const visible = this.visibleFollowPosition(visual.group.position, this.camera.position);
          const transition = this.camera.position.clone().lerp(visible, Math.min(dt * 8, 1));
          this.camera.position.copy(this.followViewClear(visual.group.position, transition) ? transition : visible);
        }
        this.camera.lookAt(this.controls.target);
      }
    }
    for (let i = this.pulses.length - 1; i >= 0; i--) {
      const pulse = this.pulses[i];
      pulse.age += dt;
      pulse.mesh.scale.setScalar(1 + pulse.age * 9);
      (pulse.mesh.material as THREE.MeshBasicMaterial).opacity = Math.max(0, 0.8 - pulse.age / 1.7);
      pulse.mesh.lookAt(this.camera.position);
      if (pulse.age > 1.4) { this.disposeObject(pulse.mesh); this.pulses.splice(i, 1); }
    }
    if (this.missionTargetVisual) {
      // Keep the hollow diamond and short label legible in perspective and top views.
      this.missionTargetVisual.diamond.quaternion.copy(this.camera.quaternion);
      this.missionTargetVisual.label.position.set(0, 14, 0).applyQuaternion(this.camera.quaternion);
    }
    this.renderer.render(this.scene, this.camera);
  };

  /** Camera visibility is deliberately independent of simulation flight bounds. */
  private followViewClear(target: THREE.Vector3, position: THREE.Vector3): boolean {
    const direction = position.clone().sub(target);
    const length = direction.length();
    if (length < 1) return false;
    const ray = new THREE.Ray(target, direction.multiplyScalar(1 / length));
    const hit = new THREE.Vector3();
    for (const box of this.cameraObstacles) {
      if (box.containsPoint(position) || box.containsPoint(target)) return false;
      if (ray.intersectBox(box, hit) && hit.distanceToSquared(target) < length * length) return false;
    }
    return true;
  }

  private visibleFollowPosition(target: THREE.Vector3, preferred: THREE.Vector3): THREE.Vector3 {
    if (this.followViewClear(target, preferred)) return preferred;
    const offset = preferred.clone().sub(target);
    const heading = Math.atan2(offset.x, offset.z);
    const distance = THREE.MathUtils.clamp(Math.hypot(offset.x, offset.z), 70, 180);
    const height = THREE.MathUtils.clamp(offset.y, 65, 150);
    // Try nearby azimuths first to retain context, then approach more steeply.
    const angles = [0, Math.PI / 4, -Math.PI / 4, Math.PI / 2, -Math.PI / 2, Math.PI * 3 / 4, -Math.PI * 3 / 4, Math.PI];
    for (const [radius, elevation] of [[distance, height], [distance * 0.7, height * 1.5], [distance * 0.4, height * 1.8], [8, 130]]) {
      for (const angle of angles) {
        const candidate = target.clone().add(new THREE.Vector3(Math.sin(heading + angle) * radius!, elevation!, Math.cos(heading + angle) * radius!));
        if (this.followViewClear(target, candidate)) return candidate;
      }
    }
    // Buildings are vertical prisms; a free drone always has a clear overhead
    // direction. The tiny horizontal component keeps OrbitControls well-defined.
    return target.clone().add(new THREE.Vector3(0.01, 140, 0.01));
  }

  private buildCity(buildings: Building[], seed: number): void {
    this.cameraObstacles.length = 0;
    for (const building of buildings) {
      this.cameraObstacles.push(new THREE.Box3(
        new THREE.Vector3(building.x - building.width / 2 - 0.65, 0, -building.y - building.depth / 2 - 0.65),
        new THREE.Vector3(building.x + building.width / 2 + 0.65, building.height + 1.2, -building.y + building.depth / 2 + 0.65),
      ));
    }
    let rng = seed >>> 0;
    const random = () => { rng = (rng * 1664525 + 1013904223) >>> 0; return rng / 4294967296; };
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(2200, 2200), new THREE.MeshStandardMaterial({ color: 0xbfc7bd, roughness: 1 }));
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    ground.position.y = -0.4;
    this.world.add(ground);

    const asphalt = new THREE.MeshStandardMaterial({ color: 0x596369, roughness: 1 });
    const sidewalk = new THREE.MeshStandardMaterial({ color: 0xc4c7bd, roughness: 1 });
    const roadBase = new THREE.Mesh(new THREE.BoxGeometry(WORLD_BOUNDS.maxX - WORLD_BOUNDS.minX, 0.2, WORLD_BOUNDS.maxY - WORLD_BOUNDS.minY), asphalt);
    roadBase.position.set((WORLD_BOUNDS.minX + WORLD_BOUNDS.maxX) / 2, -0.08, -(WORLD_BOUNDS.minY + WORLD_BOUNDS.maxY) / 2);
    roadBase.receiveShadow = true;
    this.world.add(roadBase);
    const linePositions: number[] = [];
    const crossingBoxes: { x: number; z: number; sx: number; sz: number }[] = [];
    for (let i = -4; i <= 4; i++) {
      const coordinate = i * 64;
      for (let dash = -285; dash <= 285; dash += 12) {
        if (Math.abs(dash % 64) < 11 || Math.abs(dash % 64) > 53) continue;
        linePositions.push(coordinate, 0.12, dash, coordinate, 0.12, dash + 5);
        linePositions.push(dash, 0.12, coordinate, dash + 5, 0.12, coordinate);
      }
      for (let j = -4; j <= 4; j++) {
        for (let stripe = -2; stripe <= 2; stripe++) {
          crossingBoxes.push({ x: coordinate + 11, z: j * 64 + stripe * 2.2, sx: 3.5, sz: 0.8 });
          crossingBoxes.push({ x: coordinate + stripe * 2.2, z: j * 64 - 11, sx: 0.8, sz: 3.5 });
        }
      }
    }
    const roadLines = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(linePositions, 3)), new THREE.LineBasicMaterial({ color: 0xc1c4b7, transparent: true, opacity: 0.65 }));
    this.world.add(roadLines);
    const crosswalks = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 0.03, 1), new THREE.MeshBasicMaterial({ color: 0xb7bdb5 }), crossingBoxes.length);
    const transform = new THREE.Object3D();
    crossingBoxes.forEach((stripe, index) => {
      transform.position.set(stripe.x, 0.15, stripe.z);
      transform.scale.set(stripe.sx, 1, stripe.sz);
      transform.updateMatrix();
      crosswalks.setMatrixAt(index, transform.matrix);
    });
    this.world.add(crosswalks);

    const bodyMaterial = new THREE.MeshStandardMaterial({ roughness: 0.9, metalness: 0.02 });
    const cityBoxes = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), bodyMaterial, buildings.length);
    cityBoxes.castShadow = true;
    cityBoxes.receiveShadow = true;
    const roofs = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshStandardMaterial({ color: 0xe1e1d7, roughness: 1 }), buildings.length);
    roofs.castShadow = true;
    roofs.receiveShadow = true;
    const equipment: { x: number; y: number; z: number; sx: number; sy: number; sz: number }[] = [];
    const windows: { x: number; y: number; z: number; sideways: boolean; color: THREE.Color }[] = [];
    const treePositions: THREE.Vector3[] = [];
    const palette = [0xd2d1c4, 0xb9c2c0, 0xe0d9cb, 0xc2c7c4, 0xd0c7b6, 0xacb8b7, 0xd5d9d1];
    buildings.forEach((building, index) => {
      const { x, y, width, depth, height } = building;
      const z = -y;
      const block = new THREE.Mesh(new THREE.BoxGeometry(50, 0.6, 50), sidewalk);
      block.position.set(x, 0.25, z);
      block.receiveShadow = true;
      this.world.add(block);
      transform.position.set(x, height / 2 + 0.55, z);
      transform.scale.set(width, height, depth);
      transform.updateMatrix();
      cityBoxes.setMatrixAt(index, transform.matrix);
      cityBoxes.setColorAt(index, new THREE.Color(palette[Math.abs(building.style) % palette.length]));
      transform.position.y = height + 0.8;
      transform.scale.set(width + 0.9, 0.65, depth + 0.9);
      transform.updateMatrix();
      roofs.setMatrixAt(index, transform.matrix);
      const roofCount = height > 70 ? 2 : 1;
      for (let roof = 0; roof < roofCount; roof++) {
        const unitHeight = 1.8 + random() * 2;
        equipment.push({ x: x + (random() - 0.5) * width * 0.45, y: height + 1.2 + unitHeight / 2, z: z + (random() - 0.5) * depth * 0.45, sx: 4 + random() * 4, sy: unitHeight, sz: 3 + random() * 5 });
      }
      const floors = Math.floor((height - 5) / 6);
      for (let floor = 0; floor < floors; floor++) {
        const windowY = 5.1 + floor * 6;
        for (let side = 0; side < 4; side++) {
          const span = side < 2 ? width : depth;
          const columns = Math.floor((span - 3) / 5.4);
          for (let column = 0; column < columns; column++) {
            const offset = (column - (columns - 1) / 2) * 5.4;
            const color = new THREE.Color(random() > 0.91 ? 0xc9be93 : random() > 0.5 ? 0x748888 : 0x849795);
            windows.push({
              x: side < 2 ? x + offset : x + (side === 2 ? -1 : 1) * (width / 2 + 0.04),
              y: windowY,
              z: side < 2 ? z + (side === 0 ? -1 : 1) * (depth / 2 + 0.04) : z + offset,
              sideways: side >= 2,
              color,
            });
          }
        }
      }
      for (const corner of [-1, 1]) {
        treePositions.push(new THREE.Vector3(x + corner * 27, 0.4, z - 23));
        if (random() > 0.35) treePositions.push(new THREE.Vector3(x + corner * 27, 0.4, z + 23));
      }
    });
    this.world.add(cityBoxes, roofs);
    const windowMesh = new THREE.InstancedMesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshStandardMaterial({ roughness: 0.45, metalness: 0.2, side: THREE.DoubleSide }), windows.length);
    windows.forEach((window, index) => {
      transform.position.set(window.x, window.y, window.z);
      transform.scale.set(2.7, 2.2, 1);
      transform.rotation.y = window.sideways ? Math.PI / 2 : 0;
      transform.updateMatrix();
      windowMesh.setMatrixAt(index, transform.matrix);
      windowMesh.setColorAt(index, window.color);
    });
    this.world.add(windowMesh);
    const roofUnits = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshStandardMaterial({ color: 0x98a29b, roughness: 0.9 }), equipment.length);
    equipment.forEach((unit, index) => {
      transform.position.set(unit.x, unit.y, unit.z);
      transform.scale.set(unit.sx, unit.sy, unit.sz);
      transform.rotation.y = 0;
      transform.updateMatrix();
      roofUnits.setMatrixAt(index, transform.matrix);
    });
    roofUnits.castShadow = true;
    roofUnits.receiveShadow = true;
    this.world.add(roofUnits);
    this.createTrees(treePositions, random);
    this.addCityBoundary();
  }

  private createTrees(positions: THREE.Vector3[], random: () => number): void {
    const leaves = new THREE.InstancedMesh(new THREE.IcosahedronGeometry(1, 1), new THREE.MeshStandardMaterial({ roughness: 1 }), positions.length);
    const trunks = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.45, 0.6, 3, 5), new THREE.MeshStandardMaterial({ color: 0x777766, roughness: 1 }), positions.length);
    const transform = new THREE.Object3D();
    positions.forEach((position, index) => {
      const size = 2.5 + random() * 1.2;
      transform.position.copy(position).add(new THREE.Vector3(0, 5.1, 0));
      transform.scale.set(size, size * 1.2, size);
      transform.rotation.y = random() * Math.PI;
      transform.updateMatrix();
      leaves.setMatrixAt(index, transform.matrix);
      leaves.setColorAt(index, new THREE.Color().setHSL(0.22 + random() * 0.06, 0.13, 0.34 + random() * 0.1));
      transform.position.copy(position).add(new THREE.Vector3(0, 1.5, 0));
      transform.scale.set(1, 1, 1);
      transform.updateMatrix();
      trunks.setMatrixAt(index, transform.matrix);
    });
    leaves.castShadow = true;
    trunks.castShadow = true;
    this.world.add(leaves, trunks);
  }

  private addCityBoundary(): void {
    const { minX, maxX, minY, maxY, maxZ } = WORLD_BOUNDS;
    const minRenderZ = -maxY;
    const maxRenderZ = -minY;
    const corners = [new THREE.Vector3(minX, 0.2, minRenderZ), new THREE.Vector3(maxX, 0.2, minRenderZ), new THREE.Vector3(maxX, 0.2, maxRenderZ), new THREE.Vector3(minX, 0.2, maxRenderZ)];
    const perimeter = [...corners, corners[0]!];
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(perimeter), new THREE.LineDashedMaterial({ color: 0x47716b, dashSize: 6, gapSize: 3, transparent: true, opacity: 0.95 }));
    line.computeLineDistances();
    this.world.add(line);
    const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x52766d });
    for (const x of [minX, maxX]) for (const z of [minRenderZ, maxRenderZ]) {
      const horizontal = new THREE.Mesh(new THREE.BoxGeometry(18, 0.08, 1.2), markerMaterial);
      horizontal.position.set(x + (x === minX ? 1 : -1) * 8.5, 0.22, z);
      const vertical = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.08, 18), markerMaterial);
      vertical.position.set(x, 0.22, z + (z === minRenderZ ? 1 : -1) * 8.5);
      this.world.add(horizontal, vertical);
    }
    // The flight volume has the same footprint as the ground map. Its top and
    // uprights make perspective displacement at altitude visually explicit.
    const top = perimeter.map(point => new THREE.Vector3(point.x, maxZ, point.z));
    const airMaterial = new THREE.LineDashedMaterial({ color: 0x587c80, dashSize: 5, gapSize: 5, transparent: true, opacity: 0.3, depthWrite: false });
    const topFrame = new THREE.Line(new THREE.BufferGeometry().setFromPoints(top), airMaterial);
    topFrame.computeLineDistances();
    const uprights = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(corners.flatMap(point => [point, new THREE.Vector3(point.x, maxZ, point.z)])), airMaterial);
    uprights.computeLineDistances();
    this.world.add(topFrame, uprights);
    const north = this.textSprite('N', '#547572', false);
    north.position.set((minX + maxX) / 2, 3, minRenderZ - 27);
    north.scale.set(21, 15, 1);
    this.world.add(north);
  }

  private createDrone(drone: Drone): DroneVisual {
    const color = drone.team === 'red' ? RED : BLUE;
    const group = new THREE.Group();
    const body = new THREE.Group();
    group.add(body);
    body.scale.setScalar(1.6);
    const metal = new THREE.MeshStandardMaterial({ color: 0x243137, roughness: 0.4, metalness: 0.45 });
    const colored = new THREE.MeshStandardMaterial({ color, roughness: 0.35, metalness: 0.2, emissive: color, emissiveIntensity: 0.15 });
    const shell = new THREE.Mesh(new THREE.SphereGeometry(1.5, 12, 8), colored);
    shell.scale.set(1, 0.43, 1.35);
    shell.castShadow = true;
    body.add(shell);
    const belly = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.55, 2.3), metal);
    belly.position.y = -0.3;
    body.add(belly);
    const eye = new THREE.Mesh(new THREE.SphereGeometry(0.35, 8, 6), new THREE.MeshBasicMaterial({ color: 0xe6ffff }));
    eye.position.set(0, -0.05, 1.8);
    body.add(eye);
    const rotors: THREE.Group[] = [];
    const armLength = Math.sqrt(3.3 ** 2 * 2);
    for (let index = 0; index < 4; index++) {
      const x = index % 2 ? 3.3 : -3.3;
      const z = index < 2 ? 3.3 : -3.3;
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.25, armLength), metal);
      arm.position.set(x / 2, -0.06, z / 2);
      arm.rotation.y = Math.atan2(x, z);
      arm.castShadow = true;
      body.add(arm);
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.55, 0.45, 0.65, 10), colored);
      motor.position.set(x, 0, z);
      body.add(motor);
      const rotor = new THREE.Group();
      rotor.position.set(x, 0.44, z);
      const blade = new THREE.Mesh(new THREE.BoxGeometry(3.7, 0.06, 0.28), metal);
      rotor.add(blade);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(1.88, 20), new THREE.MeshBasicMaterial({ color: 0x253f48, transparent: true, opacity: 0.15, side: THREE.DoubleSide, depthWrite: false }));
      disc.rotation.x = -Math.PI / 2;
      rotor.add(disc);
      body.add(rotor);
      rotors.push(rotor);
    }
    const label = this.textSprite(drone.id, drone.team === 'red' ? '#f57b68' : '#37c2d9');
    label.position.y = 11;
    label.scale.set(19, 6.6, 1);
    group.add(label);
    const selection = new THREE.Mesh(new THREE.RingGeometry(8.4, 9, 40), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, side: THREE.DoubleSide, depthWrite: false }));
    selection.rotation.x = -Math.PI / 2;
    selection.position.y = -1.8;
    selection.visible = this.selectedId === drone.id;
    group.add(selection);
    group.position.copy(toThree(drone.position));
    group.traverse(object => { object.userData.droneId = drone.id; });
    const groundRing = new THREE.Mesh(new THREE.RingGeometry(3.7, 4.5, 24), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.65, side: THREE.DoubleSide, depthWrite: false }));
    groundRing.rotation.x = -Math.PI / 2;
    const tether = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.23, depthWrite: false }));
    const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.62, depthWrite: false }));
    const path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color, transparent: true, opacity: 0.5, dashSize: 3, gapSize: 3, depthWrite: false }));
    this.droneLayer.add(group, groundRing, tether, trail, path);
    return { group, body, rotors, label, groundRing, tether, trail, path, selection, from: group.position.clone(), target: group.position.clone(), velocity: toThree(drone.velocity), active: drone.active, team: drone.team };
  }

  private textSprite(text: string, accent: string, badge = true): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 96;
    const ctx = canvas.getContext('2d')!;
    if (badge) {
      ctx.fillStyle = '#192c31';
      ctx.globalAlpha = 0.94;
      ctx.beginPath();
      ctx.roundRect(5, 10, 246, 76, 16);
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.fillStyle = accent;
      ctx.beginPath();
      ctx.arc(36, 48, 8, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.fillStyle = badge ? '#f7faf9' : accent;
    ctx.font = '600 40px "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, badge ? 143 : 128, 49);
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    return new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthWrite: false, depthTest: true, toneMapped: false }));
  }

  private syncMissionTarget(): void {
    const target = this.selectedId ? this.missionTargets.get(this.selectedId) : undefined;
    if (!this.options.showPaths || !target) {
      this.clearMissionTarget();
      return;
    }
    let visual = this.missionTargetVisual;
    if (!visual) {
      const group = new THREE.Group();
      group.name = 'selected-blue-mission-target';
      const diamond = new THREE.Mesh(
        new THREE.RingGeometry(5.6, 7.2, 4),
        new THREE.MeshBasicMaterial({ color: MISSION_GOLD, transparent: true, opacity: 0.98, side: THREE.DoubleSide, depthWrite: false, depthTest: false, toneMapped: false }),
      );
      diamond.renderOrder = 5;
      const label = this.textSprite('任务目标', '#e7b24c');
      label.scale.set(28, 9.4, 1);
      label.material.depthTest = false;
      label.renderOrder = 6;
      const groundRing = new THREE.Mesh(
        new THREE.RingGeometry(7.8, 8.7, 40),
        new THREE.MeshBasicMaterial({ color: MISSION_GOLD, transparent: true, opacity: 0.82, side: THREE.DoubleSide, depthWrite: false, toneMapped: false }),
      );
      groundRing.rotation.x = -Math.PI / 2;
      const tether = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineDashedMaterial({ color: MISSION_GOLD, transparent: true, opacity: 0.8, dashSize: 3, gapSize: 2, depthWrite: false, toneMapped: false }),
      );
      group.add(diamond, label, groundRing, tether);
      this.droneLayer.add(group);
      visual = { group, diamond, label, groundRing, tether };
      this.missionTargetVisual = visual;
    }
    visual.group.position.copy(target);
    visual.groundRing.position.set(0, 0.32 - target.y, 0);
    this.setLine(visual.tether, [new THREE.Vector3(0, 0.5 - target.y, 0), new THREE.Vector3()]);
  }

  private clearMissionTarget(): void {
    if (this.missionTargetVisual) {
      this.disposeObject(this.missionTargetVisual.group);
      this.missionTargetVisual = null;
    }
  }

  private setLine(line: THREE.Line, points: THREE.Vector3[]): void {
    const existing = line.geometry.getAttribute('position');
    if (existing && existing.count === points.length) {
      points.forEach((point, index) => existing.setXYZ(index, point.x, point.y, point.z));
      existing.needsUpdate = true;
      line.geometry.computeBoundingSphere();
    } else {
      line.geometry.dispose();
      line.geometry = new THREE.BufferGeometry().setFromPoints(points.length > 0 ? points : [new THREE.Vector3()]);
    }
    if (line.material instanceof THREE.LineDashedMaterial) line.computeLineDistances();
  }

  private addPulse(position: THREE.Vector3): void {
    const mesh = new THREE.Mesh(new THREE.RingGeometry(1.5, 2, 40), new THREE.MeshBasicMaterial({ color: RED, transparent: true, opacity: 0.8, side: THREE.DoubleSide, depthWrite: false }));
    mesh.position.copy(position);
    this.droneLayer.add(mesh);
    this.pulses.push({ mesh, age: 0 });
  }

  private readonly handlePointerDown = (event: PointerEvent): void => {
    this.pointerStart.set(event.clientX, event.clientY);
  };

  private readonly handlePointerUp = (event: PointerEvent): void => {
    if (event.button !== 0 || this.pointerStart.distanceTo(new THREE.Vector2(event.clientX, event.clientY)) > 5) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.set((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const targets = [...this.visuals.values()].filter(visual => visual.active).map(visual => visual.group);
    const intersection = this.raycaster.intersectObjects(targets, true)[0];
    if (intersection) {
      const id = intersection.object.userData.droneId as string;
      this.selectDrone(id);
      this.onSelect?.(id);
    }
  };

  private removeVisual(visual: DroneVisual): void {
    for (const object of [visual.group, visual.groundRing, visual.tether, visual.trail, visual.path]) this.disposeObject(object);
  }

  private clearGroup(group: THREE.Group): void {
    for (const object of [...group.children]) this.disposeObject(object);
  }

  private disposeObject(root: THREE.Object3D): void {
    root.removeFromParent();
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    root.traverse(object => {
      // Instanced attributes are owned by the mesh, not its base geometry.
      if (object instanceof THREE.InstancedMesh) object.dispose();
      if (object instanceof THREE.DirectionalLight) object.shadow.dispose();
      const renderable = object as THREE.Mesh;
      if (renderable.geometry) geometries.add(renderable.geometry);
      if (renderable.material) for (const material of Array.isArray(renderable.material) ? renderable.material : [renderable.material]) materials.add(material);
    });
    for (const geometry of geometries) geometry.dispose();
    for (const material of materials) {
      const map = (material as THREE.MeshBasicMaterial).map;
      map?.dispose();
      material.dispose();
    }
  }
}
