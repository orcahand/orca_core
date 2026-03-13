import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const MAX_TAXEL_FORCE = 5;
const FINGERS = ['thumb', 'index', 'middle', 'ring', 'pinky'];
const USE_THICK_ARROWS = true;

const scenes = {};
let taxelCoordinates = null;
let modelInfo = null;
let initialized = false;
let displayMode = 'direction'; // 'magnitude' or 'direction'
let colorScheme = 'heat'; // 'heat', 'intensity', or 'orca'
let forceThreshold = 0; // 0 = no filtering
let arrowLengthMult = 1.0;
let arrowThicknessMult = 1.0;

async function fetchCoordinates() {
    if (taxelCoordinates) return taxelCoordinates;
    const resp = await fetch('/api/taxel_coordinates');
    taxelCoordinates = await resp.json();
    return taxelCoordinates;
}

async function fetchModelInfo() {
    if (modelInfo) return modelInfo;
    const resp = await fetch('/api/sensor_model_info');
    modelInfo = await resp.json();
    return modelInfo;
}

function computeRawNormal(coords, index) {
    const p = new THREE.Vector3(coords[index].x, coords[index].y, coords[index].z);

    const dists = [];
    for (let i = 0; i < coords.length; i++) {
        if (i === index) continue;
        const q = new THREE.Vector3(coords[i].x, coords[i].y, coords[i].z);
        dists.push({ i, dist: p.distanceTo(q) });
    }
    dists.sort((a, b) => a.dist - b.dist);
    const neighbors = dists.slice(0, 6);

    // Sort neighbors angularly for consistent cross products
    const vecs = neighbors.map(n => {
        return {
            i: n.i,
            v: new THREE.Vector3(
                coords[n.i].x - p.x,
                coords[n.i].y - p.y,
                coords[n.i].z - p.z,
            ),
        };
    });

    // Approximate normal from first cross product to define a reference plane
    const refNormal = new THREE.Vector3().crossVectors(vecs[0].v, vecs[1].v).normalize();
    if (refNormal.lengthSq() < 0.001) refNormal.set(0, 0, 1);

    // Project neighbors into tangent plane and sort by angle
    const tangentX = new THREE.Vector3();
    const up = new THREE.Vector3(0, 1, 0);
    if (Math.abs(refNormal.dot(up)) > 0.99) up.set(1, 0, 0);
    tangentX.crossVectors(up, refNormal).normalize();
    const tangentY = new THREE.Vector3().crossVectors(refNormal, tangentX).normalize();

    vecs.sort((a, b) => {
        const angleA = Math.atan2(a.v.dot(tangentY), a.v.dot(tangentX));
        const angleB = Math.atan2(b.v.dot(tangentY), b.v.dot(tangentX));
        return angleA - angleB;
    });

    // Accumulate cross products of angularly-sorted consecutive neighbors
    const normal = new THREE.Vector3(0, 0, 0);
    for (let i = 0; i < vecs.length; i++) {
        const next = (i + 1) % vecs.length;
        normal.add(new THREE.Vector3().crossVectors(vecs[i].v, vecs[next].v));
    }
    normal.normalize();

    return normal;
}

function computeAllNormals(coords) {
    const positions = coords.map(c => new THREE.Vector3(c.x, c.y, c.z));
    const centroid = new THREE.Vector3(0, 0, 0);
    positions.forEach(p => centroid.add(p));
    centroid.divideScalar(positions.length);

    const rawNormals = coords.map((_, i) => computeRawNormal(coords, i));

    // Centroid is inside the convex fingertip, so "away from centroid" = outward.
    // Orient all normals with a clear centroid vector using the centroid test.
    let maxDist = 0;
    const centroidDists = positions.map(p => {
        const d = p.distanceTo(centroid);
        if (d > maxDist) maxDist = d;
        return d;
    });

    const ambiguousThreshold = maxDist * 0.15;
    const oriented = new Array(coords.length).fill(false);

    // First pass: orient normals where centroid direction is clear
    for (let i = 0; i < coords.length; i++) {
        if (centroidDists[i] > ambiguousThreshold) {
            const toPoint = new THREE.Vector3().subVectors(positions[i], centroid);
            if (rawNormals[i].dot(toPoint) < 0) rawNormals[i].negate();
            oriented[i] = true;
        }
    }

    // Second pass: orient ambiguous points (near centroid) using nearest oriented neighbor
    for (let i = 0; i < coords.length; i++) {
        if (oriented[i]) continue;

        let bestDist = Infinity;
        let bestIdx = -1;
        for (let j = 0; j < coords.length; j++) {
            if (!oriented[j]) continue;
            const d = positions[i].distanceTo(positions[j]);
            if (d < bestDist) { bestDist = d; bestIdx = j; }
        }

        if (bestIdx >= 0 && rawNormals[i].dot(rawNormals[bestIdx]) < 0) {
            rawNormals[i].negate();
        }
        oriented[i] = true;
    }

    return rawNormals;
}

async function loadSTLMesh(modelName) {
    try {
        const resp = await fetch(`/api/stl_file/${modelName}`);
        if (!resp.ok) return null;
        const buffer = await resp.arrayBuffer();

        const loader = new STLLoader();
        const geometry = loader.parse(buffer);
        geometry.computeVertexNormals();

        const material = new THREE.MeshPhongMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: 0.22,
            side: THREE.DoubleSide,
            depthWrite: false,
            shininess: 30,
        });

        return new THREE.Mesh(geometry, material);
    } catch (e) {
        console.warn(`Failed to load STL for ${modelName}:`, e);
        return null;
    }
}

function createCustomArrow(position, sphereRadius) {
    const group = new THREE.Group();
    group.position.copy(position);

    const shaftRadius = sphereRadius * 0.6;
    const headRadius = sphereRadius * 1.8;
    const headHeight = sphereRadius * 3.5;

    const shaftGeometry = new THREE.CylinderGeometry(shaftRadius, shaftRadius, 1, 8);
    shaftGeometry.translate(0, 0.5, 0);
    const shaftMaterial = new THREE.MeshPhongMaterial({
        color: 0xff0000,
        emissive: 0x220000,
        shininess: 30,
    });
    const shaft = new THREE.Mesh(shaftGeometry, shaftMaterial);
    group.add(shaft);

    const headGeometry = new THREE.ConeGeometry(headRadius, headHeight, 8);
    headGeometry.translate(0, 0.5, 0);
    const headMaterial = new THREE.MeshPhongMaterial({
        color: 0xff0000,
        emissive: 0x220000,
        shininess: 30,
    });
    const head = new THREE.Mesh(headGeometry, headMaterial);
    group.add(head);

    group.visible = false;
    group.userData = { shaft, head, shaftGeometry, headGeometry, headHeight };
    return group;
}

function updateCustomArrow(arrow, direction, length, color) {
    const { shaft, head, headHeight } = arrow.userData;

    // Update colors with emissive glow
    const emissive = color.clone().multiplyScalar(0.15);
    shaft.material.color.copy(color);
    shaft.material.emissive.copy(emissive);
    head.material.color.copy(color);
    head.material.emissive.copy(emissive);

    // Scale shaft to desired length (minus head), apply multipliers
    const scaledLength = length * arrowLengthMult;
    const shaftLength = Math.max(scaledLength - headHeight, scaledLength * 0.5);
    const thickScale = arrowThicknessMult;
    shaft.scale.set(thickScale, shaftLength, thickScale);

    // Position head at end of shaft, scale head thickness
    head.position.set(0, shaftLength, 0);
    head.scale.set(thickScale, 1, thickScale);

    // Orient the group to point in the given direction
    const up = new THREE.Vector3(0, 1, 0);
    const quaternion = new THREE.Quaternion().setFromUnitVectors(up, direction.clone().normalize());
    arrow.quaternion.copy(quaternion);
}

function createFingerScene(finger, coords) {
    const container = document.getElementById('taxels-3d-container');

    const wrapper = document.createElement('div');
    wrapper.className = 'taxel-3d-finger';
    wrapper.dataset.finger = finger;

    const label = document.createElement('div');
    label.className = 'taxel-finger-label';
    label.textContent = finger.charAt(0).toUpperCase() + finger.slice(1);
    wrapper.appendChild(label);

    const canvas = document.createElement('canvas');
    wrapper.appendChild(canvas);
    container.appendChild(wrapper);

    const width = 280;
    const height = 300;

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setClearColor(0x1a1d26);

    const scene = new THREE.Scene();

    const positions = coords.map(c => new THREE.Vector3(c.x, c.y, c.z));
    const box = new THREE.Box3().setFromPoints(positions);
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    camera.position.set(center.x, center.y - maxDim * 0.3, center.z + maxDim * 1.8);
    camera.lookAt(center);

    const controls = new OrbitControls(camera, canvas);
    controls.target.copy(center);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.update();

    // Lighting — darker ambient, stronger directional for contrast
    scene.add(new THREE.AmbientLight(0xffffff, 0.4));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(center.x + maxDim, center.y + maxDim, center.z + maxDim * 2);
    scene.add(dirLight);
    const backLight = new THREE.DirectionalLight(0xffffff, 0.4);
    backLight.position.set(center.x - maxDim, center.y - maxDim, center.z - maxDim);
    scene.add(backLight);

    // Taxel points
    const taxelMeshes = [];
    const normals = computeAllNormals(coords);
    const sphereRadius = maxDim * 0.012;
    const geometry = new THREE.SphereGeometry(sphereRadius, 8, 8);

    for (let i = 0; i < coords.length; i++) {
        const material = new THREE.MeshPhongMaterial({
            color: 0x4a5568,
            transparent: true,
            opacity: 0.4,
            shininess: 3,
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(coords[i].x, coords[i].y, coords[i].z);
        scene.add(mesh);
        taxelMeshes.push(mesh);
    }

    // Arrows
    const arrows = [];
    const arrowHeadLen = sphereRadius * 6;
    const arrowHeadWidth = sphereRadius * 4.5;

    if (USE_THICK_ARROWS) {
        for (let i = 0; i < coords.length; i++) {
            const pos = new THREE.Vector3(coords[i].x, coords[i].y, coords[i].z);
            const arrow = createCustomArrow(pos, sphereRadius);
            scene.add(arrow);
            arrows.push(arrow);
        }
    } else {
        for (let i = 0; i < coords.length; i++) {
            const arrow = new THREE.ArrowHelper(
                new THREE.Vector3(0, 0, 1),
                new THREE.Vector3(coords[i].x, coords[i].y, coords[i].z),
                0, 0xff0000, arrowHeadLen, arrowHeadWidth,
            );
            arrow.visible = false;
            scene.add(arrow);
            arrows.push(arrow);
        }
    }

    const state = {
        renderer, scene, camera, controls, canvas,
        taxelMeshes, arrows, normals, coords,
        sphereRadius, maxDim, center, animating: false,
        arrowHeadLen, arrowHeadWidth,
    };

    scenes[finger] = state;

    function animate() {
        if (!state.animating) return;
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }
    state.startAnimation = () => {
        if (!state.animating) {
            state.animating = true;
            animate();
        }
    };
    state.stopAnimation = () => { state.animating = false; };

    const ro = new ResizeObserver(entries => {
        for (const entry of entries) {
            const w = entry.contentRect.width;
            const h = 300;
            camera.aspect = w / h;
            camera.updateProjectionMatrix();
            renderer.setSize(w, h);
        }
    });
    ro.observe(wrapper);
}

async function addSTLModelToScene(finger, modelName) {
    const s = scenes[finger];
    if (!s) return;

    const mesh = await loadSTLMesh(modelName);
    if (!mesh) return;

    // STL and taxel coordinates share the same coordinate frame — no transform needed
    s.scene.add(mesh);
    s.stlMesh = mesh;

    // Refit camera to include both taxels and STL
    const combinedBox = new THREE.Box3().setFromObject(s.scene);
    const combinedCenter = combinedBox.getCenter(new THREE.Vector3());
    const combinedSize = combinedBox.getSize(new THREE.Vector3());
    const combinedMax = Math.max(combinedSize.x, combinedSize.y, combinedSize.z);
    s.controls.target.copy(combinedCenter);
    s.camera.position.set(
        combinedCenter.x,
        combinedCenter.y - combinedMax * 0.3,
        combinedCenter.z + combinedMax * 1.6,
    );
    s.controls.update();
}

// --- Coloring functions ---

const INACTIVE_COLOR = new THREE.Color(0x4a5568);

function forceToDirectionColor(fx, fy, fz, magnitude) {
    const absX = Math.abs(fx);
    const absY = Math.abs(fy);
    const alpha = Math.min(magnitude / MAX_TAXEL_FORCE, 1);

    if (magnitude < 0.1) return INACTIVE_COLOR;

    if (absX > absY) {
        return fx > 0
            ? new THREE.Color(0.94, 0.27, 0.27).lerp(new THREE.Color(0.29, 0.33, 0.37), 1 - alpha)
            : new THREE.Color(0.02, 0.71, 0.83).lerp(new THREE.Color(0.29, 0.33, 0.37), 1 - alpha);
    } else {
        return fy > 0
            ? new THREE.Color(0.06, 0.73, 0.51).lerp(new THREE.Color(0.29, 0.33, 0.37), 1 - alpha)
            : new THREE.Color(0.96, 0.62, 0.04).lerp(new THREE.Color(0.29, 0.33, 0.37), 1 - alpha);
    }
}

function forceToMagnitudeColor(magnitude) {
    if (magnitude < 0.1) return INACTIVE_COLOR;
    const t = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
    const hue = (1 - t) * 0.667; // blue -> red
    return new THREE.Color().setHSL(hue, 0.9, 0.45 + t * 0.15);
}

// --- Arrow color scheme functions (3D) ---

function arrowColorHeat(magnitude) {
    const t = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
    const hue = (1 - t) * 0.667;
    return new THREE.Color().setHSL(hue, 0.9, 0.45 + t * 0.15);
}

function arrowColorIntensity(magnitude) {
    const t = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
    const l = 0.1 + t * 0.85; // dark gray → near-white
    return new THREE.Color(l, l, l);
}

// ORCA palette: #232534 → #474f5e → #7f8ea2 → #bfc7d1 → #e5e7eb
const ORCA_STOPS = [
    new THREE.Color(0x474f5e), // low
    new THREE.Color(0x7f8ea2), // mid-low
    new THREE.Color(0xbfc7d1), // mid-high
    new THREE.Color(0xe5e7eb), // high
];

function arrowColorOrca(magnitude) {
    const t = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
    const scaled = t * (ORCA_STOPS.length - 1);
    const idx = Math.min(Math.floor(scaled), ORCA_STOPS.length - 2);
    const frac = scaled - idx;
    return ORCA_STOPS[idx].clone().lerp(ORCA_STOPS[idx + 1], frac);
}

function getArrowColor3D(magnitude) {
    switch (colorScheme) {
        case 'intensity': return arrowColorIntensity(magnitude);
        case 'orca': return arrowColorOrca(magnitude);
        default: return arrowColorHeat(magnitude);
    }
}

function updateScene(finger, taxelData) {
    const s = scenes[finger];
    if (!s || !taxelData) return;

    for (let i = 0; i < taxelData.length && i < s.taxelMeshes.length; i++) {
        const [fx, fy, fz] = taxelData[i];
        const magnitude = Math.sqrt(fx * fx + fy * fy + fz * fz);

        // If below threshold, reset to inactive and hide arrow
        const belowThreshold = forceThreshold > 0 && magnitude < forceThreshold;

        // Color based on current display mode
        const color = belowThreshold
            ? INACTIVE_COLOR
            : (displayMode === 'magnitude'
                ? forceToMagnitudeColor(magnitude)
                : forceToDirectionColor(fx, fy, fz, magnitude));
        s.taxelMeshes[i].material.color.copy(color);
        s.taxelMeshes[i].material.opacity = (!belowThreshold && magnitude > 0.1) ? 0.95 : 0.4;

        const showArrow = magnitude > 0.15 && !belowThreshold;

        if (showArrow) {
            const normal = s.normals[i];
            const up = new THREE.Vector3(0, 1, 0);
            if (Math.abs(normal.dot(up)) > 0.99) up.set(1, 0, 0);
            const tangentX = new THREE.Vector3().crossVectors(up, normal).normalize();
            const tangentY = new THREE.Vector3().crossVectors(normal, tangentX).normalize();

            const dir = new THREE.Vector3()
                .addScaledVector(tangentX, fx)
                .addScaledVector(tangentY, fy)
                .addScaledVector(normal, fz);
            dir.normalize();

            const norm = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
            const arrowLen = s.sphereRadius * (6 + norm * 24);
            const arrowColor = getArrowColor3D(magnitude);

            if (USE_THICK_ARROWS) {
                updateCustomArrow(s.arrows[i], dir, arrowLen, arrowColor);
                s.arrows[i].visible = true;
            } else {
                s.arrows[i].setDirection(dir);
                s.arrows[i].setLength(arrowLen * arrowLengthMult, s.arrowHeadLen * arrowThicknessMult, s.arrowHeadWidth * arrowThicknessMult);
                s.arrows[i].setColor(arrowColor);
                s.arrows[i].visible = true;
            }
        } else {
            s.arrows[i].visible = false;
        }
    }
}

// --- Sync display mode from 2D controls ---

function readDisplayMode() {
    const magToggle = document.getElementById('magnitude-mode-toggle');
    if (magToggle && magToggle.checked) return 'magnitude';
    return 'direction';
}

// Watch for mode changes from the radio buttons
const observer = new MutationObserver(() => { displayMode = readDisplayMode(); });
document.querySelectorAll('input[name="taxel-mode"]').forEach(el => {
    el.addEventListener('change', () => { displayMode = readDisplayMode(); });
});

// Watch for color scheme changes
const colorSchemeSelect = document.getElementById('color-scheme-select');
if (colorSchemeSelect) {
    colorSchemeSelect.addEventListener('change', (e) => {
        colorScheme = e.target.value;
    });
}

// Watch for threshold changes
window.addEventListener('threshold-changed', (e) => {
    forceThreshold = e.detail.threshold;
});

// Watch for arrow size changes
window.addEventListener('arrow-size-changed', (e) => {
    arrowLengthMult = e.detail.length;
    arrowThicknessMult = e.detail.thickness;
});

// --- Event listeners ---

window.addEventListener('taxel-data-update', (e) => {
    const taxels = e.detail;
    FINGERS.forEach(finger => {
        if (taxels[finger]) updateScene(finger, taxels[finger]);
    });
});

window.addEventListener('taxel-view-changed', async (e) => {
    if (e.detail.view === '3d') {
        await init3D();
        Object.values(scenes).forEach(s => s.startAnimation());
    } else {
        Object.values(scenes).forEach(s => s.stopAnimation());
    }
});

async function init3D() {
    if (initialized) return;

    const [coords, info] = await Promise.all([fetchCoordinates(), fetchModelInfo()]);

    // Create scenes for each finger
    FINGERS.forEach(finger => {
        if (coords[finger] && coords[finger].length > 0) {
            createFingerScene(finger, coords[finger]);
        }
    });
    initialized = true;

    // Load STL models in background
    const promises = FINGERS.map(async finger => {
        const fi = info[finger];
        if (!fi || !fi.has_stl) return;
        await addSTLModelToScene(finger, fi.model);
    });
    Promise.all(promises).then(() => {
        console.log('STL models loaded');
    });
}

// Self-initialize: module scripts are deferred, so the DOM is ready.
// Check if 3D is the active view and initialize immediately.
const view3dToggle = document.getElementById('view-3d-toggle');
if (view3dToggle && view3dToggle.checked) {
    displayMode = readDisplayMode();
    init3D().then(() => {
        Object.values(scenes).forEach(s => s.startAnimation());
    });
}
