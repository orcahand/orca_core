const MAX_FORCE_SCALE = 10;
const MIN_CIRCLE_RADIUS = 2;
const MAX_CIRCLE_RADIUS = 40;
const VISUALIZATION_RADIUS = 70;
const MAX_TAXEL_FORCE = 5;

const socket = io();

// State
let currentMode = 'taxels';
let taxelDisplayMode = 'direction'; // 'magnitude', 'direction', or 'arrows'
let arrowColorScheme = 'heat'; // 'heat', 'intensity', or 'orca'
let forceThreshold = 0.5; // default threshold in N
let arrowLengthMult = 1.0;
let arrowThicknessMult = 1.0;
let taxelCounts = { thumb: 127, index: 52, middle: 31, ring: 31, pinky: 31 };
let taxelGridsInitialized = false;
let activeSensors = {};
let taxelCoordinates = null; // Will be fetched from server

socket.on('connect', () => {
    console.log('WebSocket connected');
});

socket.on('disconnect', () => {
    console.log('WebSocket disconnected');
});

socket.on('force_update', (forces) => {
    updateForces(forces);
    updateActiveSensorsFromForces(forces);
});

socket.on('taxel_update', (taxels) => {
    updateTaxels(taxels);
    updateActiveSensorsFromTaxels(taxels);
});

socket.on('combined_update', (data) => {
    if (data.forces) {
        updateForces(data.forces);
        updateActiveSensorsFromForces(data.forces);
    }
    if (data.taxels) {
        updateTaxels(data.taxels);
        updateActiveSensorsFromTaxels(data.taxels);
    }
});

socket.on('mode_changed', (data) => {
    currentMode = data.mode;
    updatePanelVisibility();
    document.getElementById('current-mode').textContent = getModeLabel(data.mode);
    updateAutoDataTypeDisplay(data.mode);
});

socket.on('connection_status', (data) => {
    if (data.connected) {
        document.getElementById('connection-status').textContent = 'Connected';
        document.getElementById('connection-status').className = 'status-indicator connected';
        document.getElementById('connect-btn').disabled = true;
        document.getElementById('disconnect-btn').disabled = false;
        document.getElementById('refresh-btn').disabled = false;
        if (data.mode) {
            currentMode = data.mode;
            document.getElementById('mode-select').value = data.mode;
        }
        updateUI();
    } else {
        document.getElementById('connection-status').textContent = 'Disconnected';
        document.getElementById('connection-status').className = 'status-indicator disconnected';
        if (data.error) {
            showError(data.error);
        }
    }
});

socket.on('error', (data) => {
    showError(data.message);
});

socket.on('config_update', (data) => {
    console.log('Sensor configuration changed:', data);
    updateSensorConfig(data);
});

let currentView = '2d';

document.getElementById('connect-btn').addEventListener('click', connect);
document.getElementById('disconnect-btn').addEventListener('click', disconnect);
document.getElementById('refresh-btn').addEventListener('click', refresh);
document.getElementById('zero-btn').addEventListener('click', zeroSensors);
document.getElementById('reset-zero-btn').addEventListener('click', resetZero);
document.getElementById('scan-btn').addEventListener('click', scanPorts);
document.getElementById('mode-select').addEventListener('change', changeMode);
document.getElementById('magnitude-mode-toggle').addEventListener('change', () => setTaxelDisplayMode('magnitude'));
document.getElementById('direction-mode-toggle').addEventListener('change', () => setTaxelDisplayMode('direction'));
document.getElementById('arrows-mode-toggle').addEventListener('change', () => setTaxelDisplayMode('arrows'));
document.getElementById('color-scheme-select').addEventListener('change', (e) => {
    arrowColorScheme = e.target.value;
});
document.getElementById('arrow-length-slider').addEventListener('input', (e) => {
    arrowLengthMult = parseFloat(e.target.value);
    document.getElementById('arrow-length-value').textContent = arrowLengthMult.toFixed(1) + 'x';
    window.dispatchEvent(new CustomEvent('arrow-size-changed', { detail: { length: arrowLengthMult, thickness: arrowThicknessMult } }));
});
document.getElementById('arrow-thickness-slider').addEventListener('input', (e) => {
    arrowThicknessMult = parseFloat(e.target.value);
    document.getElementById('arrow-thickness-value').textContent = arrowThicknessMult.toFixed(1) + 'x';
    window.dispatchEvent(new CustomEvent('arrow-size-changed', { detail: { length: arrowLengthMult, thickness: arrowThicknessMult } }));
});
document.getElementById('threshold-toggle').addEventListener('change', (e) => {
    const input = document.getElementById('threshold-input');
    input.disabled = !e.target.checked;
    forceThreshold = e.target.checked ? parseFloat(input.value) || 0 : 0;
    window.dispatchEvent(new CustomEvent('threshold-changed', { detail: { threshold: forceThreshold } }));
});
document.getElementById('threshold-input').addEventListener('input', (e) => {
    const toggle = document.getElementById('threshold-toggle');
    if (toggle.checked) {
        forceThreshold = parseFloat(e.target.value) || 0;
        window.dispatchEvent(new CustomEvent('threshold-changed', { detail: { threshold: forceThreshold } }));
    }
});

// Auto-scan on page load
scanPorts();

function getModeLabel(mode) {
    switch (mode) {
        case 'resultant': return 'Resultant Force';
        case 'taxels': return 'Taxels Only';
        case 'combined': return 'Combined';
        default: return mode;
    }
}

async function scanPorts() {
    const select = document.getElementById('port-select');
    const scanBtn = document.getElementById('scan-btn');
    scanBtn.disabled = true;
    scanBtn.textContent = '...';
    try {
        const response = await fetch('/api/ports');
        const ports = await response.json();
        const previousValue = select.value;
        select.innerHTML = '';
        if (ports.length === 0) {
            const opt = document.createElement('option');
            opt.value = '/dev/ttyACM0';
            opt.textContent = 'No ports found — /dev/ttyACM0';
            select.appendChild(opt);
        } else {
            ports.forEach(p => {
                const opt = document.createElement('option');
                opt.value = p.device;
                const label = p.is_sensor_adapter ? `${p.device} (Sensor Adapter)` : `${p.device} — ${p.description}`;
                opt.textContent = label;
                if (p.is_sensor_adapter) opt.style.fontWeight = '600';
                select.appendChild(opt);
            });
            // Re-select previous value if still present, otherwise keep first (best match)
            if ([...select.options].some(o => o.value === previousValue)) {
                select.value = previousValue;
            }
        }
    } catch (error) {
        showError('Port scan failed: ' + error.message);
    } finally {
        scanBtn.disabled = false;
        scanBtn.textContent = 'Scan';
    }
}

async function fetchTaxelCoordinates() {
    try {
        const response = await fetch('/api/taxel_coordinates');
        taxelCoordinates = await response.json();
        return taxelCoordinates;
    } catch (error) {
        console.error('Failed to fetch taxel coordinates:', error);
        return null;
    }
}

async function connect() {
    const port = document.getElementById('port-select').value;
    const mode = document.getElementById('mode-select').value;
    try {
        // Fetch coordinates before connecting
        if (!taxelCoordinates) {
            await fetchTaxelCoordinates();
        }

        const response = await fetch('/api/connect', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({port: port, mode: mode})
        });
        const data = await response.json();
        if (data.success) {
            showError(null);
            currentMode = data.mode;
            if (data.config && data.config.num_taxels) {
                taxelCounts = data.config.num_taxels;
            }
            document.getElementById('zero-btn').disabled = false;
            initializeTaxelGrids();
            updatePanelVisibility();
            updateAutoDataTypeDisplay(data.mode);
            updateUI();
        } else {
            showError(data.message);
        }
    } catch (error) {
        showError('Connection failed: ' + error.message);
    }
}

async function disconnect() {
    try {
        const response = await fetch('/api/disconnect', {method: 'POST'});
        const data = await response.json();
        if (data.success) {
            showError(null);
            document.getElementById('connection-status').textContent = 'Disconnected';
            document.getElementById('connection-status').className = 'status-indicator disconnected';
            document.getElementById('connect-btn').disabled = false;
            document.getElementById('disconnect-btn').disabled = true;
            document.getElementById('refresh-btn').disabled = true;
            document.getElementById('zero-btn').disabled = true;
            document.getElementById('reset-zero-btn').disabled = true;
            document.getElementById('reset-zero-btn').style.display = 'none';
            document.getElementById('stream-status').style.display = 'none';
        }
    } catch (error) {
        showError('Disconnect failed: ' + error.message);
    }
}

async function zeroSensors() {
    const btn = document.getElementById('zero-btn');
    btn.disabled = true;
    btn.textContent = 'Zeroing...';
    try {
        const response = await fetch('/api/zero', {method: 'POST'});
        const data = await response.json();
        if (data.success) {
            showError(null);
            document.getElementById('reset-zero-btn').style.display = '';
            document.getElementById('reset-zero-btn').disabled = false;
        } else {
            showError(data.message);
        }
    } catch (error) {
        showError('Zero failed: ' + error.message);
    } finally {
        btn.disabled = false;
        btn.textContent = 'Zero';
    }
}

async function resetZero() {
    try {
        const response = await fetch('/api/clear_zero', {method: 'POST'});
        const data = await response.json();
        if (data.success) {
            showError(null);
            document.getElementById('reset-zero-btn').style.display = 'none';
            document.getElementById('reset-zero-btn').disabled = true;
        } else {
            showError(data.message);
        }
    } catch (error) {
        showError('Reset zero failed: ' + error.message);
    }
}

async function changeMode() {
    const mode = document.getElementById('mode-select').value;
    try {
        const response = await fetch('/api/mode', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({mode: mode})
        });
        const data = await response.json();
        if (data.success) {
            currentMode = data.mode;
            updatePanelVisibility();
            document.getElementById('current-mode').textContent = getModeLabel(data.mode);
            updateAutoDataTypeDisplay(data.mode);
        } else {
            showError(data.message);
            document.getElementById('mode-select').value = currentMode;
        }
    } catch (error) {
        showError('Mode change failed: ' + error.message);
        document.getElementById('mode-select').value = currentMode;
    }
}

function setTaxelDisplayMode(mode) {
    taxelDisplayMode = mode;

    const magLegend = document.querySelector('.magnitude-legend');
    const dirLegend = document.querySelector('.direction-legend');
    const arrowsLegend = document.querySelector('.arrows-legend');

    magLegend.style.display = mode === 'magnitude' ? 'inline-block' : 'none';
    dirLegend.style.display = mode === 'direction' ? 'inline-block' : 'none';
    arrowsLegend.style.display = mode === 'arrows' ? 'inline-block' : 'none';

    // Clear arrows when switching away from arrows mode
    if (mode !== 'arrows') {
        clearAllArrows();
    }
}

function clearAllArrows() {
    document.querySelectorAll('.taxel-arrow').forEach(el => el.remove());
}

function updatePanelVisibility() {
    const forcesPanel = document.getElementById('forces-panel');
    const taxelsPanel = document.getElementById('taxels-panel');

    switch (currentMode) {
        case 'resultant':
            forcesPanel.style.display = 'block';
            taxelsPanel.style.display = 'none';
            break;
        case 'taxels':
            forcesPanel.style.display = 'none';
            taxelsPanel.style.display = 'block';
            break;
        case 'combined':
            forcesPanel.style.display = 'block';
            taxelsPanel.style.display = 'block';
            break;
    }
}

function initializeTaxelGrids() {
    const container = document.getElementById('taxels-container');
    container.innerHTML = '';

    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];

    fingers.forEach(finger => {
        const coords = taxelCoordinates ? taxelCoordinates[finger] : null;
        const numTaxels = coords ? coords.length : (taxelCounts[finger] || 31);

        const fingerDiv = document.createElement('div');
        fingerDiv.className = 'taxel-finger';
        fingerDiv.dataset.finger = finger;

        const label = document.createElement('div');
        label.className = 'taxel-finger-label';
        label.textContent = finger.charAt(0).toUpperCase() + finger.slice(1);
        fingerDiv.appendChild(label);

        if (coords && coords.length > 0) {
            // Use coordinate-based SVG rendering
            const svg = createCoordinateSVG(finger, coords);
            fingerDiv.appendChild(svg);
        } else {
            // Fallback to simple grid if no coordinates
            const grid = createFallbackGrid(finger, numTaxels);
            fingerDiv.appendChild(grid);
        }

        container.appendChild(fingerDiv);
    });

    taxelGridsInitialized = true;
}

function createCoordinateSVG(finger, coords) {
    // Calculate bounds
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;

    coords.forEach(c => {
        minX = Math.min(minX, c.x);
        maxX = Math.max(maxX, c.x);
        minY = Math.min(minY, c.y);
        maxY = Math.max(maxY, c.y);
    });

    const dataWidth = maxX - minX;
    const dataHeight = maxY - minY;

    // SVG dimensions - scale based on finger size
    const padding = 8;
    const taxelRadius = finger === 'thumb' ? 5 : 5;
    const scale = finger === 'thumb' ? 7.5 : 7;

    const svgWidth = dataWidth * scale + padding * 2;
    const svgHeight = dataHeight * scale + padding * 2;

    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    svg.setAttribute('class', 'taxel-svg');
    svg.setAttribute('viewBox', `0 0 ${svgWidth} ${svgHeight}`);
    svg.setAttribute('width', svgWidth);
    svg.setAttribute('height', svgHeight);
    svg.dataset.finger = finger;

    // Add taxel circles
    coords.forEach((coord, index) => {
        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');

        // Transform coordinates to SVG space
        // X: left-to-right maps to SVG x
        // Y: sensor Y (proximal-to-distal) maps to SVG y (top-to-bottom, inverted)
        const svgX = (coord.x - minX) * scale + padding;
        const svgY = svgHeight - ((coord.y - minY) * scale + padding); // Invert Y

        circle.setAttribute('cx', svgX);
        circle.setAttribute('cy', svgY);
        circle.setAttribute('r', taxelRadius);
        circle.setAttribute('fill', '#1a1a1a');
        circle.setAttribute('stroke', '#2a2a2a');
        circle.setAttribute('stroke-width', '0.5');
        circle.setAttribute('class', 'taxel-circle');
        circle.setAttribute('id', `taxel-${finger}-${index}`);
        circle.dataset.taxelIndex = index;

        svg.appendChild(circle);
    });

    return svg;
}

function createFallbackGrid(finger, numTaxels) {
    // Simple fallback grid when coordinates are not available
    const grid = document.createElement('div');
    grid.className = 'taxel-grid';
    grid.dataset.finger = finger;

    const cols = finger === 'thumb' ? 11 : 6;
    const rows = Math.ceil(numTaxels / cols);

    let taxelIndex = 0;
    for (let row = 0; row < rows; row++) {
        const rowDiv = document.createElement('div');
        rowDiv.className = 'taxel-row';

        for (let col = 0; col < cols && taxelIndex < numTaxels; col++) {
            const cell = document.createElement('div');
            cell.className = 'taxel-cell';
            cell.dataset.taxelIndex = taxelIndex;
            cell.id = `taxel-${finger}-${taxelIndex}`;
            rowDiv.appendChild(cell);
            taxelIndex++;
        }

        grid.appendChild(rowDiv);
    }

    return grid;
}

function updateTaxels(taxels) {
    if (!taxelGridsInitialized) return;

    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];

    fingers.forEach(finger => {
        const fingerTaxels = taxels[finger];
        if (!fingerTaxels) return;

        fingerTaxels.forEach((taxelData, index) => {
            const element = document.getElementById(`taxel-${finger}-${index}`);
            if (!element) return;

            const [fx, fy, fz] = taxelData;
            const magnitude = Math.sqrt(fx * fx + fy * fy + fz * fz);

            // Check if it's an SVG circle or a div cell
            const isSVG = element.tagName.toLowerCase() === 'circle';

            // If below threshold, reset to default and skip
            if (forceThreshold > 0 && magnitude < forceThreshold) {
                if (isSVG) {
                    element.setAttribute('fill', '#1a1a1a');
                } else {
                    element.style.backgroundColor = '#1a1a1a';
                }
                return;
            }

            if (taxelDisplayMode === 'arrows' && isSVG) {
                // Arrows mode - show 3D direction with intensity coloring
                updateTaxelArrow(element, fx, fy, fz, magnitude);
            } else if (taxelDisplayMode === 'direction') {
                // Direction-based coloring
                const absX = Math.abs(fx);
                const absY = Math.abs(fy);

                let color = '#1a1a1a';
                if (magnitude >= 0.1) {
                    const alpha = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
                    const opacity = 0.3 + alpha * 0.7;

                    if (absX > absY) {
                        if (fx > 0) {
                            color = `rgba(239, 68, 68, ${opacity})`;
                        } else {
                            color = `rgba(6, 182, 212, ${opacity})`;
                        }
                    } else {
                        if (fy > 0) {
                            color = `rgba(16, 185, 129, ${opacity})`;
                        } else {
                            color = `rgba(245, 158, 11, ${opacity})`;
                        }
                    }
                }

                if (isSVG) {
                    element.setAttribute('fill', color);
                } else {
                    element.style.backgroundColor = color;
                }
            } else {
                // Magnitude-based coloring (grayscale)
                const normalized = Math.min(magnitude / MAX_TAXEL_FORCE, 1);
                const gray = Math.round(60 + normalized * 180);
                const color = `rgb(${gray}, ${gray}, ${gray})`;

                if (isSVG) {
                    element.setAttribute('fill', color);
                } else {
                    element.style.backgroundColor = color;
                }
            }
        });
    });
}

function getArrowColor2D(normalized) {
    switch (arrowColorScheme) {
        case 'intensity': {
            const l = Math.round(15 + normalized * 85);
            return `hsl(0, 0%, ${l}%)`;
        }
        case 'orca': {
            // ORCA palette gradient: #474f5e → #7f8ea2 → #bfc7d1 → #e5e7eb
            const stops = [
                [71, 79, 94],    // #474f5e
                [127, 142, 162], // #7f8ea2
                [191, 199, 209], // #bfc7d1
                [229, 231, 235], // #e5e7eb
            ];
            const scaled = normalized * (stops.length - 1);
            const idx = Math.min(Math.floor(scaled), stops.length - 2);
            const frac = scaled - idx;
            const r = Math.round(stops[idx][0] + (stops[idx + 1][0] - stops[idx][0]) * frac);
            const g = Math.round(stops[idx][1] + (stops[idx + 1][1] - stops[idx][1]) * frac);
            const b = Math.round(stops[idx][2] + (stops[idx + 1][2] - stops[idx][2]) * frac);
            return `rgb(${r}, ${g}, ${b})`;
        }
        default: { // 'heat'
            const hue = (1 - normalized) * 240;
            const saturation = 70 + normalized * 30;
            const lightness = 55 - normalized * 10;
            return `hsl(${hue}, ${saturation}%, ${lightness}%)`;
        }
    }
}

function updateTaxelArrow(circleElement, fx, fy, fz, magnitude) {
    const svg = circleElement.closest('svg');
    if (!svg) return;

    const cx = parseFloat(circleElement.getAttribute('cx'));
    const cy = parseFloat(circleElement.getAttribute('cy'));
    const taxelId = circleElement.id;
    const arrowId = `arrow-${taxelId}`;

    // Remove existing arrow
    const existingArrow = document.getElementById(arrowId);
    if (existingArrow) existingArrow.remove();

    // Reset circle to light gray background
    circleElement.setAttribute('fill', '#0a0a0a');

    // Don't draw arrow if force is too small or below threshold
    if (magnitude < 0.1) return;
    if (forceThreshold > 0 && magnitude < forceThreshold) return;

    // Calculate arrow properties
    const normalized = Math.min(magnitude / MAX_TAXEL_FORCE, 1);

    // XY magnitude for arrow direction in plane
    const xyMag = Math.sqrt(fx * fx + fy * fy);

    // Arrow length based on XY magnitude, with Z affecting it
    const baseLength = (4 + normalized * 10) * arrowLengthMult;
    const zFactor = 1 + Math.abs(fz) / MAX_TAXEL_FORCE * 0.5;
    const arrowLength = baseLength * (xyMag > 0.1 ? 1 : 0.3) * zFactor;

    // Arrow direction (in SVG coordinates, Y is inverted)
    let angle = 0;
    if (xyMag > 0.1) {
        angle = Math.atan2(-fy, fx); // Negative fy because SVG Y is down
    }

    // End point of arrow
    const endX = cx + Math.cos(angle) * arrowLength;
    const endY = cy + Math.sin(angle) * arrowLength;

    // Color based on selected scheme
    const color = getArrowColor2D(normalized);

    // Create arrow group
    const arrowGroup = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    arrowGroup.setAttribute('id', arrowId);
    arrowGroup.setAttribute('class', 'taxel-arrow');

    // Arrow line
    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('x1', cx);
    line.setAttribute('y1', cy);
    line.setAttribute('x2', endX);
    line.setAttribute('y2', endY);
    line.setAttribute('stroke', color);
    line.setAttribute('stroke-width', (2 + normalized * 2.5) * arrowThicknessMult);
    line.setAttribute('stroke-linecap', 'round');
    arrowGroup.appendChild(line);

    // Arrowhead (triangle)
    if (arrowLength > 4) {
        const headLength = (3 + normalized * 3) * arrowThicknessMult;
        const headAngle = 0.6; // radians, about 35 degrees

        const head1X = endX - Math.cos(angle - headAngle) * headLength;
        const head1Y = endY - Math.sin(angle - headAngle) * headLength;
        const head2X = endX - Math.cos(angle + headAngle) * headLength;
        const head2Y = endY - Math.sin(angle + headAngle) * headLength;

        const arrowhead = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
        arrowhead.setAttribute('points', `${endX},${endY} ${head1X},${head1Y} ${head2X},${head2Y}`);
        arrowhead.setAttribute('fill', color);
        arrowGroup.appendChild(arrowhead);
    }

    svg.appendChild(arrowGroup);
}

async function refresh() {
    try {
        const response = await fetch('/api/refresh');
        const data = await response.json();
        if (data.connected) {
            updateStatus(data);
            if (data.forces) updateForces(data.forces);
            if (data.mode) {
                currentMode = data.mode;
                document.getElementById('mode-select').value = data.mode;
                updatePanelVisibility();
            }
        } else {
            showError(data.error || 'Not connected');
        }
    } catch (error) {
        showError('Refresh failed: ' + error.message);
    }
}

async function updateUI() {
    try {
        // Ensure we have coordinates
        if (!taxelCoordinates) {
            await fetchTaxelCoordinates();
        }

        const response = await fetch('/api/status');
        const data = await response.json();
        if (data.connected) {
            document.getElementById('connection-status').textContent = 'Connected';
            document.getElementById('connection-status').className = 'status-indicator connected';
            document.getElementById('connect-btn').disabled = true;
            document.getElementById('disconnect-btn').disabled = false;
            document.getElementById('refresh-btn').disabled = false;
            document.getElementById('stream-status').style.display = 'block';
            updateStatus(data);

            if (data.taxels) {
                taxelCounts = data.taxels;
                if (!taxelGridsInitialized) {
                    initializeTaxelGrids();
                }
            }

            if (data.mode) {
                currentMode = data.mode;
                document.getElementById('mode-select').value = data.mode;
                document.getElementById('current-mode').textContent = getModeLabel(data.mode);
                updatePanelVisibility();
            }

            const forcesResponse = await fetch('/api/forces');
            const forces = await forcesResponse.json();
            updateForces(forces);
        } else {
            document.getElementById('connection-status').textContent = 'Disconnected';
            document.getElementById('connection-status').className = 'status-indicator disconnected';
        }
    } catch (error) {
        showError('Update failed: ' + error.message);
    }
}

function updateAutoDataTypeDisplay(mode) {
    const resultant = mode === 'resultant' || mode === 'combined';
    const taxels = mode === 'taxels' || mode === 'combined';
    document.getElementById('auto-data-type').innerHTML = `
        Resultant Force: ${resultant ? '✓' : '✗'}<br>
        Individual Taxels: ${taxels ? '✓' : '✗'}
    `;
}

function updateStatus(data) {
    document.getElementById('hardware-version').textContent = data.hardware_version || '-';

    if (data.mode) {
        document.getElementById('current-mode').textContent = getModeLabel(data.mode);
        updateAutoDataTypeDisplay(data.mode);
    }

    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];
    fingers.forEach(finger => {
        const card = document.querySelector(`.sensor-card[data-finger="${finger}"]`);
        const status = card.querySelector('.sensor-status');
        const taxels = card.querySelector('.sensor-taxels span');

        if (data.sensors && data.sensors[finger]) {
            status.className = 'sensor-status connected';
            status.textContent = '●';
            if (data.taxels) {
                taxels.textContent = data.taxels[finger] || 0;
            }
        } else {
            status.className = 'sensor-status disconnected';
            status.textContent = '●';
            taxels.textContent = '-';
        }
    });
}

function updateForces(forces) {
    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];

    fingers.forEach(finger => {
        if (!forces[finger]) return;

        const [fx, fy, fz] = forces[finger];
        const magnitude = Math.sqrt(fx*fx + fy*fy + fz*fz);

        document.getElementById(`fx-${finger}`).textContent = fx.toFixed(1);
        document.getElementById(`fy-${finger}`).textContent = fy.toFixed(1);
        document.getElementById(`fz-${finger}`).textContent = fz.toFixed(1);
        document.getElementById(`mag-${finger}`).textContent = magnitude.toFixed(1);

        const circle = document.getElementById(`force-circle-${finger}`);
        if (circle) {
            const centerX = 100;
            const centerY = 100;

            const normalizedMagnitude = Math.min(magnitude / MAX_FORCE_SCALE, 1);
            const radius = MIN_CIRCLE_RADIUS + (normalizedMagnitude * (MAX_CIRCLE_RADIUS - MIN_CIRCLE_RADIUS));

            const angle = Math.atan2(fy, fx);
            const distance = Math.min(normalizedMagnitude * VISUALIZATION_RADIUS, VISUALIZATION_RADIUS);
            const circleX = centerX + distance * Math.cos(angle);
            const circleY = centerY - distance * Math.sin(angle);

            circle.setAttribute('cx', circleX);
            circle.setAttribute('cy', circleY);
            circle.setAttribute('r', radius);

            const opacity = Math.min(0.3 + normalizedMagnitude * 0.7, 1);
            const color = magnitude > 1 ? '#ef4444' : '#3b82f6';
            circle.setAttribute('fill', color);
            circle.setAttribute('opacity', opacity);
        }
    });
}


function showError(message) {
    const panel = document.getElementById('error-panel');
    const msg = document.getElementById('error-message');
    if (message) {
        msg.textContent = message;
        panel.style.display = 'block';
    } else {
        panel.style.display = 'none';
    }
}

function updateActiveSensorsFromForces(forces) {
    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];
    fingers.forEach(finger => {
        if (forces[finger]) {
            activeSensors[finger] = true;
            updateSensorStatusDisplay(finger, true);
        }
    });
}

function updateActiveSensorsFromTaxels(taxels) {
    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];
    fingers.forEach(finger => {
        if (taxels[finger] && taxels[finger].length > 0) {
            activeSensors[finger] = true;
            updateSensorStatusDisplay(finger, true, taxels[finger].length);
        }
    });
}

function updateSensorStatusDisplay(finger, connected, taxelCount) {
    const card = document.querySelector(`.sensor-card[data-finger="${finger}"]`);
    if (!card) return;

    const status = card.querySelector('.sensor-status');
    if (connected) {
        status.className = 'sensor-status connected';
        status.textContent = '●';
    } else {
        status.className = 'sensor-status disconnected';
        status.textContent = '●';
    }

    if (taxelCount !== undefined) {
        const taxelsSpan = card.querySelector('.sensor-taxels span');
        if (taxelsSpan) {
            taxelsSpan.textContent = taxelCount;
        }
    }
}

function updateSensorConfig(data) {
    const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];

    fingers.forEach(finger => {
        const card = document.querySelector(`.sensor-card[data-finger="${finger}"]`);
        if (!card) return;

        const status = card.querySelector('.sensor-status');
        const taxelsSpan = card.querySelector('.sensor-taxels span');

        const isConnected = data.sensors && data.sensors[finger];

        if (isConnected) {
            status.className = 'sensor-status connected';
            status.textContent = '●';
            if (taxelsSpan && data.taxels) {
                taxelsSpan.textContent = data.taxels[finger] || 0;
            }
        } else {
            status.className = 'sensor-status disconnected';
            status.textContent = '●';
            if (taxelsSpan) {
                taxelsSpan.textContent = '-';
            }
        }

        // Update activeSensors tracking
        activeSensors[finger] = isConnected;
    });

    // Update taxel counts for grid reinitialization if needed
    if (data.taxels) {
        taxelCounts = data.taxels;
    }
}
