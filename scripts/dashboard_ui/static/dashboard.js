const MAX_TEMP = 70;
const DIAG_INTERVAL_MS = 2000;
const SLIDER_SEND_INTERVAL_MS = 50;

let torqueOn = false;
let pendingPositions = {};
let sliderSendTimer = null;

// Throttled slider: only send the latest value every 50ms
function onSlider(joint, value) {
    const valEl = document.getElementById('val-' + joint);
    if (valEl) valEl.textContent = parseFloat(value).toFixed(1);

    if (!torqueOn) return;

    pendingPositions[joint] = parseFloat(value);

    if (!sliderSendTimer) {
        sliderSendTimer = setTimeout(() => {
            const toSend = pendingPositions;
            pendingPositions = {};
            sliderSendTimer = null;
            fetch('/api/set_positions', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(toSend)
            });
        }, SLIDER_SEND_INTERVAL_MS);
    }
}

function readPositions() {
    fetch('/api/positions')
        .then(r => r.json())
        .then(data => {
            for (const [joint, val] of Object.entries(data)) {
                if (val === null) continue;
                const slider = document.getElementById('slider-' + joint);
                const valEl = document.getElementById('val-' + joint);
                if (slider) slider.value = val;
                if (valEl) valEl.textContent = val.toFixed(1);
            }
        });
}

function toggleTorque() {
    const action = torqueOn ? 'disable' : 'enable';
    fetch('/api/torque', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: action})
    })
    .then(r => r.json())
    .then(data => {
        torqueOn = data.torque_on;
        const btn = document.getElementById('torque-btn');
        if (torqueOn) {
            btn.textContent = 'Disable Torque';
            btn.className = 'btn btn-disable';
            readPositions();
        } else {
            btn.textContent = 'Enable Torque';
            btn.className = 'btn btn-enable';
        }
    });
}

function goNeutral() {
    if (!torqueOn) return;
    fetch('/api/neutral', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: '{}'})
        .then(r => r.json())
        .then(data => {
            if (!data.positions) return;
            for (const [joint, val] of Object.entries(data.positions)) {
                const slider = document.getElementById('slider-' + joint);
                const valEl = document.getElementById('val-' + joint);
                if (slider) slider.value = val;
                if (valEl) valEl.textContent = val.toFixed(1);
            }
        });
}

// Poll diagnostics
function fetchDiagnostics() {
    fetch('/api/diagnostics')
        .then(r => r.json())
        .then(data => {
            if (data.error) { console.error('diag error:', data.error); return; }
            for (const [joint, temp] of Object.entries(data.temps || {})) {
                const el = document.getElementById('temp-' + joint);
                if (!el) continue;
                el.textContent = temp.toFixed(0) + '\u00B0C';
                const pct = temp / MAX_TEMP * 100;
                el.className = 'col-temp' + (pct >= 90 ? ' temp-crit' : pct >= 70 ? ' temp-warn' : '');
            }
            for (const [joint, cur] of Object.entries(data.currents || {})) {
                const el = document.getElementById('cur-' + joint);
                if (!el) continue;
                el.textContent = cur.toFixed(0) + 'mA';
                const abs = Math.abs(cur);
                el.className = 'col-current' + (abs > 800 ? ' cur-crit' : abs > 400 ? ' cur-warn' : '');
            }
        })
        .catch(() => {});
}

setInterval(fetchDiagnostics, DIAG_INTERVAL_MS);
fetchDiagnostics();
readPositions();
