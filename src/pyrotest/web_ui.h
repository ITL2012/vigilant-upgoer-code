// Embedded web UI for the PYRO TEST BENCH (see main.cpp).
#ifndef PYROTEST_WEB_UI_H
#define PYROTEST_WEB_UI_H

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PYRO TEST BENCH</title>
<style>
    :root { --bg:#0d0f12; --bg2:#161a22; --border:#232d38; --text:#f5f6f7; --dim:#8a99ad;
            --accent:#00c8ff; --ok:#00ff66; --warn:#ffaa00; --danger:#ff3355; }
    * { box-sizing:border-box; margin:0; padding:0; }
    body { background:var(--bg); color:var(--text); font-family:monospace; padding:16px; }
    h1 { font-size:1.2rem; letter-spacing:2px; }
    .flex { display:flex; align-items:center; gap:10px; flex-wrap:wrap; }
    .pill { padding:4px 14px; border-radius:20px; font-weight:bold; font-size:0.9rem; }
    .pill-armed { background:var(--danger); color:#fff; animation:pulse 1s infinite; }
    .pill-disarmed { background:#1a1f28; color:var(--dim); border:1px solid var(--border); }
    @keyframes pulse { 0%,100%{opacity:1;} 50%{opacity:0.55;} }
    .panel { background:var(--bg2); border:1px solid var(--border); border-radius:10px; padding:14px; margin-top:12px; }
    .panel h2 { font-size:0.95rem; color:var(--accent); margin-bottom:10px; letter-spacing:1px; }
    .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:10px; }
    .card { background:#1a1f28; border:1px solid var(--border); border-radius:8px; padding:12px; }
    .card h3 { font-size:0.8rem; color:var(--dim); text-transform:uppercase; letter-spacing:1px; margin-bottom:8px; }
    label { display:block; font-size:0.7rem; color:var(--dim); margin:6px 0 2px; text-transform:uppercase; }
    input[type=text], input[type=number] {
        width:100%; background:var(--bg); color:var(--text); border:1px solid var(--border);
        border-radius:4px; padding:6px 8px; font-family:monospace; font-size:0.9rem;
    }
    button { cursor:pointer; border:none; border-radius:6px; padding:10px 16px;
             font-family:monospace; font-weight:bold; font-size:0.9rem; letter-spacing:1px; }
    .btn-ok    { background:var(--ok); color:#000; }
    .btn-warn  { background:var(--warn); color:#000; }
    .btn-danger{ background:var(--danger); color:#fff; }
    .btn-dim   { background:#1a1f28; color:var(--accent); border:1px solid var(--border); }
    .btn-block { width:100%; }
    button:disabled { opacity:0.35; cursor:not-allowed; }
    .stat { font-size:1.5rem; font-weight:bold; font-variant-numeric:tabular-nums; }
    .stat.small { font-size:1.05rem; }
    .dim { color:var(--dim); }
    .ok   { color:var(--ok); }
    .warn { color:var(--warn); }
    .danger { color:var(--danger); }
    .firing { color:var(--warn); animation:pulse 0.5s infinite; }
    .log-box { background:#0a0c10; border:1px solid var(--border); border-radius:8px; padding:10px;
               font-size:0.75rem; height:260px; overflow-y:auto; white-space:pre-wrap; color:#7aa2cc; }
    .code-display { font-size:2.2rem; font-weight:bold; color:var(--warn); text-align:center;
                    background:var(--bg); border:1px dashed var(--warn); border-radius:8px;
                    padding:12px; margin:8px 0; letter-spacing:8px; }
    .hint { font-size:0.72rem; color:var(--dim); margin-top:4px; }
    a { color:var(--accent); }
</style>
</head>
<body>
<div class="flex">
    <h1>PYRO TEST BENCH</h1>
    <span id="arm-pill" class="pill pill-disarmed">DISARMED</span>
    <span class="dim">uptime <span id="uptime">0</span>s</span>
    <span style="flex:1"></span>
    <a href="http://192.168.4.1:81/update" target="_blank" style="font-size:0.75rem;">Firmware OTA :81</a>
</div>

<!-- ARM / DISARM -->
<div class="panel">
    <h2>ARM CONTROL</h2>
    <div class="flex">
        <button class="btn-danger" id="btn-arm" onclick="armBegin()">ARM</button>
        <input type="text" id="arm-code" placeholder="enter code" style="max-width:140px;">
        <button class="btn-ok" onclick="armConfirm()">CONFIRM ARM</button>
        <button class="btn-dim" onclick="disarm()">DISARM</button>
        <span class="dim" id="arm-countdown" style="font-size:0.8rem;"></span>
    </div>
    <div id="code-box" style="display:none;">
        <div class="code-display" id="code-val">0000</div>
        <div class="hint">Type this code into the box and press CONFIRM ARM. Code expires after 10s.</div>
    </div>
    <div class="hint">Arming powers the 5V rail and the arm rail (same as the flight program).<br>
        Fires are blocked while DISARMED. Auto-disarm after 90s armed without firing.</div>
</div>

<!-- CHANNELS -->
<div class="panel">
    <h2>CHANNELS</h2>
    <div class="grid" id="chan-grid"></div>
    <div class="flex" style="margin-top:10px;">
        <button class="btn-danger" onclick="fireAll()">FIRE ALL ENABLED</button>
        <button class="btn-dim" onclick="clearResults()">Clear Results</button>
    </div>
</div>

<!-- LOG -->
<div class="panel">
    <h2>EVENT LOG</h2>
    <div class="log-box" id="log"></div>
</div>

<script>
const CHAN = 3;
let armed = false;

function $id(id) { return document.getElementById(id); }

function fmtUs(us) {
    if (!us) return "—";
    if (us < 1000) return us + " us";
    if (us < 1000000) return (us / 1000).toFixed(1) + " ms";
    return (us / 1000000).toFixed(3) + " s";
}

function buildChannels() {
    const grid = $id('chan-grid');
    grid.innerHTML = '';
    for (let i = 0; i < CHAN; i++) {
        const card = document.createElement('div');
        card.className = 'card';
        card.innerHTML = `
            <h3>Channel ${i}</h3>
            <label>Label</label>
            <input type="text" id="lbl-${i}" maxlength="19">
            <label>Pin (GPIO, 999 = none)</label>
            <input type="number" id="pin-${i}" min="0" max="47">
            <label>Pulse (ms, max 5000)</label>
            <input type="number" id="ms-${i}" min="1" max="5000">
            <label style="display:inline;text-transform:none;margin-right:8px;">Enabled
                <input type="checkbox" id="en-${i}" style="accent-color:var(--accent);"></label>
            <div class="flex" style="margin-top:8px;">
                <button class="btn-dim" onclick="saveCfg(${i})" style="flex:1;">Save Config</button>
                <button class="btn-danger" id="fire-${i}" onclick="fire(${i})" style="flex:1;">FIRE</button>
            </div>
            <div style="margin-top:8px;font-size:0.8rem;">
                <span class="dim">last fire: </span><span id="last-${i}" class="dim">—</span>
                <span id="state-${i}" style="display:block;margin-top:2px;"></span>
            </div>`;
        grid.appendChild(card);
    }
}

function poll() {
    fetch('/status').then(r => r.json()).then(d => {
        armed = d.armed;
        const pill = $id('arm-pill');
        pill.innerText = armed ? "ARMED" : "DISARMED";
        pill.className = 'pill ' + (armed ? 'pill-armed' : 'pill-disarmed');
        $id('uptime').innerText = d.uptime_s;
        $id('btn-arm').disabled = armed;
        $id('arm-countdown').innerText =
            armed ? `auto-disarm in ${d.auto_disarm_s - d.armed_for_s}s` : '';

        for (let i = 0; i < CHAN; i++) {
            const c = d.channels[i];
            const fireBtn = $id('fire-' + i);
            fireBtn.disabled = !armed;

            const cur = ($id('pin-' + i).value == '' ? -1 : parseInt($id('pin-' + i).value));
            const curMs = ($id('ms-' + i).value == '' ? -1 : parseInt($id('ms-' + i).value));
            if (cur != c.pin) $id('pin-' + i).value = c.pin;
            if (curMs != c.pulse_ms) $id('ms-' + i).value = c.pulse_ms;
            if ($id('lbl-' + i).value != c.label) $id('lbl-' + i).value = c.label;
            $id('en-' + i).checked = c.enabled;

            const stateEl = $id('state-' + i);
            if (c.firing) {
                stateEl.className = 'firing';
                stateEl.innerText = '● FIRING NOW';
                $id('last-' + i).innerText = 'since ' + (c.started_ms / 1000).toFixed(1) + 's';
            } else {
                stateEl.className = '';
                stateEl.innerText = c.fired ? 'measured ' + fmtUs(c.measured_us) : '';
                $id('last-' + i).innerText = c.fired ? 'fired @ ' + (c.started_ms / 1000).toFixed(1) + 's' : '—';
            }
        }
    }).catch(() => {});
}
setInterval(poll, 300);

function pollLog() {
    fetch('/log').then(r => r.text()).then(t => {
        const box = $id('log');
        if (box.innerText != t) {
            box.innerText = t;
            box.scrollTop = box.scrollHeight;
        }
    }).catch(() => {});
}
setInterval(pollLog, 1000);

function armBegin() {
    if (!confirm('ARM the test bench? The 5V rail and arm rail will go HIGH once you confirm the code.')) return;
    fetch('/arm_begin', {method:'POST'}).then(r => r.text()).then(code => {
        if (code.length == 4) {
            $id('code-val').innerText = code;
            $id('code-box').style.display = 'block';
        } else {
            alert(code);
        }
    });
}
function armConfirm() {
    const code = $id('arm-code').value.trim();
    if (!code) { alert('Enter the code first.'); return; }
    fetch('/arm?code=' + encodeURIComponent(code), {method:'POST'})
        .then(r => r.text()).then(t => {
            if (t != 'ARMED') alert(t);
            $id('code-box').style.display = 'none';
            $id('arm-code').value = '';
            poll();
        });
}
function disarm() {
    if (!confirm('DISARM the test bench? 5V + arm rails go LOW.')) return;
    fetch('/disarm', {method:'POST'}).then(() => poll());
}

function saveCfg(i) {
    const pin = $id('pin-' + i).value;
    const ms  = $id('ms-' + i).value;
    const lbl = $id('lbl-' + i).value;
    const en  = $id('en-' + i).checked ? '1' : '0';
    fetch(`/config?ch=${i}&pin=${pin}&ms=${ms}&label=${encodeURIComponent(lbl)}&en=${en}`, {method:'POST'})
        .then(r => r.text()).then(t => { if (t != 'saved') alert(t); poll(); });
}

function fire(i) {
    if (!confirm(`FIRE channel ${i}? It will go HIGH for the configured pulse.`)) return;
    fetch('/fire?ch=' + i, {method:'POST'})
        .then(r => r.text()).then(t => { if (t != 'fired') alert(t); poll(); });
}
function fireAll() {
    if (!confirm('FIRE ALL ENABLED CHANNELS simultaneously?')) return;
    fetch('/fire_all', {method:'POST'}).then(() => poll());
}
function clearResults() {
    fetch('/reset_fire', {method:'POST'}).then(() => poll());
}

buildChannels();
poll();
</script>
</body>
</html>
)rawliteral";

#endif // PYROTEST_WEB_UI_H
