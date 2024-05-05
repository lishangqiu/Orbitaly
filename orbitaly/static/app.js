/* Orbitaly console — no framework, no build step. */
"use strict";

const $ = (id) => document.getElementById(id);

/* Chart tokens (kept in sync with style.css) */
const C = {
  grid: "#2c2c2a",
  axis: "#383835",
  ink: "#ffffff",
  ink2: "#c3c2b7",
  ink3: "#898781",
  surface: "#1a1a19",
  series1: "#3987e5", // satellite + pass track
  series2: "#d95926", // antenna pointing
};

let state = null;           // latest WebSocket snapshot
let satellites = [];        // catalog from /api/satellites
let selectedId = null;      // satellite selected in the list
let selectedPasses = [];    // passes for the selected satellite
let dopplerEdited = false;  // don't clobber the form while the user types

/* ---------- helpers ---------- */

const fmtDeg = (v) => (v == null ? "—" : `${v.toFixed(1)}°`);
const fmtMHz = (hz) => (hz / 1e6).toFixed(4) + " MHz";
const fmtTime = (unix) => new Date(unix * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
const fmtCountdown = (unix) => {
  let s = Math.max(0, Math.round(unix - Date.now() / 1000));
  const h = Math.floor(s / 3600); s %= 3600;
  const m = Math.floor(s / 60); s %= 60;
  return h > 0 ? `${h}h ${String(m).padStart(2, "0")}m` : m > 0 ? `${m}m ${String(s).padStart(2, "0")}s` : `${s}s`;
};

let apiError = "";
let apiErrorAt = 0;

async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: body ? { "Content-Type": "application/json" } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!res.ok) {
    // An interlock refusal (409) is the interesting case: the operator asked
    // for motion and did not get it, so the reason has to reach the screen.
    let detail = await res.text();
    try { detail = JSON.parse(detail).detail ?? detail; } catch { /* plain text */ }
    apiError = detail;
    apiErrorAt = Date.now();
    console.warn("POST failed", url, detail);
    if (state) render();
  }
  return res;
}

/* ---------- WebSocket ---------- */

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => {
    $("conn").classList.add("online");
    $("conn-label").textContent = "live";
  };
  ws.onmessage = (ev) => { state = JSON.parse(ev.data); render(); };
  ws.onclose = () => {
    $("conn").classList.remove("online");
    $("conn-label").textContent = "offline";
    setTimeout(connect, 2000);
  };
}

/* ---------- satellite list ---------- */

async function refreshCatalog() {
  try {
    const res = await fetch("/api/satellites");
    satellites = (await res.json()).satellites;
    renderSatList();
  } catch (e) { /* offline; retry next cycle */ }
}

function renderSatList() {
  const filter = $("sat-search").value.trim().toLowerCase();
  const box = $("sat-list");
  box.innerHTML = "";
  for (const sat of satellites) {
    if (filter && !sat.name.toLowerCase().includes(filter) && !String(sat.norad_id).includes(filter)) continue;
    const row = document.createElement("div");
    row.className = "sat-row" + (sat.norad_id === selectedId ? " selected" : "");
    const up = sat.elevation > 0;
    row.innerHTML = `
      <span class="sat-name">${sat.name}${sat.has_transponders ? '<span class="tag" title="transponder data available">RF</span>' : ""}</span>
      <span class="el-badge ${up ? "up" : ""}">${sat.elevation.toFixed(0)}°</span>`;
    row.onclick = () => selectSatellite(sat.norad_id);
    box.appendChild(row);
  }
}

async function selectSatellite(id) {
  selectedId = id;
  renderSatList();
  $("passes-title").textContent = "Upcoming passes — loading";
  try {
    const res = await fetch(`/api/satellites/${id}/passes?hours=24`);
    selectedPasses = (await res.json()).passes;
  } catch (e) { selectedPasses = []; }
  renderPasses();
}

function renderPasses() {
  const sat = satellites.find((s) => s.norad_id === selectedId);
  $("passes-title").textContent = sat ? `Passes — ${sat.name} (24 h)` : "Upcoming passes";
  const box = $("pass-list");
  box.innerHTML = "";
  if (selectedId != null) {
    const trackBtn = document.createElement("button");
    trackBtn.className = "btn primary";
    trackBtn.style.margin = "4px 0 8px";
    trackBtn.textContent = `Track ${sat ? sat.name : selectedId}`;
    trackBtn.onclick = () => post(`/api/track/${selectedId}`);
    box.appendChild(trackBtn);
  }
  if (!selectedPasses.length) {
    const empty = document.createElement("div");
    empty.className = "dim";
    empty.textContent = selectedId == null ? "Select a satellite to see passes." : "No passes above minimum elevation in the next 24 h.";
    box.appendChild(empty);
    return;
  }
  for (const p of selectedPasses) {
    const row = document.createElement("div");
    row.className = "pass-row";
    row.innerHTML = `
      <span>AOS ${fmtTime(p.aos)} <span class="dim">in ${fmtCountdown(p.aos)}</span></span>
      <span class="max-el">${p.max_elevation.toFixed(0)}°</span>
      <span class="dim">${Math.round(p.duration_s / 60)} min</span>
      <span class="dim">${p.aos_azimuth.toFixed(0)}–${p.los_azimuth.toFixed(0)}°</span>`;
    box.appendChild(row);
  }
}

/* ---------- sky plot ---------- */

function polarXY(cx, cy, radius, azDeg, elDeg) {
  const r = radius * (90 - Math.min(Math.max(elDeg, 0), 90)) / 90;
  const a = (azDeg - 90) * Math.PI / 180;
  return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
}

function drawSky() {
  const canvas = $("skyplot");
  const ctx = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height;
  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 24;
  ctx.clearRect(0, 0, w, h);

  // Recessive chrome: hairline elevation rings + spokes, solid, one step off surface
  ctx.lineWidth = 1;
  ctx.font = "11px system-ui, sans-serif";
  for (const el of [0, 30, 60]) {
    ctx.beginPath();
    ctx.arc(cx, cy, R * (90 - el) / 90, 0, 2 * Math.PI);
    ctx.strokeStyle = el === 0 ? C.axis : C.grid;
    ctx.stroke();
  }
  for (let az = 0; az < 360; az += 30) {
    const [x, y] = polarXY(cx, cy, R, az, 0);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y);
    ctx.strokeStyle = az % 90 === 0 ? C.axis : C.grid;
    ctx.stroke();
  }
  // Cardinal + ring tick labels in muted ink
  ctx.fillStyle = C.ink3;
  ctx.textAlign = "center";
  for (const [az, label] of [[0, "N"], [90, "E"], [180, "S"], [270, "W"]]) {
    const [x, y] = polarXY(cx, cy, R + 14, az, 0);
    ctx.fillText(label, x, y + 4);
  }
  ctx.textAlign = "left";
  for (const el of [30, 60]) {
    const [x, y] = polarXY(cx, cy, R, 0, el);
    ctx.fillText(`${el}°`, x + 3, y - 3);
  }
  ctx.textAlign = "center";

  const tracked = state && state.tracked;

  // Pass track: 2px series line, round caps
  if (tracked && tracked.pass && tracked.pass.profile) {
    ctx.beginPath();
    let started = false;
    for (const pt of tracked.pass.profile) {
      if (pt.el < 0) continue;
      const [x, y] = polarXY(cx, cy, R, pt.az, pt.el);
      started ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      started = true;
    }
    ctx.strokeStyle = C.series1;
    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 2;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.lineWidth = 1;
  }

  // Antenna pointing: series-2 crosshair
  if (state && state.rotator) {
    const az = ((state.rotator.azimuth % 360) + 360) % 360;
    const el = state.rotator.elevation;
    const [x, y] = polarXY(cx, cy, R, az, el);
    ctx.strokeStyle = C.series2;
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x - 13, y); ctx.lineTo(x + 13, y);
    ctx.moveTo(x, y - 13); ctx.lineTo(x, y + 13);
    ctx.stroke();
    ctx.lineWidth = 1;
  }

  // Satellite: >=8px marker with a 2px surface ring; name label in ink, not series color
  if (tracked && tracked.observation && tracked.observation.elevation > -5) {
    const o = tracked.observation;
    const [x, y] = polarXY(cx, cy, R, o.azimuth, o.elevation);
    ctx.beginPath(); ctx.arc(x, y, 7, 0, 2 * Math.PI);
    ctx.fillStyle = C.surface; ctx.fill();          // surface ring
    ctx.beginPath(); ctx.arc(x, y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = o.elevation >= 0 ? C.series1 : C.ink3;
    ctx.fill();
    ctx.fillStyle = C.ink2;
    ctx.fillText(tracked.name, x, y - 13);
  }
}

/* ---------- render loop ---------- */

/* The rotator will refuse to move until it knows where it is, so say so
   plainly rather than letting Track look broken. */
function renderRotatorStatus(rot, tracker) {
  const banner = $("rot-status");
  const text = $("rot-status-text");
  const clear = $("btn-clear-fault");
  let message = "";
  let level = "info";

  if (apiError && Date.now() - apiErrorAt < 6000) {
    message = apiError;
    level = "error";
  } else if (rot.fault) {
    message = rot.fault;
    level = "error";
  } else if (rot.homing) {
    message = "Homing — finding the endstop";
  } else if (!rot.homed) {
    message = "Not homed. Press Home before tracking.";
    level = "warn";
  } else if (tracker.blocked) {
    message = tracker.blocked;
    level = "warn";
  }

  banner.hidden = !message;
  banner.className = "status-banner " + level;
  text.textContent = message;
  clear.hidden = !rot.fault;
}

function render() {
  if (!state) return;
  $("station-name").textContent = state.station.name;

  const chip = $("tracker-chip");
  chip.textContent = state.tracker.state;
  chip.className = "chip " + state.tracker.state;

  const rot = state.rotator;
  $("rd-az").textContent = fmtDeg(rot.azimuth);
  $("rd-el").textContent = fmtDeg(rot.elevation);
  $("rd-az-t").textContent = rot.moving ? `target ${fmtDeg(rot.target_azimuth)}` : "";
  $("rd-el-t").textContent = rot.moving ? `target ${fmtDeg(rot.target_elevation)}` : "";
  $("rot-moving").textContent = rot.moving ? "moving" : "stopped";
  $("rot-moving").className = "chip" + (rot.moving ? " moving" : "");
  $("rot-fault").textContent = rot.fault ? `Fault: ${rot.fault}` : "";
  $("wrap-warning").textContent = state.tracker.wrap_warning || "";
  renderRotatorStatus(rot, state.tracker);
  renderRig(state.rig);
  renderSchedule(state.schedule);

  const tleAge = state.tle.fetched_at ? Math.round((state.time - state.tle.fetched_at) / 3600) : null;
  $("tle-info").textContent = `${state.tle.satellite_count} satellites · TLEs ${tleAge == null ? "not loaded" : tleAge + " h old"}`;

  const info = $("tracked-info");
  if (state.tracked && state.tracked.observation) {
    const o = state.tracked.observation;
    info.textContent =
      `${state.tracked.name} · az ${o.azimuth.toFixed(1)}° el ${o.elevation.toFixed(1)}° · ` +
      `${o.range_km.toFixed(0)} km ${o.range_rate_km_s > 0 ? "receding" : "approaching"} ` +
      `at ${Math.abs(o.range_rate_km_s).toFixed(2)} km/s · ${o.sunlit ? "sunlit" : "in eclipse"}`;
    renderTransponders(o);
  } else {
    info.textContent = "No satellite being tracked.";
    $("xpndr-section").hidden = true;
  }
  renderDoppler2m(state.doppler_2m);
  drawSky();
}

/* ---------- 2 m doppler panel ---------- */

function renderDoppler2m(d) {
  if (!d) return;
  const chipEl = $("doppler-state");
  chipEl.textContent = d.active ? "live" : "idle";
  chipEl.className = "chip" + (d.active ? " tracking" : "");

  if (!dopplerEdited) {
    $("dop-up").value = (d.uplink_hz / 1e6).toFixed(3);
    $("dop-down").value = (d.downlink_hz / 1e6).toFixed(3);
  }

  const table = $("doppler-table");
  if (!d.active) {
    table.innerHTML = `<tr><td colspan="3">Track a satellite for live correction.</td></tr>`;
    return;
  }
  const row = (label, corrected, shift, rate) => `
    <tr>
      <td>${label}</td>
      <td class="val">${fmtMHz(corrected)}</td>
      <td class="aux">${shift >= 0 ? "+" : ""}${(shift / 1e3).toFixed(2)} kHz · ${rate >= 0 ? "+" : ""}${rate.toFixed(1)} Hz/s</td>
    </tr>`;
  table.innerHTML =
    row("TX", d.uplink_corrected_hz, d.uplink_corrected_hz - d.uplink_hz, d.uplink_rate_hz_s) +
    row("RX", d.downlink_corrected_hz, d.downlink_shift_hz, d.downlink_rate_hz_s);
}

/* ---------- scheduler ---------- */

function renderSchedule(schedule) {
  const button = $("btn-sched");
  const label = $("sched-next");
  if (!schedule || !schedule.watching) {
    // Nothing on the watch list, so the control would do nothing. Hide it.
    button.hidden = true;
    label.textContent = "";
    return;
  }
  button.hidden = false;
  button.textContent = schedule.enabled ? "Auto-track: on" : "Auto-track: off";
  button.className = "btn small" + (schedule.enabled ? " active" : "");

  if (schedule.engaged) {
    label.textContent = `working ${schedule.engaged.name}`;
  } else if (schedule.upcoming && schedule.upcoming.length) {
    const next = schedule.upcoming[0];
    label.textContent = `next ${next.name} in ${fmtCountdown(next.aos)} · ${next.max_elevation}°`;
  } else {
    label.textContent = schedule.enabled ? "no passes planned" : "";
  }
}

/* ---------- radio panel ---------- */

function renderRig(rig) {
  const section = $("rig-section");
  if (!rig || !rig.present) { section.hidden = true; return; }
  section.hidden = false;

  const chip = $("rig-state");
  const live = rig.connected && rig.enabled;
  chip.textContent = !rig.enabled ? "off" : rig.connected ? "tuning" : "no radio";
  chip.className = "chip" + (live ? " tracking" : rig.enabled ? " acquiring" : "");
  $("rig-error").textContent = rig.error || "";
  $("btn-rig-toggle").textContent = rig.enabled ? "Disable tuning" : "Enable tuning";

  const table = $("rig-table");
  if (rig.target_rx_hz == null) {
    table.innerHTML = `<tr><td colspan="3">Idle — waiting for a tracked satellite.</td></tr>`;
    return;
  }
  // Target is what doppler says; rig is where the radio actually sits. The gap
  // between them is the deadband doing its job, not an error.
  const row = (label, target, actual) => `
    <tr>
      <td>${label}</td>
      <td class="val">${fmtMHz(target)}</td>
      <td class="aux">rig ${actual == null ? "—" : fmtMHz(actual)}</td>
    </tr>`;
  table.innerHTML =
    row("TX", rig.target_tx_hz, rig.rig_tx_hz) +
    row("RX", rig.target_rx_hz, rig.rig_rx_hz) +
    `<tr><td>${rig.mode || "—"}</td><td class="aux" colspan="2">deadband ${rig.deadband_hz} Hz · ${rig.tunes} retunes</td></tr>`;
}

/* ---------- transponder panel ---------- */

function renderTransponders(obs) {
  const section = $("xpndr-section");
  const box = $("doppler-box");
  if (!obs.transponders || !obs.transponders.length) { section.hidden = true; return; }
  section.hidden = false;
  box.innerHTML = "";
  for (const t of obs.transponders) {
    const row = document.createElement("div");
    row.className = "xp-row";
    let html = `<div class="name">${t.name} · ${t.mode}</div>`;
    if (t.downlink_corrected_hz) {
      const shift = t.downlink_corrected_hz - t.downlink_hz;
      html += `<div class="freq">RX <b>${fmtMHz(t.downlink_corrected_hz)}</b><span class="shift">${shift >= 0 ? "+" : ""}${(shift / 1e3).toFixed(1)} kHz</span></div>`;
    }
    if (t.uplink_corrected_hz) {
      const shift = t.uplink_corrected_hz - t.uplink_hz;
      html += `<div class="freq">TX <b>${fmtMHz(t.uplink_corrected_hz)}</b><span class="shift">${shift >= 0 ? "+" : ""}${(shift / 1e3).toFixed(1)} kHz</span></div>`;
    }
    row.innerHTML = html;
    box.appendChild(row);
  }
}

/* ---------- controls ---------- */

document.querySelectorAll(".jog").forEach((btn) => {
  btn.onclick = () => post("/api/rotator/jog", {
    d_az: parseFloat(btn.dataset.daz),
    d_el: parseFloat(btn.dataset.del),
  });
});
$("goto-form").onsubmit = (e) => {
  e.preventDefault();
  post("/api/rotator/goto", { az: parseFloat($("goto-az").value), el: parseFloat($("goto-el").value) });
};
$("btn-stop").onclick = () => post("/api/rotator/stop");
$("btn-estop").onclick = () => {
  if (confirm("Cut motion immediately?\n\nThe axes lose their position reference and must be re-homed.")) {
    post("/api/rotator/estop");
  }
};
$("btn-clear-fault").onclick = () => post("/api/rotator/fault/clear");
$("btn-rig-toggle").onclick = () => post("/api/rig", { enabled: !(state && state.rig && state.rig.enabled) });
$("btn-sched").onclick = () =>
  post(`/api/schedule/${state && state.schedule && state.schedule.enabled ? "disable" : "enable"}`);
$("btn-park").onclick = () => post("/api/rotator/park");
$("btn-home").onclick = () => post("/api/rotator/home");
$("btn-untrack").onclick = () => post("/api/track/stop");
$("tle-refresh").onclick = async () => {
  $("tle-refresh").disabled = true;
  await post("/api/tle/refresh");
  await refreshCatalog();
  $("tle-refresh").disabled = false;
};
$("sat-search").oninput = renderSatList;

$("dop-up").oninput = $("dop-down").oninput = () => { dopplerEdited = true; };
$("doppler-form").onsubmit = async (e) => {
  e.preventDefault();
  await post("/api/doppler", {
    uplink_hz: parseFloat($("dop-up").value) * 1e6,
    downlink_hz: parseFloat($("dop-down").value) * 1e6,
  });
  dopplerEdited = false;
};

/* ---------- boot ---------- */

connect();
refreshCatalog();
setInterval(refreshCatalog, 15000);
setInterval(renderPasses, 30000); // keep countdowns fresh
