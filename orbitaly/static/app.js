/* Orbitaly dashboard — no framework, no build step. */
"use strict";

const $ = (id) => document.getElementById(id);

let state = null;           // latest WebSocket snapshot
let satellites = [];        // catalog from /api/satellites
let selectedId = null;      // satellite selected in the list
let selectedPasses = [];    // passes for the selected satellite

/* ---------- helpers ---------- */

const fmtDeg = (v) => (v == null ? "—" : `${v.toFixed(1)}°`);
const fmtMHz = (hz) => (hz / 1e6).toFixed(4) + " MHz";
const fmtTime = (unix) => new Date(unix * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
const fmtCountdown = (unix) => {
  let s = Math.max(0, Math.round(unix - Date.now() / 1000));
  const h = Math.floor(s / 3600); s %= 3600;
  const m = Math.floor(s / 60); s %= 60;
  return h > 0 ? `${h}h${String(m).padStart(2, "0")}m` : m > 0 ? `${m}m${String(s).padStart(2, "0")}s` : `${s}s`;
};

async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: body ? { "Content-Type": "application/json" } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!res.ok) console.warn("POST failed", url, await res.text());
  return res;
}

/* ---------- WebSocket ---------- */

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => $("conn-dot").className = "dot online";
  ws.onmessage = (ev) => { state = JSON.parse(ev.data); render(); };
  ws.onclose = () => {
    $("conn-dot").className = "dot offline";
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
      <span class="sat-name">${sat.name}${sat.has_transponders ? ' <span class="xpndr">⇅</span>' : ""}</span>
      <span class="el-badge ${up ? "up" : ""}">${up ? "▲ " : ""}${sat.elevation.toFixed(0)}°</span>`;
    row.onclick = () => selectSatellite(sat.norad_id);
    box.appendChild(row);
  }
}

async function selectSatellite(id) {
  selectedId = id;
  renderSatList();
  $("passes-title").textContent = "Upcoming passes — loading…";
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
      <span>AOS ${fmtTime(p.aos)} <span class="dim">(in ${fmtCountdown(p.aos)})</span></span>
      <span class="max-el">${p.max_elevation.toFixed(0)}°</span>
      <span class="dim">${Math.round(p.duration_s / 60)} min</span>
      <span class="dim">${p.aos_azimuth.toFixed(0)}°→${p.los_azimuth.toFixed(0)}°</span>`;
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

  // Elevation rings at 0/30/60 + cardinal spokes
  ctx.strokeStyle = "#263042";
  ctx.fillStyle = "#7d8ba1";
  ctx.font = "11px sans-serif";
  ctx.textAlign = "center";
  for (const el of [0, 30, 60]) {
    ctx.beginPath();
    ctx.arc(cx, cy, R * (90 - el) / 90, 0, 2 * Math.PI);
    ctx.stroke();
  }
  for (let az = 0; az < 360; az += 30) {
    const [x, y] = polarXY(cx, cy, R, az, 0);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y);
    ctx.strokeStyle = az % 90 === 0 ? "#324158" : "#1c2534";
    ctx.stroke();
  }
  for (const [az, label] of [[0, "N"], [90, "E"], [180, "S"], [270, "W"]]) {
    const [x, y] = polarXY(cx, cy, R + 14, az, 0);
    ctx.fillText(label, x, y + 4);
  }

  const tracked = state && state.tracked;

  // Pass arc for the tracked satellite
  if (tracked && tracked.pass && tracked.pass.profile) {
    ctx.beginPath();
    let started = false;
    for (const pt of tracked.pass.profile) {
      if (pt.el < 0) continue;
      const [x, y] = polarXY(cx, cy, R, pt.az, pt.el);
      started ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      started = true;
    }
    ctx.strokeStyle = "#4da3ff88";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.lineWidth = 1;
  }

  // Antenna pointing crosshair
  if (state && state.rotator) {
    const az = ((state.rotator.azimuth % 360) + 360) % 360;
    const el = state.rotator.elevation;
    const [x, y] = polarXY(cx, cy, R, az, el);
    ctx.strokeStyle = "#ffb454";
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x - 13, y); ctx.lineTo(x + 13, y);
    ctx.moveTo(x, y - 13); ctx.lineTo(x, y + 13);
    ctx.stroke();
  }

  // Satellite dot
  if (tracked && tracked.observation && tracked.observation.elevation > -5) {
    const o = tracked.observation;
    const [x, y] = polarXY(cx, cy, R, o.azimuth, o.elevation);
    ctx.fillStyle = o.elevation >= 0 ? "#3ddc84" : "#7d8ba1";
    ctx.beginPath(); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = "#dbe4f0";
    ctx.fillText(tracked.name, x, y - 12);
  }
}

/* ---------- render loop ---------- */

function render() {
  if (!state) return;
  $("station-name").textContent = "· " + state.station.name;

  const chip = $("tracker-chip");
  chip.textContent = state.tracker.state;
  chip.className = "chip " + state.tracker.state;

  const rot = state.rotator;
  $("rd-az").textContent = fmtDeg(rot.azimuth);
  $("rd-el").textContent = fmtDeg(rot.elevation);
  $("rd-az-t").textContent = rot.moving ? `→ ${fmtDeg(rot.target_azimuth)}` : "";
  $("rd-el-t").textContent = rot.moving ? `→ ${fmtDeg(rot.target_elevation)}` : "";
  $("rot-moving").textContent = rot.moving ? "moving" : "stopped";
  $("rot-moving").className = "chip " + (rot.moving ? "acquiring" : "idle");
  $("rot-fault").textContent = rot.fault || "";
  $("wrap-warning").textContent = state.tracker.wrap_warning || "";

  const tleAge = state.tle.fetched_at ? Math.round((state.time - state.tle.fetched_at) / 3600) : null;
  $("tle-info").textContent = `${state.tle.satellite_count} sats · TLEs ${tleAge == null ? "not loaded" : tleAge + "h old"}`;

  const info = $("tracked-info");
  if (state.tracked && state.tracked.observation) {
    const o = state.tracked.observation;
    info.textContent =
      `${state.tracked.name} — az ${o.azimuth.toFixed(1)}° el ${o.elevation.toFixed(1)}° · ` +
      `${o.range_km.toFixed(0)} km · ${o.range_rate_km_s > 0 ? "receding" : "approaching"} ` +
      `${Math.abs(o.range_rate_km_s).toFixed(2)} km/s${o.sunlit ? " · ☀ sunlit" : " · eclipse"}`;
    renderDoppler(o);
  } else {
    info.textContent = "No satellite being tracked.";
    $("doppler-box").innerHTML = "";
  }
  drawSky();
}

function renderDoppler(obs) {
  const box = $("doppler-box");
  if (!obs.transponders || !obs.transponders.length) { box.innerHTML = ""; return; }
  box.innerHTML = "<div class='panel-title'><span>Doppler-corrected frequencies</span></div>";
  for (const t of obs.transponders) {
    const row = document.createElement("div");
    row.className = "xp-row";
    let html = `<div class="name">${t.name} · ${t.mode}</div>`;
    if (t.downlink_corrected_hz) {
      const shift = t.downlink_corrected_hz - t.downlink_hz;
      html += `<div class="freq">▼ ${fmtMHz(t.downlink_corrected_hz)}<span class="shift">${shift >= 0 ? "+" : ""}${(shift / 1e3).toFixed(1)} kHz</span></div>`;
    }
    if (t.uplink_corrected_hz) {
      const shift = t.uplink_corrected_hz - t.uplink_hz;
      html += `<div class="freq">▲ ${fmtMHz(t.uplink_corrected_hz)}<span class="shift">${shift >= 0 ? "+" : ""}${(shift / 1e3).toFixed(1)} kHz</span></div>`;
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

/* ---------- boot ---------- */

connect();
refreshCatalog();
setInterval(refreshCatalog, 15000);
setInterval(renderPasses, 30000); // keep countdowns fresh
