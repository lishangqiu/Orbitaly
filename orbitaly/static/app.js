/* Orbitaly console — no framework, no build step. */
"use strict";

const $ = (id) => document.getElementById(id);

/* Chart tokens, read from style.css rather than restated here, so the two
   canvases land on whichever palette the configured theme (ui.theme, stamped
   onto <html data-theme> by the server) put on :root. The literals are
   fallbacks for a stylesheet that has not applied — the dark values, matching
   what index.html ships with. Read once: the theme is fixed for the life of
   the page. */
const rootStyle = getComputedStyle(document.documentElement);
const token = (name, fallback) => rootStyle.getPropertyValue(name).trim() || fallback;

const C = {
  grid: token("--grid", "#2c2c2a"),
  axis: token("--axis", "#383835"),
  ink: token("--ink", "#ffffff"),
  ink2: token("--ink-2", "#c3c2b7"),
  ink3: token("--ink-3", "#898781"),
  surface: token("--surface", "#1a1a19"),
  series1: token("--series-1", "#3987e5"), // satellite + pass track
  series2: token("--series-2", "#d95926"), // antenna pointing
  nightshade: token("--nightshade", "rgba(0, 0, 0, 0.30)"),
};

/* Track colors, in display-set order. The tracked satellite keeps series-1 so
   it matches the sky plot; the antenna has no meaning on a map, which frees
   series-2 for the next satellite. The display set is capped at this length
   rather than at some larger number with colors repeating: past six the map
   is spaghetti anyway. */
const SERIES = ["#3987e5", "#d95926", "#2eae8e", "#c07fd0", "#d4b02c", "#5fc4d8"].map(
  (fallback, i) => token(`--series-${i + 1}`, fallback)
);
const MAX_DISPLAY = SERIES.length;

/* The endpoint's samples are a minute apart, so refetching faster than this
   would return the same track. Between fetches the markers slide along it. */
const TRACK_REFRESH_S = 60;

let state = null;           // latest WebSocket snapshot
let satellites = [];        // catalog from /api/satellites
let selectedId = null;      // satellite selected in the list
let selectedPasses = [];    // passes for the selected satellite
let dopplerEdited = false;  // don't clobber the form while the user types

/* -- map state -- */
let activeView = localStorage.getItem("orbitaly.view") === "map" ? "map" : "sky";
let pinned = new Set(readPinned());   // norad ids, a browser-local preference
let world = null;                     // vendored coastlines, fetched once
let tracks = new Map();               // norad_id -> ground track
let trackPending = new Set();
let hoverId = null;
let mapMarkers = [];                  // last drawn marker positions, for hit testing
let displaySetKey = "";               // membership signature, to notice changes

function readPinned() {
  try {
    const raw = JSON.parse(localStorage.getItem("orbitaly.pinned") || "[]");
    return Array.isArray(raw) ? raw.map(Number).filter(Number.isFinite) : [];
  } catch (e) {
    return [];
  }
}

function savePinned() {
  localStorage.setItem("orbitaly.pinned", JSON.stringify([...pinned]));
}

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
  ws.onmessage = (ev) => {
    state = JSON.parse(ev.data);
    render();
    // Tracking or the schedule changing alters the display set, so fetch the
    // newcomers' tracks rather than waiting for the next poll.
    const key = displaySet().shown.map((e) => e.id).join(",");
    if (key !== displaySetKey) {
      displaySetKey = key;
      refreshTracks();
    }
  };
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
    // The map reads positions, altitudes and band state out of the catalog, so
    // a fresh catalog is fresh map data. Redraw now rather than leaving it a
    // WS tick stale.
    if (activeView === "map" && state) render();
  } catch (e) { /* offline; retry next cycle */ }
}

function renderSatList() {
  const filter = $("sat-search").value.trim().toLowerCase();
  const box = $("sat-list");
  // Rebuilt whole every 15 s poll, under whatever the operator was reading:
  // keep the scroll where they left it. A 97-row catalog snapping back to the
  // top four times a minute is its own kind of broken.
  const scroll = box.scrollTop;
  box.innerHTML = "";
  for (const sat of satellites) {
    if (filter && !sat.name.toLowerCase().includes(filter) && !String(sat.norad_id).includes(filter)) continue;
    const row = document.createElement("div");
    row.className = "sat-row" + (sat.norad_id === selectedId ? " selected" : "");
    const up = sat.elevation > 0;
    const isPinned = pinned.has(sat.norad_id);
    row.innerHTML = `
      <button class="pin${isPinned ? " on" : ""}" title="${isPinned ? "Unpin from map" : "Pin to map"}" aria-pressed="${isPinned}"></button>
      <span class="sat-name">${sat.name}${sat.has_transponders ? '<span class="tag" title="transponder data available">RF</span>' : ""}</span>
      <span class="el-badge ${up ? "up" : ""}">${sat.elevation.toFixed(0)}°</span>`;
    row.onclick = () => selectSatellite(sat.norad_id);
    row.querySelector(".pin").onclick = (e) => {
      e.stopPropagation(); // pinning is not selecting
      togglePin(sat.norad_id);
    };
    box.appendChild(row);
  }
  box.scrollTop = scroll;
}

async function selectSatellite(id) {
  selectedId = id;
  renderSatList();
  refreshTracks();
  if (state) render();
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

/* Size a canvas's backing store by devicePixelRatio and hand back a context
   already scaled to CSS pixels, so all the drawing code below can work in
   layout units and still be sharp on a hiDPI screen. */
function prepareCanvas(canvas, aspect) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(canvas.clientWidth));
  const h = Math.max(1, Math.round(w / aspect));
  const backingW = Math.round(w * dpr);
  const backingH = Math.round(h * dpr);
  if (canvas.width !== backingW || canvas.height !== backingH) {
    canvas.width = backingW;
    canvas.height = backingH;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

function drawSky() {
  const { ctx, w, h } = prepareCanvas($("skyplot"), 1);
  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 24;

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

/* ---------- map view ---------- */

/* Which satellites the map draws, and which get a range ring.
   Tracked first, so it keeps series-1 and matches the sky plot. */
function displaySet() {
  const wanted = [];
  const want = (id, ring) => {
    if (id != null) wanted.push([Number(id), ring]);
  };
  if (state && state.tracked) want(state.tracked.norad_id, true);
  // Selected comes second, before anything that could crowd it out, for two
  // reasons. The cap below used to evict it — a tracked satellite plus five
  // pinned meant clicking a sixth drew nothing at all, and only "showing 6 of
  // 7" hinted at why. And it gets a ring: the map's most common state is
  // nothing tracked, nothing pinned, one satellite clicked, and "Est. range"
  // was a legend key that state could not draw. With one satellite selected
  // there is no clutter for a ring to add to.
  want(selectedId, true);
  for (const id of pinned) want(id, true);
  if (state && state.schedule && state.schedule.upcoming) {
    for (const p of state.schedule.upcoming) want(p.norad_id, true);
  }

  const byId = new Map();
  for (const [id, ring] of wanted) {
    const existing = byId.get(id);
    if (existing) existing.ring = existing.ring || ring;
    else byId.set(id, { id, ring });
  }
  const all = [...byId.values()];
  const shown = all.slice(0, MAX_DISPLAY);
  shown.forEach((entry, i) => {
    entry.color = SERIES[i % SERIES.length];
  });
  return { shown, total: all.length };
}

function stationPoint() {
  const s = state && state.station;
  if (!s || s.latitude == null) return null;
  return s;
}

/* The elevation the map may call "in view" — the server resolves this against
   the tracker's own minimum, so the map cannot promise a pass the rest of the
   software would refuse. */
function maskDeg() {
  const s = state && state.station;
  return s && s.mask_deg != null ? s.mask_deg : 5.0;
}

function catalogRow(id) {
  return satellites.find((s) => s.norad_id === id) || null;
}

function satelliteName(id) {
  const row = catalogRow(id);
  if (row) return row.name;
  if (state && state.tracked && state.tracked.norad_id === id) return state.tracked.name;
  return `NORAD ${id}`;
}

/* -- ground track fetching -- */

async function loadWorld() {
  try {
    const res = await fetch("/world.json");
    world = await res.json();
    if (activeView === "map") render();
  } catch (e) { /* the map still draws, just without coastlines */ }
}

async function refreshTracks() {
  if (activeView !== "map") return;
  const { shown } = displaySet();
  const want = new Set(shown.map((e) => e.id));
  for (const id of [...tracks.keys()]) if (!want.has(id)) tracks.delete(id);

  // Fetched one at a time on purpose. Each of these is a vectorized SGP4 solve
  // on a host that may also be feeding motion segments in soft real time; six
  // at once would be six of them landing on a Pi's cores together, and the
  // markers slide along whatever track is already loaded meanwhile.
  const now = Date.now() / 1000;
  for (const id of want) {
    const have = tracks.get(id);
    if (have && now - have.fetchedAt < TRACK_REFRESH_S) continue;
    if (trackPending.has(id)) continue;
    trackPending.add(id);
    try {
      const res = await fetch(`/api/satellites/${id}/groundtrack`);
      if (res.ok) {
        const body = await res.json();
        tracks.set(id, {
          // [lon, lat, elevation, time, altitude] — everything past the first
          // two rides along through the antimeridian split, so in-view arcs
          // survive being cut at the frame and the ring can size itself from
          // the track instead of waiting on the 15 s catalog poll.
          points: body.points.map((p) => [p.lon, p.lat, p.el, p.t, p.alt_km]),
          step: body.step_s || 60,
          fetchedAt: now,
        });
      } else {
        // A TLE that will not propagate. Remember the failure so it is not
        // re-asked every cycle — but only until the next refresh interval, so
        // a TLE update can put the track back.
        tracks.set(id, { points: [], step: 60, fetchedAt: now, failed: true });
      }
    } catch (e) {
      /* offline; try again next cycle */
    } finally {
      trackPending.delete(id);
    }
    if (activeView === "map") render();
  }
}

/* Where a satellite is right now, interpolated along its track so the marker
   moves at the WS cadence instead of jumping every time the catalog polls. */
function positionAt(track, now) {
  const pts = track && track.points;
  if (!pts || !pts.length) return null;
  if (now <= pts[0][3]) return pts[0];
  if (now >= pts[pts.length - 1][3]) return pts[pts.length - 1];
  let i = Math.floor((now - pts[0][3]) / track.step);
  i = Math.max(0, Math.min(pts.length - 2, i));
  const a = pts[i];
  const b = pts[i + 1];
  const span = b[3] - a[3];
  return Geo.interpolatePoint(a, b, span > 0 ? (now - a[3]) / span : 0);
}

function nowIndexOf(track, now) {
  const pts = track.points;
  let i = Math.round((now - pts[0][3]) / track.step);
  return Math.max(0, Math.min(pts.length - 1, i));
}

/* A track is only good for the window it covers. If ground-track fetches keep
   failing — offline, or a TLE that has stopped propagating — `now` eventually
   walks past the last sample; the index clamps, the whole track draws faint,
   and the marker freezes at the terminus. A frozen marker is a lie, where a
   missing line is only a gap, so past either end the track is dropped and the
   catalog position takes over. */
function usableTrack(id, now) {
  const track = tracks.get(id);
  const pts = track && track.points;
  if (!pts || pts.length < 2) return null;
  return now >= pts[0][3] && now <= pts[pts.length - 1][3] ? track : null;
}

/* -- drawing primitives -- */

function strokePolyline(ctx, w, h, points, width, alpha, color, cap) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.globalAlpha = alpha;
  ctx.lineCap = cap || "round";
  ctx.lineJoin = "round";
  for (const part of Geo.splitAntimeridian(points)) {
    ctx.beginPath();
    part.forEach((p, i) => {
      const [x, y] = Geo.project(p[0], p[1], w, h);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
}

/* A projection window has two ends and neither of them means anything: the
   trail simply stops at its oldest sample, and the forecast stops a little
   past one orbit — beside the live marker, since the Earth turns ~24° under
   one LEO orbit. A hard line-end in open water is indistinguishable from a
   severed track, and was read as one.

   So *every* layer drawn from a track — base line, in-view emphasis,
   scheduled window — goes through the same taper, which ramps to zero across
   the last few samples at each end. Fading only the base line was the earlier
   attempt at this, and it failed: the full-alpha in-view overlay kept drawing
   over an invisible base and chopping dead in open air, which is the brightest
   ink on the map ending at nothing. Interior boundaries — a mask crossing, an
   AOS — are not tapered; those ends are real, and they are now interpolated
   onto the crossing itself (Geo.clipRuns*) instead of snapping to a sample. */
const TRACK_FADE_SAMPLES = 8;

function windowTaper(track) {
  const pts = track.points;
  const first = pts[0][3];
  const last = pts[pts.length - 1][3];
  const span = Math.max(1, TRACK_FADE_SAMPLES * track.step);
  return (p) =>
    Math.max(0, Math.min(1, Math.min(p[3] - first, last - p[3]) / span));
}

/* Stroke a polyline with a per-segment alpha from `taper`. Full-weight
   segments are batched into a single path, so only the fading ends — a
   handful of segments — cost a stroke each. */
function strokeTapered(ctx, w, h, points, width, alpha, color, taper) {
  if (points.length < 2) return;
  let batch = [];
  const flush = () => {
    if (batch.length >= 2) strokePolyline(ctx, w, h, batch, width, alpha, color);
    batch = [];
  };
  for (let i = 0; i < points.length - 1; i++) {
    const f = Math.min(taper(points[i]), taper(points[i + 1]));
    if (f >= 1) {
      if (!batch.length) batch.push(points[i]);
      batch.push(points[i + 1]);
      continue;
    }
    flush();
    if (f > 0.02) {
      // Butt caps: a fading line is drawn one segment at a time, and round
      // caps would overlap at every joint — each one compositing over its
      // neighbour into a bright bead, so the taper reads as a dotted line
      // rather than a fade. Consecutive segments share an endpoint exactly,
      // so nothing opens up between them.
      strokePolyline(ctx, w, h, [points[i], points[i + 1]], width, alpha * f, color, "butt");
    }
  }
  flush();
}

/* Night side, as the region below the terminator curve
       lat(lon) = atan( -cos(lon - lon_sun) / tan(dec) )
   closed off to whichever pole is in darkness. Drawn first and very softly:
   it is context for the eclipse flag, not data. */
function drawNight(ctx, w, h, now) {
  const [lonSun, latSun] = Geo.solarSubpoint(now);
  const tanDec = Math.tan((latSun * Math.PI) / 180);
  const points = [];
  for (let lon = -180; lon <= 180; lon += 2) {
    let lat = (Math.atan(-Math.cos(((lon - lonSun) * Math.PI) / 180) / tanDec) * 180) / Math.PI;
    if (!Number.isFinite(lat)) lat = 0;
    points.push([lon, lat]);
  }
  const capLat = latSun > 0 ? -90 : 90;
  ctx.beginPath();
  points.forEach((p, i) => {
    const [x, y] = Geo.project(p[0], p[1], w, h);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  });
  const [xEnd, yCap] = Geo.project(180, capLat, w, h);
  ctx.lineTo(xEnd, yCap);
  ctx.lineTo(Geo.project(-180, capLat, w, h)[0], yCap);
  ctx.closePath();
  ctx.fillStyle = C.nightshade;
  ctx.fill();
}

function drawGraticule(ctx, w, h) {
  ctx.lineWidth = 1;
  for (let lon = -180; lon <= 180; lon += 30) {
    const [x] = Geo.project(lon, 0, w, h);
    ctx.strokeStyle = lon === 0 ? C.axis : C.grid;
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let lat = -90; lat <= 90; lat += 30) {
    const [, y] = Geo.project(0, lat, w, h);
    ctx.strokeStyle = lat === 0 ? C.axis : C.grid;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
}

function drawCoastlines(ctx, w, h) {
  if (!world || !world.lines) return;
  ctx.strokeStyle = C.axis;
  ctx.lineWidth = 1;
  ctx.lineJoin = "round";
  ctx.beginPath();
  for (const line of world.lines) {
    for (const part of Geo.splitAntimeridian(line)) {
      part.forEach((p, i) => {
        const [x, y] = Geo.project(p[0], p[1], w, h);
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      });
    }
  }
  ctx.stroke();
}

function drawStation(ctx, w, h, station) {
  const [x, y] = Geo.project(station.longitude, station.latitude, w, h);
  ctx.strokeStyle = C.ink2;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x, y - 5);
  ctx.lineTo(x + 5, y);
  ctx.lineTo(x, y + 5);
  ctx.lineTo(x - 5, y);
  ctx.closePath();
  ctx.stroke();
  ctx.fillStyle = C.ink3;
  ctx.font = "11px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.fillText(station.name || "Station", x, y + 17);
}

/* The circle a satellite's subpoint must be inside for us to work it. Centred
   on the station, and sized from *this* satellite's altitude — which is why
   it is drawn per satellite rather than once. */
function drawRangeRing(ctx, w, h, station, altitudeKm, color) {
  const radius = Geo.visibilityRadiusKm(altitudeKm, maskDeg());
  if (!(radius > 0)) return;
  const ring = Geo.greatCircle(station.latitude, station.longitude, radius, 180);
  strokePolyline(ctx, w, h, ring, 1, 0.5, color);
}

/* The satellite's own horizon — everywhere on Earth that can see it. A
   different circle from the one above, and conflating the two is a common
   way for a map to mislead. */
function drawFootprint(ctx, w, h, lat, lon, footprintKm, color) {
  if (!(footprintKm > 0)) return;
  const ring = Geo.greatCircle(lat, lon, footprintKm, 120);
  strokePolyline(ctx, w, h, ring, 1, 0.18, color);
}

function drawMarker(ctx, w, h, lon, lat, color, label, dim, emphasis) {
  const [x, y] = Geo.project(lon, lat, w, h);
  const r = emphasis ? 5 : 4;
  ctx.globalAlpha = 1;
  ctx.beginPath();
  ctx.arc(x, y, r + 2, 0, 2 * Math.PI);
  ctx.fillStyle = C.surface; // surface ring, so a marker stays legible on a track
  ctx.fill();
  ctx.beginPath();
  ctx.arc(x, y, r, 0, 2 * Math.PI);
  ctx.fillStyle = color;
  ctx.globalAlpha = dim ? 0.45 : 1; // in eclipse
  ctx.fill();
  ctx.globalAlpha = 1;
  if (label) {
    // Names in ink, never in series color — same rule as the sky plot.
    ctx.fillStyle = emphasis ? C.ink : C.ink2;
    ctx.font = "11px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(label, x, y - 10);
  }
  return [x, y];
}

function drawMap() {
  const { ctx, w, h } = prepareCanvas($("worldmap"), 2);
  const now = state ? state.time : Date.now() / 1000;
  const station = stationPoint();
  const mask = maskDeg();
  const { shown, total } = displaySet();

  drawNight(ctx, w, h, now);
  drawGraticule(ctx, w, h);
  drawCoastlines(ctx, w, h);

  const tracked = state && state.tracked;
  const markers = [];

  for (const entry of shown) {
    const track = usableTrack(entry.id, now);
    const isTracked = tracked && tracked.norad_id === entry.id;
    const row = catalogRow(entry.id);
    const band = isTracked ? tracked.band : row && row.band;
    // Geometry alone must not promise a pass the radio cannot hear. Satellites
    // whose transponders are all out of band keep their track and ring, but
    // lose the in-view emphasis. No transponder data means no claim either
    // way, so those fall back to geometry — which is why the legend says
    // "in view" and not "workable".
    const mayHighlight = band !== "out_of_band";

    let lon = null;
    let lat = null;
    // Altitude only sets the ring's radius, so take it from whichever source
    // has it first: the ground track we fetched, the 15 s catalog poll, or —
    // below — the 1 Hz feed for the tracked satellite. Reading it from the
    // catalog alone left the ring absent for the first seconds after a load,
    // because a ring the station could already draw waited on a poll it did
    // not need.
    let altitudeKm = row ? row.altitude_km : null;

    if (track) {
      const taper = windowTaper(track);
      const i = nowIndexOf(track, now);
      const behind = track.points.slice(0, i + 1);
      const ahead = track.points.slice(i);
      // The step from base to emphasis is 2 px @ 0.6 -> 3 px @ 1. It used to
      // be 1.5 @ 0.85 -> 3 @ 1, and on a hiDPI screen that thin a base read as
      // nothing at all, so the highlights looked like free-floating strokes
      // rather than emphasis on a continuous line.
      strokeTapered(ctx, w, h, behind, 1.5, 0.3, entry.color, taper);
      strokeTapered(ctx, w, h, ahead, 2, 0.6, entry.color, taper);
      if (mayHighlight) {
        for (const run of Geo.clipRunsAtMask(behind, mask)) {
          strokeTapered(ctx, w, h, run, 3, 0.4, entry.color, taper);
        }
        for (const run of Geo.clipRunsAtMask(ahead, mask)) {
          strokeTapered(ctx, w, h, run, 3, 1, entry.color, taper);
        }
      }
      const scheduled = scheduledWindow(entry.id);
      if (scheduled) drawScheduled(ctx, w, h, track, scheduled, entry.color, taper);

      const here = positionAt(track, now);
      if (here) {
        lon = here[0];
        lat = here[1];
        if (here[4] != null) altitudeKm = here[4];
      }
    }

    // The tracked satellite's own position comes from the 1 Hz feed rather
    // than from an interpolated track or a 15 s catalog poll.
    if (isTracked && tracked.observation) {
      lon = tracked.observation.longitude;
      lat = tracked.observation.latitude;
      altitudeKm = tracked.observation.altitude_km;
    } else if (lon == null && row) {
      lon = row.longitude;
      lat = row.latitude;
    }

    // The ring is centred on the station and sized from altitude alone, so it
    // is drawn before the marker bail-out below: not knowing yet where the
    // satellite is says nothing about how far away we could hear it.
    if (entry.ring && station && altitudeKm) {
      drawRangeRing(ctx, w, h, station, altitudeKm, entry.color);
    }

    if (lon == null || lat == null) continue;

    if (isTracked && tracked.observation && tracked.observation.footprint_km) {
      drawFootprint(ctx, w, h, lat, lon, tracked.observation.footprint_km, entry.color);
    }
    markers.push({ entry, lon, lat, isTracked });
  }

  if (station) drawStation(ctx, w, h, station);

  for (const m of markers) {
    const dim =
      m.isTracked && state.tracked.observation ? !state.tracked.observation.sunlit : false;
    const [x, y] = drawMarker(
      ctx,
      w,
      h,
      m.lon,
      m.lat,
      m.entry.color,
      satelliteName(m.entry.id),
      dim,
      m.isTracked || m.entry.id === hoverId
    );
    m.x = x;
    m.y = y;
  }
  mapMarkers = markers;

  // An empty display set draws a bare map under a legend naming six things
  // that are not on it. Say which, rather than leaving it looking broken.
  $("map-count").textContent =
    total === 0
      ? "nothing selected — pin a satellite to draw it"
      : total > shown.length
        ? `showing ${shown.length} of ${total}`
        : "";
}

function scheduledWindow(id) {
  const upcoming = (state && state.schedule && state.schedule.upcoming) || [];
  const entry = upcoming.find((p) => p.norad_id === id);
  return entry ? [entry.aos, entry.los] : null;
}

/* The slice of orbit the station has actually committed to. This is the one
   planning cue a pass list cannot show: which part of which orbit is ours.

   Clipped at the interpolated AOS and LOS rather than at the first and last
   samples inside the window — a whole-sample filter puts the committed
   window's ends up to a sample (~450 km of ground at LEO) from the times the
   scheduler actually committed to. */
function drawScheduled(ctx, w, h, track, [aos, los], color, taper) {
  for (const run of Geo.clipRunsInWindow(track.points, aos, los)) {
    strokeTapered(ctx, w, h, run, 3, 1, color, taper);
    // Square ends, so the committed window reads as a window and not just as
    // a brighter stretch of track — but only at a real AOS or LOS, never
    // where the window simply runs off the end of the projection.
    ctx.fillStyle = color;
    for (const p of [run[0], run[run.length - 1]]) {
      if (Math.abs(p[3] - aos) > 1 && Math.abs(p[3] - los) > 1) continue;
      const [x, y] = Geo.project(p[0], p[1], w, h);
      ctx.globalAlpha = taper(p);
      ctx.fillRect(x - 2.5, y - 2.5, 5, 5);
    }
    ctx.globalAlpha = 1;
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
  const hovered = activeView === "map" && hoverId != null ? hoverReadout(hoverId) : null;
  if (hovered) {
    info.textContent = hovered;
  } else if (state.tracked && state.tracked.observation) {
    const o = state.tracked.observation;
    info.textContent =
      `${state.tracked.name} · az ${o.azimuth.toFixed(1)}° el ${o.elevation.toFixed(1)}° · ` +
      `${o.range_km.toFixed(0)} km ${o.range_rate_km_s > 0 ? "receding" : "approaching"} ` +
      `at ${Math.abs(o.range_rate_km_s).toFixed(2)} km/s · ${o.sunlit ? "sunlit" : "in eclipse"}`;
  } else {
    info.textContent = "No satellite being tracked.";
  }
  if (state.tracked && state.tracked.observation) {
    renderTransponders(state.tracked.observation);
  } else {
    $("xpndr-section").hidden = true;
  }
  renderDoppler2m(state.doppler_2m);
  if (activeView === "map") drawMap();
  else drawSky();
}

/* One line about whatever the pointer is over, in the slot the tracked
   readout normally uses. Includes why a satellite is not workable — the map
   should be able to answer "why isn't that one highlighted". */
function hoverReadout(id) {
  const row = catalogRow(id);
  const parts = [satelliteName(id)];
  if (row) {
    parts.push(`az ${row.azimuth.toFixed(0)}° el ${row.elevation.toFixed(0)}°`);
    parts.push(`${row.range_km.toFixed(0)} km`);
  }
  const scheduled = scheduledWindow(id);
  if (scheduled) parts.push(`AOS in ${fmtCountdown(scheduled[0])}`);
  const reason = BAND_REASON[row ? row.band : null];
  if (reason) parts.push(reason);
  return parts.join(" · ");
}

/* Kept in step with orbitaly/core/bands.py */
const BAND_REASON = {
  out_of_band: "no downlink in station bands",
  rx_only: "receive only — no uplink in station bands",
  unknown: "no transponder data",
  two_way: null,
};

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

/* ---------- map controls ---------- */

function togglePin(id) {
  if (pinned.has(id)) pinned.delete(id);
  else pinned.add(id);
  savePinned();
  renderSatList();
  refreshTracks();
  if (state) render();
}

function setView(view) {
  activeView = view;
  localStorage.setItem("orbitaly.view", view);
  $("view-sky").hidden = view !== "sky";
  $("view-map").hidden = view !== "map";
  $("tab-sky").setAttribute("aria-selected", String(view === "sky"));
  $("tab-map").setAttribute("aria-selected", String(view === "map"));
  if (view === "map") {
    if (!world) loadWorld();
    refreshTracks();
  } else {
    hoverId = null;
  }
  if (state) render();
}

$("tab-sky").onclick = () => setView("sky");
$("tab-map").onclick = () => setView("map");

/* Nearest-marker hit test. No floating tooltips over the canvas — the readout
   goes in the same slot the tracked satellite uses. */
function markerAt(event) {
  const canvas = $("worldmap");
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  let best = null;
  let bestDistance = 14;
  for (const m of mapMarkers) {
    const d = Math.hypot(m.x - x, m.y - y);
    if (d < bestDistance) {
      best = m;
      bestDistance = d;
    }
  }
  return best;
}

$("worldmap").onmousemove = (e) => {
  const hit = markerAt(e);
  const id = hit ? hit.entry.id : null;
  if (id !== hoverId) {
    hoverId = id;
    $("worldmap").style.cursor = id == null ? "crosshair" : "pointer";
    if (state) render();
  }
};
$("worldmap").onmouseleave = () => {
  if (hoverId != null) {
    hoverId = null;
    if (state) render();
  }
};
$("worldmap").onclick = (e) => {
  const hit = markerAt(e);
  if (hit) selectSatellite(hit.entry.id);
};

/* Canvases are sized from their layout box, so a resize needs a redraw. */
let resizeTimer = null;
window.addEventListener("resize", () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => {
    if (state) render();
  }, 120);
});

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

setView(activeView);
connect();
refreshCatalog();
setInterval(refreshCatalog, 15000);
setInterval(renderPasses, 30000); // keep countdowns fresh
setInterval(refreshTracks, 15000); // no-op unless the map is showing
