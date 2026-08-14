"use strict";

const fragment = new URLSearchParams(location.hash.slice(1));
const token = fragment.get("token") || "";
history.replaceState(null, "", location.pathname);
const headers = {"X-Waddle-Token": token, "X-Waddle-Request": "1"};
let state = null;
let deadman = null;
let deadmanGeneration = 0;
let motionQueue = Promise.resolve();
let cameraTimer = null;
let localControl = false;
let activeTaskKey = null;

async function api(path, options = {}) {
  const request = {...options, headers: {...headers, ...(options.headers || {})}};
  if (request.body !== undefined) request.headers["Content-Type"] = "application/json";
  const response = await fetch(path, request);
  if (!response.ok) {
    let detail = `${response.status} ${response.statusText}`;
    try { detail = (await response.json()).error || detail; } catch (_) {}
    throw new Error(detail);
  }
  return response;
}

async function post(path, value = {}) {
  return (await api(path, {method: "POST", body: JSON.stringify(value)})).json();
}

function setMotion(message) { document.querySelector("#motion-status").textContent = message; }
function queueMotion(work) { const current = motionQueue.then(work, work); motionQueue = current.catch(() => {}); return current; }
function splitNames(value) { return value.split(",").map(item => item.trim()).filter(Boolean); }

async function loadState() {
  try {
    state = await (await api("/api/state")).json();
    localControl = Boolean(state.local_handoff_ready);
    document.querySelector("#connection").textContent = state.plane_connected ? "Plane connected" : "Local only / plane disconnected";
    document.querySelector("#state").textContent = JSON.stringify({episode_id: state.episode_id, episode_state: state.episode_state, gate_mode: state.gate_mode, claim_active: state.claim_active, active_claim_id: state.active_claim_id, provenance: state.provenance, plane_connected: state.plane_connected, task_sessions_negotiated: state.task_sessions_negotiated, calibration_measurements_negotiated: state.calibration_measurements_negotiated, workspace_artifacts_negotiated: state.workspace_artifacts_negotiated, execution_backend: state.execution_backend, local_handoff_ready: state.local_handoff_ready, estop_unregistered: state.estop_unregistered}, null, 2);
    const inc = state.increments;
    for (const [id, key] of [["#joint-step", "joint_step_rad"], ["#linear-step", "linear_step_m"], ["#angular-step", "angular_step_rad"]]) { const input = document.querySelector(id); if (!input.dataset.edited) input.value = inc[key]; }
    renderJog(state.jog_targets || []);
    renderCameras(state.cameras || []);
  } catch (error) { document.querySelector("#connection").textContent = `Unavailable: ${error.message}`; }
}

function stopDeadman() {
  deadmanGeneration += 1;
  if (deadman) { clearInterval(deadman.heartbeat); clearInterval(deadman.steps); }
  deadman = null;
  localControl = false;
  queueMotion(() => post("/api/jog/release")).catch(error => setMotion(error.message));
}

function startDeadman(intent) {
  if (!localControl) { setMotion("Take local control before jogging."); return; }
  const generation = ++deadmanGeneration;
  deadman = {generation, heartbeat: null, steps: null};
  queueMotion(async () => {
    if (!deadman || deadman.generation !== generation) return;
    const result = await post("/api/jog", intent);
    if (!deadman || deadman.generation !== generation) return;
    setMotion(result.accepted ? "Jog accepted; the owner's envelope still decides the whole command." : `Refused (${result.code}): ${result.detail}`);
    if (!result.accepted) { stopDeadman(); return; }
    deadman.heartbeat = setInterval(() => queueMotion(async () => {
      if (!deadman || deadman.generation !== generation) return;
      const result = await post("/api/jog/heartbeat");
      if (!result.accepted) { setMotion(`Refused (${result.code}): ${result.detail}`); stopDeadman(); }
    }).catch(error => { setMotion(error.message); stopDeadman(); }), 250);
    deadman.steps = setInterval(() => queueMotion(async () => {
      if (!deadman || deadman.generation !== generation) return;
      const result = await post("/api/jog", intent);
      if (!result.accepted) { setMotion(`Refused (${result.code}): ${result.detail}`); stopDeadman(); }
    }).catch(error => { setMotion(error.message); stopDeadman(); }), 250);
  }).catch(error => { setMotion(error.message); stopDeadman(); });
}

function jogButton(label, intent) {
  const button = document.createElement("button"); button.textContent = label;
  button.addEventListener("pointerdown", event => { event.preventDefault(); button.setPointerCapture(event.pointerId); startDeadman(intent); });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(name, stopDeadman);
  return button;
}

function renderJog(targets) {
  const root = document.querySelector("#jog"); root.replaceChildren();
  for (const target of targets) {
    const group = document.createElement("div"); const heading = document.createElement("h3"); heading.textContent = target.part || "Robot"; group.append(heading);
    if (target.unsupported) { const p = document.createElement("p"); p.className = "muted"; p.textContent = `Jog unsupported for ${target.kind}`; group.append(p); root.append(group); continue; }
    if (target.kind === "joint_position") target.joints.forEach((name, index) => { const row = document.createElement("div"); row.className = "jog-row"; const text = document.createElement("span"); text.textContent = name; row.append(text, jogButton("−", {kind:"joint", index, direction:-1, part:target.part}), jogButton("+", {kind:"joint", index, direction:1, part:target.part})); group.append(row); });
    if (target.kind === "ee_pose_delta") [["X", "linear", 0], ["Y", "linear", 1], ["Z", "linear", 2], ["Roll", "angular", 0], ["Pitch", "angular", 1], ["Yaw", "angular", 2]].forEach(([name, kind, index]) => { const row = document.createElement("div"); row.className = "jog-row"; const text = document.createElement("span"); text.textContent = `${name} (${target.frame}, ${target.delta_frame})`; row.append(text, jogButton("−", {kind, index, direction:-1, part:target.part}), jogButton("+", {kind, index, direction:1, part:target.part})); group.append(row); });
    root.append(group);
  }
}

function renderCameras(cameras) {
  const root = document.querySelector("#cameras"); const names = new Set(cameras.map(camera => camera.name));
  for (const old of [...root.querySelectorAll("figure")]) if (!names.has(old.dataset.camera)) old.remove();
  for (const camera of cameras) if (!root.querySelector(`[data-camera="${CSS.escape(camera.name)}"]`)) {
    const figure = document.createElement("figure"); figure.dataset.camera = camera.name;
    const caption = document.createElement("figcaption"); caption.textContent = `${camera.name} — ${camera.width}×${camera.height}`;
    const canvas = document.createElement("canvas"); canvas.width = camera.width; canvas.height = camera.height;
    canvas.onclick = event => submitCalibrationClick(figure, event);
    figure.append(caption, canvas); root.append(figure);
  }
  if (!cameraTimer) cameraTimer = setInterval(refreshCameras, 500);
}

async function refreshCameras() {
  for (const figure of document.querySelectorAll("#cameras figure")) try {
    const response = await api(`/api/cameras/${encodeURIComponent(figure.dataset.camera)}`);
    const width = Number(response.headers.get("X-Waddle-Width")); const height = Number(response.headers.get("X-Waddle-Height")); const rgb = new Uint8Array(await response.arrayBuffer());
    figure.dataset.frameSequence = response.headers.get("X-Waddle-Frame-Sequence") || "";
    if (rgb.length !== width * height * 3) continue;
    const rgba = new Uint8ClampedArray(width * height * 4); for (let i=0, j=0; i<rgb.length; i+=3, j+=4) { rgba[j]=rgb[i]; rgba[j+1]=rgb[i+1]; rgba[j+2]=rgb[i+2]; rgba[j+3]=255; }
    const canvas = figure.querySelector("canvas"); if (canvas.width !== width || canvas.height !== height) { canvas.width=width; canvas.height=height; } canvas.getContext("2d").putImageData(new ImageData(rgba, width, height), 0, 0);
  } catch (_) {}
}

async function submitCalibrationClick(figure, event) {
  const status = document.querySelector("#calibration-status"); const calibrationId = document.querySelector("#calibration-id").value; const sampleId = document.querySelector("#sample-id").value;
  if (!calibrationId || !sampleId || !figure.dataset.frameSequence) { status.textContent = "Calibration id, sample id, and a managed RGB-D frame are required."; return; }
  const canvas = figure.querySelector("canvas"); const rect = canvas.getBoundingClientRect(); const x = Math.min(canvas.width - 1, Math.max(0, Math.floor((event.clientX - rect.left) * canvas.width / rect.width))); const y = Math.min(canvas.height - 1, Math.max(0, Math.floor((event.clientY - rect.top) * canvas.height / rect.height)));
  try { const result = await post("/api/calibration/click", {calibration_id: calibrationId, sample_id: sampleId, camera: figure.dataset.camera, frame_sequence: Number(figure.dataset.frameSequence), x, y}); status.textContent = JSON.stringify(result.measurement, null, 2); }
  catch (error) { status.textContent = error.message; }
}

async function loadTasks() {
  try { const {tasks} = await (await api("/api/tasks")).json(); const root = document.querySelector("#task-list"); root.replaceChildren(); for (const task of tasks) { const row = document.createElement("button"); row.className = `task-row${task.key === activeTaskKey ? " active" : ""}`; row.textContent = `${task.name} — ${task.task_session_id || "creating…"}`; row.onclick = () => { activeTaskKey = task.key; renderTaskHistory(task.history); loadTasks(); }; root.append(row); if (task.key === activeTaskKey) renderTaskHistory(task.history); } } catch (_) {}
}

function renderTaskHistory(history) { const log = document.querySelector("#task-log"); log.textContent = history.map(event => event.text || event.detail || `[${event.kind}]`).filter(Boolean).join("\n"); }
async function pollTask(requestId) { try { const result = await (await api(`/api/tasks/events?request_id=${encodeURIComponent(requestId)}`)).json(); await loadTasks(); if (!result.events.some(event => ["done", "interrupted", "unavailable", "error"].includes(event.kind))) setTimeout(() => pollTask(requestId), 0); } catch (error) { document.querySelector("#task-log").textContent += `\n${error.message}`; } }

async function loadBackends() { try { const result = await (await api("/api/execution/backends")).json(); const select = document.querySelector("#execution-backend"); select.replaceChildren(); for (const backend of result.backends) { const option = document.createElement("option"); option.value = backend.id; option.textContent = backend.local ? `${backend.label} — ${backend.name}` : backend.label; option.selected = backend.id === result.selected; select.append(option); } } catch (error) { document.querySelector("#execution-status").textContent = error.message; } }

async function loadRecordings() {
  const root = document.querySelector("#recordings");
  try { const {recordings} = await (await api("/api/recordings")).json(); root.replaceChildren(); for (const item of recordings) { const row = document.createElement("div"); row.className="recording"; const text=document.createElement("div"); text.textContent=`${item.task || item.episode_id} — ${item.outcome} — ${item.t_start_unix_ns || "?"} .. ${item.t_end_unix_ns || "?"}`; row.append(text); for (const kind of item.downloads) { const button=document.createElement("button"); button.textContent=`Download ${kind}`; button.onclick=async()=>{ const response=await api(`/api/recordings/download?entry=${item.entry}&kind=${kind}`); const blob=await response.blob(); const url=URL.createObjectURL(blob); const a=document.createElement("a"); a.href=url; a.download=`${item.episode_id}.${kind === "mcap" ? "mcap" : "sidecar.json"}`; a.click(); URL.revokeObjectURL(url); }; row.append(button); } root.append(row); } } catch(error) { root.textContent=error.message; }
}

document.querySelector("#estop").onclick = () => post("/api/estop").then(result => setMotion(`E-stop ${result.status}; this is not confirmation.`)).catch(error => setMotion(error.message));
document.querySelector("#handoff").onclick = () => post("/api/handoff").then(result => { localControl = result.accepted; setMotion(result.accepted ? "Local control is ready for one jog gesture." : `Refused (${result.code}): ${result.detail}`); }).catch(error => setMotion(error.message));
for (const input of document.querySelectorAll(".increments input")) input.addEventListener("input", () => input.dataset.edited="1");
document.querySelector("#save-increments").onclick = () => post("/api/config", {joint_step_rad:Number(document.querySelector("#joint-step").value), linear_step_m:Number(document.querySelector("#linear-step").value), angular_step_rad:Number(document.querySelector("#angular-step").value)}).then(() => { for (const input of document.querySelectorAll(".increments input")) delete input.dataset.edited; setMotion("Increments updated for this UI run."); }).catch(error => setMotion(error.message));
document.querySelector("#task-create").onsubmit = async event => { event.preventDefault(); try { const result = await post("/api/tasks/create", {name: document.querySelector("#task-name").value}); activeTaskKey = result.key; pollTask(result.request_id); loadTasks(); } catch (error) { document.querySelector("#task-log").textContent = error.message; } };
document.querySelector("#task-message").onsubmit = async event => { event.preventDefault(); if (!activeTaskKey) return; const operation = event.submitter.value; try { const result = await post(`/api/tasks/${operation}`, {key: activeTaskKey, text: document.querySelector("#task-text").value}); document.querySelector("#task-text").value = ""; pollTask(result.request_id); } catch (error) { document.querySelector("#task-log").textContent += `\n${error.message}`; } };
document.querySelector("#select-backend").onclick = () => post("/api/execution/select", {backend_id: document.querySelector("#execution-backend").value}).then(result => { document.querySelector("#execution-status").textContent = `${result.backend.label} selected.`; loadBackends(); }).catch(error => document.querySelector("#execution-status").textContent = error.message);
document.querySelector("#request-artifact").onclick = async () => { const status = document.querySelector("#artifact-status"); try { const result = await post("/api/artifacts", {graph_ids: splitNames(document.querySelector("#artifact-graphs").value), calibration_names: splitNames(document.querySelector("#artifact-calibrations").value)}); const ready = await (await api(`/api/artifacts/events?request_id=${encodeURIComponent(result.request_id)}`)).json(); status.textContent = JSON.stringify(ready.events, null, 2); } catch (error) { status.textContent = error.message; } };
document.querySelector("#refresh-recordings").onclick=loadRecordings;
window.addEventListener("blur", stopDeadman); window.addEventListener("pagehide", stopDeadman);
loadState(); loadTasks(); loadBackends(); loadRecordings(); setInterval(loadState, 1000);
