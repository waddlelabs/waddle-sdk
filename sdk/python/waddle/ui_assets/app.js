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

function queueMotion(work) {
  const current = motionQueue.then(work, work);
  motionQueue = current.catch(() => {});
  return current;
}

async function loadState() {
  try {
    state = await (await api("/api/state")).json();
    document.querySelector("#connection").textContent = state.plane_connected ? "Plane connected" : "Local only / plane disconnected";
    document.querySelector("#state").textContent = JSON.stringify({episode_id: state.episode_id, episode_state: state.episode_state, gate_mode: state.gate_mode, claim_active: state.claim_active, active_claim_id: state.active_claim_id, provenance: state.provenance, plane_connected: state.plane_connected, chat_negotiated: state.chat_negotiated, agent_invited: state.agent_invited, agent_engaged: state.agent_engaged, estop_unregistered: state.estop_unregistered}, null, 2);
    document.querySelector("#chat-status").textContent = state.plane_connected && state.chat_negotiated ? "Chat transport is available when this session's invited host is alive." : "Chat is unavailable on this connection. Local state, e-stop, jog, cameras, and recordings remain available.";
    const inc = state.increments;
    for (const [id, key] of [["#joint-step", "joint_step_rad"], ["#linear-step", "linear_step_m"], ["#angular-step", "angular_step_rad"]]) {
      const input = document.querySelector(id); if (!input.dataset.edited) input.value = inc[key];
    }
    renderJog(state.jog_targets || []);
    renderCameras(state.cameras || []);
  } catch (error) {
    document.querySelector("#connection").textContent = `Unavailable: ${error.message}`;
  }
}

function stopDeadman() {
  deadmanGeneration += 1;
  if (deadman) {
    clearInterval(deadman.heartbeat);
    clearInterval(deadman.steps);
  }
  deadman = null;
  // Release is serialized behind any already-running jog request. A quick
  // pointer-up therefore cannot be overtaken by that request's late reply,
  // and queued step/heartbeat callbacks see the changed generation and no-op.
  queueMotion(() => post("/api/jog/release")).catch(error => setMotion(error.message));
}

function startDeadman(intent) {
  stopDeadman();
  const generation = ++deadmanGeneration;
  deadman = {generation, heartbeat: null, steps: null};
  queueMotion(async () => {
    if (!deadman || deadman.generation !== generation) return;
    const result = await post("/api/jog", intent);
    if (!deadman || deadman.generation !== generation) return;
    setMotion(result.accepted ? "Jog step accepted; owner's envelope still decides the whole command." : `Refused (${result.code}): ${result.detail}`);
    if (!result.accepted) { stopDeadman(); return; }
    deadman.heartbeat = setInterval(() => queueMotion(async () => {
      if (!deadman || deadman.generation !== generation) return;
      const heartbeat = await post("/api/jog/heartbeat");
      if (!heartbeat.accepted) {
        setMotion(`Refused (${heartbeat.code}): ${heartbeat.detail}`);
        stopDeadman();
      }
    }).catch(error => { setMotion(error.message); stopDeadman(); }), 250);
    deadman.steps = setInterval(() => queueMotion(async () => {
      if (!deadman || deadman.generation !== generation) return;
      const step = await post("/api/jog", intent);
      if (!step.accepted) {
        setMotion(`Refused (${step.code}): ${step.detail}`);
        stopDeadman();
      }
    }).catch(error => { setMotion(error.message); stopDeadman(); }), 250);
  }).catch(error => { setMotion(error.message); stopDeadman(); });
}

function jogButton(label, intent) {
  const button = document.createElement("button");
  button.textContent = label;
  button.addEventListener("pointerdown", event => { event.preventDefault(); button.setPointerCapture(event.pointerId); startDeadman(intent); });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(name, stopDeadman);
  return button;
}

function renderJog(targets) {
  const root = document.querySelector("#jog"); root.replaceChildren();
  for (const target of targets) {
    const group = document.createElement("div");
    const heading = document.createElement("h3"); heading.textContent = target.part || "Robot"; group.append(heading);
    if (target.unsupported) { const p = document.createElement("p"); p.className = "muted"; p.textContent = `Jog unsupported for ${target.kind}`; group.append(p); root.append(group); continue; }
    if (target.kind === "joint_position") target.joints.forEach((name, index) => {
      const row = document.createElement("div"); row.className = "jog-row"; const text = document.createElement("span"); text.textContent = name; row.append(text, jogButton("−", {kind:"joint", index, direction:-1, part:target.part}), jogButton("+", {kind:"joint", index, direction:1, part:target.part})); group.append(row);
    });
    if (target.kind === "ee_pose_delta") [["X", "linear", 0], ["Y", "linear", 1], ["Z", "linear", 2], ["Roll", "angular", 0], ["Pitch", "angular", 1], ["Yaw", "angular", 2]].forEach(([name, kind, index]) => {
      const row = document.createElement("div"); row.className = "jog-row"; const text = document.createElement("span"); text.textContent = `${name} (${target.frame}, ${target.delta_frame})`; row.append(text, jogButton("−", {kind, index, direction:-1, part:target.part}), jogButton("+", {kind, index, direction:1, part:target.part})); group.append(row);
    });
    root.append(group);
  }
}

function renderCameras(cameras) {
  const root = document.querySelector("#cameras");
  const names = new Set(cameras.map(c => c.name));
  for (const old of [...root.querySelectorAll("figure")]) if (!names.has(old.dataset.camera)) old.remove();
  for (const camera of cameras) if (!root.querySelector(`[data-camera="${CSS.escape(camera.name)}"]`)) {
    const figure = document.createElement("figure"); figure.dataset.camera = camera.name; const caption = document.createElement("figcaption"); caption.textContent = `${camera.name} — ${camera.width}×${camera.height}`; const canvas = document.createElement("canvas"); canvas.width = camera.width; canvas.height = camera.height; figure.append(caption, canvas); root.append(figure);
  }
  if (!cameraTimer) cameraTimer = setInterval(refreshCameras, 500);
}

async function refreshCameras() {
  for (const figure of document.querySelectorAll("#cameras figure")) try {
    const response = await api(`/api/cameras/${encodeURIComponent(figure.dataset.camera)}`);
    const width = Number(response.headers.get("X-Waddle-Width")); const height = Number(response.headers.get("X-Waddle-Height")); const rgb = new Uint8Array(await response.arrayBuffer());
    if (rgb.length !== width * height * 3) continue;
    const rgba = new Uint8ClampedArray(width * height * 4); for (let i=0, j=0; i<rgb.length; i+=3, j+=4) { rgba[j]=rgb[i]; rgba[j+1]=rgb[i+1]; rgba[j+2]=rgb[i+2]; rgba[j+3]=255; }
    const canvas = figure.querySelector("canvas"); if (canvas.width !== width || canvas.height !== height) { canvas.width=width; canvas.height=height; } canvas.getContext("2d").putImageData(new ImageData(rgba, width, height), 0, 0);
  } catch (_) {}
}

async function loadRecordings() {
  const root = document.querySelector("#recordings");
  try { const {recordings} = await (await api("/api/recordings")).json(); root.replaceChildren(); for (const item of recordings) { const row = document.createElement("div"); row.className="recording"; const text=document.createElement("div"); text.textContent=`${item.task || item.episode_id} — ${item.outcome} — ${item.t_start_unix_ns || "?"} .. ${item.t_end_unix_ns || "?"}`; row.append(text); for (const kind of item.downloads) { const button=document.createElement("button"); button.textContent=`Download ${kind}`; button.onclick=async()=>{ const response=await api(`/api/recordings/download?entry=${item.entry}&kind=${kind}`); const blob=await response.blob(); const url=URL.createObjectURL(blob); const a=document.createElement("a"); a.href=url; a.download=`${item.episode_id}.${kind === "mcap" ? "mcap" : "sidecar.json"}`; a.click(); URL.revokeObjectURL(url); }; row.append(button); } root.append(row); } } catch(error) { root.textContent=error.message; }
}

async function pollChat(requestId) {
  let after=0; const log=document.querySelector("#chat-log");
  while (true) { try { const {events}=await (await api(`/api/chat/events?request_id=${encodeURIComponent(requestId)}&after=${after}`)).json(); for (const event of events) { after=Math.max(after,event.sequence); if (event.kind==="text") log.textContent += event.text; if (event.detail) log.textContent += `\n${event.detail}\n`; if (["done","unavailable","error"].includes(event.kind)) return; } } catch(error) { log.textContent += `\nChat unavailable: ${error.message}\nLocal controls remain available.\n`; return; } }
}

document.querySelector("#estop").onclick = () => post("/api/estop").then(r => setMotion(`E-stop ${r.status}; this is not confirmation.`)).catch(error => setMotion(error.message));
for (const input of document.querySelectorAll(".increments input")) input.addEventListener("input", () => input.dataset.edited="1");
document.querySelector("#save-increments").onclick = () => post("/api/config", {joint_step_rad:Number(document.querySelector("#joint-step").value), linear_step_m:Number(document.querySelector("#linear-step").value), angular_step_rad:Number(document.querySelector("#angular-step").value)}).then(() => { for (const input of document.querySelectorAll(".increments input")) delete input.dataset.edited; setMotion("Increments updated for this UI run."); }).catch(error => setMotion(error.message));
document.querySelector("#chat-form").onsubmit = async event => { event.preventDefault(); const text=document.querySelector("#chat-text").value; const log=document.querySelector("#chat-log"); log.textContent += `\nYou: ${text}\nHost: `; try { const result=await post("/api/chat", {text}); document.querySelector("#chat-text").value=""; pollChat(result.request_id); } catch(error) { log.textContent += `\nChat unavailable: ${error.message}\nLocal controls remain available.\n`; } };
document.querySelector("#refresh-recordings").onclick=loadRecordings;
window.addEventListener("blur", stopDeadman); window.addEventListener("pagehide", stopDeadman);
loadState(); loadRecordings(); setInterval(loadState, 1000);
