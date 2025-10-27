// === Utility functions ===
async function jget(url) {
  const res = await fetch(url);
  return res.json();
}
async function jpost(url, data) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(data || {})
  });
  return res.json();
}

// === Element references ===
const robotPill = document.querySelector("#robotStatus");
const portsEl = document.querySelector("#portsJson");
const mapView = document.querySelector("#mapView");
const imgSel = document.querySelector("#imageTopic");
const pcSel = document.querySelector("#pcTopic");
const imgView = document.querySelector("#imgView");
const pcView = document.querySelector("#pcView");

// === Refresh robot status + topic dropdowns ===
async function refreshStatus() {
  try {
    const s = await jget("/api/status");
    console.log("Status response:", s);

    robotPill.textContent = s.robot_on ? "● ONLINE" : "● OFFLINE";
    robotPill.classList.toggle("on", !!s.robot_on);

    // isi dropdown image & pointcloud
    imgSel.innerHTML = s.image_topics.map(t => `<option value="${t}">${t}</option>`).join("");
    pcSel.innerHTML = s.pointcloud_topics.map(t => `<option value="${t}">${t}</option>`).join("");

  } catch (err) {
    console.error("refreshStatus error:", err);
  }
}

// === Refresh port list bar ===
async function refreshPorts() {
  try {
    const p = await jget("/api/ports");
    portsEl.innerHTML = `
      <span>🔌 Arduino: ${p.ttyUSB.length ? p.ttyUSB.join(", ") : "—"}</span>
      <span>🧭 RPLIDAR: ${p.ttyACM.length ? p.ttyACM.join(", ") : "—"}</span>
      <span>📷 RealSense: ${p.video.length ? p.video.join(", ") : "—"}</span>
    `;
  } catch (err) {
    console.error("refreshPorts error:", err);
  }
}

// === Snapshot handlers ===
async function snapMap() {
  const u = `/api/map.png?_${Date.now()}`;
  mapView.src = u;
  const dl = document.querySelector("#btnMapDL");
  if (dl) dl.href = u;
}

async function snapImage() {
  const t = imgSel.value;
  if (!t) return;
  imgView.src = `/api/image.png?topic=${encodeURIComponent(t)}&_=${Date.now()}`;
}

async function snapPC() {
  const t = pcSel.value;
  if (!t) return;
  pcView.src = `/api/pointcloud.png?topic=${encodeURIComponent(t)}&_=${Date.now()}`;
}

// === Teleoperation ===
Array.from(document.querySelectorAll(".key")).forEach(b =>
  b.addEventListener("click", () => jpost("/api/teleop/wasd", { key: b.dataset.k }))
);
document.addEventListener("keydown", e => {
  const k = e.key.toUpperCase();
  if (["W", "A", "S", "D", "X"].includes(k))
    jpost("/api/teleop/wasd", { key: k });
});
document.querySelector("#btnStop").addEventListener("click", () => jpost("/api/stop", {}));

// === Navigation ===
const gx = document.querySelector("#goalX");
const gy = document.querySelector("#goalY");
const gyaw = document.querySelector("#goalYaw");
document.querySelector("#btnSendGoal").addEventListener("click", () => {
  const yawRad = (parseFloat(gyaw.value) || 0) * Math.PI / 180;
  jpost("/api/nav2/goal", {
    x: parseFloat(gx.value) || 0,
    y: parseFloat(gy.value) || 0,
    yaw: yawRad
  });
});

// === Joystick control ===
const joy = document.querySelector("#joystick");
const knob = joy.querySelector(".knob");
let jActive = false;

joy.addEventListener("mousedown", startJoy);
joy.addEventListener("touchstart", startJoy);
joy.addEventListener("dragstart", e => e.preventDefault());
["mousemove", "touchmove"].forEach(ev => document.addEventListener(ev, moveJoy));
["mouseup", "mouseleave", "touchend", "touchcancel"].forEach(ev => document.addEventListener(ev, stopJoy));

function startJoy(ev) {
  jActive = true;
  moveJoy(ev);
}

function moveJoy(ev) {
  if (!jActive) return;
  const r = joy.getBoundingClientRect();
  const p = ev.touches ? ev.touches[0] : ev;
  const dx = p.clientX - (r.left + r.width / 2);
  const dy = p.clientY - (r.top + r.height / 2);
  const radius = r.width / 2;
  const dist = Math.min(Math.sqrt(dx * dx + dy * dy), radius);
  const angle = Math.atan2(dy, dx);
  const x = Math.cos(angle) * dist;
  const y = Math.sin(angle) * dist;

  // update visual knob
  knob.style.left = `${50 + (x / radius) * 50}%`;
  knob.style.top = `${50 + (y / radius) * 50}%`;

  // normalize range [-1, 1]
  const nx = x / radius;
  const ny = y / radius;

  const forward = -ny;
  const turn = nx;
  const key = Math.abs(forward) > Math.abs(turn)
    ? forward > 0 ? "W" : "S"
    : turn > 0 ? "D" : "A";

  jpost("/api/teleop/wasd", { key });
}

function stopJoy() {
  if (!jActive) return;
  jActive = false;
  knob.style.left = "50%";
  knob.style.top = "50%";
  jpost("/api/teleop/wasd", { key: "X" });
}

// === Event listeners ===
document.querySelector("#btnMap").addEventListener("click", snapMap);
document.querySelector("#btnSnapImage").addEventListener("click", snapImage);
document.querySelector("#btnPC").addEventListener("click", snapPC);

imgSel.addEventListener("change", snapImage);
pcSel.addEventListener("change", snapPC);

// === Auto-initialize ===
async function initDashboard() {
  await refreshPorts();
  await refreshStatus();
  snapMap();
  // auto-refresh loop
  setInterval(refreshPorts, 5000);
  setInterval(refreshStatus, 10000);
  setInterval(snapMap, 2000);
}
initDashboard();
