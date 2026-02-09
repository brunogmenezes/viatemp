const express = require('express');
const bodyParser = require('body-parser');
const cors = require('cors');
const mqtt = require('mqtt');
const path = require('path');
const fs = require('fs');
const multer = require('multer');

const app = express();
app.use(cors());
app.use(bodyParser.json({ limit: '2mb' }));
// Load auth routes
const authRouter = require('./auth');
app.use('/api/auth', authRouter);

// Create HTTP server and attach Socket.IO for realtime
const http = require('http');
const server = http.createServer(app);
const { Server } = require('socket.io');
const io = new Server(server, { cors: { origin: '*' } });

io.on('connection', (socket) => {
  console.log('Web client connected', socket.id);
  // send initial state
  socket.emit('devices_updated', getDeviceList());
});

const MQTT_URL = process.env.MQTT_URL || 'mqtt://localhost:1883';
const MQTT_USER = process.env.MQTT_USER;
const MQTT_PASS = process.env.MQTT_PASS;
const HEARTBEAT_TTL_MS = parseInt(process.env.HEARTBEAT_TTL_MS || '120000', 10);
const FIRMWARE_BASE_URL = process.env.FIRMWARE_BASE_URL || '';

// In-memory device registry: mac -> device
const devices = {};
const DEVICES_FILE = path.join(__dirname, 'devices.json');
let saveDevicesTimer = null;

const FIRMWARE_DIR = path.join(__dirname, 'firmware');
const FIRMWARE_INDEX_FILE = path.join(__dirname, 'firmware.json');
let firmwareIndex = [];

function ensureFirmwareDir() {
  if (!fs.existsSync(FIRMWARE_DIR)) {
    fs.mkdirSync(FIRMWARE_DIR, { recursive: true });
  }
}

function loadFirmwareIndex() {
  if (!fs.existsSync(FIRMWARE_INDEX_FILE)) return;
  try {
    const raw = fs.readFileSync(FIRMWARE_INDEX_FILE, 'utf8');
    const data = JSON.parse(raw);
    if (Array.isArray(data)) firmwareIndex = data;
  } catch (e) {
    console.warn('Failed to load firmware index', e.message);
  }
}

function saveFirmwareIndex() {
  try {
    fs.writeFileSync(FIRMWARE_INDEX_FILE, JSON.stringify(firmwareIndex, null, 2));
  } catch (e) {
    console.warn('Failed to save firmware index', e.message);
  }
}

function sanitizeVersion(value) {
  const v = String(value || '').trim();
  if (!v) return '';
  if (!/^[0-9A-Za-z._-]+$/.test(v)) return '';
  return v;
}

function compareVersions(a, b) {
  const pa = String(a || '').split('.').map(n => parseInt(n, 10));
  const pb = String(b || '').split('.').map(n => parseInt(n, 10));
  const len = Math.max(pa.length, pb.length);
  for (let i = 0; i < len; i += 1) {
    const na = Number.isFinite(pa[i]) ? pa[i] : 0;
    const nb = Number.isFinite(pb[i]) ? pb[i] : 0;
    if (na > nb) return 1;
    if (na < nb) return -1;
  }
  return 0;
}

function getLatestFirmware() {
  if (!firmwareIndex.length) return null;
  return firmwareIndex.reduce((best, current) => {
    if (!best) return current;
    if (compareVersions(current.version, best.version) > 0) return current;
    if (compareVersions(current.version, best.version) === 0) {
      return (current.uploadedAt || '') > (best.uploadedAt || '') ? current : best;
    }
    return best;
  }, null);
}

ensureFirmwareDir();
loadFirmwareIndex();

const storage = multer.diskStorage({
  destination: (req, file, cb) => cb(null, FIRMWARE_DIR),
  filename: (req, file, cb) => {
    const v = sanitizeVersion(req.body && req.body.version);
    const base = v ? `firmware_v${v}` : 'firmware';
    const name = `${base}_${Date.now()}.bin`;
    cb(null, name);
  }
});

const upload = multer({
  storage,
  limits: { fileSize: 50 * 1024 * 1024 },
  fileFilter: (req, file, cb) => {
    const ext = path.extname(file.originalname).toLowerCase();
    if (ext !== '.bin') return cb(new Error('invalid_file_type'));
    cb(null, true);
  }
});

function loadDevicesFromDisk() {
  if (!fs.existsSync(DEVICES_FILE)) return;
  try {
    const raw = fs.readFileSync(DEVICES_FILE, 'utf8');
    const data = JSON.parse(raw);
    if (data && typeof data === 'object') {
      Object.keys(data).forEach((k) => {
        if (data[k] && typeof data[k] === 'object') devices[k] = data[k];
      });
      console.log('Devices loaded from disk:', Object.keys(devices).length);
    }
  } catch (e) {
    console.warn('Failed to load devices from disk', e.message);
  }
}

function scheduleSaveDevices() {
  if (saveDevicesTimer) clearTimeout(saveDevicesTimer);
  saveDevicesTimer = setTimeout(() => {
    try {
      fs.writeFileSync(DEVICES_FILE, JSON.stringify(devices, null, 2));
    } catch (e) {
      console.warn('Failed to save devices to disk', e.message);
    }
  }, 1000);
}

loadDevicesFromDisk();

function getDeviceList() {
  const now = Date.now();
  return Object.values(devices).map((d) => {
    const lastSeen = d.lastSeen || 0;
    const online = lastSeen > 0 && (now - lastSeen) <= HEARTBEAT_TTL_MS;
    return { ...d, lastSeen, online };
  });
}

const mqttOptions = {};
if (MQTT_USER) mqttOptions.username = MQTT_USER;
if (MQTT_PASS) mqttOptions.password = MQTT_PASS;
if (Object.keys(mqttOptions).length > 0) console.log('MQTT auth enabled for user', MQTT_USER ? MQTT_USER : '<unknown>');
const mqttClient = mqtt.connect(MQTT_URL, mqttOptions);
const db = require('./db');

const jwt = require('jsonwebtoken');

function verifyTokenMiddleware(req, res, next) {
  const auth = req.headers['authorization'];
  if (!auth) return res.status(401).json({ error: 'missing_token' });
  const parts = auth.split(' ');
  if (parts.length !== 2 || parts[0] !== 'Bearer') return res.status(401).json({ error: 'invalid_token' });
  const token = parts[1];
  try {
    const payload = jwt.verify(token, process.env.JWT_SECRET || 'dev_secret');
    req.user = payload;
    next();
  } catch (e) {
    return res.status(401).json({ error: 'invalid' });
  }
}

mqttClient.on('connect', () => {
  console.log('Connected to MQTT broker', MQTT_URL);
  mqttClient.subscribe('devices/announce', { qos: 0 });
  mqttClient.subscribe('devices/heartbeat', { qos: 0 });
  mqttClient.subscribe('esp32/temperature', { qos: 0 });
});

mqttClient.on('message', (topic, payload) => {
  try {
    const msg = JSON.parse(payload.toString());
    if (topic === 'devices/announce') {
      const mac = msg.mac;
      if (!mac) return;
      const key = mac.replace(/:/g, '').toLowerCase();
      devices[key] = devices[key] || {};
      devices[key].mac = mac;
      devices[key].ip = msg.ip || devices[key].ip;
      devices[key].version = msg.version || devices[key].version;
      devices[key].lastSeen = Date.now();
      devices[key].adopted = devices[key].adopted || false;
      scheduleSaveDevices();
      console.log('Device announced:', mac);
      // broadcast update to web clients
      io.emit('devices_updated', getDeviceList());
    }
    if (topic === 'devices/heartbeat') {
      const mac = msg.mac;
      if (!mac) return;
      const key = mac.replace(/:/g, '').toLowerCase();
      devices[key] = devices[key] || {};
      devices[key].mac = mac;
      devices[key].ip = msg.ip || devices[key].ip;
      devices[key].version = msg.version || devices[key].version;
      devices[key].uptime = typeof msg.uptime === 'number' ? msg.uptime : devices[key].uptime;
      devices[key].rssi = typeof msg.rssi === 'number' ? msg.rssi : devices[key].rssi;
      devices[key].heap = typeof msg.heap === 'number' ? msg.heap : devices[key].heap;
      devices[key].lastSeen = Date.now();
      devices[key].adopted = devices[key].adopted || false;
      scheduleSaveDevices();
      io.emit('devices_updated', getDeviceList());
    }
    if (topic === 'esp32/temperature') {
      const mac = msg.mac;
      if (!mac) return;
      const key = mac.replace(/:/g, '').toLowerCase();
      devices[key] = devices[key] || {};
      devices[key].mac = mac;
      if (typeof msg.ip === 'string' && msg.ip.length) devices[key].ip = msg.ip;
      if (typeof msg.version === 'string' && msg.version.length) devices[key].version = msg.version;
      if (typeof msg.sensor === 'string' && msg.sensor.length) devices[key].name = msg.sensor;
      if (typeof msg.location === 'string' && msg.location.length) devices[key].location = msg.location;
      devices[key].uptime = typeof msg.uptime === 'number' ? msg.uptime : devices[key].uptime;
      devices[key].rssi = typeof msg.rssi === 'number' ? msg.rssi : devices[key].rssi;
      devices[key].heap = typeof msg.heap === 'number' ? msg.heap : devices[key].heap;
      const temp = typeof msg.temperature === 'number' ? msg.temperature :
        (typeof msg.temperature === 'string' ? parseFloat(msg.temperature) : null);
      if (Number.isFinite(temp)) devices[key].temperature = temp;
      devices[key].lastSeen = Date.now();
      devices[key].adopted = devices[key].adopted || false;
      scheduleSaveDevices();
      io.emit('devices_updated', getDeviceList());
    }
  } catch (e) {
    console.warn('Invalid MQTT payload', e.message);
  }
});

// Firmware APIs
app.get('/api/firmware', (req, res) => {
  res.json(firmwareIndex);
});

app.get('/api/firmware/latest', (req, res) => {
  const latest = getLatestFirmware();
  if (!latest) return res.status(200).json(null);
  res.json(latest);
});

app.post('/api/firmware/upload', verifyTokenMiddleware, (req, res) => {
  upload.single('firmware')(req, res, (err) => {
    if (err) {
      if (err.code === 'LIMIT_FILE_SIZE') return res.status(413).json({ error: 'file_too_large' });
      return res.status(400).json({ error: err.message || 'upload_failed' });
    }
    const version = sanitizeVersion(req.body && req.body.version);
    if (!version) {
      if (req.file && req.file.path) fs.unlink(req.file.path, () => {});
      return res.status(400).json({ error: 'invalid_version' });
    }
    if (!req.file) return res.status(400).json({ error: 'missing_file' });

    const entry = {
      version,
      filename: req.file.filename,
      originalName: req.file.originalname,
      size: req.file.size,
      uploadedAt: new Date().toISOString()
    };
    firmwareIndex.push(entry);
    saveFirmwareIndex();
    res.json(entry);
  });
});

// API
app.get('/api/devices', (req, res) => {
  res.json(getDeviceList());
});

app.get('/favicon.ico', (req, res) => {
  res.status(204).end();
});

app.get('/api/devices/unadopted', (req, res) => {
  const arr = getDeviceList().filter(d => !d.adopted);
  res.json(arr);
});

// Adopt a device: POST /api/devices/:mac/adopt { name, location }
app.post('/api/devices/:mac/adopt', verifyTokenMiddleware, async (req, res) => {
  const mac = req.params.mac.replace(/:/g, '').toLowerCase();
  const device = devices[mac];
  if (!device) return res.status(404).json({ error: 'Device not found' });
  const { name, location, location_id } = req.body;
  device.name = name || device.name || '';
  // If location_id provided, resolve name from DB
  if (location_id) {
    try {
      const r = await db.query('SELECT id, name FROM locations WHERE id = $1 LIMIT 1', [location_id]);
      if (r.rowCount > 0) {
        device.location = r.rows[0].name;
        device.location_id = r.rows[0].id;
      }
    } catch (e) {
      console.warn('Error fetching location', e.message);
    }
  } else if (location) {
    device.location = location;
  }
  device.adopted = true;
  scheduleSaveDevices();

  // Publish config to device
  const cfgTopic = `devices/${mac}/config`;
  const payloadObj = { name: device.name };
  if (device.location_id) payloadObj.location_id = device.location_id;
  else if (device.location) payloadObj.location = device.location;
  const payload = JSON.stringify(payloadObj);
  mqttClient.publish(cfgTopic, payload, { qos: 0 }, (err) => {
    if (err) console.warn('Publish error', err.message);
  });

  // notify web clients
  io.emit('devices_updated', getDeviceList());

  res.json(device);
});

// Remove adoption / reset device to unadopted state
app.delete('/api/devices/:mac', verifyTokenMiddleware, (req, res) => {
  const mac = req.params.mac.replace(/:/g, '').toLowerCase();
  const device = devices[mac];
  if (!device) return res.status(404).json({ error: 'Device not found' });

  device.adopted = false;
  device.name = '';
  device.location = '';
  device.location_id = null;
  scheduleSaveDevices();

  // Inform device that it was unadopted
  const cfgTopic = `devices/${mac}/config`;
  const payload = JSON.stringify({ removed: true });
  mqttClient.publish(cfgTopic, payload, { qos: 0 }, (err) => {
    if (err) console.warn('Publish error', err.message);
  });

  console.log('Device unadopted:', device.mac);
  // notify web clients
  io.emit('devices_updated', getDeviceList());

  res.json({ ok: true, device });
});

// Restart a device via MQTT command
app.post('/api/devices/:mac/restart', verifyTokenMiddleware, (req, res) => {
  const mac = req.params.mac.replace(/:/g, '').toLowerCase();
  const device = devices[mac];
  if (!device) return res.status(404).json({ error: 'Device not found' });
  const cmdTopic = `devices/${mac}/command`;
  const payload = JSON.stringify({ action: 'restart' });
  mqttClient.publish(cmdTopic, payload, { qos: 0 }, (err) => {
    if (err) {
      console.warn('Publish error', err.message);
      return res.status(500).json({ error: 'publish_failed' });
    }
    res.json({ ok: true });
  });
});

// Update device firmware via MQTT command
app.post('/api/devices/:mac/update', verifyTokenMiddleware, (req, res) => {
  const mac = req.params.mac.replace(/:/g, '').toLowerCase();
  const device = devices[mac];
  if (!device) return res.status(404).json({ error: 'Device not found' });

  let target = null;
  const requestedVersion = sanitizeVersion(req.body && req.body.version);
  if (requestedVersion) {
    target = firmwareIndex.find(fw => fw.version === requestedVersion) || null;
  }
  if (!target) target = getLatestFirmware();
  if (!target) return res.status(404).json({ error: 'no_firmware' });

  const forwardedProto = req.headers['x-forwarded-proto'];
  const baseUrl = FIRMWARE_BASE_URL || (forwardedProto ? `${forwardedProto}://${req.get('host')}` : `${req.protocol}://${req.get('host')}`);
  const url = `${baseUrl}/firmware/${target.filename}`;

  const cmdTopic = `devices/${mac}/command`;
  const payload = JSON.stringify({ action: 'update', url, version: target.version });
  mqttClient.publish(cmdTopic, payload, { qos: 0 }, (err) => {
    if (err) {
      console.warn('Publish error', err.message);
      return res.status(500).json({ error: 'publish_failed' });
    }
    res.json({ ok: true, target });
  });
});

// Serve static frontend
app.use('/', express.static(path.join(__dirname, '..', 'client')));

// Serve firmware files
app.use('/firmware', express.static(FIRMWARE_DIR));

// Locations API (protected write operations)
const locationsRouter = require('./locations');
// GET is public, protect POST/PUT/DELETE
app.use('/api/locations', locationsRouter);

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => console.log(`Server listening on http://localhost:${PORT}`));
