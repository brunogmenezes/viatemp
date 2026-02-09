const express = require('express');
const bodyParser = require('body-parser');
const cors = require('cors');
const mqtt = require('mqtt');
const path = require('path');
const fs = require('fs');
const multer = require('multer');
const https = require('https');
const net = require('net');

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
const ZABBIX_ITEM_KEYS = {
  uptime: 'viatemp.uptime',
  version: 'viatemp.version',
  temperature: 'viatemp.temperature'
};

const ZABBIX_CONFIG_FILE = path.join(__dirname, 'zabbix.json');
const zabbixConfig = {
  enabled: process.env.ZABBIX_ENABLED === '1',
  apiUrl: process.env.ZABBIX_API_URL || '',
  apiToken: process.env.ZABBIX_API_TOKEN || '',
  hostGroup: process.env.ZABBIX_HOST_GROUP || 'VIATEMP',
  templateName: process.env.ZABBIX_TEMPLATE || 'VIATEMP',
  authUser: process.env.ZABBIX_USER || '',
  authPass: process.env.ZABBIX_PASS || '',
  senderHost: process.env.ZABBIX_SENDER_HOST || '',
  senderPort: parseInt(process.env.ZABBIX_SENDER_PORT || '10051', 10)
};
let zabbixSession = { token: '', expiresAt: 0 };
let zabbixHistoryPushUnsupported = false;

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

function loadZabbixConfig() {
  if (!fs.existsSync(ZABBIX_CONFIG_FILE)) return;
  try {
    const raw = fs.readFileSync(ZABBIX_CONFIG_FILE, 'utf8');
    const data = JSON.parse(raw);
    if (typeof data.enabled === 'boolean') zabbixConfig.enabled = data.enabled;
    if (typeof data.apiUrl === 'string' && data.apiUrl.trim().length) zabbixConfig.apiUrl = data.apiUrl.trim();
    if (typeof data.hostGroup === 'string' && data.hostGroup.trim().length) zabbixConfig.hostGroup = data.hostGroup.trim();
  } catch (e) {
    console.warn('Failed to load Zabbix config', e.message);
  }
}

function saveZabbixConfig() {
  const payload = {
    enabled: !!zabbixConfig.enabled,
    apiUrl: zabbixConfig.apiUrl,
    hostGroup: zabbixConfig.hostGroup
  };
  try {
    fs.writeFileSync(ZABBIX_CONFIG_FILE, JSON.stringify(payload, null, 2));
  } catch (e) {
    console.warn('Failed to save Zabbix config', e.message);
  }
}

function getZabbixPublicConfig() {
  return {
    enabled: !!zabbixConfig.enabled,
    apiUrl: zabbixConfig.apiUrl,
    hostGroup: zabbixConfig.hostGroup,
    tokenConfigured: !!zabbixConfig.apiToken
  };
}

function isZabbixEnabled() {
  const hasToken = !!zabbixConfig.apiToken;
  const hasUserPass = !!(zabbixConfig.authUser && zabbixConfig.authPass);
  return !!(zabbixConfig.enabled && zabbixConfig.apiUrl && (hasToken || hasUserPass));
}

function zabbixHttpRequest(payload, headers) {
  return new Promise((resolve, reject) => {
    const apiUrl = new URL(zabbixConfig.apiUrl);
    const options = {
      method: 'POST',
      hostname: apiUrl.hostname,
      port: apiUrl.port || (apiUrl.protocol === 'https:' ? 443 : 80),
      path: apiUrl.pathname + apiUrl.search,
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(payload),
        ...headers
      }
    };
    const client = apiUrl.protocol === 'https:' ? https : http;
    const req = client.request(options, (res) => {
      let body = '';
      res.on('data', (chunk) => { body += chunk; });
      res.on('end', () => {
        try {
          const json = JSON.parse(body || '{}');
          if (json.error) {
            const err = new Error(json.error.message || 'zabbix_error');
            err.code = json.error.code;
            err.data = json.error.data;
            err.method = json.error.data && json.error.data.method ? json.error.data.method : undefined;
            return reject(err);
          }
          resolve(json.result);
        } catch (e) {
          reject(e);
        }
      });
    });
    req.on('error', reject);
    req.write(payload);
    req.end();
  });
}

function getZabbixSenderHost() {
  if (zabbixConfig.senderHost) return zabbixConfig.senderHost;
  if (zabbixConfig.apiUrl) {
    try {
      const apiUrl = new URL(zabbixConfig.apiUrl);
      return apiUrl.hostname;
    } catch (e) {
      return '';
    }
  }
  return '';
}

function zabbixSenderRequest(data) {
  const host = getZabbixSenderHost();
  const port = Number.isFinite(zabbixConfig.senderPort) ? zabbixConfig.senderPort : 10051;
  if (!host) return Promise.reject(new Error('zabbix_sender_host_missing'));

  const now = Date.now();
  const clock = Math.floor(now / 1000);
  const keys = Array.isArray(data) ? data.map(d => d.key).filter(Boolean) : [];
  console.log('Zabbix sender payload', { host, port, clock, keys, count: Array.isArray(data) ? data.length : 0 });

  const payloadObj = {
    request: 'sender data',
    data,
    clock
  };
  const payload = JSON.stringify(payloadObj);
  const header = Buffer.from('ZBXD\x01');
  const length = Buffer.alloc(8);
  length.writeUInt32LE(Buffer.byteLength(payload), 0);
  length.writeUInt32LE(0, 4);
  const packet = Buffer.concat([header, length, Buffer.from(payload)]);

  return new Promise((resolve, reject) => {
    const socket = new net.Socket();
    const timer = setTimeout(() => {
      socket.destroy();
      reject(new Error('zabbix_sender_timeout'));
    }, 6000);

    const chunks = [];
    socket.on('data', (chunk) => { chunks.push(chunk); });
    socket.on('error', (e) => {
      clearTimeout(timer);
      reject(e);
    });
    socket.on('close', () => {
      clearTimeout(timer);
      const buf = Buffer.concat(chunks);
      if (buf.length < 13) return resolve(null);
      const jsonStart = 13;
      const jsonStr = buf.slice(jsonStart).toString('utf8');
      try {
        const json = JSON.parse(jsonStr);
        if (json.response && json.response !== 'success') {
          const err = new Error('zabbix_sender_failed');
          err.info = json.info;
          return reject(err);
        }
        resolve(json);
      } catch (e) {
        resolve(null);
      }
    });

    socket.connect(port, host, () => {
      socket.write(packet);
    });
  });
}

async function zabbixLogin() {
  const payload = JSON.stringify({
    jsonrpc: '2.0',
    method: 'user.login',
    params: {
      user: zabbixConfig.authUser,
      password: zabbixConfig.authPass
    },
    id: Date.now()
  });
  const token = await zabbixHttpRequest(payload, {});
  if (typeof token === 'string' && token.length) {
    zabbixSession = { token, expiresAt: Date.now() + (10 * 60 * 1000) };
    return token;
  }
  throw new Error('zabbix_login_failed');
}

async function getZabbixAuth() {
  if (zabbixConfig.apiToken) return { useBearer: true, token: zabbixConfig.apiToken };
  if (!(zabbixConfig.authUser && zabbixConfig.authPass)) return { useBearer: false, token: '' };
  if (zabbixSession.token && zabbixSession.expiresAt > Date.now()) {
    return { useBearer: false, token: zabbixSession.token };
  }
  const token = await zabbixLogin();
  return { useBearer: false, token };
}

async function zabbixRequest(method, params) {
  if (!isZabbixEnabled()) return null;
  const auth = await getZabbixAuth();
  const payloadObj = {
    jsonrpc: '2.0',
    method,
    params,
    id: Date.now()
  };
  if (!auth.useBearer && auth.token) payloadObj.auth = auth.token;
  const payload = JSON.stringify(payloadObj);
  const headers = auth.useBearer && auth.token ? { Authorization: `Bearer ${auth.token}` } : {};

  try {
    return await zabbixHttpRequest(payload, headers);
  } catch (e) {
    const shouldRetry = !auth.useBearer && e && e.message === 'Not authorized.';
    if (shouldRetry && zabbixConfig.authUser && zabbixConfig.authPass) {
      zabbixSession = { token: '', expiresAt: 0 };
      const refreshed = await getZabbixAuth();
      if (!refreshed.useBearer && refreshed.token) payloadObj.auth = refreshed.token;
      const retryPayload = JSON.stringify(payloadObj);
      const retryHeaders = refreshed.useBearer && refreshed.token ? { Authorization: `Bearer ${refreshed.token}` } : {};
      return await zabbixHttpRequest(retryPayload, retryHeaders);
    }
    e.method = method;
    throw e;
  }
}

async function ensureZabbixHostAndItems(device) {
  if (!isZabbixEnabled() || !device || !device.name) return null;
  if (device.zabbixEnsuring) return null;
  device.zabbixEnsuring = true;
  try {
    const macSuffix = (device.mac || '').replace(/:/g, '').toLowerCase();
    const hostName = macSuffix ? `${device.name}-${macSuffix}` : device.name;
    const groupResult = await zabbixRequest('hostgroup.get', {
      output: ['groupid', 'name'],
      filter: { name: [zabbixConfig.hostGroup] }
    });
    let groupId = groupResult && groupResult.length ? groupResult[0].groupid : null;
    if (!groupId) {
      const created = await zabbixRequest('hostgroup.create', { name: zabbixConfig.hostGroup });
      groupId = created && created.groupids ? created.groupids[0] : null;
    }
    if (!groupId) throw new Error('zabbix_group_failed');

    const templateResult = await zabbixRequest('template.get', {
      output: ['templateid', 'host'],
      filter: { host: [zabbixConfig.templateName] }
    });
    const templateId = templateResult && templateResult.length ? templateResult[0].templateid : null;
    if (!templateId) throw new Error('zabbix_template_missing');

    const hostResult = await zabbixRequest('host.get', {
      output: ['hostid', 'host'],
      selectParentTemplates: ['templateid', 'name'],
      selectInterfaces: ['interfaceid', 'ip', 'useip', 'dns', 'port'],
      filter: { host: [hostName] }
    });
    let hostId = hostResult && hostResult.length ? hostResult[0].hostid : null;
    const parentTemplates = hostResult && hostResult.length ? hostResult[0].parentTemplates || [] : [];
    const interfaces = hostResult && hostResult.length ? hostResult[0].interfaces || [] : [];
    const hasTemplate = parentTemplates.some((tpl) => tpl.templateid === templateId);
    if (!hostId) {
      const ip = device.ip || '127.0.0.1';
      const createdHost = await zabbixRequest('host.create', {
        host: hostName,
        groups: [{ groupid: groupId }],
        templates: [{ templateid: templateId }],
        interfaces: [{ type: 1, main: 1, useip: 1, ip, dns: '', port: '10050' }]
      });
      hostId = createdHost && createdHost.hostids ? createdHost.hostids[0] : null;
    } else if (!hasTemplate) {
      const templates = parentTemplates.map((tpl) => ({ templateid: tpl.templateid }));
      templates.push({ templateid: templateId });
      await zabbixRequest('host.update', { hostid: hostId, templates });
    }

    if (hostId && device.ip && interfaces.length) {
      const mainInterface = interfaces.find((iface) => iface.useip === '1' || iface.useip === 1) || interfaces[0];
      if (mainInterface && mainInterface.ip && mainInterface.ip !== device.ip) {
        await zabbixRequest('hostinterface.update', {
          interfaceid: mainInterface.interfaceid,
          ip: device.ip,
          useip: 1,
          dns: ''
        });
      }
    }
    if (!hostId) throw new Error('zabbix_host_failed');

    device.zabbix = {
      hostid: hostId,
      host: hostName,
      templateid: templateId
    };
    device.zabbixEnsured = true;
    scheduleSaveDevices();
    return device.zabbix;
  } finally {
    device.zabbixEnsuring = false;
  }
}

async function pushZabbixValues(device, values) {
  if (!isZabbixEnabled() || !device || !device.zabbix) return;
  const entries = [];
  const clock = Math.floor(Date.now() / 1000);
  const useSender = zabbixHistoryPushUnsupported || !device.zabbix.items;
  if (useSender) {
    const senderEntries = [];
    const host = (device.zabbix && device.zabbix.host) || device.name || device.mac;
    if (values.uptime !== undefined) senderEntries.push({ host, key: ZABBIX_ITEM_KEYS.uptime, value: Number(values.uptime), clock });
    if (values.version !== undefined) senderEntries.push({ host, key: ZABBIX_ITEM_KEYS.version, value: String(values.version), clock });
    if (values.temperature !== undefined) senderEntries.push({ host, key: ZABBIX_ITEM_KEYS.temperature, value: Number(values.temperature), clock });
    if (senderEntries.length) await zabbixSenderRequest(senderEntries);
    return;
  }

  if (values.uptime !== undefined && device.zabbix.items.uptime) {
    entries.push({ itemid: device.zabbix.items.uptime, value: Number(values.uptime), clock });
  }
  if (values.version !== undefined && device.zabbix.items.version) {
    entries.push({ itemid: device.zabbix.items.version, value: String(values.version), clock });
  }
  if (values.temperature !== undefined && device.zabbix.items.temperature) {
    entries.push({ itemid: device.zabbix.items.temperature, value: Number(values.temperature), clock });
  }
  if (!entries.length) return;

  try {
    await zabbixRequest('history.push', entries);
  } catch (e) {
    if (e && e.message === 'Invalid params.') {
      await zabbixRequest('history.push', { data: entries });
      return;
    }
    if (e && e.message === 'Method not found.') {
      zabbixHistoryPushUnsupported = true;
      const senderEntries = [];
      const host = (device.zabbix && device.zabbix.host) || device.name || device.mac;
      if (values.uptime !== undefined) senderEntries.push({ host, key: ZABBIX_ITEM_KEYS.uptime, value: Number(values.uptime), clock });
      if (values.version !== undefined) senderEntries.push({ host, key: ZABBIX_ITEM_KEYS.version, value: String(values.version), clock });
      if (values.temperature !== undefined) senderEntries.push({ host, key: ZABBIX_ITEM_KEYS.temperature, value: Number(values.temperature), clock });
      if (senderEntries.length) await zabbixSenderRequest(senderEntries);
      return;
    }
    throw e;
  }
}

function queueZabbixUpdate(device, values) {
  if (!device || !device.adopted) return;
  if (!isZabbixEnabled()) return;
  setImmediate(async () => {
    try {
      if (!device.zabbixEnsured) await ensureZabbixHostAndItems(device);
      await pushZabbixValues(device, values);
    } catch (e) {
      const info = [];
      if (e && e.method) info.push(`method=${e.method}`);
      if (e && e.code !== undefined) info.push(`code=${e.code}`);
      if (e && e.data) info.push(`data=${e.data}`);
      console.warn('Zabbix update failed', e && e.message ? e.message : 'unknown_error', info.join(' '));
    }
  });
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
loadZabbixConfig();

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
      queueZabbixUpdate(devices[key], {
        uptime: devices[key].uptime,
        version: devices[key].version
      });
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
      queueZabbixUpdate(devices[key], {
        temperature: devices[key].temperature,
        uptime: devices[key].uptime,
        version: devices[key].version
      });
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

// Zabbix config APIs
app.get('/api/zabbix/config', verifyTokenMiddleware, (req, res) => {
  res.json(getZabbixPublicConfig());
});

app.post('/api/zabbix/config', verifyTokenMiddleware, (req, res) => {
  const body = req.body || {};
  if (typeof body.enabled === 'boolean') zabbixConfig.enabled = body.enabled;
  if (typeof body.apiUrl === 'string' && body.apiUrl.trim().length) zabbixConfig.apiUrl = body.apiUrl.trim();
  if (typeof body.hostGroup === 'string' && body.hostGroup.trim().length) zabbixConfig.hostGroup = body.hostGroup.trim();
  saveZabbixConfig();
  res.json(getZabbixPublicConfig());
});

app.get('/api/zabbix/diagnose', verifyTokenMiddleware, async (req, res) => {
  if (!isZabbixEnabled()) return res.status(400).json({ error: 'zabbix_disabled' });

  const result = {
    ok: false,
    apiUrl: zabbixConfig.apiUrl,
    hostGroup: zabbixConfig.hostGroup,
    checks: {}
  };

  try {
    result.checks.apiVersion = await zabbixRequest('apiinfo.version', {});
  } catch (e) {
    result.checks.apiVersion = { error: e.message || 'failed', code: e.code, data: e.data };
    return res.status(500).json(result);
  }

  try {
    const groups = await zabbixRequest('hostgroup.get', {
      output: ['groupid', 'name'],
      filter: { name: [zabbixConfig.hostGroup] }
    });
    result.checks.hostGroup = groups;
    result.ok = true;
    return res.json(result);
  } catch (e) {
    result.checks.hostGroup = { error: e.message || 'failed', code: e.code, data: e.data };
    return res.status(500).json(result);
  }
});

app.get('/api/zabbix/diagnose-public', async (req, res) => {
  if (!isZabbixEnabled()) return res.status(400).json({ error: 'zabbix_disabled' });

  const result = {
    ok: false,
    apiUrl: zabbixConfig.apiUrl,
    hostGroup: zabbixConfig.hostGroup,
    checks: {}
  };

  try {
    result.checks.apiVersion = await zabbixRequest('apiinfo.version', {});
  } catch (e) {
    result.checks.apiVersion = { error: e.message || 'failed', code: e.code, data: e.data };
    return res.status(500).json(result);
  }

  try {
    const groups = await zabbixRequest('hostgroup.get', {
      output: ['groupid', 'name'],
      filter: { name: [zabbixConfig.hostGroup] }
    });
    result.checks.hostGroup = groups;
    result.ok = true;
    return res.json(result);
  } catch (e) {
    result.checks.hostGroup = { error: e.message || 'failed', code: e.code, data: e.data };
    return res.status(500).json(result);
  }
});

app.get('/api/zabbix/auth-status', verifyTokenMiddleware, (req, res) => {
  const hasToken = !!zabbixConfig.apiToken;
  const hasUserPass = !!(zabbixConfig.authUser && zabbixConfig.authPass);
  const authMode = hasToken ? 'token' : (hasUserPass ? 'userpass' : 'none');
  res.json({
    enabled: !!zabbixConfig.enabled,
    apiUrl: zabbixConfig.apiUrl,
    hostGroup: zabbixConfig.hostGroup,
    authMode,
    hasToken,
    hasUserPass,
    sessionActive: !!(zabbixSession.token && zabbixSession.expiresAt > Date.now())
  });
});

app.get('/api/zabbix/auth-status-public', (req, res) => {
  const hasToken = !!zabbixConfig.apiToken;
  const hasUserPass = !!(zabbixConfig.authUser && zabbixConfig.authPass);
  const authMode = hasToken ? 'token' : (hasUserPass ? 'userpass' : 'none');
  res.json({
    enabled: !!zabbixConfig.enabled,
    apiUrl: zabbixConfig.apiUrl,
    hostGroup: zabbixConfig.hostGroup,
    authMode,
    hasToken,
    hasUserPass,
    sessionActive: !!(zabbixSession.token && zabbixSession.expiresAt > Date.now())
  });
});

app.post('/api/devices/:mac/zabbix/sync', verifyTokenMiddleware, async (req, res) => {
  const mac = req.params.mac.replace(/:/g, '').toLowerCase();
  const device = devices[mac];
  if (!device) return res.status(404).json({ error: 'Device not found' });
  if (!isZabbixEnabled()) return res.status(400).json({ error: 'zabbix_disabled' });

  try {
    await ensureZabbixHostAndItems(device);
    await pushZabbixValues(device, {
      uptime: device.uptime,
      version: device.version,
      temperature: device.temperature
    });
    res.json({ ok: true, zabbix: device.zabbix || null });
  } catch (e) {
    const info = [];
    if (e && e.method) info.push(`method=${e.method}`);
    if (e && e.code !== undefined) info.push(`code=${e.code}`);
    if (e && e.data) info.push(`data=${e.data}`);
    console.warn('Zabbix sync failed', e && e.message ? e.message : 'unknown_error', info.join(' '));
    res.status(500).json({ error: 'zabbix_sync_failed' });
  }
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

  if (isZabbixEnabled()) {
    try {
      await ensureZabbixHostAndItems(device);
    } catch (e) {
      console.warn('Zabbix ensure failed on adopt', e.message);
    }
  }

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

  if (isZabbixEnabled()) {
    setImmediate(async () => {
      try {
        let hostId = device.zabbix && device.zabbix.hostid ? device.zabbix.hostid : null;
        if (!hostId && device.zabbix && device.zabbix.host) {
          const hostResult = await zabbixRequest('host.get', {
            output: ['hostid', 'host'],
            filter: { host: [device.zabbix.host] }
          });
          if (hostResult && hostResult.length) hostId = hostResult[0].hostid;
        }
        if (hostId) await zabbixRequest('host.delete', [hostId]);
      } catch (e) {
        const info = [];
        if (e && e.method) info.push(`method=${e.method}`);
        if (e && e.code !== undefined) info.push(`code=${e.code}`);
        if (e && e.data) info.push(`data=${e.data}`);
        console.warn('Zabbix delete failed', e && e.message ? e.message : 'unknown_error', info.join(' '));
      }
    });
  }

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
