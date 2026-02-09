// Fetch devices from backend
async function fetchDevices() {
  const res = await fetch('/api/devices', { headers: getAuthHeaders() });
  return await res.json();
}

function getToken(){
  return localStorage.getItem('viatemp_token');
}

function getAuthHeaders(opts = {}){
  const token = getToken();
  const includeJson = opts.includeJson !== false;
  const h = {};
  if (includeJson) h['Content-Type'] = 'application/json';
  if (token) h['Authorization'] = 'Bearer ' + token;
  return h;
}

function showToast(message, opts = {}){
  const type = opts.type || 'error';
  const title = opts.title || (type === 'success' ? 'Sucesso' : 'Erro');
  const timeout = typeof opts.timeout === 'number' ? opts.timeout : 3500;
  let container = document.querySelector('.toast-container');
  if (!container) {
    container = document.createElement('div');
    container.className = 'toast-container';
    document.body.appendChild(container);
  }
  const t = document.createElement('div');
  t.className = 'toast ' + (type === 'success' ? 'success' : 'error');
  t.innerHTML = `<div class="t-body"><div class="t-title">${title}</div><div class="t-text">${message}</div></div><button class="t-close">✕</button>`;
  const closeBtn = t.querySelector('.t-close');
  closeBtn.addEventListener('click', () => t.remove());
  container.appendChild(t);
  if (timeout > 0) setTimeout(() => t.remove(), timeout);
}

function confirmAction(message, opts = {}){
  return new Promise((resolve) => {
    const modal = document.getElementById('confirm-modal');
    const titleEl = document.getElementById('confirm-title');
    const messageEl = document.getElementById('confirm-message');
    const btnOk = document.getElementById('confirm-ok');
    const btnCancel = document.getElementById('confirm-cancel');
    const btnClose = document.getElementById('confirm-close');
    if (!modal || !btnOk || !btnCancel) return resolve(false);
    titleEl.textContent = opts.title || 'Confirmar acao';
    messageEl.textContent = message;
    if (opts.okText) btnOk.textContent = opts.okText;
    if (opts.cancelText) btnCancel.textContent = opts.cancelText;
    const close = (value) => {
      modal.setAttribute('aria-hidden','true');
      modal.classList.remove('open');
      btnOk.onclick = null;
      btnCancel.onclick = null;
      if (btnClose) btnClose.onclick = null;
      resolve(value);
    };
    btnOk.onclick = () => close(true);
    btnCancel.onclick = () => close(false);
    if (btnClose) btnClose.onclick = () => close(false);
    modal.setAttribute('aria-hidden','false');
    modal.classList.add('open');
  });
}

// Group devices by location
function groupByLocation(devices) {
  const map = {};
  devices.forEach(d => {
    const loc = (d.location && d.location.trim().length) ? d.location : 'Sem Local';
    map[loc] = map[loc] || [];
    map[loc].push(d);
  });
  return map;
}

// Create card element for a device
const HEARTBEAT_TTL_MS = 120000;
const FIRMWARE_CACHE_MS = 60000;
let latestFirmware = null;
let latestFirmwareLoadedAt = 0;

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

async function refreshFirmwareLatest(force) {
  const now = Date.now();
  if (!force && latestFirmwareLoadedAt && (now - latestFirmwareLoadedAt) < FIRMWARE_CACHE_MS) return latestFirmware;
  try {
    const res = await fetch('/api/firmware/latest', { headers: getAuthHeaders() });
    if (!res.ok) {
      latestFirmware = null;
      latestFirmwareLoadedAt = now;
      return null;
    }
    latestFirmware = await res.json();
    latestFirmwareLoadedAt = now;
    return latestFirmware;
  } catch (e) {
    latestFirmware = null;
    latestFirmwareLoadedAt = now;
    return null;
  }
}

function isUpdateAvailable(deviceVersion) {
  if (!latestFirmware || !latestFirmware.version) return false;
  if (!deviceVersion) return false;
  return compareVersions(deviceVersion, latestFirmware.version) < 0;
}

function formatUptime(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return 'Uptime --';
  const days = Math.floor(seconds / 86400);
  seconds %= 86400;
  const hours = Math.floor(seconds / 3600);
  seconds %= 3600;
  const minutes = Math.floor(seconds / 60);
  seconds %= 60;
  const parts = [];
  if (days > 0) parts.push(days + 'd');
  if (hours > 0 || days > 0) parts.push(hours + 'h');
  if (minutes > 0 || hours > 0 || days > 0) parts.push(minutes + 'm');
  parts.push(seconds + 's');
  return 'Uptime ' + parts.join(' ');
}

function createDeviceElement(d) {
  const tpl = document.getElementById('device-template');
  if (!tpl) return null;
  const el = tpl.content.cloneNode(true);
  el.querySelector('.name').textContent = d.name || d.mac;
  const status = el.querySelector('.status');
  status.textContent = d.adopted ? 'Adotado' : 'Disponivel';
  status.classList.add(d.adopted ? 'status-adopted' : 'status-available');
  const chipLocation = el.querySelector('.chip-location');
  chipLocation.textContent = d.location && d.location.trim().length ? d.location : 'Sem local';
  const chipTemp = el.querySelector('.chip-temp');
  if (chipTemp) {
    const temp = typeof d.temperature === 'number' ? d.temperature :
      (typeof d.temperature === 'string' ? parseFloat(d.temperature) : null);
    chipTemp.textContent = Number.isFinite(temp) ? `Temp ${temp.toFixed(1)}°C` : 'Temp —';
  }
  const chipIp = el.querySelector('.chip-ip');
  chipIp.textContent = d.ip ? `IP ${d.ip}` : 'IP desconhecido';
  const chipUptime = el.querySelector('.chip-uptime');
  if (chipUptime) {
    chipUptime.textContent = formatUptime(Number(d.uptime));
  }
  const chipLife = el.querySelector('.chip-life');
  const lastSeen = typeof d.lastSeen === 'number' ? d.lastSeen : 0;
  const online = typeof d.online === 'boolean' ? d.online : (lastSeen > 0 && (Date.now() - lastSeen) <= HEARTBEAT_TTL_MS);
  chipLife.textContent = online ? 'Online' : 'Offline';
  chipLife.classList.add(online ? 'chip-online' : 'chip-offline');
  el.querySelector('.meta').textContent = `MAC ${d.mac}${d.version ? ' • v' + d.version : ''}`;
  const btnAdopt = el.querySelector('.adopt');
  const btnDetails = el.querySelector('.details');
  const btnRemove = el.querySelector('.remove');
  const btnRestart = el.querySelector('.restart');
  const btnUpdate = el.querySelector('.update');
  if (d.adopted) {
    btnAdopt.textContent = 'Adotado';
    btnAdopt.disabled = true;
    btnAdopt.classList.add('disabled');
  }
  if (btnRestart && (!d.adopted || !online)) {
    btnRestart.remove();
  }
  const updateAvailable = isUpdateAvailable(d.version);
  if (btnUpdate && (!updateAvailable || !online)) {
    btnUpdate.remove();
  }
  btnAdopt.addEventListener('click', () => openDeviceModal(d, 'adopt'));
  btnDetails.addEventListener('click', () => openDeviceModal(d, 'details'));
  if (btnRestart && d.adopted && online) btnRestart.addEventListener('click', async () => {
    if (!online) {
      showToast('Sensor offline, nao foi possivel reiniciar', { type: 'error', title: 'Reinicio' });
      return;
    }
    const ok = await confirmAction('Reiniciar este sensor agora?', {
      title: 'Reiniciar sensor',
      okText: 'Reiniciar',
      cancelText: 'Cancelar'
    });
    if (!ok) return;
    const mac = d.mac.replace(/:/g, '').toLowerCase();
    const res = await fetch(`/api/devices/${mac}/restart`, {
      method: 'POST',
      headers: getAuthHeaders()
    });
    if (res.ok) {
      showToast('Comando de reinicio enviado', { type: 'success', title: 'Reinicio' });
      return;
    }
    if (res.status === 401) {
      showToast('Sessao expirada. Faca login novamente.', { type: 'error', title: 'Reinicio' });
      setTimeout(() => { window.location.href = '/login.html'; }, 800);
      return;
    }
    let msg = 'Falha ao enviar comando de reinicio';
    try {
      const body = await res.json();
      if (body && body.error) msg = `Falha ao enviar comando de reinicio (${body.error})`;
    } catch (e) {
      // ignore parse errors
    }
    showToast(msg, { type: 'error', title: 'Reinicio' });
  });
  if (btnUpdate && updateAvailable && online) btnUpdate.addEventListener('click', async () => {
    const ok = await confirmAction(`Atualizar firmware para v${latestFirmware.version}?`, {
      title: 'Atualizar firmware',
      okText: 'Atualizar',
      cancelText: 'Cancelar'
    });
    if (!ok) return;
    const mac = d.mac.replace(/:/g, '').toLowerCase();
    const res = await fetch(`/api/devices/${mac}/update`, {
      method: 'POST',
      headers: getAuthHeaders(),
      body: JSON.stringify({ version: latestFirmware.version })
    });
    if (res.ok) {
      showToast('Comando de atualizacao enviado', { type: 'success', title: 'Firmware' });
      return;
    }
    if (res.status === 401) {
      showToast('Sessao expirada. Faca login novamente.', { type: 'error', title: 'Firmware' });
      setTimeout(() => { window.location.href = '/login.html'; }, 800);
      return;
    }
    let msg = 'Falha ao enviar comando de atualizacao';
    try {
      const body = await res.json();
      if (body && body.error) msg = `Falha ao enviar comando de atualizacao (${body.error})`;
    } catch (e) {
      // ignore parse errors
    }
    showToast(msg, { type: 'error', title: 'Firmware' });
  });
  btnRemove.addEventListener('click', async () => {
    const ok = await confirmAction('Remover adoção deste dispositivo? Isso fará com que ele volte para não adotados.', {
      title: 'Remover adocao',
      okText: 'Remover',
      cancelText: 'Cancelar'
    });
    if (!ok) return;
    const mac = d.mac.replace(/:/g,'').toLowerCase();
    const res = await fetch(`/api/devices/${mac}`, { method: 'DELETE', headers: getAuthHeaders() });
    if (res.ok) {
      d.adopted = false;
      d.name = '';
      d.location = '';
      loadAndRender();
      showToast('Adoção removida', { type: 'success', title: 'Remoção' });
    } else {
      showToast('Erro ao remover adoção', { type: 'error', title: 'Remoção' });
    }
  });
  return el;
}

function renderKpis(devices) {
  const visible = devices.filter(d => d.adopted || d.online);
  const total = visible.length;
  const adopted = visible.filter(d => d.adopted).length;
  const available = visible.filter(d => !d.adopted).length;
  const noLoc = visible.filter(d => !d.location || !d.location.trim().length).length;
  const offline = visible.filter(d => d.adopted && !d.online).length;
  const t = document.getElementById('kpi-total');
  const a = document.getElementById('kpi-adopted');
  const v = document.getElementById('kpi-available');
  const n = document.getElementById('kpi-noloc');
  const o = document.getElementById('kpi-offline');
  if (t) t.textContent = total;
  if (a) a.textContent = adopted;
  if (v) v.textContent = available;
  if (n) n.textContent = noLoc;
  if (o) o.textContent = offline;
}

// Render sidebar with locations as a simple tree
function renderSidebar(groups, onlineDevices, allDevices) {
  const ul = document.getElementById('locations');
  ul.innerHTML = '';
  const allLi = document.createElement('li');
  allLi.className = 'location-item';
  const visibleAll = Array.isArray(allDevices) ? allDevices.filter(d => d.adopted || d.online) : [];
  const allCount = visibleAll.length;
  allLi.textContent = `Todos (${allCount})`;
  allLi.addEventListener('click', () => {
    setActiveFilter({ type: 'all' });
    renderAllGrouped(visibleAll || []);
  });
  ul.appendChild(allLi);

  Object.keys(groups).sort().forEach(loc => {
    const li = document.createElement('li');
    li.className = 'location-item';
    const count = groups[loc].length;
    li.innerHTML = `<span class="loc-name">${loc}</span> <span class="count">${count}</span>`;
    li.addEventListener('click', (e) => {
      const wasActive = li.classList.contains('active');
      Array.from(document.querySelectorAll('.location-item.active')).forEach(n => n.classList.remove('active'));
      if (!wasActive) {
        li.classList.add('active');
        setActiveFilter({ type: 'location', value: loc });
        renderMain(groups[loc]);
      } else {
        li.classList.remove('active');
        setActiveFilter(null);
        renderMain([]);
      }
    });
    ul.appendChild(li);
  });

  // Keep active class in sync after rerender
  const active = getActiveFilter();
  if (active && active.type === 'location') {
    const items = Array.from(ul.querySelectorAll('.location-item'));
    items.forEach(item => {
      const nameEl = item.querySelector('.loc-name');
      if (nameEl && nameEl.textContent === active.value) item.classList.add('active');
    });
  }
}

// Render main grid with device cards
function renderMain(devices) {
  const content = document.getElementById('content');
  if (!content) return;
  content.innerHTML = '';
  if (!devices || devices.length === 0) {
    content.innerHTML = '<div class="empty">Nenhum sensor encontrado nesta seleção.</div>';
    return;
  }
  devices.forEach(d => {
    const el = createDeviceElement(d);
    if (el) content.appendChild(el);
  });
}

function renderSection(content, title, devices) {
  if (!content) return;
  const section = document.createElement('div');
  section.className = 'group-section';
  const header = document.createElement('div');
  header.className = 'group-title';
  header.textContent = `${title} (${devices.length})`;
  section.appendChild(header);
  const list = document.createElement('div');
  list.className = 'group-list';
  if (!devices.length) {
    const empty = document.createElement('div');
    empty.className = 'group-empty';
    empty.textContent = 'Nenhum sensor nesta categoria.';
    list.appendChild(empty);
  } else {
    devices.forEach(d => {
      const el = createDeviceElement(d);
      if (el) list.appendChild(el);
    });
  }
  section.appendChild(list);
  content.appendChild(section);
}

function renderAllGrouped(devices) {
  const content = document.getElementById('content');
  if (!content) return;
  content.innerHTML = '';
  if (!devices || devices.length === 0) {
    content.innerHTML = '<div class="empty">Nenhum sensor encontrado nesta seleção.</div>';
    return;
  }
  const unadopted = devices.filter(d => !d.adopted && d.online);
  const online = devices.filter(d => d.adopted && d.online);
  const offline = devices.filter(d => d.adopted && !d.online);
  renderSection(content, 'Nao adotados', unadopted);
  renderSection(content, 'Online', online);
  renderSection(content, 'Offline', offline);
}

// Load devices and render both sidebar and main
async function loadAndRender() {
  await refreshFirmwareLatest();
  const devices = await fetchDevices();
  const onlineDevices = devices.filter(d => d.online);
  const groupedDevices = devices.filter(d => d.adopted || d.online);
  renderKpis(devices);
  const groups = groupByLocation(groupedDevices);
  renderSidebar(groups, onlineDevices, devices);
  const active = getActiveFilter();
  if (active && active.type === 'location') {
    renderMain(groups[active.value] || []);
    return;
  }
  if (active && active.type === 'all') {
    renderAllGrouped(groupedDevices);
    return;
  }
  if (active && active.type === 'unadopted') {
    const onlineUnadopted = onlineDevices.filter(d => !d.adopted);
    if (onlineUnadopted.length) renderMain(onlineUnadopted);
    else renderMain([]);
    return;
  }
  renderAllGrouped(groupedDevices);
}

// Event listeners
const filterAll = document.getElementById('filter-all');
if (filterAll) filterAll.addEventListener('click', async () => { const devices = await fetchDevices(); const visible = devices.filter(d => d.adopted || d.online); setActiveFilter({ type: 'all' }); renderAllGrouped(visible); });
const filterUnadopted = document.getElementById('filter-unadopted');
if (filterUnadopted) filterUnadopted.addEventListener('click', async () => { const devices = await fetchDevices(); setActiveFilter({ type: 'unadopted' }); renderMain(devices.filter(d => !d.adopted && d.online)); });
const searchInput = document.getElementById('search');
if (searchInput) searchInput.addEventListener('input', async (e) => {
  const q = e.target.value.toLowerCase().trim();
  const devices = await fetchDevices();
  const visible = devices.filter(d => d.adopted || d.online);
  if (!q) {
    const onlineDevices = devices.filter(d => d.online);
    const groupedDevices = visible;
    const groups = groupByLocation(groupedDevices);
    const active = getActiveFilter();
    if (active && active.type === 'location') {
      renderMain(groups[active.value] || []);
      return;
    }
    if (active && active.type === 'all') {
      renderAllGrouped(groupedDevices);
      return;
    }
    if (active && active.type === 'unadopted') {
      renderMain(onlineDevices.filter(d => !d.adopted));
      return;
    }
    renderAllGrouped(groupedDevices);
    return;
  }
  const filtered = visible.filter(d => ((d.name || '').toLowerCase().includes(q) || (d.location || '').toLowerCase().includes(q)));
  setActiveFilter(null);
  renderMain(filtered);
});

// Persist current sidebar selection to avoid auto-reset on refresh
let activeFilter = { type: 'all' };
function setActiveFilter(filter) {
  activeFilter = filter;
}
function getActiveFilter() {
  return activeFilter;
}

if (document.getElementById('content')) {
  loadAndRender();
  setInterval(loadAndRender, 30000);

  // Realtime updates via Socket.IO
  if (typeof io !== 'undefined') {
    const socket = io();
    socket.on('connect', () => console.log('Connected to realtime server'));
    socket.on('devices_updated', () => {
      loadAndRender();
    });
  }
}

// Modal implementation
const modal = document.getElementById('modal');
const modalBody = document.getElementById('modal-body');
const modalTitle = document.getElementById('modal-title');
const modalClose = document.getElementById('modal-close');
const modalAdopt = document.getElementById('modal-adopt');
const modalCancel = document.getElementById('modal-cancel');
let modalDevice = null;
let modalMode = 'details';

function openDeviceModal(d, mode) {
  if (!modal || !modalBody || !modalTitle) return;
  modalDevice = d;
  modalMode = mode || 'details';
  modalTitle.textContent = modalMode === 'details' ? 'Detalhes do sensor' : (d.name || d.mac);
  const statusText = d.adopted ? 'Adotado' : 'Disponivel';
  const locationText = d.location && d.location.trim().length ? d.location : 'Sem local';
  const formHtml = modalMode === 'adopt' ? `
      <div class="modal-form">
        <div class="field">
          <label>Nome</label>
          <input id="modal-name" type="text" value="${d.name || ''}" required />
          <div class="field-error" id="modal-name-error">Informe um nome.</div>
        </div>
        <div class="field">
          <label>Localidade</label>
          <select id="modal-location-select" required><option value="">Carregando...</option></select>
          <div class="field-error" id="modal-location-error">Selecione uma localidade.</div>
        </div>
      </div>
  ` : '';
  modalBody.innerHTML = `
    <div class="modal-grid ${modalMode === 'details' ? 'only-info' : ''}">
      <div class="modal-info">
        <div class="info-row"><span>Status</span><strong>${statusText}</strong></div>
        <div class="info-row"><span>MAC</span><strong>${d.mac}</strong></div>
        <div class="info-row"><span>IP</span><strong>${d.ip || '—'}</strong></div>
        <div class="info-row"><span>Local</span><strong>${locationText}</strong></div>
        <div class="info-row"><span>Versao</span><strong>${d.version || '—'}</strong></div>
      </div>
      ${formHtml}
    </div>
  `;
  modal.setAttribute('aria-hidden', 'false');
  modal.classList.add('open');
  if (modalAdopt) modalAdopt.classList.toggle('hidden', modalMode !== 'adopt');

  if (modalMode === 'adopt') {
    (async () => {
      try {
        const res = await fetch('/api/locations');
        if (!res.ok) throw new Error('Não foi possível carregar localidades');
        const locations = await res.json();
        const sel = document.getElementById('modal-location-select');
        sel.innerHTML = '<option value="">Selecione uma localidade</option>';
        locations.forEach(loc => {
          const opt = document.createElement('option');
          opt.value = loc.id;
          opt.textContent = loc.name;
          if (modalDevice.location_id && String(modalDevice.location_id) === String(loc.id)) opt.selected = true;
          else if (!modalDevice.location_id && modalDevice.location && modalDevice.location === loc.name) opt.selected = true;
          sel.appendChild(opt);
        });
      } catch (e) {
        const sel = document.getElementById('modal-location-select');
        if (sel) sel.innerHTML = '<option value="">Erro ao carregar localidades</option>';
      }
    })();
    const nameInput = document.getElementById('modal-name');
    const sel = document.getElementById('modal-location-select');
    const updateState = () => {
      const name = nameInput ? nameInput.value.trim() : '';
      const locationId = sel ? sel.value : '';
      const nameField = nameInput ? nameInput.closest('.field') : null;
      const locationField = sel ? sel.closest('.field') : null;
      const nameError = document.getElementById('modal-name-error');
      const locError = document.getElementById('modal-location-error');
      const nameOk = !!name;
      const locOk = !!locationId;
      if (nameField) nameField.classList.toggle('invalid', !nameOk);
      if (locationField) locationField.classList.toggle('invalid', !locOk);
      if (nameError) nameError.style.display = nameOk ? 'none' : 'block';
      if (locError) locError.style.display = locOk ? 'none' : 'block';
      if (modalAdopt) modalAdopt.disabled = !(nameOk && locOk);
    };
    if (nameInput) nameInput.addEventListener('input', updateState);
    if (sel) sel.addEventListener('change', updateState);
    updateState();
  }
}

function closeModal() { if (!modal) return; modalDevice = null; modal.setAttribute('aria-hidden','true'); modal.classList.remove('open'); }
if (modalClose) modalClose.addEventListener('click', closeModal);
if (modalCancel) modalCancel.addEventListener('click', closeModal);
if (modalAdopt) modalAdopt.addEventListener('click', async () => {
  if (!modalDevice) return;
  const mac = modalDevice.mac.replace(/:/g,'').toLowerCase();
  const nameInput = document.getElementById('modal-name');
  const sel = document.getElementById('modal-location-select');
  const name = nameInput ? nameInput.value.trim() : '';
  const locationId = sel ? sel.value : '';
  if (!name) {
    showToast('Informe um nome para o sensor', { type: 'error', title: 'Validacao' });
    return;
  }
  if (!locationId) {
    showToast('Selecione uma localidade', { type: 'error', title: 'Validacao' });
    return;
  }
  const body = { name, location_id: locationId };
  const res = await fetch(`/api/devices/${mac}/adopt`, { method: 'POST', headers: getAuthHeaders(), body: JSON.stringify(body) });
  if (res.ok) {
    closeModal();
    loadAndRender();
    showToast('Sensor adotado com sucesso', { type: 'success', title: 'Adoção' });
  } else {
    showToast('Erro ao adotar o sensor', { type: 'error', title: 'Adoção' });
  }
});

// Sidebar toggle
const toggleSidebar = document.getElementById('toggle-sidebar');
if (toggleSidebar) toggleSidebar.addEventListener('click', () => document.querySelector('.sidebar').classList.toggle('collapsed'));

// Auth: ensure logged in
let currentUser = null;

function updateUserUI(){
  const name = currentUser && currentUser.username ? currentUser.username : '—';
  const nameEl = document.getElementById('user-name');
  if (nameEl) nameEl.textContent = name;
  const avatarEl = document.getElementById('user-avatar');
  if (avatarEl) avatarEl.textContent = name && name !== '—' ? name.charAt(0).toUpperCase() : 'U';
  const profileName = document.getElementById('profile-name');
  if (profileName) profileName.textContent = name;
  const profileUser = document.getElementById('profile-username');
  if (profileUser) profileUser.textContent = name;
  const profileAvatar = document.getElementById('profile-avatar');
  if (profileAvatar) profileAvatar.textContent = name && name !== '—' ? name.charAt(0).toUpperCase() : 'U';
}

function initUserMenu(){
  const userMenu = document.getElementById('user-menu');
  const userTrigger = document.getElementById('user-trigger');
  const userDropdown = document.getElementById('user-dropdown');
  if (!userMenu || !userTrigger || !userDropdown) return;
  const closeMenu = () => userDropdown.classList.remove('open');
  userTrigger.addEventListener('click', (e) => {
    e.stopPropagation();
    userDropdown.classList.toggle('open');
  });
  document.addEventListener('click', (e) => {
    if (!userMenu.contains(e.target)) closeMenu();
  });
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') closeMenu();
  });

  const profileOpen = document.getElementById('profile-open');
  const profileModal = document.getElementById('profile-modal');
  const profileClose = document.getElementById('profile-close');
  const profileCloseBtn = document.getElementById('profile-close-btn');
  if (profileOpen && profileModal) {
    profileOpen.addEventListener('click', () => {
      closeMenu();
      profileModal.setAttribute('aria-hidden', 'false');
      profileModal.classList.add('open');
    });
  }
  const closeProfile = () => {
    if (!profileModal) return;
    profileModal.setAttribute('aria-hidden', 'true');
    profileModal.classList.remove('open');
  };
  if (profileClose) profileClose.addEventListener('click', closeProfile);
  if (profileCloseBtn) profileCloseBtn.addEventListener('click', closeProfile);
  if (profileModal) {
    const backdrop = profileModal.querySelector('.modal-backdrop');
    if (backdrop) backdrop.addEventListener('click', closeProfile);
  }
}

async function checkAuth(){
  const token = getToken();
  if (!token) { location.href = '/login.html'; return; }
  // verify and set username
  try {
    const r = await fetch('/api/auth/me', { headers: { 'Authorization': 'Bearer ' + token } });
    if (!r.ok) { localStorage.removeItem('viatemp_token'); location.href = '/login.html'; return; }
    const body = await r.json();
    currentUser = body;
    updateUserUI();
  } catch (e) {
    localStorage.removeItem('viatemp_token'); location.href = '/login.html';
  }
}

const logoutBtn = document.getElementById('logout');
if (logoutBtn) logoutBtn.addEventListener('click', ()=>{ localStorage.removeItem('viatemp_token'); location.href = '/login.html'; });

if (document.getElementById('user-name')) {
  checkAuth();
  initUserMenu();
}
