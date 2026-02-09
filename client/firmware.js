async function fetchFirmwareList() {
  const res = await fetch('/api/firmware', { headers: getAuthHeaders() });
  if (!res.ok) return [];
  const data = await res.json();
  return Array.isArray(data) ? data : [];
}

async function fetchLatestFirmware() {
  const res = await fetch('/api/firmware/latest', { headers: getAuthHeaders() });
  if (!res.ok) return null;
  return await res.json();
}

function renderLatest(latest) {
  const container = document.getElementById('firmware-latest');
  if (!container) return;
  if (!latest) {
    container.className = 'firmware-empty';
    container.textContent = 'Nenhum firmware enviado.';
    return;
  }
  container.className = 'firmware-latest-card';
  container.innerHTML = `
    <div class="firmware-meta">
      <div><strong>Versao:</strong> ${latest.version}</div>
      <div><strong>Arquivo:</strong> ${latest.originalName || latest.filename}</div>
      <div><strong>Enviado em:</strong> ${new Date(latest.uploadedAt).toLocaleString()}</div>
      <div><strong>Tamanho:</strong> ${(latest.size / 1024).toFixed(1)} KB</div>
    </div>
    <a class="btn btn-ghost" href="/firmware/${latest.filename}" target="_blank" rel="noreferrer">Baixar</a>
  `;
}

function renderList(list) {
  const container = document.getElementById('firmware-list');
  if (!container) return;
  if (!list.length) {
    container.innerHTML = '<div class="empty">Nenhum firmware enviado.</div>';
    return;
  }
  container.innerHTML = list.map(item => {
    const dt = item.uploadedAt ? new Date(item.uploadedAt).toLocaleString() : '—';
    const size = item.size ? `${(item.size / 1024).toFixed(1)} KB` : '—';
    return `
      <div class="firmware-item">
        <div class="firmware-info">
          <div class="firmware-version">v${item.version}</div>
          <div class="firmware-name">${item.originalName || item.filename}</div>
          <div class="firmware-meta-row">${dt} • ${size}</div>
        </div>
        <a class="btn btn-ghost" href="/firmware/${item.filename}" target="_blank" rel="noreferrer">Baixar</a>
      </div>
    `;
  }).join('');
}

async function loadFirmware() {
  try {
    const [list, latest] = await Promise.all([fetchFirmwareList(), fetchLatestFirmware()]);
    const sorted = list.slice().sort((a, b) => {
      const ta = a.uploadedAt ? new Date(a.uploadedAt).getTime() : 0;
      const tb = b.uploadedAt ? new Date(b.uploadedAt).getTime() : 0;
      return tb - ta;
    });
    renderLatest(latest);
    renderList(sorted);
  } catch (e) {
    showToast('Erro ao carregar firmwares', { type: 'error', title: 'Firmware' });
  }
}

const form = document.getElementById('firmware-form');
if (form) {
  form.addEventListener('submit', async (e) => {
    e.preventDefault();
    const versionInput = document.getElementById('firmware-version');
    const fileInput = document.getElementById('firmware-file');
    const version = versionInput ? versionInput.value.trim() : '';
    const file = fileInput && fileInput.files ? fileInput.files[0] : null;
    if (!version || !file) {
      showToast('Informe a versao e selecione o arquivo .bin', { type: 'error', title: 'Firmware' });
      return;
    }
    const data = new FormData();
    data.append('version', version);
    data.append('firmware', file);

    try {
      const res = await fetch('/api/firmware/upload', {
        method: 'POST',
        headers: getAuthHeaders({ includeJson: false }),
        body: data
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        const msg = body && body.error ? body.error : 'Falha ao enviar firmware';
        showToast(msg, { type: 'error', title: 'Firmware' });
        return;
      }
      showToast('Firmware enviado com sucesso', { type: 'success', title: 'Firmware' });
      if (versionInput) versionInput.value = '';
      if (fileInput) fileInput.value = '';
      loadFirmware();
    } catch (err) {
      showToast('Erro ao enviar firmware', { type: 'error', title: 'Firmware' });
    }
  });
}

if (document.getElementById('firmware-list')) {
  loadFirmware();
}
