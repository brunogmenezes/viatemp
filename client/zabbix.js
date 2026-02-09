async function loadZabbixConfig() {
  try {
    const res = await fetch('/api/zabbix/config', { headers: getAuthHeaders() });
    if (!res.ok) throw new Error('config_failed');
    const data = await res.json();
    const apiInput = document.getElementById('zabbix-api');
    const groupInput = document.getElementById('zabbix-group');
    const enabledInput = document.getElementById('zabbix-enabled');
    if (apiInput) apiInput.value = data.apiUrl || '';
    if (groupInput) groupInput.value = data.hostGroup || '';
    if (enabledInput) enabledInput.checked = !!data.enabled;

    const status = document.getElementById('zabbix-status');
    if (status) {
      const tokenState = data.tokenConfigured ? 'Token configurado' : 'Token ausente';
      status.innerHTML = `
        <div><strong>Status:</strong> ${data.enabled ? 'Ativo' : 'Inativo'}</div>
        <div><strong>Token:</strong> ${tokenState}</div>
        <div><strong>API URL:</strong> ${data.apiUrl || '—'}</div>
        <div><strong>Host group:</strong> ${data.hostGroup || '—'}</div>
      `;
    }
  } catch (e) {
    showToast('Erro ao carregar configuracao do Zabbix', { type: 'error', title: 'Zabbix' });
  }
}

const form = document.getElementById('zabbix-form');
if (form) {
  form.addEventListener('submit', async (e) => {
    e.preventDefault();
    const apiUrl = document.getElementById('zabbix-api').value.trim();
    const hostGroup = document.getElementById('zabbix-group').value.trim();
    const enabled = document.getElementById('zabbix-enabled').checked;
    if (!apiUrl || !hostGroup) {
      showToast('Informe a URL e o host group', { type: 'error', title: 'Zabbix' });
      return;
    }
    try {
      const res = await fetch('/api/zabbix/config', {
        method: 'POST',
        headers: getAuthHeaders(),
        body: JSON.stringify({ apiUrl, hostGroup, enabled })
      });
      if (!res.ok) throw new Error('save_failed');
      showToast('Configuracao salva', { type: 'success', title: 'Zabbix' });
      loadZabbixConfig();
    } catch (e) {
      showToast('Erro ao salvar configuracao', { type: 'error', title: 'Zabbix' });
    }
  });
}

if (document.getElementById('zabbix-status')) {
  loadZabbixConfig();
}
