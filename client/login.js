// login.js - handles login flow and toasts
(function(){
  function $(id){return document.getElementById(id)}
  const loginBtn = $('login');
  const alertsContainer = $('login-alerts');

  function showToast(message, opts={type:'error',title:'' ,timeout:4000}){
    // create container if needed
    let container = document.querySelector('.toast-container');
    if (!container){ container = document.createElement('div'); container.className = 'toast-container'; document.body.appendChild(container); }
    const t = document.createElement('div');
    t.className = 'toast ' + (opts.type === 'success' ? 'success' : 'error');
    t.innerHTML = `<div class="t-body"><div class="t-title">${opts.title || (opts.type==='success'?'Sucesso':'Erro')}</div><div class="t-text">${message}</div></div><button class="t-close">✕</button>`;
    const closeBtn = t.querySelector('.t-close');
    closeBtn.addEventListener('click', ()=>{ t.remove(); });
    container.appendChild(t);
    if (opts.timeout && opts.timeout>0){ setTimeout(()=>{ t.remove(); }, opts.timeout); }
    return t;
  }

  async function doLogin(){
    const u = $('username').value.trim();
    const p = $('password').value;
    if (!u || !p){
      showInlineError('Preencha usuário e senha');
      return;
    }
    loginBtn.disabled = true;
    loginBtn.textContent = 'Entrando...';
    try {
      // 1. Tenta login local
      let res = await fetch('/api/auth/login',{
        method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({username:u,password:p})
      });
      if (res.ok){
        const body = await res.json();
        localStorage.setItem('viatemp_token', body.token);
        showToast('Você foi autenticado com sucesso',{type:'success',title:'Bem-vindo',timeout:1500});
        setTimeout(()=>{ location.href = '/'; }, 700);
        return;
      }
      // 2. Se falhar, tenta login externo Jupiter
      res = await fetch('https://api.jupiter.com.br/action/Usuario/logar', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify({ usuario: u, senha: p })
      });
      if (res.ok) {
        const body = await res.json();
        window._jupiterDebug = body; // Salva globalmente para inspeção
        // Ajuste automático para nomes de campos
        const accessToken = body.accessToken || body.token || body.jwt || '';
        let usuario = body.usuario || body.username || body.user || u;
        if (typeof usuario === 'object' && usuario !== null) usuario = String(usuario.username || usuario.user || u || '');
        if (!accessToken || !usuario || typeof usuario !== 'string') {
          showInlineError('Resposta da API Jupiter inválida');
          showToast('Resposta da API Jupiter inválida',{type:'error',title:'Erro'});
          return;
        }
        // Troca o token Jupiter por um JWT local
        const extRes = await fetch('/api/auth/login-external', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ accessToken, usuario })
        });
        if (extRes.ok) {
          const extBody = await extRes.json();
          localStorage.setItem('viatemp_token', extBody.token);
          showToast('Login externo realizado com sucesso',{type:'success',title:'Bem-vindo',timeout:1500});
          setTimeout(()=>{ location.href = '/'; }, 700);
          return;
        } else {
          showInlineError('Falha ao validar token externo');
          showToast('Falha ao validar token externo',{type:'error',title:'Erro'});
        }
      } else {
        let msg = 'Falha ao autenticar';
        try { const j = await res.json(); if (j && j.message) msg = j.message; } catch(e){}
        showInlineError(msg);
        showToast(msg,{type:'error',title:'Erro'});
      }
    } catch (e){
      showInlineError('Erro de conexão com o servidor');
      showToast('Erro de conexão com o servidor',{type:'error',title:'Conexão'});
    } finally {
      loginBtn.disabled = false;
      loginBtn.textContent = 'Entrar';
    }
  }

  function showInlineError(msg){
    alertsContainer.innerHTML = `<div class="toast error" style="position:relative"><div class="t-body"><div class="t-title">Erro</div><div class="t-text">${msg}</div></div><button class="t-close">✕</button></div>`;
    const btn = alertsContainer.querySelector('.t-close'); if (btn) btn.addEventListener('click', ()=> alertsContainer.innerHTML='');
  }

  loginBtn.addEventListener('click', doLogin);
  document.addEventListener('keypress', (e)=>{ if(e.key==='Enter') doLogin(); });
})();
