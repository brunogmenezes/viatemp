const express = require('express');
const router = express.Router();
const db = require('./db');
const path = require('path');
require('dotenv').config({ path: path.join(__dirname, '.env') });
const jwt = require('jsonwebtoken');

// POST /api/auth/login { username, password }
router.post('/login', async (req, res) => {
  const { username, password } = req.body;
  if (!username || !password) return res.status(400).json({ error: 'missing' });
  try {
    // Use pgcrypto crypt() verification directly in SQL
    const q = `SELECT id, username, full_name FROM users WHERE username = $1 AND password_hash = crypt($2, password_hash) LIMIT 1`;
    const r = await db.query(q, [username, password]);
    if (r.rowCount === 0) return res.status(401).json({ error: 'invalid' });
    const user = r.rows[0];
    const token = jwt.sign({ sub: user.id, username: user.username }, process.env.JWT_SECRET || 'dev_secret', { expiresIn: '8h' });
    res.json({ token, user: { id: user.id, username: user.username, full_name: user.full_name } });
  } catch (e) {
    console.error(e);
    res.status(500).json({ error: 'server' });
  }
});

// GET /api/auth/me -> validate token and return user
router.get('/me', (req, res) => {
  const auth = req.headers['authorization'];
  if (!auth) return res.status(401).json({ error: 'missing_token' });
  const parts = auth.split(' ');
  if (parts.length !== 2 || parts[0] !== 'Bearer') return res.status(401).json({ error: 'invalid_token' });
  const token = parts[1];
  try {
    const payload = jwt.verify(token, process.env.JWT_SECRET || 'dev_secret');
    // return minimal info
    res.json({ id: payload.sub, username: payload.username });
  } catch (e) {
    return res.status(401).json({ error: 'invalid' });
  }
});

// POST /api/auth/login-external { accessToken, usuario }
router.post('/login-external', async (req, res) => {
  const { accessToken, usuario } = req.body;
  console.log('[login-external] accessToken:', accessToken);
  console.log('[login-external] usuario:', usuario);
  if (!accessToken || !usuario) return res.status(400).json({ error: 'missing' });
  try {
    const jwtParts = accessToken.split('.');
    if (jwtParts.length !== 3) return res.status(401).json({ error: 'invalid_token' });
    const payload = JSON.parse(Buffer.from(jwtParts[1], 'base64').toString('utf8'));
    if (!payload) return res.status(401).json({ error: 'invalid_token' });
    // Preferir username do payload se disponível
    let safeUsername = String(usuario).substring(0, 64);
    let safeFullName = safeUsername;
    if (payload && payload.usuario) safeUsername = String(payload.usuario).substring(0, 64);
    if (payload && payload.nome) safeFullName = String(payload.nome).substring(0, 64);
    console.log('[login-external] safeUsername:', safeUsername);
    console.log('[login-external] safeFullName:', safeFullName);
    const q1 = `SELECT id, username, full_name FROM users WHERE username = $1 LIMIT 1`;
    const r1 = await db.query(q1, [safeUsername]);
    let user = null;
    if (r1.rowCount > 0) {
      user = r1.rows[0];
    } else {
      const q2 = `INSERT INTO users (username, full_name, password_hash) VALUES ($1, $2, crypt($3, gen_salt('bf'))) RETURNING id, username, full_name`;
      const r2 = await db.query(q2, [safeUsername, safeFullName, Math.random().toString(36).slice(-8)]);
      user = r2.rows[0];
    }
    const token = jwt.sign({ sub: user.id, username: user.username, external: true }, process.env.JWT_SECRET || 'dev_secret', { expiresIn: '8h' });
    res.json({ token, user: { id: user.id, username: user.username, full_name: user.full_name, external: true } });
  } catch (e) {
    console.error('[login-external] error:', e);
    res.status(500).json({ error: 'server' });
  }
});

module.exports = router;
