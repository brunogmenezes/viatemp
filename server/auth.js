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

module.exports = router;
