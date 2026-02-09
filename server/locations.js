const express = require('express');
const router = express.Router();
const db = require('./db');

// GET /api/locations
router.get('/', async (req, res) => {
  try {
    const r = await db.query('SELECT id, name, description, created_at FROM locations ORDER BY name');
    res.json(r.rows);
  } catch (e) { console.error(e); res.status(500).json({ error: 'server' }); }
});

// POST /api/locations { name, description }
router.post('/', async (req, res) => {
  const { name, description } = req.body;
  if (!name) return res.status(400).json({ error: 'missing_name' });
  try {
    const q = 'INSERT INTO locations(name, description) VALUES($1,$2) RETURNING id, name, description, created_at';
    const r = await db.query(q, [name, description || null]);
    res.json(r.rows[0]);
  } catch (e) { console.error(e); res.status(500).json({ error: 'server' }); }
});

// PUT /api/locations/:id { name, description }
router.put('/:id', async (req, res) => {
  const id = parseInt(req.params.id,10);
  const { name, description } = req.body;
  if (!name) return res.status(400).json({ error: 'missing_name' });
  try {
    const q = 'UPDATE locations SET name=$1, description=$2 WHERE id=$3 RETURNING id, name, description, created_at';
    const r = await db.query(q, [name, description || null, id]);
    if (r.rowCount === 0) return res.status(404).json({ error: 'not_found' });
    res.json(r.rows[0]);
  } catch (e) { console.error(e); res.status(500).json({ error: 'server' }); }
});

// DELETE /api/locations/:id
router.delete('/:id', async (req, res) => {
  const id = parseInt(req.params.id,10);
  try {
    await db.query('DELETE FROM locations WHERE id=$1', [id]);
    res.json({ ok: true });
  } catch (e) { console.error(e); res.status(500).json({ error: 'server' }); }
});

module.exports = router;
