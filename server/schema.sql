-- Schema for Viatemp
BEGIN;

CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- Users (for login)
CREATE TABLE IF NOT EXISTS users (
  id SERIAL PRIMARY KEY,
  username VARCHAR(64) UNIQUE NOT NULL,
  password_hash TEXT NOT NULL,
  full_name VARCHAR(128),
  role VARCHAR(32) DEFAULT 'admin',
  created_at TIMESTAMPTZ DEFAULT now()
);

-- Locations (localidades)
CREATE TABLE IF NOT EXISTS locations (
  id SERIAL PRIMARY KEY,
  name VARCHAR(128) UNIQUE NOT NULL,
  description TEXT,
  created_at TIMESTAMPTZ DEFAULT now()
);

-- Sensors
CREATE TABLE IF NOT EXISTS sensors (
  mac VARCHAR(17) PRIMARY KEY,
  ip INET,
  version VARCHAR(32),
  name VARCHAR(64),
  location_id INTEGER REFERENCES locations(id) ON DELETE SET NULL,
  adopted BOOLEAN DEFAULT FALSE,
  last_seen TIMESTAMPTZ,
  created_at TIMESTAMPTZ DEFAULT now(),
  updated_at TIMESTAMPTZ DEFAULT now()
);

CREATE INDEX IF NOT EXISTS idx_sensors_location ON sensors(location_id);
CREATE INDEX IF NOT EXISTS idx_sensors_lastseen ON sensors(last_seen);

-- Adoption history (audit)
CREATE TABLE IF NOT EXISTS adoption_history (
  id SERIAL PRIMARY KEY,
  mac VARCHAR(17) REFERENCES sensors(mac) ON DELETE CASCADE,
  location_id INTEGER REFERENCES locations(id) ON DELETE SET NULL,
  action VARCHAR(16) NOT NULL,
  performed_by INTEGER REFERENCES users(id),
  performed_at TIMESTAMPTZ DEFAULT now()
);

-- Example admin user (password 'admin123')
INSERT INTO users (username, password_hash, full_name, role)
SELECT 'admin', crypt('admin123', gen_salt('bf')), 'Administrador', 'admin'
WHERE NOT EXISTS (SELECT 1 FROM users WHERE username='admin');

COMMIT;
