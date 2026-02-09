# Viatemp - Adoção de sensores via MQTT

Rápido scaffold para adotar sensores ESP32 via MQTT.

Backend:
- Node.js (Express) que escuta `devices/announce` e publica `devices/{mac}/config` quando adotar.

Frontend:
- Interface simples em `client/` para listar sensores não adotados e enviar adoção.

Broker MQTT:
- Use `docker-compose up -d` para iniciar Mosquitto local (porta 1883).

Como rodar:

1) Subir Mosquitto:

```bash
docker-compose up -d
```

2) Instalar dependências e iniciar server:

```bash
cd server
npm install
npm start
```

3) Acesse a UI em `http://localhost:3000/` e adote sensores.

ESP32:
- O sketch deve publicar um `announce` em `devices/announce` com payload JSON: `{ "mac": "AA:BB:CC:...", "ip": "192.168.x.x", "version": "3.0.0" }`.
- O backend publicará `devices/{mac}/config` com `{ "name": "...", "location": "..." }`.
# viatemp