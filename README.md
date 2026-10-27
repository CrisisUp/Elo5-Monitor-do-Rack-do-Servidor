# 📋 RACK MONITOR — Sistema de Monitoramento de Rack com ESP32

[![ESP32](https://img.shields.io/badge/ESP32-DevKit-blue?logo=espressif&logoColor=white)](https://www.espressif.com/)
[![MQTT](https://img.shields.io/badge/MQTT-broker.emqx.io-purple?logo=eclipse-mosquitto&logoColor=white)](https://mqtt.org/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-orange?logo=platformio&logoColor=white)](https://platformio.org/)
[![Doxygen](https://img.shields.io/badge/Docs-Doxygen-blue?logo=readthedocs&logoColor=white)](./docs/html/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-7.4-brightgreen)](./CHANGELOG.md)

Sistema IoT de monitoramento de rack de servidores com ESP32, sensores locais,
dashboard web em tempo real, controle remoto via MQTT e Dashboard Local de
contingência (SSE).

Versão atual: **firmware v7.4**
Ambiente de desenvolvimento: **PlatformIO (VS Code) + Wokwi (protótipo) + MQTTX**

---

## 🎯 Visão Geral

Sistema IoT de monitoramento de rack de servidores com ESP32, sensores locais, dashboard web em tempo real, controle remoto via MQTT e Dashboard Local de contingência (SSE).

## 📚 Documentação Técnica Completa

A documentação detalhada de todas as classes, métodos, structs e variáveis foi gerada automaticamente com **Doxygen**.

👉 **[Clique aqui para acessar a documentação (HTML)](./docs/html/index.html)**

*Ou, se preferir a versão online hospedada no GitHub Pages:*
👉 **[Acessar versão online](https://github.com/CrisisUp/Elo5-Monitor-do-Rack-do-Servidor)**

A documentação cobre:

- **Classes:** `Alarme`, `Audio`, `DisplayOLED`, `NotificadorMQTT`, `DashboardWeb`, `RingBufferLog`, `BotaoPanico`, etc.
- **Structs:** `LeiturasSensores`, `EventoRFID`, `EventoLog`
- **Enums:** `EstadoSistema`, `ModoVigilante`, `EstadoAmbiental`, `NivelLog`, `FonteTempo`
- **Funções:** `_verificarAmbiente()`, `executarModoTeste()`, `montarJSONStatus()`, etc.

## 📦 Arquitetura Final

```text
┌─────────────────┐         ┌──────────────┐         ┌──────────────┐
│  ESP32          │  MQTT   │  Broker      │  WSS    │  Site Web    │
│  (hardware)     │ ──────► │  (emqx.io)   │ ◄─────► │  (Externo)   │
│                 │ ◄────── │              │         │              │
└─────────────────┘         └──────────────┘         └──────────────┘
     │                                                   │
     ├─ Sensores (PIR, LDR, US, Porta, RFID, DHT)        │
     ├─ Atuadores (LED, Buzzer, OLED)                    │
     ├─ Máquina de Estados (Alarme)                      │
     ├─ Dashboard Local (SSE na porta 8080)              │
     ├─ OTA (ArduinoOTA)                                 │
     └─ NVS (Persistência)                               │
```

⚠️ Limitação do Wokwi gratuito: o ESP32 simulado só existe enquanto a aba
está aberta. Ao fechar, o dispositivo "morre", o LWT dispara offline e o
dashboard fica offline. Não há gateway privado nem simulação 24/7 no plano gratuito.

## 🧩 Componentes do Sistema

### 1. Código do ESP32 (`src/main.cpp`)

#### Arquitetura modular com classes

| Classe | Responsabilidade |
| :--- | :--- |
| **Relogio** | Timestamp via RTC DS3231 (fallback para NTP, depois millis) |
| **LDR** | Sensor de luz com histerese e debounce |
| **Ultrassonico** | HC-SR04 com média móvel de 5 amostras |
| **Porta** | Switch com debounce de 200ms |
| **PIR** | Sensor de movimento |
| **Botao** | Multifunção (curto/duplo/longo) com debounce |
| **BotaoPanico** | Pushbutton com ISR (resposta < 30ms) |
| **LeitorRFID** | MFRC522 que emite eventos (não decide nada) |
| **LedRGB** | LED de status com PWM |
| **Audio** | Buzzers + melodias por estado |
| **DisplayOLED** | Renderização (consome estado, não lê sensores) |
| **Alarme** | Fonte única de verdade — máquina de estados |
| **NotificadorMQTT** | Observador do Alarme + recebe comandos (segurança e ambiente) |
| **DashboardWeb** | Servidor HTTP local (SSE) para contingência offline |
| **RingBufferLog** | Buffer circular de 100 eventos em RAM (auditoria pós-incidente) |
| **NVS (Preferences)** | Persistência de config, contadores e modo (ARMADO/DESARMADO) |

### Estados do sistema

- 🟢 **SEGURO** — tudo normal
- 🔵 **ACESSO_RFID** — cartão acabou de passar
- 🟡 **AVISO_SALA** — movimento na sala (não invasão!)
- 🟡 **APROXIMACAO_RACK** — alguém perto do rack
- 🔴 **EMERGENCIA_RACK** — porta aberta (invasão real)
- ⚪ **INVASAO_SALA** — reservado (não usado por padrão)

### Modos

- **ARMADO** — vigia ativo
- **DESARMADO** — manutenção (rearme bloqueado se porta aberta)

### Filtros implementados

| Sensor | Filtro |
| :--- | :--- |
| LDR | histerese 1800–2200 + debounce 800 ms |
| Ultrassônico | média móvel de 5 amostras; rejeita <2 cm e >400 cm |
| Porta | debounce 200 ms |
| Botão | debounce 250 ms + clique duplo <400 ms + clique longo >3000 ms |
| Pânico (ISR) | FALLING edge, cooldown de 2s |
| Estados | debounce de transição 300 ms |

### 🌡️ Alarmes ambientais (v7.4)

| Faixa | Temperatura | Umidade | Ação |
| :--- | :--- | :--- | :--- |
| Normal | < 30 °C | < 65 % | — |
| Alerta | ≥ 30 °C | ≥ 65 % | Log WARN + `rack/logs` |
| Crítico | ≥ 35 °C | ≥ 80 % | Log CRIT + `rack/logs` |
| Taxa de subida | ≥ 1 °C/min | — | Log CRIT + `rack/logs` |

- **Histerese:** 2 °C (temperatura) / 5 % (umidade) para desligar o alarme.
- **Estado consolidado** (`ambiente`: `OK` / `ALERTA` / `CRITICO`) exposto no JSON de status.
- **Limiares configuráveis** via Serial (`SET TEMP_ALERTA`, `SET TEMP_CRIT`, `SET UMID_ALERTA`, `SET UMID_CRIT`) e via MQTT (`SET_TEMP_*`, `SET_UMID_*` no tópico de ambiente).
- **Buzzer ambiental opcional** (`#define ALARME_SONORO_AMBIENTAL 0` por padrão) — respeita prioridade de segurança: nunca bipa durante `EMERGENCIA_RACK` ou `INVASAO_SALA`.

### 📡 Comunicação MQTT (v7.4)

#### Tópicos (com prefixo configurável)

O prefixo `rack_monitor_ABCD1234` é **único do seu projeto**. Troque por algo seu
no firmware **e** no dashboard. Isso **não é segurança** — só reduz ruído de
curiosos no broker público.

| Tópico | Direção | Retain | Conteúdo |
| :--- | :--- | :---: | :--- |
| `rack_monitor_ABCD1234/status` | ESP32 → Site | ✅ | JSON com estado, modo, sensores, contadores, flags ambientais, `time_source` |
| `rack_monitor_ABCD1234/status/lwt` | Broker → Site | ✅ | `{"status":"online"}` ou `{"status":"offline"}` (LWT) |
| `rack_monitor_ABCD1234/comando` | Site → ESP32 | ❌ | `ARMAR`, `DESARMAR`, `STATUS`, `RESET <token>`, `TESTE` |
| `rack_monitor_ABCD1234/ambiente` | ESP32 → Site | ✅ | JSON dedicado: temp, umid, estado ambiental, `sensor_dht_ok`, limiares |
| `rack_monitor_ABCD1234/comando/ambiente` | Site → ESP32 | ❌ | `SET_TEMP_ALERTA`, `SET_TEMP_CRIT`, `SET_UMID_ALERTA`, `SET_UMID_CRIT`, `PANICO`, `STATUS_AMB` |
| `rack_monitor_ABCD1234/logs` | ESP32 → Site | ❌ | Eventos detalhados (porta, cartão, transições, ambiente, OTA, pânico) |
| `rack_monitor_ABCD1234/heartbeat` | ESP32 → Site | ❌ | Sinal de vida a cada 30 s |

#### Payload de status (exemplo)

```json
{
  "estado": "NORMAL",
  "modo": "ARMADO",
  "ts": "2026-10-26T19:47:05",
  "time_source": "RTC",
  "porta": false,
  "luz": false,
  "dist_cm": 166.3,
  "ativo": false,
  "ip": "10.10.0.2",
  "uid": "01020304",
  "acessos": 5,
  "invasoes": 1,
  "rearmes": 1,
  "alerta_temp": false,
  "critico_temp": false,
  "alerta_umid": false,
  "critico_umid": false,
  "taxa_temp_alerta": false,
  "ambiente": "OK",
  "temp_c": 18.5,
  "umid_pct": 47.5
}
```

#### Features MQTT

- ✅ **LWT em tópico dedicado** (`.../status/lwt`) — não sobrescreve o status retido real
- ✅ **Retain no status** — novos assinantes recebem o último estado imediatamente
- ✅ **Retain no ambiente** — idem, em tópico separado
- ✅ **Republicação automática** do status completo após reconectar
- ✅ **Rate limit de `STATUS`** (10 s) — protege contra clientes abusivos
- ✅ **`RESET` com token** — contadores de segurança (`invasoes`, `rearmes`) não são zerados
- ✅ **Log de motivo de desconexão** (`rc=...`)
- ✅ **Heartbeat a cada 30 s** — permite ao dashboard detectar queda em 60 s
- ✅ **Reconexão automática** (Wi-Fi e MQTT a cada 5 s)
- ✅ **Buffer MQTT** aumentado para 512 bytes
- ✅ **KeepAlive MQTT** configurado para 120 s (reduz `rc=-3` em broker público)

### 💾 Persistência NVS

#### Salvos na flash do ESP32

**Configuração:**

- `limiar` (distância de aproximação)
- `timeout` (tempo de desarme)
- `t_alert`, `t_crit`, `u_alert`, `u_crit` (limiares ambientais)
- `modo` (ARMADO / DESARMADO — persistido entre reboots e OTA)

**Contadores:**

- `invasoes`, `aberturas`, `acessos`, `desarmes`, `rearmes`

**Cartões RFID:**

- `total_cards` + `card_0`, `card_1`, … (deduplicados)

#### Comportamento

- Salva ao mudar config (`SET LIMIAR`, `SET TIMEOUT`, `SET TEMP_*`, `SET UMID_*`, `RESET`, `ADD_CARD`)
- Salva **na hora** ao mudar modo (`ARMAR`, `DESARMAR`, timeout) — garante persistência após OTA
- Salva a cada 60 s **apenas se algum contador mudou** (dirty flag)
- Carrega no `setup()` antes de iniciar a lógica

> ⚠️ **Limitação do Wokwi:** a flash virtual é zerada ao parar a simulação.
> Em hardware real, persiste normalmente.

---

### 🖥️ Dashboard Externo (`index.html`)

#### Design

Dark mode com paleta semântica de cores.

#### Estrutura visual

- **Header** — logo + indicador de conexão + status do heartbeat ESP32
- **Cards de Status** — Modo, Estado, Porta, IP, UID
- **Cards de Contadores** — Acessos RFID, Invasões, Rearmes Bloqueados, Temperatura, Umidade
- **Botões de Controle** — Armar, Desarmar, Modo Teste, Reset (token via `prompt`), Atualizar agora
- **Gráfico histórico** — distância e temperatura (Chart.js, últimos 60 pontos, persistido em `localStorage`)
- **Log de Eventos** — ações físicas + comandos remotos
- **Histórico de Status** — transições de estado

#### Sistema de cores semântico

| Cor | Significado | Onde aparece |
| :---: | :--- | :--- |
| 🟢 Verde | Tudo normal | Estado `NORMAL`, `ACESSO RFID`, conexão online |
| 🔵 Azul | Informação/manutenção | Modo `DESARMADO`, IP, acessos |
| 🟡 Amarelo | Aviso | `AVISO SALA`, `APROXIMAÇÃO`, ambiente `ALERTA` |
| 🔴 Vermelho | Perigo/crítico | `EMERGENCIA`, invasão, porta aberta (armado), ambiente `CRITICO` |
| ⚪ Branco | Neutro | Modo `ARMADO`, porta fechada |

#### Tecnologias

- `mqtt.js` (via CDN) — conexão WebSocket
- `wss://broker.emqx.io:8084/mqtt` — conexão segura
- `Chart.js` (via CDN) — gráfico histórico
- Fonte Inter (Google Fonts)
- JavaScript puro (sem frameworks)

#### Comportamento (v4.0)

- **Sem polling agressivo** — o dashboard se atualiza por:
  - **Retain** (ao conectar, recebe o último status e ambiente)
  - **Transições** (firmware publica automaticamente)
  - **Heartbeat** (a cada 30 s)
  - **`STATUS` inicial** (1× ao conectar)
- **LWT** assinado em `.../status/lwt` — detecta queda imediata do ESP32
- **`RESET` com token** via `prompt()` — o token não fica hardcoded no HTML
- **Botão "Atualizar agora"** — pede um `STATUS` sob demanda (respeitando o rate limit de 10 s); útil para diagnóstico
- **Feedback visual de comandos** — botões desabilitam até confirmação via `rack/status` (ou timeout de 5 s)
- **Histórico do gráfico** persistido em `localStorage`
- **`temp_c: -999`** (DHT falhou) — não é plotado no gráfico
- **`sensor_dht_ok: false`** — exibe alerta no dashboard de ambiente

#### Correções aplicadas (v4.0)

- ✅ Bug do `toFixed()` em mensagens sem `dist_cm`
- ✅ Separação entre logs de eventos e histórico de status
- ✅ Timestamp sempre exibido (formato `DD/MM HH:MM:SS`)
- ✅ Quebra de palavras longas (`word-break: break-word`)
- ✅ Nomes amigáveis para estados (`NORMAL`, não `SEGURO`)
- ✅ Notificação de navegador em emergência (sem repetir)
- ✅ `reconnectPeriod: 5000` explícito + feedback de reconexão

### 🌐 Dashboard Local (SSE) — v7.4

Além do dashboard externo (via MQTT), o ESP32 serve uma **página de contingência** própria.

- **Acesso:** `http://<IP-do-ESP32>:8080/` (na mesma rede local).
- **Tecnologia:** `WebServer` + **SSE (Server-Sent Events)** — HTTP puro, sem WebSocket.
- **Funciona offline:** Não depende do broker MQTT nem da internet. Se o Wi-Fi do local cair, o ESP32 continua servindo o dashboard para quem estiver na mesma LAN.
- **Atualização:** Tempo real (eventos de porta, temperatura, logs) via `EventSource`.
- **Rotas:**
  - `/` — Página HTML (autocontida, ~4KB)
  - `/events` — Stream SSE (`text/event-stream`)
  - `/api/status` — JSON pontual do status (fallback)
  - `/api/hist` — JSON com os últimos 20 eventos do ring buffer
- **HTML embutido:** O HTML/CSS/JS está em `PROGMEM` no firmware — zero dependência de filesystem.
- **Single-client:** O `WebServer` do ESP32 atende um cliente SSE por vez (limitação conhecida do core). Ideal para um navegador de contingência.

> ⚠️ **Nota Wokwi:** No simulador gratuito, o acesso externo à porta é bloqueado. O dashboard local funciona perfeitamente no hardware físico.

---

### 🧪 MQTTX (cliente MQTT gráfico)

#### O que é

Um cliente MQTT gráfico (Web ou Desktop) que serve como "WhatsApp" para
conversar com o ESP32 via broker.

#### Por que é útil

- **Ver mensagens em tempo real** — inscrever em `.../status` e `.../logs`
- **Enviar comandos** — publicar em `.../comando` para testar `ARMAR`, `DESARMAR`, `RESET <token>`, `TESTE`
- **Testar Retain e LWT** — verificar se o broker entregou a mensagem retida ou o testamento
- **Debug de comunicação** — isolar se o problema é ESP32, broker ou site

#### Como usar (MQTTX Desktop — TCP 1883)

1. Conecte em `mqtt://broker.emqx.io:1883`
2. Inscreva em `rack_monitor_ABCD1234/#` (recebe todos os tópicos do prefixo)
3. Publique em `rack_monitor_ABCD1234/comando`:
   - `STATUS` — pede status (respeita rate limit de 10 s)
   - `ARMAR` / `DESARMAR` — muda modo
   - `TESTE` — inicia autodiagnóstico
   - `RESET rack2026` — zera contadores operacionais (preserva segurança)
4. Publique em `rack_monitor_ABCD1234/comando/ambiente`:
   - `PANICO` — dispara pânico remoto
   - `SET_TEMP_CRIT 38` — ajusta limiar crítico de temperatura
   - `STATUS_AMB` — pede um ambiente sob demanda

#### MQTTX Web (WSS 8084)

1. Acesse `mqttx.app/web-client`
2. Conecte em `wss://broker.emqx.io:8084/mqtt`
3. Mesmos tópicos/comandos do Desktop

#### Conceitos MQTT aprendidos

- **Broker** — intermediário que recebe e distribui mensagens
- **Publish/Subscribe** — modelo de comunicação (não é cliente-servidor)
- **Tópicos** — "canais" onde mensagens circulam
- **QoS** — qualidade de serviço (0, 1, 2)
- **Retain** — broker guarda a última mensagem do tópico
- **LWT** — mensagem automática publicada se cliente desconectar sem avisar

### 🖥️ Comandos Serial (firmware v7.4)

| Comando | Descrição |
| :--- | :--- |
| `SET LIMIAR <5-400>` | Ajusta limiar ultrassônico (persiste) |
| `SET TIMEOUT <0-1440>` | Timeout de desarme em minutos (persiste) |
| `SET TEMP_ALERTA <15-50>` | Limiar de temperatura para alerta (persiste) |
| `SET TEMP_CRIT <20-60>` | Limiar de temperatura crítico (persiste) |
| `SET UMID_ALERTA <30-90>` | Limiar de umidade para alerta (persiste) |
| `SET UMID_CRIT <40-100>` | Limiar de umidade crítico (persiste) |
| `STATUS` | Status completo (contadores, limiares, Wi-Fi, MQTT, OTA, histórico) |
| `ARMAR` | Força `ARMADO` (se porta fechada) |
| `DESARMAR` | Força `DESARMADO` |
| `TESTE` | Modo autodiagnóstico |
| `PANICO` | Dispara pânico manual |
| `RTC` | Mostra hora e fonte de tempo |
| `RTC YYYY-MM-DD HH:MM:SS` | Ajusta RTC manualmente |
| `NTP` | Força sincronização NTP manual |
| `OTA` | Mostra status do serviço OTA (hostname, porta, senha) |
| `WEB` | Mostra a URL do Dashboard Local |
| `HIST` | Despeja os últimos 100 eventos do ring buffer no Serial |
| `HIST_JSON` | Despeja o histórico em formato JSON |
| `HIST_CLEAR` | Limpa o histórico de eventos |
| `RESET` | Zera contadores operacionais (preserva segurança) |
| `ADD_CARD <UID>` | Cadastra cartão na NVS (deduplicado) |
| `LIST_CARDS` | Lista cartões autorizados |
| `HELP` | Lista comandos |

#### Botão físico (multifunção)

- **Curto** — reset operacional
- **Longo** — rearme forçado
- **Duplo** — modo teste

#### Botão de pânico (GPIO 26)

- **ISR** (FALLING edge) — resposta em < 30 ms
- **Cooldown** de 2 s (evita bounce mecânico)
- Dispara `INVASAO_SALA` imediatamente (ignora debounce de estado)
- Ignorado em modo `DESARMADO` (evita fadiga de alarme)

---

### 📁 Estrutura de Arquivos (PlatformIO)

```text
RackMonitor-Elo5/
├── platformio.ini      # Configuração do PlatformIO (board, libs)
├── src/
│   └── main.cpp        # Firmware ESP32 (código unificado v7.4)
├── server/
│   └── public/
│       └── index.html  # Dashboard externo (MQTT/WebSocket)
├── docs/               # Documentação Doxygen
├── diagram.json        # Circuito no Wokwi
├── CHANGELOG.md        # Histórico de versões
└── README.md           # Este arquivo
```

#### Bibliotecas necessárias (`platformio.ini`)

```ini
lib_deps = 
    miguelbalboa/MFRC522@^1.4.11
    adafruit/Adafruit GFX Library@^1.11.11
    adafruit/Adafruit SSD1306@^2.5.13
    adafruit/RTClib@^2.1.4
    adafruit/DHT sensor library@^1.4.6
    knolleary/PubSubClient@^2.8
```

### 🎓 Princípios de Arquitetura Aplicados

1. **Sensores não decidem** — só medem e filtram
2. **Atuadores não leem sensores** — só consomem estado
3. **Alarme é fonte única de verdade** — recebe struct/eventos
4. **Eventos são fatos** — botão foi pressionado, cartão foi lido
5. **Toda transição é logada** — auditoria completa
6. **`loop()` só orquestra** — update → coleta eventos → renderiza
7. **MQTT é observador** — comandos remotos pedem via API pública
8. **NVS é passiva** — não interfere na lógica do alarme
9. **JSON é montado uma vez** — `montarJSONStatus()` reutilizado por MQTT e Dashboard
10. **Logs alimentam o ring buffer** — toda chamada a `logSerial()` é auditável

### ✅ O que está funcionando (v7.4 / v4.0)

#### Firmware

- ☑ Sensores com filtros e debounce
- ☑ Máquina de estados com transições
- ☑ RFID + botão emitindo eventos
- ☑ Vigilante com bloqueio de rearme
- ☑ Wi-Fi + MQTT (LWT, Retain, comandos segregados)
- ☑ NVS (persistência de config, contadores, limiares ambientais, cartões, **modo**)
- ☑ Logs de auditoria via MQTT
- ☑ Alarmes ambientais (temp/umid) com histerese, crítico e taxa de subida
- ☑ Estado ambiental consolidado no JSON de status
- ☑ Rate limit de `STATUS` (10 s) + `RESET` com token
- ☑ Dirty flag para reduzir desgaste da flash
- ☑ **NTP + RTC Sync** com fallback para millis
- ☑ **OTA (ArduinoOTA)** com callbacks de progresso
- ☑ **Botão de pânico** com ISR (< 30 ms)
- ☑ **Ring Buffer** de 100 eventos em RAM (auditoria pós-incidente)
- ☑ **Dashboard Local (SSE)** na porta 8080 (contingência offline)
- ☑ **Persistência do modo** entre reboots e OTA

#### Dashboard Externo

- ☑ Conexão WebSocket com broker
- ☑ Atualização em tempo real via retain + transições + heartbeat
- ☑ Sistema de cores semântico
- ☑ Logs separados (eventos + histórico)
- ☑ Comandos remotos funcionais (`RESET` com token via `prompt`)
- ☑ Timestamp em todos os logs
- ☑ Gráfico histórico com persistência em `localStorage`
- ☑ Detecção de queda do ESP32 via LWT + heartbeat
- ☑ Sem polling agressivo

#### Dashboard Local

- ☑ Página HTML autocontida (~4KB, sem CDN)
- ☑ SSE em tempo real (`text/event-stream`)
- ☑ Fallback JSON (`/api/status`)
- ☑ Histórico de eventos (`/api/hist`)
- ☑ Reconexão automática do `EventSource`

### ⚠️ Limitações do ambiente gratuito

| Limitação | Impacto |
| :--- | :--- |
| Wokwi gratuito só roda com a aba aberta | ESP32 "morre" ao fechar; dashboard fica offline |
| Gateway público do Wokwi | ESP32 não recebe conexões de entrada (dashboard local não acessível) |
| Broker público `broker.emqx.io` | Sem autenticação; qualquer um pode publicar em `.../comando` |
| Token de `RESET` no firmware | `rack2026` é público — troque por um valor seu antes de produção |
| Flash virtual do Wokwi zera ao parar | NVS não persiste entre sessões de simulação |

---

### 🔐 Segurança — Recomendações para Produção

Antes de qualquer demonstração ou uso real, **troque estas credenciais** no firmware:

| Constante | Valor padrão | Ação |
| :--- | :--- | :--- |
| `MQTT_RESET_TOKEN` | `"rack2026"` | Troque por string forte (16+ chars aleatórios) |
| `OTA_PASSWORD` | `"rack@2026#ota"` | Troque por senha forte |
| `WIFI_SSID` / `WIFI_PASSWORD` | `"Wokwi-GUEST"` / `""` | Use as credenciais da sua rede real |
| `OTA_HOSTNAME` | `"rack-ABCD1234"` | Personalize (evita conflito com outros ESP32) |

**Nunca commite essas senhas no GitHub.** Use gerenciador de senhas ou `.env` fora do repositório.

---

### 🚀 Próximos passos sugeridos

- **HMAC nos comandos MQTT** — assinar `ARMAR`/`DESARMAR` com SHA256 + timestamp (anti-replay) para evitar que qualquer um desarme o rack via broker público.
- **Broker privado** — migrar do `broker.emqx.io` público para um HiveMQ Cloud (free tier) com TLS.
- **TLS no firmware** — `WiFiClientSecure` + porta 8883.
- **Hospedar o dashboard** — GitHub Pages, Vercel ou Netlify (gratuito).
- **Testes em hardware real** — validar OTA, NTP e Dashboard Local no ESP32 físico.
- **Notificação push** — integrar com Telegram ou ntfy.sh para alertas críticos.
- **Dois sensores DHT** — entrada e saída de ar do rack (pontos quentes).
- **Persistência do histórico** — backend com InfluxDB / SQLite para séries temporais.

---

### 📌 Lembretes importantes

- **Backup:** baixe uma cópia do `src/main.cpp` e `index.html` para o seu computador.
- **Bibliotecas:** devem estar no `platformio.ini` (PlatformIO) ou `libraries.txt` (Wokwi).
- **`Wokwi-GUEST`** — rede Wi-Fi virtual do Wokwi (sem senha).
- **`broker.emqx.io`** — broker público gratuito (limite de mensagens).
- **Porta 8084 (WSS)** — usada pelo navegador; **1883 (TCP)** — usada pelo ESP32.
- **Porta 8080 (HTTP)** — usada pelo Dashboard Local no hardware físico.
- **Prefixos de tópico** — o `rack_monitor_ABCD1234` deve ser idêntico no firmware, MQTTX e dashboard.
- **Token de RESET** — troque `rack2026` por um valor seu no firmware antes de qualquer demonstração pública.

---

## 📚 Referências e Agradecimentos

Este projeto foi construído sobre os ombros de gigantes. Agradecimentos às seguintes documentações e ferramentas:

### Bibliotecas e Ferramentas

- [**PlatformIO**](https://platformio.org/) — IDE e build system para embarcados
- [**Wokwi**](https://docs.wokwi.com/) — Simulador de ESP32 (gratuito)
- [**PubSubClient**](https://pubsubclient.knolleary.net/) — Cliente MQTT para Arduino/ESP32
- [**MFRC522**](https://github.com/miguelbalboa/rfid) — Biblioteca do leitor RFID
- [**Adafruit SSD1306**](https://learn.adafruit.com/monochrome-oled-breakouts) + [**GFX**](https://learn.adafruit.com/adafruit-gfx-graphics-library) — Display OLED
- [**RTClib**](https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout) — Driver do RTC DS3231
- [**DHT sensor library**](https://learn.adafruit.com/dht) — Sensor DHT22
- [**Preferences**](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html) — NVS (persistência em flash)
- [**ArduinoOTA**](https://arduino-esp32.embarkx.in/.../ArduinoOTA) — Atualização Over-The-Air
- [**WebServer**](https://github.com/espressif/arduino-esp32/tree/master/libraries/WebServer) — Servidor HTTP built-in
- [**Doxygen**](https://www.doxygen.nl/) — Geração automática da documentação técnica

### Conceitos e Tutoriais

- [MQTT Essentials (HiveMQ)](https://www.hivemq.com/mqtt-essentials/) — Excelente introdução aos conceitos de MQTT
- [MQTT Retain and LWT (EMQX)](https://www.emqx.com/en/blog/mqtt-retained-messages) — Documentação sobre Retain e Last Will
- [ESP32 Strapping Pins (Espressif)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html) — Cuidados com GPIOs 0, 2, 12 e 15
- [Server-Sent Events (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events) — Referência do SSE
- [ArduinoOTA (ESP32)](https://arduino-esp32.embarkx.in/...) — Como funciona o OTA no ESP32

### Inspiração

- Documentação da [Arduino Reference](https://www.arduino.cc/reference/en/) — Boas práticas de código embarcado
- [The Twelve-Factor App](https://12factor.net/) — Princípios de design de software

Este é o `README.md` completo, atualizado para a **v7.4**. Você pode copiar o bloco de código acima e substituir o conteúdo do seu arquivo `README.md` atual. Todas as tabelas, badges, negritos e ícones estão preservados no formato Markdown.