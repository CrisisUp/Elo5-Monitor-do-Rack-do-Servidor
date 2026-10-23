# 📋 RACK MONITOR — Sistema de Monitoramento de Rack com ESP32

Sistema IoT de monitoramento de rack de servidores com ESP32, sensores locais,
dashboard web em tempo real e controle remoto via MQTT.

Versão atual: **firmware v6.4 / dashboard v3.1**
Ambiente de desenvolvimento: **Wokwi (gratuito) + MQTTX (gratuito) + broker público**

---

## 🎯 Visão Geral

Sistema IoT de monitoramento de rack de servidores com ESP32, sensores locais, dashboard web em tempo real e controle remoto via MQTT.

## 📦 Arquitetura Final

```text
┌─────────────┐         ┌──────────────┐         ┌──────────────┐
│  ESP32      │  MQTT   │  Broker      │  WSS    │  Site Web    │
│  (Wokwi)    │ ──────► │  (emqx.io)   │ ◄─────► │  (Local)     │
│             │ ◄────── │              │         │              │
└─────────────┘         └──────────────┘         └──────────────┘
     │                                                   │
     ├─ Sensores (PIR, LDR, US, Porta, RFID)             │
     ├─ Atuadores (LED, Buzzer, OLED)                    │
     ├─ Máquina de Estados (Alarme)                      │
     └─ NVS (Persistência)                               │
```

⚠️ Limitação do Wokwi gratuito: o ESP32 simulado só existe enquanto a aba
está aberta. Ao fechar, o dispositivo "morre", o LWT dispara offline e o
dashboard fica offline. Não há gateway privado nem simulação 24/7 no plano gratuito.

## 🧩 Componentes do Sistema

### 1. Código do ESP32 (sketch.ino)

#### Arquitetura modular com classes

| Classe | Responsabilidade |
| :--- | :--- |
| **Relogio** | Timestamp via RTC DS3231 (fallback para millis) |
| **LDR** | Sensor de luz com histerese e debounce |
| **Ultrassonico** | HC-SR04 com média móvel de 5 amostras |
| **Porta** | Switch com debounce de 200ms |
| **PIR** | Sensor de movimento |
| **Botao** | Multifunção (curto/duplo/longo) com debounce |
| **LeitorRFID** | MFRC522 que emite eventos (não decide nada) |
| **LedRGB** | LED de status com PWM |
| **Audio** | Buzzers + melodias por estado |
| **DisplayOLED** | Renderização (consome estado, não lê sensores) |
| **Alarme** | Fonte única de verdade — máquina de estados |
| **NotificadorMQTT** | Observador do Alarme + recebe comandos |
| **NVS (Preferences)** | Persistência de config e contadores |

## Estados do sistema

- 🟢 SEGURO — tudo normal
- 🔵 ACESSO_RFID — cartão acabou de passar
- 🟡 AVISO_SALA — movimento na sala (não invasão!)
- 🟡 APROXIMACAO_RACK — alguém perto do rack
- 🔴 EMERGENCIA_RACK — porta aberta (invasão real)
- ⚪ INVASAO_SALA — reservado (não usado por padrão)

### Modos

- ARMADO — vigia ativo
- DESARMADO — manutenção (rearme bloqueado se porta aberta)

### Filtros implementados

| Sensor | Filtro |
| :--- | :--- |
| LDR | histerese 1800–2200 + debounce 800 ms |
| Ultrassônico | média móvel de 5 amostras; rejeita <2 cm e >400 cm |
| Porta | debounce 200 ms |
| Botão | debounce 250 ms + clique duplo <400 ms + clique longo >3000 ms |
| Estados | debounce de transição 300 ms |

### 🌡️ Alarmes ambientais (v6.4)

| Faixa | Temperatura | Umidade | Ação |
| :--- | :--- | :--- | :--- |
| Normal | < 30 °C | < 65 % | — |
| Alerta | ≥ 30 °C | ≥ 65 % | Log WARN + `rack/logs` |
| Crítico | ≥ 35 °C | ≥ 80 % | Log CRIT + `rack/logs` |
| Taxa de subida | ≥ 1 °C/min | — | Log CRIT + `rack/logs` |

- **Histerese:** 2 °C (temperatura) / 5 % (umidade) para desligar o alarme.
- **Estado consolidado** (`ambiente`: `OK` / `ALERTA` / `CRITICO`) exposto no JSON de status.
- **Limiares configuráveis** via Serial (`SET TEMP_ALERTA`, `SET TEMP_CRIT`, `SET UMID_ALERTA`, `SET UMID_CRIT`) e persistidos na NVS.
- **Buzzer ambiental opcional** (`#define ALARME_SONORO_AMBIENTAL 0` por padrão) — respeita prioridade de segurança: nunca bipa durante `EMERGENCIA_RACK` ou `INVASAO_SALA`.

### 📡 Comunicação MQTT (v6.4)

#### Tópicos (com prefixo configurável)

O prefixo `rack_monitor_ABCD1234` é **único do seu projeto**. Troque por algo seu
no firmware **e** no dashboard. Isso **não é segurança** — só reduz ruído de
curiosos no broker público.

| Tópico | Direção | Retain | Conteúdo |
| :--- | :--- | :---: | :--- |
| `rack_monitor_ABCD1234/status` | ESP32 → Site | ✅ | JSON com estado, modo, sensores, contadores, flags ambientais |
| `rack_monitor_ABCD1234/status/lwt` | Broker → Site | ✅ | `{"status":"online"}` ou `{"status":"offline"}` (LWT) |
| `rack_monitor_ABCD1234/comando` | Site → ESP32 | ❌ | `ARMAR`, `DESARMAR`, `STATUS`, `RESET <token>`, `TESTE` |
| `rack_monitor_ABCD1234/logs` | ESP32 → Site | ❌ | Eventos detalhados (porta, cartão, transições, ambiente) |
| `rack_monitor_ABCD1234/heartbeat` | ESP32 → Site | ❌ | Sinal de vida a cada 30 s |

#### Payload de status (exemplo)

```json
{
  "estado": "NORMAL",
  "modo": "ARMADO",
  "ts": "2026-10-23T19:24:26",
  "porta": false,
  "luz": false,
  "dist_cm": 318.8,
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
  "temp_c": 24.5,
  "umid_pct": 48.0
}
```

### Features MQTT

- ✅ **LWT em tópico dedicado** (`.../status/lwt`) — não sobrescreve o status retido real
- ✅ **Retain no status** — novos assinantes recebem o último estado imediatamente
- ✅ **Republicação automática** do status completo após reconectar
- ✅ **Rate limit de `STATUS`** (2 s) — protege contra clientes abusivos
- ✅ **`RESET` com token** — contadores de segurança (`invasoes`, `rearmes`) não são zerados
- ✅ **Log de motivo de desconexão** (`rc=...`)
- ✅ **Heartbeat a cada 30 s** — permite ao dashboard detectar queda em 60 s
- ✅ **Reconexão automática** (Wi-Fi e MQTT a cada 5 s)
- ✅ **Buffer MQTT** aumentado para 512 bytes

---

### 💾 Persistência NVS

#### Salvos na flash do ESP32

**Configuração:**
- `limiar` (distância de aproximação)
- `timeout` (tempo de desarme)
- `t_alert`, `t_crit`, `u_alert`, `u_crit` (limiares ambientais)

**Contadores:**
- `invasoes`, `aberturas`, `acessos`, `desarmes`, `rearmes`

**Cartões RFID:**
- `total_cards` + `card_0`, `card_1`, … (deduplicados)

#### Comportamento

- Salva ao mudar config (`SET LIMIAR`, `SET TIMEOUT`, `SET TEMP_*`, `SET UMID_*`, `RESET`, `ADD_CARD`)
- Salva a cada 60 s **apenas se algum contador mudou** (dirty flag)
- Carrega no `setup()` antes de iniciar a lógica

> ⚠️ **Limitação do Wokwi:** a flash virtual é zerada ao parar a simulação.
> Em hardware real, persiste normalmente.

---

### 🖥️ Dashboard Web (index.html)

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

#### Comportamento (v3.1)

- **Sem polling agressivo** — o dashboard se atualiza por:
  - **Retain** (ao conectar, recebe o último status)
  - **Transições** (firmware publica automaticamente)
  - **Heartbeat** (a cada 30 s)
  - **`STATUS` inicial** (1× ao conectar)
- **LWT** assinado em `.../status/lwt` — detecta queda imediata do ESP32
- **`RESET` com token** via `prompt()` — o token não fica hardcoded no HTML
- **Botão "Atualizar agora"** — pede um `STATUS` sob demanda (respeitando o rate limit de 2 s); útil para diagnóstico
- **Feedback visual de comandos** — botões desabilitam até confirmação via `rack/status` (ou timeout de 5 s)
- **Histórico do gráfico** persistido em `localStorage`
- **`temp_c: -999`** (DHT falhou) — não é plotado no gráfico

#### Correções aplicadas

- ✅ Bug do `toFixed()` em mensagens sem `dist_cm`
- ✅ Separação entre logs de eventos e histórico de status
- ✅ Timestamp sempre exibido (formato `DD/MM HH:MM:SS`)
- ✅ Quebra de palavras longas (`word-break: break-word`)
- ✅ Nomes amigáveis para estados (`NORMAL`, não `SEGURO`)
- ✅ Notificação de navegador em emergência (sem repetir)
- ✅ `reconnectPeriod: 5000` explícito + feedback de reconexão

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
   - `STATUS` — pede status (respeita rate limit de 2 s)
   - `ARMAR` / `DESARMAR` — muda modo
   - `TESTE` — inicia autodiagnóstico
   - `RESET rack2026` — zera contadores operacionais (preserva segurança)

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

---

### 🖥️ Comandos Serial (firmware)

| Comando | Descrição |
| :--- | :--- |
| `SET LIMIAR <5-400>` | Ajusta limiar ultrassônico (persiste) |
| `SET TIMEOUT <0-1440>` | Timeout de desarme em minutos (persiste) |
| `SET TEMP_ALERTA <15-50>` | Limiar de temperatura para alerta (persiste) |
| `SET TEMP_CRIT <20-60>` | Limiar de temperatura crítico (persiste) |
| `SET UMID_ALERTA <30-90>` | Limiar de umidade para alerta (persiste) |
| `SET UMID_CRIT <40-100>` | Limiar de umidade crítico (persiste) |
| `STATUS` | Status completo (contadores, limiares, Wi-Fi, MQTT) |
| `ARMAR` | Força `ARMADO` (se porta fechada) |
| `DESARMAR` | Força `DESARMADO` |
| `TESTE` | Modo autodiagnóstico |
| `RTC` | Mostra hora do RTC |
| `RTC YYYY-MM-DD HH:MM:SS` | Ajusta RTC |
| `RESET` | Zera contadores operacionais (preserva segurança) |
| `ADD_CARD <UID>` | Cadastra cartão na NVS (deduplicado) |
| `LIST_CARDS` | Lista cartões autorizados |
| `HELP` | Lista comandos |

#### Botão físico (multifunção)

- **Curto** — reset operacional
- **Longo** — rearme forçado
- **Duplo** — modo teste

---

### 📁 Estrutura de Arquivos

```text
rack-monitor/
├── sketch.ino          # Firmware ESP32 (Wokwi)
├── diagram.json        # Circuito no Wokwi
├── libraries.txt       # Dependências (PubSubClient, etc.)
└── server/
    └── public/
        └── index.html  # Dashboard web

#### Bibliotecas necessárias (`libraries.txt`)

- `PubSubClient`
- `MFRC522`
- `Adafruit SSD1306`
- `Adafruit GFX Library`
- `RTClib`
- `DHT sensor library`

---

### 🎓 Princípios de Arquitetura Aplicados

1. **Sensores não decidem** — só medem e filtram
2. **Atuadores não leem sensores** — só consomem estado
3. **Alarme é fonte única de verdade** — recebe struct/eventos
4. **Eventos são fatos** — botão foi pressionado, cartão foi lido
5. **Toda transição é logada** — auditoria completa
6. **`loop()` só orquestra** — update → coleta eventos → renderiza
7. **MQTT é observador** — comandos remotos pedem via API pública
8. **NVS é passiva** — não interfere na lógica do alarme

---

### ✅ O que está funcionando (v6.4 / v3.1)

#### Firmware

- ☑ Sensores com filtros e debounce
- ☑ Máquina de estados com transições
- ☑ RFID + botão emitindo eventos
- ☑ Vigilante com bloqueio de rearme
- ☑ Wi-Fi + MQTT (LWT, Retain, comandos)
- ☑ NVS (persistência de config, contadores, limiares ambientais, cartões)
- ☑ Logs de auditoria via MQTT
- ☑ Alarmes ambientais (temp/umid) com histerese, crítico e taxa de subida
- ☑ Estado ambiental consolidado no JSON de status
- ☑ Rate limit de `STATUS` + `RESET` com token
- ☑ Dirty flag para reduzir desgaste da flash

#### Dashboard

- ☑ Conexão WebSocket com broker
- ☑ Atualização em tempo real via retain + transições + heartbeat
- ☑ Sistema de cores semântico
- ☑ Logs separados (eventos + histórico)
- ☑ Comandos remotos funcionais (`RESET` com token via `prompt`)
- ☑ Timestamp em todos os logs
- ☑ Gráfico histórico com persistência em `localStorage`
- ☑ Detecção de queda do ESP32 via LWT + heartbeat
- ☑ Sem polling agressivo

---

### ⚠️ Limitações do ambiente gratuito

| Limitação | Impacto |
| :--- | :--- |
| Wokwi gratuito só roda com a aba aberta | ESP32 "morre" ao fechar; dashboard fica offline |
| Gateway público do Wokwi | ESP32 não recebe conexões de entrada |
| Broker público `broker.emqx.io` | Sem autenticação; qualquer um pode publicar em `.../comando` |
| Token de `RESET` no firmware | `rack2026` é público — troque por um valor seu antes de produção |
| Flash virtual do Wokwi zera ao parar | NVS não persiste entre sessões de simulação |

---

### 🚀 Próximos passos sugeridos

- **Autenticação MQTT** — usuário/senha no broker (HiveMQ Cloud free tier, EMQX Cloud)
- **Broker privado** — em produção, nunca usar `broker.emqx.io` público
- **TLS no firmware** — `WiFiClientSecure` + porta 8883 (testar se o Wokwi gratuito permite)
- **Hospedar o dashboard** — GitHub Pages, Vercel ou Netlify (gratuito)
- **Testes em hardware real** — migrar do Wokwi para ESP32 físico
- **Dois sensores DHT** — entrada e saída de ar do rack (pontos quentes)
- **Persistência do histórico** — backend com InfluxDB / SQLite para séries temporais
- **Notificação por e-mail / webhook** — em vez de só dashboard

---

### 📌 Lembretes importantes

- **Backup:** baixe uma cópia do `sketch.ino` e `index.html` para o seu computador (Wokwi pode perder).
- **Bibliotecas:** devem estar no `libraries.txt` do Wokwi.
- **`Wokwi-GUEST`** — rede Wi-Fi virtual do Wokwi (sem senha).
- **`broker.emqx.io`** — broker público gratuito (limite de mensagens).
- **Porta 8084 (WSS)** — usada pelo navegador; **1883 (TCP)** — usada pelo ESP32.
- **Prefixos de tópico** — o `rack_monitor_ABCD1234` deve ser idêntico no firmware, MQTTX e dashboard.
- **Token de RESET** — troque `rack2026` por um valor seu no firmware antes de qualquer demonstração pública.