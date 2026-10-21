# 📋 RESUMO DO PROJETO — RACK MONITOR

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

- LDR: histerese 1800-2200 + debounce 800ms
- Ultrassônico: média móvel de 5 amostras
- Porta: debounce 200ms
- Botão: debounce 250ms + janela de clique duplo 400ms + clique longo 3000ms
- Estados: debounce de transição 300ms

### 2. Comunicação MQTT

#### Tópicos

| Tópico | Direção | Conteúdo |
| :--- | :--- | :--- |
| `rack/status` | ESP32 → Site | JSON com estado, modo, sensores, contadores (retained) |
| `rack/comando` | Site → ESP32 | Comandos: `ARMAR`, `DESARMAR`, `STATUS`, `RESET`, `TESTE` |
| `rack/logs` | ESP32 → Site | Eventos detalhados (porta, cartão, transições) |

#### Payload de status (exemplo)

```json
{
  "estado": "SEGURO",
  "modo": "ARMADO",
  "ts": "2026-10-20T19:24:26",
  "porta": false,
  "luz": false,
  "dist_cm": 318.8,
  "ativo": false,
  "ip": "10.10.0.2",
  "uid": "01020304",
  "acessos": 5,
  "invasoes": 1,
  "rearmes": 1
}
```

### Features MQTT implementadas

- ✅ LWT (Last Will) — broker publica {"status":"offline"} se ESP32 cair
- ✅ Retain — novos assinantes recebem último estado imediatamente
- ✅ Comandos remotos — ARMAR/DESARMAR via MQTT
- ✅ Logs de auditoria — tópico separado para eventos detalhados
- ✅ Reconexão automática — Wi-Fi e MQTT a cada 5s
- ✅ Anti-flood — trava no site (só envia STATUS após resposta)

### 1. Persistência NVS

#### Salvos na flash do ESP32

- limiar (distância de aproximação)
- timeout (tempo de desarme)
- invasoes, aberturas, acessos, desarmes (contadores)

#### Comportamento

- Salva ao mudar config (SET LIMIAR, SET TIMEOUT, RESET)
- Salva a cada 60s (contadores)
- Carrega no setup() antes de iniciar lógica

## ⚠️ Limitação do Wokwi: a flash virtual é zerada ao parar a simulação. Em hardware real, persiste normalmente.

### 2. Site Web (index.html)

#### Design: Dark Mode profissional com paleta semântica de cores

### Estrutura visual

- Header — logo + indicador de conexão
- Cards de Status (linha 1) — Modo, Estado, Porta, IP, UID
- Cards de Contadores (linha 2) — Acessos RFID, Invasões, Rearmes Bloqueados
- Botões de Controle — Armar, Desarmar, Modo Teste, Reset
- Log de Eventos — ações físicas (porta, cartão, comandos)
- Histórico de Status — transições de estado

Sistema de cores semântico:

| Cor | Significado | Onde aparece |
| :--- | :--- | :--- |
| 🟢 **Verde** | Tudo normal | Estado SEGURO, conexão online |
| 🔵 **Azul** | Informação/manutenção | Modo DESARMADO, IP, acessos |
| 🟡 **Amarelo** | Aviso | AVISO_SALA, APROXIMAÇÃO, porta em manutenção |
| 🔴 **Vermelho** | Perigo/crítico | EMERGENCIA, invasão, porta aberta (armado) |
| ⚪ **Branco** | Neutro | Modo ARMADO, porta fechada |

Bordas dinâmicas: o card de estado ganha borda colorida + glow conforme o estado.

### Tecnologias

- mqtt.js (via CDN) — conexão WebSocket com broker
- wss://broker.emqx.io:8084/mqtt — conexão segura
- Fonte Inter (Google Fonts)
- JavaScript puro (sem frameworks)

### Correções aplicadas

- ✅ Bug do toFixed() em mensagens sem dist_cm
- ✅ Anti-flood (trava aguardandoResposta)
- ✅ Separação entre logs de eventos e histórico de status
- ✅ Timestamp sempre exibido (formato DD/MM HH:MM:SS)
- ✅ Quebra de palavras longas (word-break: break-word)
- ✅ Nomes amigáveis para estados (EMERGENCIA, não EMERGENCIA_RACK)

### 🆕 Conceito Novo: MQTTX

O que é: Um cliente MQTT gráfico (Web ou Desktop) que serve como "WhatsApp" para conversar com o ESP32 via broker.

Por que foi útil:

- Ver mensagens em tempo real — inscrever em rack/status e rack/logs
- Enviar comandos — publicar em rack/comando para testar ARMAR, DESARMAR, etc.
- Testar Retain e LWT — verificar se o broker entregou mensagem retida ou testamento
- Debug de comunicação — isolar se o problema é ESP32, broker ou site

Como usar:

- Acesse mqttx.app/web-client
- Conecte em wss://broker.emqx.io:8084/mqtt
- Inscreva em rack/status (ver estado) e rack/logs (ver eventos)
- Para comandos: publique em rack/comando com payload ARMAR ou DESARMAR
- Conceitos MQTT aprendidos:
- Broker — intermediário que recebe e distribui mensagens
- Publish/Subscribe — modelo de comunicação (não é cliente-servidor)
- Tópicos — "canais" onde mensagens circulam
- QoS — qualidade de serviço (0, 1, 2)
- Retain — broker guarda última mensagem do tópico
- LWT — mensagem automática publicada se cliente desconectar sem avisar

## 📁 Estrutura de Arquivos

```text
rack-monitor/
├── sketch.ino          # Firmware ESP32 (Wokwi)
├── diagram.json        # Circuito no Wokwi
├── libraries.txt       # Dependências (PubSubClient, etc.)
└── server/
    └── public/
        └── index.html  # Dashboard web
```

## 🎓 Princípios de Arquitetura Aplicados

Sensores não decidem — só medem e filtram
Atuadores não leem sensores — só consomem estado
Alarme é fonte única de verdade — recebe struct/eventos
Eventos são fatos — botão foi pressionado, cartão foi lido
Toda transição é logada — auditoria completa
loop() só orquestra — update → coleta eventos → renderiza
MQTT é observador — comandos remotos pedem via API pública
NVS é passiva — não interfere na lógica do alarme

## ✅ O Que Está Funcionando

### Firmware

- ☑ Sensores com filtros e debounce
- ☑ Máquina de estados com transições
- ☑ RFID + botão emitindo eventos
- ☑ Vigilante com bloqueio de rearme
- ☑ Wi-Fi + MQTT (LWT, Retain, comandos)
- ☑ NVS (persistência de config e contadores)
- ☑ Logs de auditoria via MQTT

### Site

- ☑ Conexão WebSocket com broker
- ☑ Cards com atualização em tempo real (polling 2s)
- ☑ Sistema de cores semântico
- ☑ Logs separados (eventos + histórico)
- ☑ Comandos remotos funcionais
- ☑ Timestamp em todos os logs
- ☑ Anti-flood

## 🚀 Próximos Passos Sugeridos (para amanhã)

- Ajustes visuais finais — testar responsividade mobile, alinhamento dos botões
Adicionar gráfico histórico — usar Chart.js para gráfico de distância/estado ao longo do tempo
- Autenticação MQTT — usuário/senha no broker (segurança)
- Migrar para broker privado — em produção, não usar emqx.io público
- Hospedar o site — GitHub Pages, Vercel ou Netlify (gratuito)
- Testes em hardware real — migrar do Wokwi para ESP32 físico
- Documentação final — README do projeto com diagrama de arquitetura

## 📌 Lembretes Importantes

- Backup: baixe uma cópia do sketch.ino e index.html para o seu computador (Wokwi pode perder).
- Bibliotecas: PubSubClient, MFRC522, Adafruit SSD1306, RTClib, Adafruit GFX devem estar no libraries.txt.
- Wokwi-GUEST — rede Wi-Fi virtual do Wokwi (sem senha).
- broker.emqx.io — broker público gratuito (limite de mensagens).
- Porta 8084 (WSS) — usada pelo navegador; 1883 (TCP) — usada pelo ESP32.

Parabéns pelo projeto! Você tem um sistema IoT completo, modular, documentado, com dashboard profissional e comunicação MQTT robusta. Amanhã é só retomar daqui. 🚀
