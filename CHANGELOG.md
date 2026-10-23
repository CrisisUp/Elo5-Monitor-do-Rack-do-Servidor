# Changelog

Todas as mudanças notáveis deste projeto serão documentadas neste arquivo.
O formato é baseado em [Keep a Changelog](https://keepachangelog.com/pt-BR/1.1.0/).

## [6.4] - 2026-10-23

### Corrigido

- Dupla publicação por STATUS (cache atualizado em `_publicar`)
- Rate limit de STATUS (2s) e RESET com token
- LWT em tópico dedicado (`rack/status/lwt`); republica status completo ao reconectar
- Log do motivo de desconexão (`rc=...`)
- Dirty flag para NVS (não grava se nada mudou)
- Removida validação morta em `LeitorRFID::ler()`
- `jsonEscape()` + `setBufferSize(512)` no PubSubClient
- Primeira leitura de PIR/LDR não gera borda falsa
- `SensorDHT` simplificado (sem leitura dupla)
- Deduplicação de cartões na NVS
- Documentação de pinos atualizada (DHT no 13)

### Adicionado

- Watchdog do DHT22 (reset ambiental após 10s sem leitura)
- Sistema de prioridade de áudio (Evita conflito entre buzzers)
- Distância inválida enviada como `null` no JSON
- Cache de estado da porta (`_ultimoEstadoPortaPublicado`)
- Modo Teste v3 com feedback detalhado (logs individuais)
- Documentação Doxygen completa

## [6.3] - 2026-10-22

### Adicionado ()

- LWT (Last Will): broker publica `{"status":"offline"}` se ESP32 cair
- Retain flag: novos assinantes recebem o último estado imediatamente
- NVS (Preferences): limiar, timeout e contadores persistem após reboot
- Comandos remotos via MQTT: `rack/comando` aceita `ARMAR`/`DESARMAR`

## [6.2] - 2026-10-20

### Adicionado (*)

- Integração Wi-Fi + MQTT: transições publicadas em broker público

## [6.1] - 2026-10-18

### Adicionado (**)

- Documentação Doxygen inicial
- Correção do duplo `update()` do botão
- Ordem de inicialização I2C

## [6.0] - 2026-10-15

### Alterado

- Refatoração modular: classes encapsuladas, struct de sensores, máquina isolada

## [5.0] - 2026-10-10

### Adicionado (***)

- Versão original em arquivo único
