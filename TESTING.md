# ✅ Checklist de Testes — Rack Monitor v7.4

Plano de testes completo para o sistema de monitoramento de rack. Cobre desde a validação básica de hardware até cenários de falha de rede e segurança.

---

## 1. 🛠️ Testes de Hardware e Sensores (Bancada)

*Objetivo: Garantir que cada componente físico responde corretamente e dentro das especificações.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 1.1 | **Display OLED** | Ligar o ESP32. | O logo "RACK MONITOR v7.4" aparece e o display não fica em branco. | ☐ |
| 1.2 | **LED RGB (Self-test)** | Observar o LED ao ligar. | Ele pisca em Vermelho, Verde e Azul em sequência e apaga. | ☐ |
| 1.3 | **Buzzer (Self-test)** | Observar/ouvir ao ligar. | O buzzer emite um bip curto de inicialização. | ☐ |
| 1.4 | **Sensor DHT22** | Abrir o Monitor Serial e digitar `STATUS`. | A temperatura e umidade aparecem com valores realistas (não `-999`). | ☐ |
| 1.5 | **Sensor Ultrassônico** | Aproximar a mão do sensor e digitar `STATUS`. | O valor de `dist_cm` varia conforme a mão se aproxima/afasta. | ☐ |
| 1.6 | **LDR (Luz)** | Cobrir o sensor com a mão e depois expor à luz. | No `STATUS`, o estado `luz` muda de `APAGADA` para `ACESA` (com debounce). | ☐ |
| 1.7 | **Sensor de Porta (Switch)** | Abrir e fechar o switch. | No `STATUS`, a `porta` muda de `fechada` para `ABERTA`. | ☐ |
| 1.8 | **Sensor PIR** | Acenar a mão em frente ao PIR. | No `STATUS`, o `movimento` aparece como detectado. | ☐ |
| 1.9 | **Leitor RFID** | Aproximar um cartão cadastrado. | O UID aparece no `STATUS` e no log (cartão autorizado). | ☐ |
| 1.10 | **Botão Físico (Curto)** | Pressionar rapidamente o botão. | O log mostra "Clique CURTO: reset operacional". | ☐ |
| 1.11 | **Botão Físico (Longo)** | Segurar o botão por mais de 3s. | O log mostra "Clique LONGO: rearme forçado". | ☐ |
| 1.12 | **Botão Físico (Duplo)** | Pressionar duas vezes rápido. | O log mostra "Clique DUPLO: modo teste". | ☐ |
| 1.13 | **Botão de Pânico** | Pressionar o botão no GPIO 26. | O sistema transita imediatamente para `INVASAO_SALA` e o log mostra "BOTAO DE PANICO ACIONADO". | ☐ |

---

## 2. 🧠 Testes de Lógica e Máquina de Estados (Firmware)

*Objetivo: Validar o comportamento do `Alarme` sem depender de rede ou dashboard.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 2.1 | **Transição SEGURO → AVISO** | Fazer o PIR detectar movimento (sistema ARMADO). | O OLED mostra "Aviso: Movimento" e o LED fica amarelo. | ☐ |
| 2.2 | **Transição SEGURO → EMERGÊNCIA** | Abrir a porta (sistema ARMADO). | O OLED mostra "Emergencia: Rack", LED vermelho e o buzzer dispara alarme. | ☐ |
| 2.3 | **Bloqueio de Rearme** | Abrir a porta e tentar armar via cartão RFID. | O sistema não arma, o OLED mostra "REARME BLOQUEADO" e o log avisa "Feche a porta". | ☐ |
| 2.4 | **Rearme Automático por Timeout** | Desarmar via cartão e esperar o timeout configurado (ex: 1 min). | O sistema volta a `ARMADO` automaticamente (se a porta estiver fechada) e o log avisa. | ☐ |
| 2.5 | **Modo Manutenção (DESARMADO)** | Passe um cartão autorizado para desarmar. | O OLED mostra "MANUTENÇÃO", LED azul pulsante e o PIR não gera alarme. | ☐ |
| 2.6 | **Persistência do Modo (NVS)** | Desarmar e reiniciar o ESP32 (botão EN). | No boot, o log mostra "Modo restaurado: DESARMADO" e o sistema não arma. | ☐ |
| 2.7 | **Histerese Ambiental** | Simular (ou esquentar) o DHT acima do limiar crítico e depois esfriar. | O buzzer de ambiente liga no crítico e **desliga** quando a temperatura cai abaixo do limiar de histerese. | ☐ |
| 2.8 | **Taxa de Subida** | (Avançado) Simular subida rápida de temperatura (> 1 °C/min). | O log gera um evento "CRIT" de subida rápida. | ☐ |

---

## 3. 🌐 Testes de Comunicação (Wi-Fi e MQTT)

*Objetivo: Garantir que o ESP32 conversa corretamente com o broker e recebe comandos.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 3.1 | **Conexão Wi-Fi** | Ligar o ESP32 e observar o Monitor Serial. | O log mostra o IP obtido (ex: `192.168.1.x`). | ☐ |
| 3.2 | **Conexão MQTT (LWT)** | Usar MQTTX para se inscrever em `.../status/lwt`. | Ao ligar, recebe `{"status":"online"}`; ao desligar, recebe `offline`. | ☐ |
| 3.3 | **Retain (Status)** | Inscrever-se em `.../status` com o MQTTX. | Recebe imediatamente o último status publicado, mesmo sem o ESP32 ter mudado de estado. | ☐ |
| 3.4 | **Retain (Ambiente)** | Inscrever-se em `.../ambiente`. | Recebe imediatamente os dados de temperatura/umidade atuais. | ☐ |
| 3.5 | **Comando ARMAR/DESARMAR** | Publicar `DESARMAR` em `.../comando` e depois `ARMAR`. | O sistema muda de modo e o dashboard/MQTTX reflete a mudança. | ☐ |
| 3.6 | **Comando STATUS** | Publicar `STATUS` em `.../comando`. | O ESP32 publica o JSON de status imediatamente (respeitando rate limit). | ☐ |
| 3.7 | **Comando RESET (Token)** | Publicar `RESET rack2026` (token correto) e depois `RESET errado`. | O reset funciona só com o token certo; com o errado, loga "negado". | ☐ |
| 3.8 | **Comando de Ambiente** | Publicar `SET_TEMP_CRIT 38` em `.../comando/ambiente`. | O limiar é atualizado, persiste na NVS e o ESP32 republica o ambiente. | ☐ |
| 3.9 | **Pânico Remoto** | Publicar `PANICO` em `.../comando/ambiente`. | O ESP32 entra em `INVASAO_SALA` imediatamente. | ☐ |
| 3.10 | **Reconexão Automática** | Desligar o roteador Wi-Fi por 10s e religar. | O log mostra "Wi-Fi desconectado" e depois "Wi-Fi reconectado" + "MQTT reconectado". | ☐ |

---

## 4. 🖥️ Testes do Dashboard Externo (Web/MQTT)

*Objetivo: Validar a interface web que consome o MQTT via WebSocket.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 4.1 | **Conexão do Dashboard** | Abrir o `index.html` no navegador. | O status "Online" aparece e os cards são preenchidos com os dados do ESP32. | ☐ |
| 4.2 | **Atualização em Tempo Real** | Abrir a porta do rack (sistema ARMADO). | O dashboard muda para vermelho e o log de eventos registra a invasão **instantaneamente**. | ☐ |
| 4.3 | **Botão "Atualizar Agora"** | Clicar no botão. | O dashboard envia `STATUS` e atualiza os dados (respeitando o rate limit). | ☐ |
| 4.4 | **Comando Reset via Dashboard** | Clicar em "Reset" e digitar o token no `prompt`. | Os contadores zeram no dashboard (menos invasões/rearmes). | ☐ |
| 4.5 | **Gráfico Histórico** | Observar o gráfico de temperatura/distância. | Os pontos são plotados e o histórico persiste ao recarregar a página (`localStorage`). | ☐ |
| 4.6 | **Detecção de Queda (LWT)** | Desligar o ESP32. | Em até 60s, o dashboard mostra "ESP32: PERDIDO!" (offline). | ☐ |

---

## 5. 🌐 Testes do Dashboard Local (SSE — v7.4)

*Objetivo: Validar a página de contingência servida pelo próprio ESP32.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 5.1 | **Acesso à Página** | Abrir `http://<IP-do-ESP32>:8080/` no navegador (mesma rede). | A página carrega com os cards de status. | ☐ |
| 5.2 | **SSE (Tempo Real)** | Abrir a porta do rack e observar o dashboard local. | O card "Porta" muda para "ABERTA" e fica vermelho **em tempo real**. | ☐ |
| 5.3 | **Fallback JSON** | Acessar `http://<IP-do-ESP32>:8080/api/status` no navegador. | Retorna um JSON com o status atual (útil para debug). | ☐ |
| 5.4 | **Histórico (Ring Buffer)** | Acessar `http://<IP-do-ESP32>:8080/api/hist`. | Retorna um JSON com os últimos 20 eventos registrados. | ☐ |
| 5.5 | **Reconexão do SSE** | Recarregar a página ou pausar/retomar a rede. | O `EventSource` reconecta sozinho e volta a atualizar. | ☐ |
| 5.6 | **Funciona Offline (Broker)** | Desligar o roteador (ou bloquear o broker). | O dashboard local **continua funcionando** (pois não depende de MQTT). | ☐ |

---

## 6. 🔄 Testes de OTA (Atualização Over-The-Air)

*Objetivo: Validar a atualização remota sem cabo USB.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 6.1 | **Descoberta do OTA** | No PlatformIO/Arduino IDE, olhar as portas de rede. | O dispositivo `rack-elo5-a1b2` aparece na lista. | ☐ |
| 6.2 | **Upload via OTA** | Fazer uma alteração mínima no código e enviar via rede. | O upload completa 100% e o ESP32 reinicia. | ☐ |
| 6.3 | **Persistência Pós-OTA** | Após o OTA, verificar o modo e os contadores no Monitor Serial. | O modo (ARMADO/DESARMADO) e contadores foram preservados na NVS. | ☐ |
| 6.4 | **Callback de Progresso** | Durante o upload, observar o Monitor Serial. | O log mostra "Atualizacao iniciada", "Progresso: X%" e "Atualizacao concluida". | ☐ |

---

## 7. 🔐 Testes de Segurança e Falhas (Edge Cases)

*Objetivo: Garantir que o sistema se comporta de forma segura em situações adversas.*

| # | Teste | Como fazer | Critério de Aceite | Status |
| :--- | :--- | :--- | :--- | :---: |
| 7.1 | **Acesso não autorizado (MQTT)** | Publicar `DESARMAR` sem HMAC (quando implementado). | O comando é ignorado (ou o sistema exige assinatura). | ☐ |
| 7.2 | **Falha do DHT22** | Desconectar o sensor DHT22 do GPIO 13. | Após 10s, o log avisa "DHT22 sem leitura" e o ambiente volta a `OK` (sem falsos alarmes). | ☐ |
| 7.3 | **Falha do RTC** | Desconectar a bateria do RTC DS3231. | O sistema usa NTP (se Wi-Fi on) ou millis (se off) e o `time_source` muda. | ☐ |
| 7.4 | **Falha do Broker MQTT** | Bloquear a porta 1883 no roteador. | O ESP32 tenta reconectar a cada 5s, mas o alarme e o dashboard local continuam funcionando. | ☐ |
| 7.5 | **Botão de Pânico em DESARMADO** | Desarmar o sistema e apertar o pânico. | O log avisa "Pânico ignorado: sistema DESARMADO" (evita fadiga de alarme). | ☐ |
| 7.6 | **Ring Buffer Cheio** | Executar `STATUS` mais de 100 vezes. | O `HIST` mostra apenas os últimos 100 eventos, sem crash. | ☐ |

---

## 📋 Como Registrar os Resultados

Sugiro criar uma planilha (Excel/Google Sheets) com as colunas:

`# | Teste | Data | Responsável | Resultado (OK/Falha) | Observações`

Isso ajuda a rastrear regressões quando você fizer atualizações no código (ex: ao implementar o HMAC, rodar novamente a seção 7.1).

---

## 🚀 Próximos Passos Após os Testes

1. **Se todos passarem:** o sistema está pronto para produção (com as credenciais trocadas, conforme a seção de Segurança do README).
2. **Se algum falhar:** isole o componente (ex: se o DHT22 falhar no teste 2.7, revise o watchdog em `_verificarAmbiente`).
3. **Implementar o HMAC (seção 7.1):** é o próximo upgrade de segurança planejado.