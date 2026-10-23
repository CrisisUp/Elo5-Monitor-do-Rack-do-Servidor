// =========================================================================
// SISTEMA DE MONITORAMENTO DE RACK - v6.4 (corrigido)
// Modularizado + Doxygen + Wi-Fi/MQTT + LWT + Retain + NVS + Comandos
// =========================================================================
// CHANGELOG
//   v6.4 - CORREÇÕES:
//          [1] Dupla publicação por STATUS (cache atualizado em _publicar).
//          [2] Rate limit de STATUS (2s) e RESET com token.
//          [3] LWT em tópico dedicado (rack/status/lwt); republica status
//              completo ao reconectar; log do motivo de desconexão.
//          [4] Dirty flag para NVS (não grava se nada mudou).
//          [5] Removida validação morta em LeitorRFID::ler().
//          [6] jsonEscape() + setBufferSize(512) no PubSubClient.
//          [7] Primeira leitura de PIR/LDR não gera borda falsa.
//          [8] SensorDHT simplificado (sem leitura dupla).
//          [9] Deduplicação de cartões na NVS.
//         [10] Documentação de pinos atualizada (DHT no 13).
//   v6.3 - LWT (Last Will): broker publica {"status":"offline"} se ESP32 cair.
//          Retain flag: novos assinantes recebem o último estado imediatamente.
//          NVS (Preferences): limiar, timeout e contadores persistem após reboot.
//          Comandos remotos via MQTT: tópico rack/comando aceita ARMAR/DESARMAR.
//   v6.2 - Integração Wi-Fi + MQTT: transições publicadas em broker público.
//   v6.1 - Doxygen; correção duplo update() do botão; ordem init I2C.
//   v6.0 - Refactor: classes encapsuladas, struct de sensores, máquina isolada.
//   v5.0 - Versão original em arquivo único.
// =========================================================================

// =========================================================================
// MAPEAMENTO DE HARDWARE - ESP32 DevKit
// =========================================================================
// [ OCUPADOS ]
//  GPIO | Módulo       | Função                    | Fio           | Observação
//  -----|--------------|---------------------------|---------------|----------------------
//   0   | LedRGB       | Canal B                   | Azul          | Strapping (boot) — ATENÇÃO
//   2   | Audio        | Buzzer D                  | Rosa          | Strapping (boot)
//   4   | LeitorRFID   | RST                       | Laranja       |
//   5   | LeitorRFID   | SS (SDA)                  | Rosa          |
//  13   | DHT22        | Dados (SDA)               | Amarelo       | Sensor temp/umid
//  14   | Botao        | Pushbutton                | Laranja       | INPUT_PULLUP
//  15   | Audio        | Buzzer E                  | Roxo Escuro   | Strapping (boot)
//  16   | Ultrassonico | ECHO                      | Laranja       |
//  17   | Ultrassonico | TRIG                      | Verde         |
//  18   | LeitorRFID   | SCK                       | Cinza         |
//  19   | LeitorRFID   | MISO                      | Verde         |
//  21   | DisplayOLED  | I2C SDA (OLED + RTC)      | Verde         | Wire.begin(21,22)
//  22   | DisplayOLED  | I2C SCL (OLED + RTC)      | Verde Claro   | Wire.begin(21,22)
//  23   | LeitorRFID   | MOSI                      | Azul Claro    |
//  25   | Porta        | Switch                    | Verde         | INPUT_PULLDOWN
//  27   | PIR          | Movimento                 | Verde Claro   |
//  32   | LedRGB       | Canal R                   | Marrom        |
//  33   | LedRGB       | Canal G                   | Verde         |
//  34   | LDR          | Leitura Analógica (AO)    | Verde         | Input-only
//
// [ LIVRES PARA EXPANSÃO ]
//  - GPIO 12 : Livre (Strapping — cuidado no boot)
//  - GPIO 26 : Livre (Ótimo para PWM/Saída)
//  - GPIO 35 : Livre (Apenas Entrada Analógica)
//  - GPIO 36 : Livre (Apenas Entrada Analógica)
//  - GPIO 39 : Livre (Apenas Entrada Analógica)
//
// [ PROIBIDOS ]
//  - GPIO 6 a 11 : Conectados à memória Flash (NUNCA USAR)
//  - GPIO 1 e 3  : Usados pela Serial USB (TX0/RX0)
//
// [ REGRAS DE USO ]
//  - Strapping pins (0, 2, 12, 15): não forçar estado contrário no boot.
//  - Input-only (34, 35, 36, 39): sem pull-up interno, sem PWM.
//  - I2C (21, 22): reservados ao barramento compartilhado OLED + RTC.
//  - Antes de adicionar periférico, verificar seção "LIVRES".
//  - Todo pino novo deve ser adicionado ao namespace `Pins` abaixo.
//  - ATENÇÃO: LED_B no GPIO 0 (strapping). Considere migrar para GPIO 26.
// =========================================================================

// =========================================================================
// ARQUITETURA DE SOFTWARE
// =========================================================================
//  Sensores → struct LeiturasSensores + Eventos → Alarme (única fonte de
//  verdade) → Atuadores (render) + NotificadorMQTT (observador/publicador).
//  NVS é persistência passiva. loop() só orquestra.
//
// REGRAS DE OURO
//  1. Sensores NÃO decidem nada — só medem e filtram.
//  2. Atuadores NÃO leem sensores — só consomem estado.
//  3. Alarme NÃO acessa hardware direto — recebe struct/eventos.
//  4. Eventos são "fatos", não "ações".
//  5. Toda transição é logada com nível (INFO/ALERT/CRIT).
//  6. loop() só orquestra.
//  7. MQTT é OBSERVADOR do Alarme.
//  8. NVS é persistência PASSIVA.
// =========================================================================

#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <vector>
#include <DHT.h>

// =========================================================================
// CONFIGURAÇÃO GLOBAL
// =========================================================================

namespace Pins {
  constexpr uint8_t RFID_SS   = 5;
  constexpr uint8_t RFID_RST  = 4;
  constexpr uint8_t RFID_SCK  = 18;
  constexpr uint8_t RFID_MISO = 19;
  constexpr uint8_t RFID_MOSI = 23;

  constexpr uint8_t PIR       = 27;
  constexpr uint8_t TRIG      = 17;
  constexpr uint8_t ECHO      = 16;
  constexpr uint8_t LDR_AO    = 34;

  constexpr uint8_t BOTAO     = 14;
  constexpr uint8_t SWITCH    = 25;

  constexpr uint8_t BUZZER_E  = 15;
  constexpr uint8_t BUZZER_D  = 2;

  constexpr uint8_t LED_R     = 32;
  constexpr uint8_t LED_G     = 33;
  constexpr uint8_t LED_B     = 0;
}

constexpr uint32_t PWM_FREQ       = 5000;
constexpr uint8_t  PWM_RESOLUTION = 8;
constexpr uint8_t  LARGURA_TELA   = 128;
constexpr uint8_t  ALTURA_TELA    = 64;
constexpr uint8_t  ENDERECO_OLED  = 0x3C;

// Filtros
constexpr int  LDR_LIMIAR_ACENDE = 2200;
constexpr int  LDR_LIMIAR_APAGA  = 1800;
constexpr unsigned long DEBOUNCE_LDR_MS = 800;

constexpr int   US_AMOSTRAS        = 5;
constexpr float US_DIST_MAX_VALIDA = 400.0f;

constexpr unsigned long DEBOUNCE_ESTADO_MS = 300;

constexpr unsigned long DEBOUNCE_BOTAO_MS       = 250;
constexpr unsigned long JANELA_CLIQUE_DUPLO_MS  = 400;
constexpr unsigned long DURACAO_CLIQUE_LONGO_MS = 3000;

constexpr unsigned long DEBOUNCE_PORTA_MS = 200;

constexpr unsigned long DURACAO_AVISO_BLOQUEIO_MS = 4000;

// Limites ambientais (ASHRAE + margem de segurança)
// Valores DEFAULT — podem ser sobrescritos pela NVS
constexpr float TEMP_ALERTA_C_DEFAULT    = 30.0f;
constexpr float TEMP_CRITICO_C_DEFAULT   = 35.0f;
constexpr float UMID_ALERTA_PCT_DEFAULT  = 65.0f;
constexpr float UMID_CRITICO_PCT_DEFAULT = 80.0f;

constexpr float TEMP_HISTERESE  = 2.0f;
constexpr float UMID_HISTERESE  = 5.0f;

// Taxa de subida de temperatura
constexpr float TAXA_ALERTA_C_POR_MIN = 1.0f;
constexpr unsigned long INTERVALO_AMOSTRA_TAXA_MS = 60000UL;

// Limiares efetivos (sobrescritos pela NVS no boot)
float limiarTempAlerta  = TEMP_ALERTA_C_DEFAULT;
float limiarTempCritico = TEMP_CRITICO_C_DEFAULT;
float limiarUmidAlerta  = UMID_ALERTA_PCT_DEFAULT;
float limiarUmidCritico = UMID_CRITICO_PCT_DEFAULT;

// Flags de estado ambiental
bool alertaTempAtivo  = false;
bool criticoTempAtivo = false;
bool alertaUmidAtivo  = false;
bool criticoUmidAtivo = false;
bool taxaTempAlerta   = false;

// Estado ambiental consolidado (exposto ao Alarme e ao JSON)
enum EstadoAmbiental { AMB_OK, AMB_ALERTA, AMB_CRITICO };
EstadoAmbiental estadoAmbiental = AMB_OK;

// Buzzer ambiental opcional (prioridade respeitada)
#define ALARME_SONORO_AMBIENTAL 0

// Amostragem da taxa de subida (uso interno de _verificarAmbiente)
float         _tempAnteriorTaxa  = -999.0f;
unsigned long _tempoAnteriorTaxa = 0;

// [4] Flag global de dirty para NVS (usada por Alarme e loop)
bool contadoresDirty = false;

const String CARTAO_AUTORIZADO_1 = "01020304";
const String CARTAO_AUTORIZADO_2 = "55667788";

constexpr unsigned long WARMUP_LDR_MS = 1500;
constexpr unsigned long WARMUP_PIR_MS = 1500;
constexpr unsigned long WARMUP_US_MS  = 1500;

std::vector<String> cartoesAutorizados;

// =========================================================================
// CONFIGURAÇÃO DE REDE (Wi-Fi + MQTT)
// =========================================================================
const char* WIFI_SSID     = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_BROKER   = "broker.emqx.io";
const int   MQTT_PORT     = 1883;

const char* MQTT_TOPIC_STATUS = "rack_monitor_ABCD1234/status";
const char* MQTT_TOPIC_LWT    = "rack_monitor_ABCD1234/status/lwt";
const char* MQTT_TOPIC_CMD    = "rack_monitor_ABCD1234/comando";
const char* MQTT_TOPIC_LOGS   = "rack_monitor_ABCD1234/logs";
const char* MQTT_TOPIC_HB     = "rack_monitor_ABCD1234/heartbeat"; 

const unsigned long MQTT_RETRY_MS = 5000;

// [2] Token para comandos destrutivos. TROQUE ESTE VALOR!
const char* MQTT_RESET_TOKEN = "rack2026";

// =========================================================================
// ENUMS E STRUCTS
// =========================================================================

enum ModoVigilante { ARMADO, DESARMADO };

enum EstadoSistema {
  SEGURO,
  AVISO_SALA,
  APROXIMACAO_RACK,
  EMERGENCIA_RACK,
  INVASAO_SALA,   // reservado (não atribuído atualmente)
  ACESSO_RFID
};

enum EventoBotao { BOTAO_NENHUM, BOTAO_CURTO, BOTAO_DUPLO, BOTAO_LONGO };

struct EventoRFID {
  bool   novo   = false;
  bool   valido = false;
  String uid    = "";
};

struct LeiturasSensores {
  bool  movimento   = false;
  bool  luzAcesa    = false;
  bool  portaAberta = false;
  float distancia   = 999.0f;
};

// =========================================================================
// PERIFÉRICOS
// =========================================================================
Adafruit_SSD1306 display(LARGURA_TELA, ALTURA_TELA, &Wire, -1);
RTC_DS3231       rtc;
MFRC522          rfid(Pins::RFID_SS, Pins::RFID_RST);

bool rtcDisponivel = false;

// =========================================================================
// MÓDULO: RELÓGIO
// =========================================================================
class Relogio {
public:
  bool begin() {
    if (rtc.begin()) {
      rtcDisponivel = true;
      if (rtc.lostPower()) {
        Serial.println(F("[WARN] RTC perdeu energia. Ajustando para hora de compilacao."));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      }
      return true;
    }
    rtcDisponivel = false;
    return false;
  }

  String timestamp() const {
    if (rtcDisponivel) {
      DateTime agora = rtc.now();
      char buf[20];
      sprintf(buf, "%02d/%02d %02d:%02d:%02d",
              agora.day(), agora.month(),
              agora.hour(), agora.minute(), agora.second());
      return String(buf);
    }
    unsigned long ms = millis();
    unsigned long s  = ms / 1000;
    unsigned long m  = s / 60;
    unsigned long h  = m / 60;
    s %= 60; m %= 60;
    char buf[12];
    sprintf(buf, "%02lu:%02lu:%02lu", h, m, s);
    return String(buf);
  }

  String timestampISO() const {
    if (rtcDisponivel) {
      DateTime agora = rtc.now();
      char buf[25];
      sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%02d",
              agora.year(), agora.month(), agora.day(),
              agora.hour(), agora.minute(), agora.second());
      return String(buf);
    }
    return String(millis());
  }

  bool disponivel() const { return rtcDisponivel; }
  RTC_DS3231& driver()    { return rtc; }
};

Relogio relogio;

// =========================================================================
// LOG CENTRALIZADO
// =========================================================================
void logSerial(const String &nivel, const String &tag, const String &mensagem) {
  Serial.print("[");
  Serial.print(relogio.timestamp());
  Serial.print("] [");
  Serial.print(nivel);
  Serial.print("]");
  if (nivel.length() == 4) Serial.print(" ");
  Serial.print(" [");
  Serial.print(tag);
  Serial.print("] ");
  Serial.println(mensagem);
}

// Protótipo (implementação no final, após NotificadorMQTT)
void enviarLogSite(const String &mensagem);

// =========================================================================
// MÓDULO: LDR
// =========================================================================
class LDR {
public:
  void begin() {
    pinMode(Pins::LDR_AO, INPUT);
    _estado = false;
    _tempoUltimaMudanca = 0;
    _tempoInicio = millis();
  }

  void update() {
    if (millis() - _tempoInicio < 1500) {
      _estado = false;
      return;
    }
    int leitura = analogRead(Pins::LDR_AO);
    if (!_estado && leitura < LDR_LIMIAR_APAGA) {
      if (millis() - _tempoUltimaMudanca >= DEBOUNCE_LDR_MS) {
        _estado = true;
        _tempoUltimaMudanca = millis();
      }
    } else if (_estado && leitura > LDR_LIMIAR_ACENDE) {
      if (millis() - _tempoUltimaMudanca >= DEBOUNCE_LDR_MS) {
        _estado = false;
        _tempoUltimaMudanca = millis();
      }
    }
  }

  bool estaAcesa() const { return _estado; }
  int  leituraBruta() const { return analogRead(Pins::LDR_AO); }

private:
  bool          _estado             = false;
  unsigned long _tempoUltimaMudanca = 0;
  unsigned long _tempoInicio        = 0;
};

LDR ldr;

// =========================================================================
// MÓDULO: ULTRASSÔNICO
// =========================================================================
class Ultrassonico {
public:
  void begin() {
    pinMode(Pins::TRIG, OUTPUT);
    pinMode(Pins::ECHO, INPUT);
    digitalWrite(Pins::TRIG, LOW);
    _tempoInicio = millis();
    _distancia   = 999.0f;
  }

  void update() {
    if (millis() - _tempoInicio < 1500) {
      _distancia = 999.0f;
      return;
    }
    float soma    = 0.0f;
    int   validas = 0;
    for (int i = 0; i < US_AMOSTRAS; i++) {
      float d = _medirBruto();
      if (d >= 2.0f && d < US_DIST_MAX_VALIDA) {
        soma += d;
        validas++;
      }
      delayMicroseconds(60);
    }
    if (validas > 0) _distancia = soma / validas;
  }

  float distancia() const { return _distancia; }

private:
  float _distancia = 999.0f;
  unsigned long _tempoInicio = 0;

  float _medirBruto() const {
    digitalWrite(Pins::TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(Pins::TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(Pins::TRIG, LOW);
    long duracao = pulseIn(Pins::ECHO, HIGH, 25000);
    float d = duracao * 0.0343f / 2.0f;
    return (d <= 0.0f) ? 999.0f : d;
  }
};

Ultrassonico ultrassonico;

// =========================================================================
// MÓDULO: PORTA
// =========================================================================
class Porta {
public:
  void begin() {
    pinMode(Pins::SWITCH, INPUT_PULLDOWN);
    _estadoFiltrado     = false;
    _brutoAnterior      = false;
    _tempoUltimaMudanca = 0;
  }

  void update() {
    bool bruto = (digitalRead(Pins::SWITCH) == HIGH);
    if (bruto != _brutoAnterior) {
      _tempoUltimaMudanca = millis();
      _brutoAnterior = bruto;
    }
    if ((millis() - _tempoUltimaMudanca) >= DEBOUNCE_PORTA_MS &&
        bruto != _estadoFiltrado) {
      _estadoFiltrado = bruto;
    }
  }

  bool estaAberta() const { return _estadoFiltrado; }

private:
  bool          _estadoFiltrado     = false;
  bool          _brutoAnterior      = false;
  unsigned long _tempoUltimaMudanca = 0;
};

Porta porta;

// =========================================================================
// MÓDULO: PIR
// =========================================================================
class PIR {
public:
  void begin() {
    pinMode(Pins::PIR, INPUT);
    _tempoInicio = millis();
  }

  void update() {
    if (millis() - _tempoInicio < 1500) {
      _movimento = false;
      return;
    }
    _movimento = (digitalRead(Pins::PIR) == HIGH);
  }

  bool detectaMovimento() const { return _movimento; }

private:
  bool          _movimento   = false;
  unsigned long _tempoInicio = 0;
};

PIR pir;

// =========================================================================
// MÓDULO: BOTÃO
// =========================================================================
class Botao {
public:
  void begin() {
    pinMode(Pins::BOTAO, INPUT_PULLUP);
    _brutoAnterior      = false;
    _filtrado           = false;
    _tempoUltimaMudanca = 0;
    _tempoPressionado   = 0;
    _cliques            = 0;
    _tempoUltimoClique  = 0;
    _tempoUltimaAcao    = 0;
  }

  EventoBotao update() {
    bool bruto = (digitalRead(Pins::BOTAO) == LOW);

    if (bruto != _brutoAnterior) {
      _tempoUltimaMudanca = millis();
      _brutoAnterior = bruto;
    }
    if ((millis() - _tempoUltimaMudanca) < DEBOUNCE_BOTAO_MS) return BOTAO_NENHUM;
    if (bruto == _filtrado) return BOTAO_NENHUM;

    _filtrado = bruto;

    if (_filtrado) {
      _tempoPressionado = millis();
      return BOTAO_NENHUM;
    }

    unsigned long duracao = millis() - _tempoPressionado;
    if (millis() - _tempoUltimaAcao < DEBOUNCE_BOTAO_MS) return BOTAO_NENHUM;

    if (duracao >= DURACAO_CLIQUE_LONGO_MS) {
      _tempoUltimaAcao = millis();
      _cliques = 0;
      return BOTAO_LONGO;
    }

    if (duracao >= 50) {
      _cliques++;
      _tempoUltimoClique = millis();
    }

    if (_cliques >= 2 && (millis() - _tempoUltimoClique < JANELA_CLIQUE_DUPLO_MS)) {
      _tempoUltimaAcao = millis();
      _cliques = 0;
      return BOTAO_DUPLO;
    }

    if (_cliques == 1 && (millis() - _tempoUltimoClique > JANELA_CLIQUE_DUPLO_MS)) {
      _tempoUltimaAcao = millis();
      _cliques = 0;
      return BOTAO_CURTO;
    }

    return BOTAO_NENHUM;
  }

private:
  bool          _brutoAnterior      = false;
  bool          _filtrado           = false;
  unsigned long _tempoUltimaMudanca = 0;
  unsigned long _tempoPressionado   = 0;
  int           _cliques            = 0;
  unsigned long _tempoUltimoClique  = 0;
  unsigned long _tempoUltimaAcao    = 0;
};

Botao botao;

// =========================================================================
// MÓDULO: RFID  [5] validação removida do leitor
// =========================================================================
class LeitorRFID {
public:
  void begin() {
    SPI.begin();
    rfid.PCD_Init();
  }

  EventoRFID ler() {
    EventoRFID ev;
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return ev;

    String uid = "";
    for (uint8_t i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) uid += "0";
      uid += String(rfid.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();

    ev.novo   = true;
    ev.uid    = uid;
    ev.valido = false;   // validação fica em Alarme::_tratarRFID

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return ev;
  }
};

LeitorRFID leitorRFID;

// =========================================================================
// MÓDULO: LED RGB
// =========================================================================
class LedRGB {
public:
  void begin() {
    ledcAttach(Pins::LED_R, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(Pins::LED_G, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(Pins::LED_B, PWM_FREQ, PWM_RESOLUTION);
    definirCor(0, 0, 0);
  }

  void definirCor(int r, int g, int b) {
    ledcWrite(Pins::LED_R, 255 - constrain(r, 0, 255));
    ledcWrite(Pins::LED_G, 255 - constrain(g, 0, 255));
    ledcWrite(Pins::LED_B, 255 - constrain(b, 0, 255));
  }

  void selfTest() {
    definirCor(255, 0, 0); delay(150);
    definirCor(0, 255, 0); delay(150);
    definirCor(0, 0, 255); delay(150);
    definirCor(0, 0, 0);
  }

  void atualizar(ModoVigilante modo, EstadoSistema estado,
                 bool bloqueioAtivo, unsigned long &tempoUltimoPisca,
                 bool &estadoPisca) {
    unsigned long agora = millis();

    if (bloqueioAtivo) {
      if ((agora / 250) % 2 == 0) definirCor(255, 200, 0);
      else                         definirCor(0, 0, 0);
      return;
    }

    if (modo == DESARMADO) {
      float fase   = (agora % 2000) / 2000.0f;
      int   brilho = (int)(127.5f * (1.0f + sin(fase * 2.0f * PI)));
      definirCor(0, 0, brilho);
      return;
    }

    switch (estado) {
      case SEGURO:
      case ACESSO_RFID:      definirCor(0, 255, 0);   break;
      case AVISO_SALA:       definirCor(255, 200, 0); break;
      case APROXIMACAO_RACK: definirCor(255, 100, 0); break;
      case INVASAO_SALA:
        if (agora - tempoUltimoPisca >= 250) {
          estadoPisca      = !estadoPisca;
          tempoUltimoPisca = agora;
        }
        definirCor(estadoPisca ? 255 : 0, 0, 0);
        break;
      case EMERGENCIA_RACK:  definirCor(255, 0, 0);   break;
    }
  }
};

LedRGB led;

// =========================================================================
// MÓDULO: ÁUDIO
// =========================================================================
class Audio {
public:
  void begin() {
    pinMode(Pins::BUZZER_E, OUTPUT);
    pinMode(Pins::BUZZER_D, OUTPUT);
    parar();
  }

  void parar() {
    noTone(Pins::BUZZER_E);
    noTone(Pins::BUZZER_D);
  }

  void beepCurto(int freq = 1200, int dur = 100) { tone(Pins::BUZZER_E, freq, dur); }

  void beepErro() {
    tone(Pins::BUZZER_D, 500, 200); delay(250);
    tone(Pins::BUZZER_D, 400, 200);
  }

  void beepSucesso() {
    tone(Pins::BUZZER_E, 800, 100);  delay(120);
    tone(Pins::BUZZER_E, 600, 150);
  }

  void tocarMelodia(EstadoSistema estado, unsigned long &tempoUltimoBip) {
    unsigned long agora = millis();
    switch (estado) {
      case SEGURO:
      case ACESSO_RFID: parar(); break;
      case AVISO_SALA:
        if (agora - tempoUltimoBip >= 700) {
          tone(Pins::BUZZER_E, 523, 100);
          tempoUltimoBip = agora;
        }
        break;
      case APROXIMACAO_RACK:
        if (agora - tempoUltimoBip >= 400) {
          tone(Pins::BUZZER_E, 750, 80);
          tempoUltimoBip = agora;
        }
        break;
      case INVASAO_SALA:
        if (agora - tempoUltimoBip >= 25) {
          static int freq = 400, passo = 25;
          tone(Pins::BUZZER_D, freq, 20);
          freq += passo;
          if (freq >= 900 || freq <= 400) passo = -passo;
          tempoUltimoBip = agora;
        }
        break;
      case EMERGENCIA_RACK:
        if (agora - tempoUltimoBip >= 160) {
          static bool alt = false;
          tone(Pins::BUZZER_D, alt ? 900 : 1300, 140);
          alt = !alt;
          tempoUltimoBip = agora;
        }
        break;
    }
  }

  void avisoBloqueio(unsigned long &tempoUltimoBip) {
    unsigned long agora = millis();
    if (agora - tempoUltimoBip >= 800) {
      tone(Pins::BUZZER_D, 500, 100);
      tempoUltimoBip = agora;
    }
  }
};

Audio audio;

// =========================================================================
// MÓDULO: DISPLAY OLED
// =========================================================================
class DisplayOLED {
public:
  void begin() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, ENDERECO_OLED)) {
      Serial.println(F("[ERRO] Falha ao inicializar SSD1306"));
    }
    display.clearDisplay();
    display.display();
  }

  void mostrarStatus(EstadoSistema estado, ModoVigilante modo,
                     const LeiturasSensores &s,
                     const String &uid, bool cartaoOk,
                     unsigned int contAberturas, unsigned int contInvasoes) {
    display.clearDisplay();
    String linha1, linha2;
    int icone = 1;

    switch (estado) {
      case SEGURO:
        linha1 = "VIGILANTE ARMADO"; linha2 = "Porta Fechada"; icone = 1;
        break;
      case ACESSO_RFID:
        linha1 = cartaoOk ? "ACESSO LIBERADO" : "ACESSO NEGADO!";
        linha2 = "UID: " + uid;
        icone  = cartaoOk ? 1 : 3;
        break;
      case AVISO_SALA:
        linha1 = "AVISO: Movimento"; linha2 = "Rack Fechado"; icone = 2;
        break;
      case APROXIMACAO_RACK:
        linha1 = "APROXIMACAO";
        linha2 = "Dist: " + String(s.distancia, 1) + "cm";
        icone  = 2;
        break;
      case INVASAO_SALA:
        linha1 = "INVASAO NA SALA!";
        linha2 = "Mov+Luz | #" + String(contInvasoes);
        icone  = 3;
        break;
      case EMERGENCIA_RACK:
        linha1 = "EMERGENCIA: RACK!";
        linha2 = "Porta Aberta #" + String(contAberturas);
        icone  = 3;
        break;
    }

    _desenharIcone(icone, 2, 2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(24, 4);
    display.println(linha1);
    display.drawLine(0, 20, 128, 20, SSD1306_WHITE);
    display.setCursor(0, 26);
    display.println(linha2);
    display.drawLine(0, 46, 128, 46, SSD1306_WHITE);
    display.setCursor(0, 52);
    display.print(relogio.timestamp());
    display.print(" AB:");
    display.print(contAberturas);
    display.display();
  }

  void mostrarDesarmado(unsigned long restanteMs, const String &uid,
                        unsigned int movManutencao) {
    display.clearDisplay();
    _desenharIcone(4, 2, 2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(24, 4);
    display.println("MANUTENCAO");
    display.drawLine(0, 20, 128, 20, SSD1306_WHITE);

    unsigned long min = restanteMs / 60000UL;
    unsigned long seg = (restanteMs % 60000UL) / 1000UL;

    display.setCursor(0, 26);
    display.print("Rearme em: ");
    display.print(min); display.print("m");
    display.print(seg); display.print("s");

    display.setCursor(0, 36);
    display.print("UID: "); display.print(uid);

    display.drawLine(0, 46, 128, 46, SSD1306_WHITE);
    display.setCursor(0, 52);
    display.print(relogio.timestamp());
    display.print("  MANUT:"); display.print(movManutencao);
    display.display();
  }

  void mostrarBloqueio(const String &motivo, unsigned int totalBloqueios,
                       const String &uid) {
    display.clearDisplay();
    _desenharIcone(5, 2, 2);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(24, 4);
    display.println("REARME BLOQUEADO");
    display.drawLine(0, 20, 128, 20, SSD1306_WHITE);
    display.setCursor(0, 26);
    display.println(motivo);
    display.setCursor(0, 38);
    display.print("Bloqueios: "); display.print(totalBloqueios);
    display.drawLine(0, 46, 128, 46, SSD1306_WHITE);
    display.setCursor(0, 52);
    display.print("UID: "); display.print(uid);
    display.display();
  }

private:
  void _desenharIcone(int tipo, int x, int y) {
    switch (tipo) {
      case 1:
        display.drawRect(x, y + 6, 12, 9, SSD1306_WHITE);
        display.fillRect(x + 2, y + 8, 8, 5, SSD1306_WHITE);
        display.drawCircle(x + 6, y + 6, 4, SSD1306_WHITE);
        break;
      case 2:
        display.drawTriangle(x + 8, y, x, y + 14, x + 16, y + 14, SSD1306_WHITE);
        display.drawLine(x + 8, y + 5, x + 8, y + 9, SSD1306_WHITE);
        display.drawPixel(x + 8, y + 12, SSD1306_WHITE);
        break;
      case 3:
        display.drawRect(x, y + 6, 12, 9, SSD1306_WHITE);
        display.drawRect(x + 2, y + 8, 8, 5, SSD1306_WHITE);
        display.drawCircle(x + 6, y + 3, 4, SSD1306_WHITE);
        display.fillRect(x + 2, y + 2, 5, 4, SSD1306_BLACK);
        display.drawLine(x + 19, y + 2, x + 19, y + 8, SSD1306_WHITE);
        display.drawPixel(x + 19, y + 11, SSD1306_WHITE);
        break;
      case 4:
        display.drawTriangle(x + 8, y, x, y + 14, x + 16, y + 14, SSD1306_WHITE);
        display.drawLine(x + 4, y + 7, x + 8, y + 11, SSD1306_WHITE);
        display.drawLine(x + 8, y + 11, x + 12, y + 5, SSD1306_WHITE);
        break;
      case 5:
        display.drawRect(x, y + 2, 12, 16, SSD1306_WHITE);
        display.fillRect(x + 8, y + 9, 2, 3, SSD1306_WHITE);
        display.drawLine(x + 15, y + 5, x + 15, y + 12, SSD1306_WHITE);
        display.drawPixel(x + 15, y + 15, SSD1306_WHITE);
        break;
    }
  }
};

DisplayOLED oled;

// =========================================================================
// MÓDULO: DHT22  [8] simplificado
// =========================================================================
#define DHT_PIN     13
#define DHT_TYPE    DHT22

class SensorDHT {
public:
  DHT dht;

  SensorDHT() : dht(DHT_PIN, DHT_TYPE) {}

  void begin() {
    dht.begin();
    logSerial("INFO", "DHT", "Sensor DHT22 inicializado no GPIO 13.");
  }

  float temperatura() {
    float t = dht.readTemperature();
    return isnan(t) ? -999.0f : t;
  }

  float umidade() {
    float u = dht.readHumidity();
    return isnan(u) ? -999.0f : u;
  }
};

SensorDHT dht;

// =========================================================================
// MÓDULO: ALARME (máquina de estados)
// =========================================================================
class Alarme {
public:
  ModoVigilante modo           = ARMADO;
  EstadoSistema estadoAtual    = SEGURO;
  EstadoSistema estadoAnterior = SEGURO;
  EstadoSistema estadoCandidato      = SEGURO;
  unsigned long tempoEstadoCandidato = 0;

  unsigned int  contadorDesarmes             = 0;
  unsigned int  contadorRearmesBloqueados    = 0;
  unsigned int  contadorAberturasPorta       = 0;
  unsigned int  contadorInvasoes             = 0;
  unsigned int  contadorAproximacoes         = 0;
  unsigned int  contadorAcessosValidos       = 0;
  unsigned int  contadorMovimentosManutencao = 0;

  unsigned long timeoutDesarme       = 30UL * 60UL * 1000UL;
  unsigned long tempoInicioDesarme   = 0;
  bool          aviso5minDisparado   = false;
  String        uidUltimoDesarme     = "";
  String        uidUltimoRearme      = "";
  unsigned long duracaoUltimoDesarme = 0;
  unsigned long ultimoUIDtempoRFID   = 0;

  String        motivoBloqueio      = "";
  unsigned long tempoBloqueioAtivo  = 0;

  bool resetContadoresPendente = false;
  bool modoTestePendente       = false;
  bool audioPararPendente      = false;

  float limiarDistancia = 50.0f;

  void update(const LeiturasSensores &s, EventoRFID rfidEv, EventoBotao botaoEv,
              Porta &porta, Audio &audio) {
    _logBordas(s);
    _logPorta(porta);

    if (rfidEv.novo) {
      ultimoUIDtempoRFID = millis();
      _tratarRFID(rfidEv, porta, audio);
    }

    if (botaoEv != BOTAO_NENHUM) {
      _tratarBotao(botaoEv, porta, audio);
    }

    if (modo == DESARMADO) {
      unsigned long decorrido = millis() - tempoInicioDesarme;
      unsigned long restante  = (timeoutDesarme > decorrido) ? (timeoutDesarme - decorrido) : 0;

      if (!aviso5minDisparado && restante <= 5UL * 60UL * 1000UL &&
          timeoutDesarme > 5UL * 60UL * 1000UL) {
        aviso5minDisparado = true;
        logSerial("WARN", "VIGILANTE", "AVISO: 5 minutos para rearme automatico!");
        audio.beepCurto(1500, 200);
      }

      if (decorrido >= timeoutDesarme) _rearmarPorTimeout(porta, audio);
    }

    if (motivoBloqueio.length() > 0 &&
        (millis() - tempoBloqueioAtivo) > DURACAO_AVISO_BLOQUEIO_MS) {
      motivoBloqueio = "";
    }

    if (modo == ARMADO) _atualizarEstados(s, rfidEv);
  }

  bool bloqueioAtivo() const { return motivoBloqueio.length() > 0; }

  bool armarViaSerial(Porta &porta, Audio &audio) {
    if (modo == ARMADO) return false;
    return _tentarArmar(porta, audio);
  }

  String nomeModo() const { return (modo == ARMADO) ? "ARMADO" : "DESARMADO"; }
  String nomeEstadoAtual() const { return _nomeEstado(estadoAtual); }

private:
  bool _estadoAnteriorPIR   = false;
  bool _estadoAnteriorLDR   = false;
  bool _portaEstadoAnterior = false;
  bool _primeiraLeitura     = true;   // [7]

  void _logBordas(const LeiturasSensores &s) {
    // [7] Na primeira leitura, apenas registra o estado, sem gerar borda falsa
    if (_primeiraLeitura) {
      _estadoAnteriorPIR = s.movimento;
      _estadoAnteriorLDR = s.luzAcesa;
      _primeiraLeitura   = false;
      return;
    }

    if (s.movimento != _estadoAnteriorPIR) {
      if (modo == DESARMADO && s.movimento) {
        contadorMovimentosManutencao++;
        contadoresDirty = true;   // [4]
        logSerial("INFO", "MANUT", "Movimento em modo manutencao (auditoria).");
      } else {
        logSerial("INFO", "SENSOR",
          s.movimento ? "Movimento detectado pelo PIR." : "Movimento cessou.");
      }
      _estadoAnteriorPIR = s.movimento;
    }
    if (s.luzAcesa != _estadoAnteriorLDR) {
      logSerial("INFO", "SENSOR",
        s.luzAcesa ? "Luzes da sala ACESAS." : "Luzes da sala APAGADAS.");
      _estadoAnteriorLDR = s.luzAcesa;
    }
  }

  void _logPorta(Porta &porta) {
    bool aberta = porta.estaAberta();
    if (aberta != _portaEstadoAnterior) {
      if (aberta) {
        contadorAberturasPorta++;
        contadoresDirty = true;   // [4]
        if (modo == ARMADO) {
          logSerial("ALERT", "PORTA", "!!! PORTA ABERTA (total: " + String(contadorAberturasPorta) + ") !!!");
          enviarLogSite("ALERTA: Porta do Rack ABERTA!");
        } else {
          logSerial("INFO", "PORTA", "Porta aberta em modo manutencao (total: " + String(contadorAberturasPorta) + ")");
          enviarLogSite("Porta aberta (manutencao).");
        }
      } else {
        logSerial("INFO", "PORTA", "Porta fechada.");
        enviarLogSite("Porta do Rack FECHADA.");
      }
      _portaEstadoAnterior = aberta;
    }
  }

  void _tratarRFID(const EventoRFID &ev, Porta &porta, Audio &audio) {
    uidUltimoDesarme = ev.uid;

    bool valido = false;
    for (const String &card : cartoesAutorizados) {
      if (card == ev.uid) { valido = true; break; }
    }

    if (valido) {
      enviarLogSite("Cartao Autorizado: " + ev.uid);
      contadorAcessosValidos++;
      contadoresDirty = true;   // [4]
      logSerial("INFO", "RFID", "Cartao AUTORIZADO: " + ev.uid);
      _alternarModo(ev.uid, porta, audio);
    } else {
      logSerial("WARN", "RFID", "Acesso NEGADO! Cartao nao cadastrado: " + ev.uid);
      enviarLogSite("Cartao NEGADO: " + ev.uid);
      audio.beepErro();
    }
  }

  void _tratarBotao(EventoBotao ev, Porta &porta, Audio &audio) {
    switch (ev) {
      case BOTAO_LONGO:
        logSerial("INFO", "BOTAO", "Clique LONGO: rearme forcado.");
        if (modo == DESARMADO) _tentarArmar(porta, audio);
        else logSerial("WARN", "BOTAO", "Ja esta ARMADO.");
        audio.beepCurto(1000, 200);
        break;
      case BOTAO_DUPLO:
        logSerial("INFO", "BOTAO", "Clique DUPLO: modo teste.");
        modoTestePendente = true;
        break;
      case BOTAO_CURTO:
        logSerial("INFO", "BOTAO", "Clique CURTO: reset operacional.");
        resetContadoresPendente = true;
        audio.beepCurto(1200, 100);
        break;
      default: break;
    }
  }

  void _alternarModo(const String &uid, Porta &porta, Audio &audio) {
    if (modo == ARMADO) {
      modo               = DESARMADO;
      tempoInicioDesarme = millis();
      uidUltimoDesarme   = uid;
      aviso5minDisparado = false;
      contadorDesarmes++;
      contadoresDirty    = true;   // [4]
      motivoBloqueio     = "";
      logSerial("INFO", "VIGILANTE",
        "DESARMADO por UID: " + uid +
        " | Timeout: " + String(timeoutDesarme / 60000) + " min");
      audio.beepSucesso();
    } else {
      _tentarArmar(porta, audio);
    }
  }

  bool _tentarArmar(Porta &porta, Audio &audio) {
    if (porta.estaAberta()) {
      contadorRearmesBloqueados++;
      contadoresDirty    = true;   // [4]
      motivoBloqueio     = "Feche a porta!";
      tempoBloqueioAtivo = millis();

      logSerial("WARN", "VIGILANTE",
        "REARME BLOQUEADO: porta aberta | " +
        String(contadorRearmesBloqueados) + " bloqueio(s) ate agora.");
      logSerial("WARN", "VIGILANTE",
        "Feche a porta e passe o cartao novamente para armar.");
      enviarLogSite("REARME BLOQUEADO: porta aberta!");
      audio.beepErro();
      return false;
    }
    duracaoUltimoDesarme = (millis() - tempoInicioDesarme) / 1000UL;
    modo                = ARMADO;
    uidUltimoRearme     = uidUltimoDesarme;
    motivoBloqueio      = "";
    logSerial("INFO", "VIGILANTE",
      "REARMADO | Duracao do desarme: " + String(duracaoUltimoDesarme) + "s");
    enviarLogSite("Sistema REARMADO com sucesso.");
    audio.beepSucesso();
    estadoAtual     = SEGURO;
    estadoAnterior  = SEGURO;
    estadoCandidato = SEGURO;
    return true;
  }

  void _rearmarPorTimeout(Porta &porta, Audio &audio) {
    if (porta.estaAberta()) {
      logSerial("WARN", "VIGILANTE",
        "TIMEOUT IGNORADO: porta aberta impede rearme automatico. Reiniciando contagem.");
      tempoInicioDesarme = millis();
      aviso5minDisparado = false;
      return;
    }
    duracaoUltimoDesarme = (millis() - tempoInicioDesarme) / 1000UL;
    modo                = ARMADO;
    uidUltimoRearme     = "TIMEOUT";
    logSerial("WARN", "VIGILANTE",
      "REARME AUTOMATICO por timeout | Duracao: " +
      String(duracaoUltimoDesarme) + "s");
    audio.beepCurto(1500, 200); delay(250);
    audio.beepCurto(2000, 300);
    estadoAtual     = SEGURO;
    estadoAnterior  = SEGURO;
    estadoCandidato = SEGURO;
  }

  void _atualizarEstados(const LeiturasSensores &s, const EventoRFID &rfidEv) {
    EstadoSistema candidato;
    bool rfidRecente = rfidEv.novo || (millis() - ultimoUIDtempoRFID < 2500);

    if (rfidRecente) {
      candidato = ACESSO_RFID;
    }
    else if (s.portaAberta) {
      candidato = EMERGENCIA_RACK;
    }
    else if (s.movimento) {
      candidato = AVISO_SALA;
    }
    else if (s.distancia < limiarDistancia && s.distancia > 0) {
      candidato = APROXIMACAO_RACK;
    }
    else {
      candidato = SEGURO;
    }

    bool critico = (candidato == EMERGENCIA_RACK);
    bool urgente = (candidato == ACESSO_RFID);

    if (critico || urgente) {
      estadoAtual          = candidato;
      estadoCandidato      = candidato;
      tempoEstadoCandidato = millis();
    } else {
      if (candidato != estadoCandidato) {
        estadoCandidato      = candidato;
        tempoEstadoCandidato = millis();
      } else if (candidato != estadoAtual &&
                 (millis() - tempoEstadoCandidato >= DEBOUNCE_ESTADO_MS)) {
        estadoAtual = candidato;
      }
    }

    if (estadoAtual != estadoAnterior) {
      String nivel = "INFO";
      if (estadoAtual == EMERGENCIA_RACK) nivel = "CRIT";
      else if (estadoAtual == APROXIMACAO_RACK) nivel = "ALERT";
      else if (estadoAtual == AVISO_SALA)       nivel = "WARN";

      logSerial(nivel, "ESTADO",
        "Transicao: " + _nomeEstado(estadoAnterior) + " -> " + _nomeEstado(estadoAtual));
      enviarLogSite("Estado alterado: " + _nomeEstado(estadoAnterior) + " -> " + _nomeEstado(estadoAtual));

      if (estadoAtual == EMERGENCIA_RACK) {
        contadorInvasoes++;
        contadoresDirty = true;   // [4]
        logSerial("CRIT", "SEGURANCA", "!!! INVASAO FISICA DETECTADA NO RACK !!!");
      }
      if (estadoAtual == APROXIMACAO_RACK) {
        contadorAproximacoes++;
        contadoresDirty = true;   // [4]
      }

      audioPararPendente = true;
      estadoAnterior = estadoAtual;
    }
  }

  static String _nomeEstado(EstadoSistema e) {
    switch (e) {
      case SEGURO:           return "NORMAL";
      case ACESSO_RFID:      return "ACESSO RFID";
      case AVISO_SALA:       return "AVISO SALA";
      case APROXIMACAO_RACK: return "APROXIMACAO RACK";
      case EMERGENCIA_RACK:  return "EMERGENCIA RACK";
      case INVASAO_SALA:     return "INVASAO SALA";
      default:               return "DESCONHECIDO";
    }
  }
};

Alarme alarme;

// =========================================================================
// MÓDULO: NVS (Persistência em Flash)
// =========================================================================
Preferences prefs;

void salvarConfig() {
  prefs.begin("rack", false);
  prefs.putFloat("limiar",  alarme.limiarDistancia);
  prefs.putULong("timeout", alarme.timeoutDesarme);
  prefs.putUInt ("invasoes",  alarme.contadorInvasoes);
  prefs.putUInt ("aberturas", alarme.contadorAberturasPorta);
  prefs.putUInt ("acessos",   alarme.contadorAcessosValidos);
  prefs.putUInt ("desarmes",  alarme.contadorDesarmes);
  prefs.putUInt ("rearmes",   alarme.contadorRearmesBloqueados);
  prefs.putFloat("t_alert", limiarTempAlerta);
  prefs.putFloat("t_crit",  limiarTempCritico);
  prefs.putFloat("u_alert", limiarUmidAlerta);
  prefs.putFloat("u_crit",  limiarUmidCritico);
  prefs.end();
  logSerial("INFO", "NVS", "Configuracoes salvas na flash.");
}

// [9] Deduplicação de cartões
void carregarCartoes() {
  cartoesAutorizados.clear();

  auto adicionarSeNovo = [](const String &uid) {
    for (const String &c : cartoesAutorizados) {
      if (c == uid) return;
    }
    cartoesAutorizados.push_back(uid);
  };

  prefs.begin("rack", true);

  adicionarSeNovo(CARTAO_AUTORIZADO_1);
  adicionarSeNovo(CARTAO_AUTORIZADO_2);

  int total = prefs.getInt("total_cards", 0);
  for (int i = 0; i < total; i++) {
    String chave = "card_" + String(i);
    String uid = prefs.getString(chave.c_str(), "");
    if (uid.length() > 0) {
      adicionarSeNovo(uid);
      logSerial("INFO", "NVS", "Cartao carregado: " + uid);
    }
  }
  prefs.end();
  logSerial("INFO", "NVS", "Total de cartoes carregados: " + String(cartoesAutorizados.size()));
}

void carregarConfig() {
  prefs.begin("rack", true);
  alarme.limiarDistancia        = prefs.getFloat("limiar", 50.0f);
  alarme.timeoutDesarme         = prefs.getULong("timeout", 30UL * 60UL * 1000UL);
  alarme.contadorInvasoes       = prefs.getUInt ("invasoes", 0);
  alarme.contadorAberturasPorta = prefs.getUInt ("aberturas", 0);
  alarme.contadorAcessosValidos = prefs.getUInt ("acessos", 0);
  alarme.contadorDesarmes       = prefs.getUInt ("desarmes", 0);
  alarme.contadorRearmesBloqueados = prefs.getUInt("rearmes", 0);
  limiarTempAlerta  = prefs.getFloat("t_alert", TEMP_ALERTA_C_DEFAULT);
  limiarTempCritico = prefs.getFloat("t_crit",  TEMP_CRITICO_C_DEFAULT);
  limiarUmidAlerta  = prefs.getFloat("u_alert", UMID_ALERTA_PCT_DEFAULT);
  limiarUmidCritico = prefs.getFloat("u_crit",  UMID_CRITICO_PCT_DEFAULT);
  prefs.end();
  logSerial("INFO", "NVS", "Configuracoes carregadas da flash.");
}

// =========================================================================
// MÓDULO: NOTIFICADOR MQTT
// =========================================================================
class NotificadorMQTT {
public:
  void begin() {
    Serial.print(F("[MQTT] Conectando ao WiFi"));
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
    unsigned long inicio = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - inicio) < 15000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F(" OK!"));
      Serial.print(F("[MQTT] IP: ")); Serial.println(WiFi.localIP());
    } else {
      Serial.println(F(" FALHOU (WiFi indisponivel, seguindo offline)"));
    }

    _client.setServer(MQTT_BROKER, MQTT_PORT);
    _client.setBufferSize(512);   // [6]

    _client.setCallback([this](char* topic, byte* payload, unsigned int length) {
      _tratarComando(topic, payload, length);
    });
  }

  void update() {
    // 1. Wi-Fi
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - _ultimoTempoReconexao > MQTT_RETRY_MS) {
        _ultimoTempoReconexao = millis();
        WiFi.reconnect();
      }
      return;
    }

    // 2. MQTT
    if (!_client.connected()) {
      // [3] log de desconexão
      if (_estavaConectado) {
        logSerial("WARN", "MQTT", "Desconectado do broker (rc=" + String(_client.state()) + ").");
        _estavaConectado = false;
      }

      if (millis() - _ultimoTempoReconexao > MQTT_RETRY_MS) {
        _ultimoTempoReconexao = millis();
        _client.disconnect();
        String clientId = "RackESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        Serial.print(F("[MQTT] Conectando ao broker... "));

        // [3] LWT em tópico dedicado
        if (_client.connect(clientId.c_str(), nullptr, nullptr,
                            MQTT_TOPIC_LWT, 1, true, "{\"status\":\"offline\"}")) {
          Serial.println(F("OK"));
          _estavaConectado = true;
          _client.publish(MQTT_TOPIC_LWT, "{\"status\":\"online\"}", true);
          _client.subscribe(MQTT_TOPIC_CMD);
          logSerial("INFO", "MQTT", String("Inscrito em: ") + MQTT_TOPIC_CMD);

          // [3] republica status completo imediatamente após conectar
          _publicar(true);
        } else {
          Serial.print(F("Falha rc=")); Serial.println(_client.state());
        }
      }
      return;
    }

    _client.loop();

    // 3. Publica transição (cache atualizado dentro de _publicar [1])
    if (alarme.estadoAtual != _ultimoEstadoPublicado ||
        alarme.modo       != _ultimoModoPublicado) {
      _publicar();
    }

    // Heartbeat
    if (millis() - _ultimoHeartbeat > INTERVALO_HEARTBEAT_MS) {
      _ultimoHeartbeat = millis();
      String hbPayload = "{\"ts\":\"" + relogio.timestampISO() +
                         "\",\"uptime_s\":" + String(millis() / 1000) + "}";
      _client.publish(MQTT_TOPIC_HB, hbPayload.c_str());
    }
  }

  bool estaConectado() {
    return _client.connected();
  }

  void publicarLog(const String &payload) {
    _client.publish(MQTT_TOPIC_LOGS, payload.c_str());
  }

private:
  WiFiClient     _wifiClient;
  PubSubClient   _client{_wifiClient};
  unsigned long  _ultimoTempoReconexao = 0;
  unsigned long  _ultimoHeartbeat      = 0;
  const unsigned long INTERVALO_HEARTBEAT_MS = 30000UL;
  EstadoSistema  _ultimoEstadoPublicado = SEGURO;
  ModoVigilante  _ultimoModoPublicado   = ARMADO;

  bool           _estavaConectado       = false;         // [3]
  unsigned long  _ultimoStatusForcado   = 0;             // [2]
  const unsigned long STATUS_RATE_LIMIT_MS = 2000UL;     // [2]

  void _publicar(bool forcar = false) {
    if (!forcar && alarme.estadoAtual == _ultimoEstadoPublicado &&
        alarme.modo == _ultimoModoPublicado) {
      return;
    }

    bool ativo = (alarme.modo == ARMADO &&
                  (alarme.estadoAtual == EMERGENCIA_RACK ||
                   alarme.estadoAtual == INVASAO_SALA));

    String payload = "{\"estado\":\"";
    payload += alarme.nomeEstadoAtual();
    payload += "\",\"modo\":\"";
    payload += alarme.nomeModo();
    payload += "\",\"ts\":\"";
    payload += relogio.timestampISO();
    payload += "\",\"porta\":";
    payload += (porta.estaAberta() ? "true" : "false");
    payload += ",\"luz\":";
    payload += (ldr.estaAcesa() ? "true" : "false");
    payload += ",\"dist_cm\":";
    payload += String(ultrassonico.distancia(), 1);
    payload += ",\"ativo\":";
    payload += (ativo ? "true" : "false");

    payload += ",\"ip\":\"";
    payload += WiFi.localIP().toString();
    payload += "\",\"uid\":\"";
    payload += (alarme.uidUltimoDesarme.length() > 0 ? alarme.uidUltimoDesarme : "-");
    payload += "\",\"acessos\":";
    payload += String(alarme.contadorAcessosValidos);
    payload += ",\"invasoes\":";
    payload += String(alarme.contadorInvasoes);
    payload += ",\"rearmes\":";
    payload += String(alarme.contadorRearmesBloqueados);

    payload += ",\"alerta_temp\":";
    payload += (alertaTempAtivo ? "true" : "false");
    payload += ",\"critico_temp\":";
    payload += (criticoTempAtivo ? "true" : "false");
    payload += ",\"alerta_umid\":";
    payload += (alertaUmidAtivo ? "true" : "false");
    payload += ",\"critico_umid\":";
    payload += (criticoUmidAtivo ? "true" : "false");
    payload += ",\"taxa_temp_alerta\":";
    payload += (taxaTempAlerta ? "true" : "false");
    payload += ",\"ambiente\":\"";
    payload += (estadoAmbiental == AMB_CRITICO ? "CRITICO" :
                estadoAmbiental == AMB_ALERTA  ? "ALERTA"  : "OK");
    payload += "\"";

    float t = dht.temperatura();
    float u = dht.umidade();
    payload += ",\"temp_c\":" + String(t, 1);
    payload += ",\"umid_pct\":" + String(u, 1);
    payload += "}";

    Serial.print(F("[MQTT] Publicando em "));
    Serial.print(MQTT_TOPIC_STATUS);
    Serial.print(F(": "));
    Serial.println(payload);

    _client.publish(MQTT_TOPIC_STATUS, payload.c_str(), true);

    // [1] cache atualizado AQUI (qualquer publicação, forçada ou não)
    _ultimoEstadoPublicado = alarme.estadoAtual;
    _ultimoModoPublicado   = alarme.modo;
  }

  void _tratarComando(char* topic, byte* payload, unsigned int length) {
    String cmd;
    for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
    cmd.trim();
    cmd.toUpperCase();

    logSerial("INFO", "MQTT_CMD", "Recebido: " + cmd);

    if (cmd == "DESARMAR") {
      if (alarme.modo == ARMADO) {
        alarme.modo               = DESARMADO;
        alarme.tempoInicioDesarme = millis();
        alarme.uidUltimoDesarme   = "MQTT";
        alarme.aviso5minDisparado = false;
        alarme.contadorDesarmes++;
        contadoresDirty = true;
        logSerial("INFO", "MQTT_CMD", "DESARMADO remoto executado.");
        enviarLogSite("Comando Remoto: DESARMAR executado.");
      } else {
        logSerial("WARN", "MQTT_CMD", "Ja desarmado, ignorando.");
        enviarLogSite("Comando Remoto: DESARMAR (ja desarmado).");
      }
    }
    else if (cmd == "ARMAR") {
      if (alarme.modo == DESARMADO) {
        if (alarme.armarViaSerial(porta, audio)) {
          logSerial("INFO", "MQTT_CMD", "ARMADO remoto executado.");
          enviarLogSite("Comando Remoto: ARMAR executado.");
        } else {
          logSerial("WARN", "MQTT_CMD", "Armar remoto BLOQUEADO (porta aberta).");
          enviarLogSite("Comando Remoto: ARMAR BLOQUEADO.");
        }
      } else {
        logSerial("WARN", "MQTT_CMD", "Ja armado, ignorando.");
        enviarLogSite("Comando Remoto: ARMAR (ja armado).");
      }
    }
    // [2] STATUS com rate limit
    else if (cmd == "STATUS") {
      if (millis() - _ultimoStatusForcado < STATUS_RATE_LIMIT_MS) {
        logSerial("WARN", "MQTT_CMD", "STATUS ignorado (rate limit).");
        return;
      }
      _ultimoStatusForcado = millis();
      logSerial("INFO", "MQTT_CMD", "Enviando STATUS sob demanda.");
      _publicar(true);
    }
    // [2] RESET com token; preserva contadores de segurança
    else if (cmd.startsWith("RESET")) {
      String token = cmd.substring(5);
      token.trim();

      if (token != MQTT_RESET_TOKEN) {
        logSerial("WARN", "MQTT_CMD", "RESET negado: token invalido.");
        enviarLogSite("Comando Remoto: RESET NEGADO (token).");
      } else {
        alarme.contadorAberturasPorta       = 0;
        alarme.contadorAproximacoes         = 0;
        alarme.contadorAcessosValidos       = 0;
        alarme.contadorMovimentosManutencao = 0;
        alarme.contadorDesarmes             = 0;
        // NÃO zera: contadorInvasoes, contadorRearmesBloqueados
        alarme.resetContadoresPendente = false;
        salvarConfig();
        logSerial("INFO", "MQTT_CMD", "RESET remoto executado (seguranca preservada).");
        enviarLogSite("Comando Remoto: RESET operacional executado.");
      }
    }
    else if (cmd == "TESTE") {
      alarme.modoTestePendente = true;
      logSerial("INFO", "MQTT_CMD", "MODO TESTE remoto executado.");
      enviarLogSite("Comando Remoto: TESTE iniciado.");
    }
    else {
      logSerial("WARN", "MQTT_CMD", "Comando desconhecido: " + cmd);
    }
  }
};

NotificadorMQTT notificador;

// =========================================================================
// ALARME AMBIENTAL (Temperatura + Umidade + taxa + crítico + consolidado)
// =========================================================================
void _verificarAmbiente(float temp, float umid) {
  if (temp == -999.0f || umid == -999.0f) return;

  // 4.1 — Taxa de subida de temperatura (amostra a cada 1 min)
  unsigned long agora = millis();
  if (_tempoAnteriorTaxa == 0) {
    _tempAnteriorTaxa  = temp;
    _tempoAnteriorTaxa = agora;
  } else if (agora - _tempoAnteriorTaxa >= INTERVALO_AMOSTRA_TAXA_MS) {
    float dtMin = (agora - _tempoAnteriorTaxa) / 60000.0f;
    float taxa  = (temp - _tempAnteriorTaxa) / dtMin;

    if (!taxaTempAlerta && taxa >= TAXA_ALERTA_C_POR_MIN) {
      taxaTempAlerta = true;
      logSerial("CRIT", "AMBIENTE",
        "SUBIDA RAPIDA: " + String(taxa, 2) + " C/min (limiar " +
        String(TAXA_ALERTA_C_POR_MIN, 2) + " C/min)");
      enviarLogSite("CRITICO: Temperatura subindo " + String(taxa, 2) + " C/min!");
    }
    else if (taxaTempAlerta && taxa < (TAXA_ALERTA_C_POR_MIN * 0.5f)) {
      taxaTempAlerta = false;
      logSerial("INFO", "AMBIENTE",
        "Taxa de subida normalizada: " + String(taxa, 2) + " C/min");
    }

    _tempAnteriorTaxa  = temp;
    _tempoAnteriorTaxa = agora;
  }

  // Temperatura — ALERTA
  if (!alertaTempAtivo && temp >= limiarTempAlerta) {
    alertaTempAtivo = true;
    logSerial("WARN", "AMBIENTE",
      "Temperatura alta: " + String(temp, 1) + "C (limiar " +
      String(limiarTempAlerta, 1) + "C)");
    enviarLogSite("ALERTA: Temperatura do rack em " + String(temp, 1) + "C");
  }
  else if (alertaTempAtivo && temp <= (limiarTempAlerta - TEMP_HISTERESE)) {
    alertaTempAtivo = false;
    logSerial("INFO", "AMBIENTE",
      "Temperatura normalizada: " + String(temp, 1) + "C");
    enviarLogSite("Temperatura do rack normalizada.");
  }

  // Temperatura — CRÍTICO
  if (!criticoTempAtivo && temp >= limiarTempCritico) {
    criticoTempAtivo = true;
    logSerial("CRIT", "AMBIENTE",
      "TEMPERATURA CRITICA: " + String(temp, 1) + "C!");
    enviarLogSite("CRITICO: Temperatura do rack em " + String(temp, 1) + "C!");
  }
  else if (criticoTempAtivo && temp <= (limiarTempCritico - TEMP_HISTERESE)) {
    criticoTempAtivo = false;
    logSerial("INFO", "AMBIENTE",
      "Temperatura critica normalizada: " + String(temp, 1) + "C");
    enviarLogSite("Temperatura critica normalizada.");
  }

  // Umidade — ALERTA
  if (!alertaUmidAtivo && umid >= limiarUmidAlerta) {
    alertaUmidAtivo = true;
    logSerial("WARN", "AMBIENTE",
      "Umidade alta: " + String(umid, 1) + "% (limiar " +
      String(limiarUmidAlerta, 1) + "%)");
    enviarLogSite("ALERTA: Umidade do rack em " + String(umid, 1) + "%");
  }
  else if (alertaUmidAtivo && umid <= (limiarUmidAlerta - UMID_HISTERESE)) {
    alertaUmidAtivo = false;
    logSerial("INFO", "AMBIENTE",
      "Umidade normalizada: " + String(umid, 1) + "%");
    enviarLogSite("Umidade do rack normalizada.");
  }

  // 4.5 — Umidade — CRÍTICO (condensação)
  if (!criticoUmidAtivo && umid >= limiarUmidCritico) {
    criticoUmidAtivo = true;
    logSerial("CRIT", "AMBIENTE",
      "UMIDADE CRITICA: " + String(umid, 1) + "% (risco de condensacao)");
    enviarLogSite("CRITICO: Umidade do rack em " + String(umid, 1) + "%!");
  }
  else if (criticoUmidAtivo && umid <= (limiarUmidCritico - UMID_HISTERESE)) {
    criticoUmidAtivo = false;
    logSerial("INFO", "AMBIENTE",
      "Umidade critica normalizada: " + String(umid, 1) + "%");
    enviarLogSite("Umidade critica normalizada.");
  }

  // 4.2 — Estado ambiental consolidado
  EstadoAmbiental novoEstado;
  if (criticoTempAtivo || criticoUmidAtivo || taxaTempAlerta) {
    novoEstado = AMB_CRITICO;
  } else if (alertaTempAtivo || alertaUmidAtivo) {
    novoEstado = AMB_ALERTA;
  } else {
    novoEstado = AMB_OK;
  }

  if (novoEstado != estadoAmbiental) {
    const char* nomes[] = { "OK", "ALERTA", "CRITICO" };
    logSerial(novoEstado == AMB_OK ? "INFO" :
              novoEstado == AMB_ALERTA ? "WARN" : "CRIT",
              "AMBIENTE",
              String("Estado ambiental: ") + nomes[estadoAmbiental] +
              " -> " + nomes[novoEstado]);
    estadoAmbiental = novoEstado;
  }

  // 4.3 — Buzzer ambiental opcional (prioridade respeitada)
#if ALARME_SONORO_AMBIENTAL
  bool segurancaCritica = (alarme.estadoAtual == EMERGENCIA_RACK ||
                           alarme.estadoAtual == INVASAO_SALA);
  if (!segurancaCritica) {
    static unsigned long _ultimoBipAmbiental = 0;
    unsigned long agoraBip = millis();

    if (estadoAmbiental == AMB_CRITICO && agoraBip - _ultimoBipAmbiental >= 3000) {
      tone(Pins::BUZZER_D, 400, 200); delay(220);
      tone(Pins::BUZZER_D, 400, 200);
      _ultimoBipAmbiental = agoraBip;
    }
    else if (estadoAmbiental == AMB_ALERTA && agoraBip - _ultimoBipAmbiental >= 8000) {
      tone(Pins::BUZZER_E, 700, 150);
      _ultimoBipAmbiental = agoraBip;
    }
  }
#endif
}

// =========================================================================
// HELPER JSON [6]
// =========================================================================
String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", (uint8_t)c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

// =========================================================================
// ENVIO DE LOG PARA O SITE
// =========================================================================
void enviarLogSite(const String &mensagem) {
  if (notificador.estaConectado()) {
    String payload = "{\"ts\":\"" + relogio.timestamp() +
                     "\",\"msg\":\"" + jsonEscape(mensagem) + "\"}";
    notificador.publicarLog(payload);
  }
}

// =========================================================================
// COMANDOS SERIAL
// =========================================================================
String bufferSerial = "";

void processarComandoSerial(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  logSerial("INFO", "CMD", "Comando recebido: " + cmd);

  if (cmd.startsWith("SET LIMIAR ")) {
    float novo = cmd.substring(11).toFloat();
    if (novo >= 5 && novo <= 400) {
      alarme.limiarDistancia = novo;
      salvarConfig();
      logSerial("INFO", "CMD", "Limiar ajustado para " + String(novo) + " cm.");
    } else {
      logSerial("WARN", "CMD", "Valor fora da faixa (5-400 cm).");
    }
  }
  else if (cmd.startsWith("SET TIMEOUT ")) {
    long minutos = cmd.substring(12).toInt();
    if (minutos >= 0 && minutos <= 1440) {
      alarme.timeoutDesarme = (unsigned long)minutos * 60UL * 1000UL;
      salvarConfig();
      logSerial("INFO", "CMD", "Timeout de desarme: " + String(minutos) + " min.");
      if (alarme.modo == DESARMADO) {
        alarme.tempoInicioDesarme = millis();
        alarme.aviso5minDisparado = false;
      }
    } else {
      logSerial("WARN", "CMD", "Timeout fora da faixa (0-1440 min).");
    }
  }
  else if (cmd.startsWith("SET TEMP_ALERTA ")) {
    float v = cmd.substring(16).toFloat();
    if (v >= 15 && v <= 50) {
      limiarTempAlerta = v;
      salvarConfig();
      logSerial("INFO", "CMD", "Limiar temp alerta: " + String(v, 1) + "C");
    } else logSerial("WARN", "CMD", "Fora da faixa (15-50).");
  }
  else if (cmd.startsWith("SET TEMP_CRIT ")) {
    float v = cmd.substring(14).toFloat();
    if (v >= 20 && v <= 60) {
      limiarTempCritico = v;
      salvarConfig();
      logSerial("INFO", "CMD", "Limiar temp critico: " + String(v, 1) + "C");
    } else logSerial("WARN", "CMD", "Fora da faixa (20-60).");
  }
  else if (cmd.startsWith("SET UMID_ALERTA ")) {
    float v = cmd.substring(16).toFloat();
    if (v >= 30 && v <= 90) {
      limiarUmidAlerta = v;
      salvarConfig();
      logSerial("INFO", "CMD", "Limiar umid alerta: " + String(v, 1) + "%");
    } else logSerial("WARN", "CMD", "Fora da faixa (30-90).");
  }
  else if (cmd.startsWith("SET UMID_CRIT ")) {
    float v = cmd.substring(14).toFloat();
    if (v >= 40 && v <= 100) {
      limiarUmidCritico = v;
      salvarConfig();
      logSerial("INFO", "CMD", "Limiar umid critico: " + String(v, 1) + "%");
    } else logSerial("WARN", "CMD", "Fora da faixa (40-100).");
  }
  else if (cmd == "STATUS") {
    Serial.println(F("----------- STATUS ATUAL -----------"));
    Serial.print(F("  Modo Vigilante:       ")); Serial.println(alarme.nomeModo());
    Serial.print(F("  Estado Alarme:        ")); Serial.println(alarme.nomeEstadoAtual());
    Serial.print(F("  Porta (filtrada):     ")); Serial.println(porta.estaAberta() ? "ABERTA" : "fechada");
    Serial.print(F("  Timeout Desarme:      ")); Serial.print(alarme.timeoutDesarme / 60000UL); Serial.println(F(" min"));
    if (alarme.modo == DESARMADO) {
      unsigned long dec = millis() - alarme.tempoInicioDesarme;
      unsigned long rst = (alarme.timeoutDesarme > dec) ? (alarme.timeoutDesarme - dec) / 1000UL : 0;
      Serial.print(F("  Rearme em:            ")); Serial.print(rst); Serial.println(F(" s"));
      Serial.print(F("  UID Desarme:          ")); Serial.println(alarme.uidUltimoDesarme);
    } else {
      Serial.print(F("  UID Ultimo Rearme:    "));
      Serial.println(alarme.uidUltimoRearme.length() ? alarme.uidUltimoRearme : "-");
    }
    Serial.print(F("  Contador Desarmes:    ")); Serial.println(alarme.contadorDesarmes);
    Serial.print(F("  Rearmes Bloqueados:   ")); Serial.println(alarme.contadorRearmesBloqueados);
    Serial.print(F("  Duracao ult. desarme: ")); Serial.print(alarme.duracaoUltimoDesarme); Serial.println(F(" s"));
    Serial.print(F("  Limiar Distancia:     ")); Serial.print(alarme.limiarDistancia); Serial.println(F(" cm"));
    Serial.print(F("  Aberturas da Porta:   ")); Serial.println(alarme.contadorAberturasPorta);
    Serial.print(F("  Acessos RFID:         ")); Serial.println(alarme.contadorAcessosValidos);
    Serial.print(F("  Invasoes:             ")); Serial.println(alarme.contadorInvasoes);
    Serial.print(F("  Aproximacoes:         ")); Serial.println(alarme.contadorAproximacoes);
    Serial.print(F("  Mov. em manutencao:   ")); Serial.println(alarme.contadorMovimentosManutencao);
    Serial.print(F("  Limiares ambientais:  T["));
    Serial.print(limiarTempAlerta, 1);   Serial.print(F("/"));
    Serial.print(limiarTempCritico, 1);  Serial.print(F("C] U["));
    Serial.print(limiarUmidAlerta, 1);   Serial.print(F("/"));
    Serial.print(limiarUmidCritico, 1);  Serial.println(F("%]"));
    Serial.print(F("  WiFi:                 "));
    Serial.println(WiFi.status() == WL_CONNECTED ? "conectado" : "desconectado");
    Serial.print(F("  MQTT:                 "));
    Serial.println(notificador.estaConectado() ? "conectado" : "desconectado");
    Serial.println(F("-------------------------------------"));
  }
  else if (cmd == "ARMAR") {
    if (alarme.modo == DESARMADO) {
      logSerial("INFO", "CMD", "Tentativa de ARMAR via Serial.");
      alarme.armarViaSerial(porta, audio);
      salvarConfig();
    } else {
      logSerial("WARN", "CMD", "Ja esta armado.");
    }
  }
  else if (cmd == "DESARMAR") {
    if (alarme.modo == ARMADO) {
      alarme.modo               = DESARMADO;
      alarme.tempoInicioDesarme = millis();
      alarme.uidUltimoDesarme   = "SERIAL";
      alarme.aviso5minDisparado = false;
      alarme.contadorDesarmes++;
      contadoresDirty = true;
      logSerial("INFO", "CMD", "Modo Vigilante DESARMADO via Serial.");
      salvarConfig();
    } else {
      logSerial("WARN", "CMD", "Ja esta desarmado.");
    }
  }
  else if (cmd == "TESTE") {
    logSerial("INFO", "CMD", "Executando modo teste via Serial.");
    alarme.modoTestePendente = true;
  }
  else if (cmd == "RESET") {
    // [2] preserva contadores de segurança
    alarme.contadorAberturasPorta       = 0;
    alarme.contadorAproximacoes         = 0;
    alarme.contadorAcessosValidos       = 0;
    alarme.contadorMovimentosManutencao = 0;
    alarme.contadorDesarmes             = 0;
    // NÃO zera: contadorInvasoes, contadorRearmesBloqueados
    salvarConfig();
    logSerial("INFO", "CMD", "Contadores operacionais zerados (seguranca preservada).");
  }
  else if (cmd.startsWith("RTC ")) {
    int ano, mes, dia, hora, min, seg;
    if (sscanf(cmd.c_str(), "RTC %d-%d-%d %d:%d:%d",
               &ano, &mes, &dia, &hora, &min, &seg) == 6) {
      relogio.driver().adjust(DateTime(ano, mes, dia, hora, min, seg));
      logSerial("INFO", "RTC",
        "RTC ajustado para " + String(ano) + "-" + String(mes) + "-" + String(dia) +
        " " + String(hora) + ":" + String(min) + ":" + String(seg));
    } else {
      logSerial("WARN", "CMD", "Formato: RTC YYYY-MM-DD HH:MM:SS");
    }
  }
  else if (cmd == "RTC") {
    logSerial("INFO", "RTC", "Hora atual: " + relogio.timestamp());
  }
  // [9] ADD_CARD com deduplicação
  else if (cmd.startsWith("ADD_CARD ")) {
    String novoUID = cmd.substring(9);
    novoUID.trim();
    if (novoUID.length() >= 8) {
      bool jaExiste = false;
      for (const String &c : cartoesAutorizados) {
        if (c == novoUID) { jaExiste = true; break; }
      }
      if (jaExiste) {
        logSerial("WARN", "CMD", "Cartao ja cadastrado: " + novoUID);
      } else {
        prefs.begin("rack", false);
        int total = prefs.getInt("total_cards", 0);
        String chave = "card_" + String(total);
        prefs.putString(chave.c_str(), novoUID);
        prefs.putInt("total_cards", total + 1);
        prefs.end();
        cartoesAutorizados.push_back(novoUID);
        logSerial("INFO", "CMD", "Cartao cadastrado na NVS: " + novoUID);
      }
    } else {
      logSerial("WARN", "CMD", "UID invalido. Use ADD_CARD <UID_HEX> (min 8 chars)");
    }
  }
  else if (cmd == "LIST_CARDS") {
    Serial.println(F("--- Cartoes Autorizados ---"));
    for (const String &card : cartoesAutorizados) {
      Serial.println(card);
    }
    Serial.println(F("---------------------------"));
  }
  else if (cmd == "HELP") {
    Serial.println(F("----------- COMANDOS DISPONIVEIS -----------"));
    Serial.println(F("  SET LIMIAR <5-400>      - Ajusta limiar ultrassonico (persiste)"));
    Serial.println(F("  SET TIMEOUT <0-1440>    - Timeout de desarme (min, persiste)"));
    Serial.println(F("  SET TEMP_ALERTA <15-50> - Limiar temp alerta (C, persiste)"));
    Serial.println(F("  SET TEMP_CRIT <20-60>   - Limiar temp critico (C, persiste)"));
    Serial.println(F("  SET UMID_ALERTA <30-90> - Limiar umid alerta (%, persiste)"));
    Serial.println(F("  SET UMID_CRIT <40-100>  - Limiar umid critico (%, persiste)"));
    Serial.println(F("  STATUS                  - Status completo"));
    Serial.println(F("  ARMAR                   - Forca ARMADO (se porta fechada)"));
    Serial.println(F("  DESARMAR                - Forca DESARMADO"));
    Serial.println(F("  TESTE                   - Modo teste"));
    Serial.println(F("  RTC                     - Mostra hora do RTC"));
    Serial.println(F("  RTC YYYY-MM-DD HH:MM:SS - Ajusta RTC"));
    Serial.println(F("  RESET                   - Zera contadores operacionais (preserva seguranca)"));
    Serial.println(F("  ADD_CARD <UID>          - Cadastra novo cartao na NVS"));
    Serial.println(F("  LIST_CARDS              - Lista cartoes autorizados"));
    Serial.println(F("  HELP                    - Lista comandos"));
    Serial.println(F("---------------------------------------------"));
    Serial.println(F("  BOTAO : curto=reset | longo=rearme | duplo=teste"));
    Serial.println(F("  MQTT  : rack/status (pub) | rack/comando (sub)"));
    Serial.println(F("  MQTT  : rack/status/lwt (online/offline) | rack/logs (eventos)"));
    Serial.println(F("  MQTT  : comandos = ARMAR | DESARMAR | STATUS | RESET <token> | TESTE"));
  }
  else {
    logSerial("WARN", "CMD", "Comando desconhecido. Digite HELP.");
  }
}

// =========================================================================
// MODO TESTE
// =========================================================================
void executarModoTeste() {
  logSerial("INFO", "TESTE", "=== INICIANDO AUTODIAGNOSTICO v2 ===");
  enviarLogSite("TESTE: iniciando autodiagnostico...");

  int ok = 0, falhas = 0;
  String resumoFalhas = "";

  // 1. OLED
  if (display.begin(SSD1306_SWITCHCAPVCC, ENDERECO_OLED)) {
    logSerial("INFO", "TESTE", "[OK] Display OLED (I2C) respondendo.");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] Display OLED NAO responde.");
    resumoFalhas += "OLED "; falhas++;
  }

  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20); display.println("MODO TESTE");
  display.setCursor(10, 40); display.println("Aguarde...");
  display.display();

  // 2. RFID
  byte versao = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (versao != 0x00 && versao != 0xFF) {
    logSerial("INFO", "TESTE", "[OK] RFID MFRC522 (v0x" + String(versao, HEX) + ").");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] RFID MFRC522 NAO responde.");
    resumoFalhas += "RFID "; falhas++;
  }

  // 3. Ultrassonico
  float d = ultrassonico.distancia();
  if (d > 0.0f && d < US_DIST_MAX_VALIDA) {
    logSerial("INFO", "TESTE", "[OK] Ultrassonico: " + String(d, 1) + " cm.");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] Ultrassonico: " + String(d, 1) + " cm.");
    resumoFalhas += "US "; falhas++;
  }

  // 4. LDR
  int ldrRaw = ldr.leituraBruta();
  logSerial("INFO", "TESTE", "[OK] LDR: " + String(ldrRaw) + " (" + String(ldr.estaAcesa() ? "ACESA" : "APAGADA") + ").");
  ok++;

  // 5. Porta
  logSerial("INFO", "TESTE", "[INFO] Porta: " + String(porta.estaAberta() ? "ABERTA" : "fechada") + ".");
  ok++;

  // 6. PIR
  logSerial("INFO", "TESTE", "[INFO] PIR: " + String(pir.detectaMovimento() ? "ATIVO" : "inativo") + ".");
  ok++;

  // 7. DHT22
  float t = dht.temperatura();
  float u = dht.umidade();
  if (t > -50.0f && t < 80.0f) {
    logSerial("INFO", "TESTE", "[OK] DHT22: " + String(t, 1) + "C / " + String(u, 1) + "%.");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] DHT22 sem leitura valida.");
    resumoFalhas += "DHT22 "; falhas++;
  }

  // 8. Buzzers
  logSerial("INFO", "TESTE", "[INFO] Testando buzzers...");
  tone(Pins::BUZZER_E, 500, 150);  delay(200);
  tone(Pins::BUZZER_E, 1000, 150); delay(200);
  tone(Pins::BUZZER_E, 2000, 150); delay(200);
  tone(Pins::BUZZER_D, 800, 300);  delay(350);
  ok++;

  // 9. LED
  logSerial("INFO", "TESTE", "[INFO] Testando LED RGB...");
  led.definirCor(255, 0, 0);     delay(300);
  led.definirCor(0, 255, 0);     delay(300);
  led.definirCor(0, 0, 255);     delay(300);
  led.definirCor(255, 255, 255); delay(300);
  led.definirCor(0, 0, 0);
  ok++;

  // Resumo
  String resumo = "TESTE CONCLUIDO: " + String(ok) + " OK";
  if (falhas > 0) resumo += ", " + String(falhas) + " FALHA(S): " + resumoFalhas;
  logSerial(falhas > 0 ? "WARN" : "INFO", "TESTE", resumo);
  enviarLogSite(resumo);

  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println("TESTE CONCLUIDO");
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
  display.setCursor(0, 20); display.print("OK:     "); display.println(ok);
  display.setCursor(0, 32); display.print("FALHAS: "); display.println(falhas);
  display.display();

  if (falhas == 0) {
    tone(Pins::BUZZER_E, 1500, 100); delay(120);
    tone(Pins::BUZZER_E, 2000, 150);
  } else {
    tone(Pins::BUZZER_D, 400, 500); delay(600);
    tone(Pins::BUZZER_D, 300, 500);
  }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  led.begin();
  led.selfTest();

  Wire.begin(21, 22);
  relogio.begin();
  oled.begin();

  ldr.begin();
  ultrassonico.begin();
  porta.begin();
  pir.begin();
  botao.begin();

  leitorRFID.begin();
  audio.begin();

  dht.begin();

  carregarConfig();
  carregarCartoes();

  Serial.println();
  Serial.println(F("================================================="));
  Serial.println(F("  SISTEMA DE MONITORAMENTO DE RACK - v6.4"));
  Serial.println(F("  LWT + Retain + NVS + Comandos Remotos MQTT"));
  Serial.println(F("================================================="));
  logSerial("INFO", "SISTEMA", "Sistema inicializado. MODO VIGILANTE: ARMADO.");
  logSerial("INFO", "RFID", "Cartao 1 Autorizado: " + CARTAO_AUTORIZADO_1);
  logSerial("INFO", "RFID", "Cartao 2 Autorizado: " + CARTAO_AUTORIZADO_2);
  logSerial("INFO", "RTC", relogio.disponivel() ? "RTC DS3231 ativo." : "RTC ausente (usando millis).");
  logSerial("INFO", "VIGILANTE", "Timeout de desarme: " + String(alarme.timeoutDesarme / 60000) + " min");
  logSerial("INFO", "VIGILANTE", "Politica: rearme BLOQUEADO se porta estiver aberta.");
  logSerial("INFO", "FILTROS",
    "LDR: " + String(LDR_LIMIAR_APAGA) + "-" + String(LDR_LIMIAR_ACENDE) +
    " deb " + String(DEBOUNCE_LDR_MS) + "ms | US media: " + String(US_AMOSTRAS));
  logSerial("INFO", "FILTROS",
    "Botao: deb " + String(DEBOUNCE_BOTAO_MS) + "ms | duplo <" +
    String(JANELA_CLIQUE_DUPLO_MS) + "ms | longo >" + String(DURACAO_CLIQUE_LONGO_MS) + "ms");
  logSerial("INFO", "FILTROS", "Porta: deb " + String(DEBOUNCE_PORTA_MS) + "ms");
  logSerial("INFO", "SISTEMA", "Digite HELP no Serial para comandos disponiveis.");
  Serial.println(F("-------------------------------------------------"));

  notificador.begin();
}

// =========================================================================
// LOOP PRINCIPAL
// =========================================================================
unsigned long tempoUltimoBip   = 0;
unsigned long tempoUltimoPisca = 0;
bool          estadoPiscaLED   = false;

// [4] salvamento só se dirty
unsigned long tempoUltimoSalvamento = 0;
constexpr unsigned long INTERVALO_SALVAMENTO_MS = 60000UL;

void loop() {
  // 1. Serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferSerial.length() > 0) {
        processarComandoSerial(bufferSerial);
        bufferSerial = "";
      }
    } else {
      bufferSerial += c;
    }
  }

  // 2. Sensores contínuos
  ldr.update();
  ultrassonico.update();
  porta.update();
  pir.update();

  // 3. Eventos
  EventoBotao evBotao = botao.update();
  EventoRFID  evRFID  = leitorRFID.ler();

  // 4. Snapshot
  LeiturasSensores s;
  s.movimento   = pir.detectaMovimento();
  s.luzAcesa    = ldr.estaAcesa();
  s.portaAberta = porta.estaAberta();
  s.distancia   = ultrassonico.distancia();

  // 5. Máquina de estados
  alarme.update(s, evRFID, evBotao, porta, audio);

  // 5.1. Alarme ambiental
  float tempAtual = dht.temperatura();
  float umidAtual = dht.umidade();
  _verificarAmbiente(tempAtual, umidAtual);

  // 6. Ações pendentes
  if (alarme.audioPararPendente) {
    alarme.audioPararPendente = false;
    audio.parar();
  }
  if (alarme.resetContadoresPendente) {
    // [2] preserva contadores de segurança
    alarme.resetContadoresPendente = false;
    alarme.contadorAberturasPorta       = 0;
    alarme.contadorAproximacoes         = 0;
    alarme.contadorAcessosValidos       = 0;
    alarme.contadorMovimentosManutencao = 0;
    alarme.contadorDesarmes             = 0;
    // NÃO zera: contadorInvasoes, contadorRearmesBloqueados
    salvarConfig();
  }
  if (alarme.modoTestePendente) {
    alarme.modoTestePendente = false;
    executarModoTeste();
  }

  // 7. Display
  if (alarme.modo == DESARMADO) {
    unsigned long decorrido = millis() - alarme.tempoInicioDesarme;
    unsigned long restante  = (alarme.timeoutDesarme > decorrido)
                                ? (alarme.timeoutDesarme - decorrido) : 0;
    oled.mostrarDesarmado(restante, alarme.uidUltimoDesarme,
                          alarme.contadorMovimentosManutencao);
  } else if (alarme.bloqueioAtivo()) {
    oled.mostrarBloqueio(alarme.motivoBloqueio,
                         alarme.contadorRearmesBloqueados,
                         alarme.uidUltimoDesarme);
    audio.avisoBloqueio(tempoUltimoBip);
  } else {
    oled.mostrarStatus(alarme.estadoAtual, alarme.modo, s,
                       alarme.uidUltimoDesarme, true,
                       alarme.contadorAberturasPorta,
                       alarme.contadorInvasoes);
    audio.tocarMelodia(alarme.estadoAtual, tempoUltimoBip);
  }

  // 8. LED
  led.atualizar(alarme.modo, alarme.estadoAtual,
                alarme.bloqueioAtivo(), tempoUltimoPisca, estadoPiscaLED);

  // 9. MQTT
  notificador.update();

  // 10. [4] Salvamento periódico apenas se dirty
  if (contadoresDirty && millis() - tempoUltimoSalvamento > INTERVALO_SALVAMENTO_MS) {
    tempoUltimoSalvamento = millis();
    contadoresDirty = false;
    salvarConfig();
  }

  delay(20);
}