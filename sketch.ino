/**
 * @file RackMonitor.ino
 * @brief Sistema de Monitoramento de Rack (ESP32).
 * 
 * Versão 7.3 - Ring buffer de eventos + HIST (debug pós-incidente).
 * 
 * Arquitetura:
 * Sensores -> Eventos -> Alarme (única fonte de verdade) -> Atuadores
 * (Display, LED, Áudio) + MQTT (observador) + OTA (manutenção) +
 * RingBufferLog (auditoria em RAM).
 */

// =========================================================================
// CHANGELOG
// =========================================================================
//   v7.3 - RING BUFFER + HIST:
//          [1] Buffer circular de 100 eventos em RAM (sem String, sem malloc).
//          [2] Struct EventoLog { uint32_t ms; uint8_t nivel; char msg[64]; }.
//          [3] logSerial() agora alimenta o buffer automaticamente.
//          [4] Comandos HIST / HIST_CLEAR / HIST_JSON.
//          [5] Push com noInterrupts()/interrupts() (compatível com ISR do pânico).
//   v7.2 - OTA (ArduinoOTA) + persistência do modo na NVS.
//   v7.1 - NTP + RTC Sync + fallback millis com flag no JSON.
//   v7.0 - Tópico ambiente dedicado + comando ambiente segregado + pânico ISR.
//   v6.5 - Cache DHT, logs não-bloqueantes, OLED com ambiente, reset robusto.
//   v6.4 - Correções anteriores (dupla publicação, rate limit, LWT, NVS dirty).
//   v6.3 - LWT (Last Will), Retain flag, NVS, comandos remotos MQTT.
//   v6.2 - Integração Wi-Fi + MQTT.
//   v6.1 - Doxygen; correção duplo update() do botão; ordem init I2C.
//   v6.0 - Refactor: classes encapsuladas, struct de sensores, máquina isolada.
//   v5.0 - Versão original em arquivo único.

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
//  26   | BotaoPanico  | Pushbutton (v7.0)         | Branco.       | INPUT_PULLUP + ISR
//  27   | PIR          | Movimento                 | Verde Claro   |
//  32   | LedRGB       | Canal R                   | Marrom        |
//  33   | LedRGB       | Canal G                   | Verde         |
//  34   | LDR          | Leitura Analógica (AO)    | Verde         | Input-only
//
// [ LIVRES PARA EXPANSÃO ]
//  - GPIO 12 : Livre (Strapping — cuidado no boot)
//  - GPIO 35 : Livre (Apenas Entrada Analógica)
//  - GPIO 36 : Livre (Apenas Entrada Analógica)
//  - GPIO 39 : Livre (Apenas Entrada Analógica)
//
// [ PROIBIDOS ]
//  - GPIO 6 a 11 : Conectados à memória Flash (NUNCA USAR)
//  - GPIO 1 e 3  : Usados pela Serial USB (TX0/RX0)
// =========================================================================

/**
 * @brief Arquitetura de Software e Regras de Ouro.
 *
 * REGRAS DE OURO
 * 1. Sensores NÃO decidem nada — só medem e filtram.
 * 2. Atuadores NÃO leem sensores — só consomem estado.
 * 3. Alarme NÃO acessa hardware direto — recebe struct/eventos.
 * 4. Eventos são "fatos", não "ações".
 * 5. Toda transição é logada com nível (INFO/ALERT/CRIT).
 * 6. loop() só orquestra.
 * 7. MQTT é OBSERVADOR do Alarme.
 * 8. NVS é persistência PASSIVA.
 */

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
#include <math.h>
#include <time.h>
#include <ArduinoOTA.h>

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

// [v7.0] Botão de pânico — GPIO 26 (livre, sem strapping, pull-up interno)
constexpr uint8_t BOTAO_PANICO = 26;

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

// Limites ambientais
constexpr float TEMP_ALERTA_C_DEFAULT    = 30.0f;
constexpr float TEMP_CRITICO_C_DEFAULT   = 35.0f;
constexpr float UMID_ALERTA_PCT_DEFAULT  = 65.0f;
constexpr float UMID_CRITICO_PCT_DEFAULT = 80.0f;

constexpr float TEMP_HISTERESE  = 2.0f;
constexpr float UMID_HISTERESE  = 5.0f;

constexpr float TAXA_ALERTA_C_POR_MIN = 1.0f;
constexpr unsigned long INTERVALO_AMOSTRA_TAXA_MS = 60000UL;

float limiarTempAlerta  = TEMP_ALERTA_C_DEFAULT;
float limiarTempCritico = TEMP_CRITICO_C_DEFAULT;
float limiarUmidAlerta  = UMID_ALERTA_PCT_DEFAULT;
float limiarUmidCritico = UMID_CRITICO_PCT_DEFAULT;

bool alertaTempAtivo  = false;
bool criticoTempAtivo = false;
bool alertaUmidAtivo  = false;
bool criticoUmidAtivo = false;
bool taxaTempAlerta   = false;

// Watchdog do DHT22
unsigned long _ultimaLeituraValidaDHT = 0;
bool          _sensorDHTPerdido       = false;
constexpr unsigned long TIMEOUT_DHT_MS = 10000UL;

// Cache das últimas leituras válidas do DHT
float _ultimaTempValida = -999.0f;
float _ultimaUmidValida = -999.0f;

enum EstadoAmbiental { AMB_OK, AMB_ALERTA, AMB_CRITICO };
EstadoAmbiental estadoAmbiental = AMB_OK;

// [v7.1] Fonte de tempo atual
enum FonteTempo { FONTE_NTP, FONTE_RTC, FONTE_MILLIS };
FonteTempo fonteTempoAtual = FONTE_MILLIS;

#define ALARME_SONORO_AMBIENTAL 0

float         _tempAnteriorTaxa  = -999.0f;
unsigned long _tempoAnteriorTaxa = 0;

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

// [v7.0] Tópicos dedicados de ambiente
const char* MQTT_TOPIC_AMBIENTE     = "rack_monitor_ABCD1234/ambiente";
const char* MQTT_TOPIC_CMD_AMBIENTE = "rack_monitor_ABCD1234/comando/ambiente";

const unsigned long MQTT_RETRY_MS = 5000;

const char* MQTT_RESET_TOKEN = "rack2026";

// =========================================================================
// [v7.1] CONFIGURAÇÃO NTP
// =========================================================================
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.google.com";
const char* NTP_SERVER_3 = "time.cloudflare.com";

const long  NTP_GMT_OFFSET_SEC      = -3 * 3600;
const int   NTP_DAYLIGHT_OFFSET_SEC = 0;

const unsigned long NTP_SYNC_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
const unsigned long NTP_SYNC_TIMEOUT_MS  = 15000UL;

// =========================================================================
// [v7.2] CONFIGURAÇÃO OTA
// =========================================================================
const char* OTA_HOSTNAME = "rack-ABCD1234";
const char* OTA_PASSWORD = "rack@2026#ota";
const uint16_t OTA_PORT  = 3232;

// =========================================================================
// [v7.3] CONFIGURAÇÃO RING BUFFER
// =========================================================================
// 100 eventos * (4+1+64) bytes ≈ 6.9KB na heap (alocado uma vez no boot).
// Sobra ~290KB de heap livre no ESP32 — folga confortável.
constexpr uint16_t HIST_TAMANHO = 100;
constexpr uint8_t  HIST_MSG_MAX = 64;   // tamanho máximo de cada mensagem

enum NivelLog : uint8_t {
  NIVEL_INFO  = 0,
  NIVEL_WARN  = 1,
  NIVEL_ALERT = 2,
  NIVEL_CRIT  = 3
};

// =========================================================================
// ENUMS E STRUCTS
// =========================================================================

enum ModoVigilante { ARMADO, DESARMADO };

enum EstadoSistema {
  SEGURO,
  AVISO_SALA,
  APROXIMACAO_RACK,
  EMERGENCIA_RACK,
  INVASAO_SALA,
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
// [v7.3] STRUCT DO RING BUFFER
// =========================================================================
// Sem String, sem ponteiro, sem alocação dinâmica. Copiável por memcpy.
// =========================================================================

struct EventoLog {
  uint32_t ms;              // millis() no momento do evento
  uint8_t  nivel;           // NivelLog
  char     msg[HIST_MSG_MAX];
};

// =========================================================================
// PERIFÉRICOS
// =========================================================================
Adafruit_SSD1306 display(LARGURA_TELA, ALTURA_TELA, &Wire, -1);
RTC_DS3231       rtc;
MFRC522          rfid(Pins::RFID_SS, Pins::RFID_RST);

bool rtcDisponivel = false;

Preferences prefs;

// =========================================================================
// [v7.3] RING BUFFER DE EVENTOS
// =========================================================================
// Buffer circular em RAM. O(n) no dump, O(1) no push.
// Protegido contra ISR via noInterrupts()/interrupts() no push — o botão
// de pânico pode chamar registrarEvento() de dentro da ISR sem corromper
// o índice circular.
// =========================================================================

class RingBufferLog {
  public:
    void begin() {
      _head  = 0;
      _count = 0;
      // Não precisa zerar o array — o _count controla o que é válido
    }

    /**
     * @brief Insere um evento no buffer. Circular: se cheio, sobrescreve
     *        o mais antigo.
     * @note  Chamar de dentro de ISR é seguro (noInterrupts é reentrante no
     *        ESP32 — desabilita temporariamente e restaura).
     */
    void push(uint8_t nivel, const char *msg) {
      noInterrupts();
      EventoLog &slot = _buffer[_head];
      slot.ms    = millis();
      slot.nivel = nivel;
      strncpy(slot.msg, msg, HIST_MSG_MAX - 1);
      slot.msg[HIST_MSG_MAX - 1] = '\0';   // garante terminação

      _head = (_head + 1) % HIST_TAMANHO;
      if (_count < HIST_TAMANHO) _count++;
      interrupts();
    }

    /**
     * @brief Despeja os eventos em ordem cronológica (mais antigo -> mais novo).
     */
    void dumpSerial() const {
      Serial.println(F("----------- HISTORICO DE EVENTOS -----------"));
      Serial.print(F("  Total no buffer: ")); Serial.println(_count);
      Serial.print(F("  Capacidade:      ")); Serial.println(HIST_TAMANHO);
      Serial.println(F("---------------------------------------------"));

      if (_count == 0) {
        Serial.println(F("  (vazio)"));
        Serial.println(F("---------------------------------------------"));
        return;
      }

      // O mais antigo está em (_head - _count + HIST_TAMANHO) % HIST_TAMANHO
      uint16_t inicio = (_head + HIST_TAMANHO - _count) % HIST_TAMANHO;
      for (uint16_t i = 0; i < _count; i++) {
        uint16_t idx = (inicio + i) % HIST_TAMANHO;
        const EventoLog &e = _buffer[idx];

        char tempo[12];
        uint32_t s = e.ms / 1000;
        uint32_t m = s / 60;
        uint32_t h = m / 60;
        s %= 60; m %= 60;
        snprintf(tempo, sizeof(tempo), "%02lu:%02lu:%02lu",
                 (unsigned long)h, (unsigned long)m, (unsigned long)s);

        Serial.print("[");
        Serial.print(tempo);
        Serial.print("] [");
        Serial.print(_nomeNivel(e.nivel));
        Serial.print("] ");
        Serial.println(e.msg);
      }
      Serial.println(F("---------------------------------------------"));
    }

    /**
     * @brief Despeja os eventos em formato JSON (uma linha por evento).
     *        Útil para colar em ferramentas de análise.
     */
    void dumpJSON() const {
      Serial.println(F("{\"eventos\":["));
      if (_count == 0) {
        Serial.println(F("]}"));
        return;
      }

      uint16_t inicio = (_head + HIST_TAMANHO - _count) % HIST_TAMANHO;
      for (uint16_t i = 0; i < _count; i++) {
        uint16_t idx = (inicio + i) % HIST_TAMANHO;
        const EventoLog &e = _buffer[idx];

        Serial.print(F("  {\"ms\":"));
        Serial.print(e.ms);
        Serial.print(F(",\"nivel\":\""));
        Serial.print(_nomeNivel(e.nivel));
        Serial.print(F("\",\"msg\":\""));
        _escapeJson(e.msg);
        Serial.print(F("\"}"));
        if (i < _count - 1) Serial.print(',');
        Serial.println();
      }
      Serial.println(F("]}"));
    }

    void clear() {
      noInterrupts();
      _head  = 0;
      _count = 0;
      interrupts();
      Serial.println(F("[INFO] Historico limpo."));
    }

    uint16_t tamanho() const { return _count; }

  private:
    EventoLog _buffer[HIST_TAMANHO];
    volatile uint16_t _head  = 0;
    volatile uint16_t _count = 0;

    static const char* _nomeNivel(uint8_t n) {
      switch (n) {
        case NIVEL_INFO:  return "INFO";
        case NIVEL_WARN:  return "WARN";
        case NIVEL_ALERT: return "ALERT";
        case NIVEL_CRIT:  return "CRIT";
        default:          return "????";
      }
    }

    static void _escapeJson(const char *s) {
      for (size_t i = 0; s[i] != '\0'; i++) {
        char c = s[i];
        switch (c) {
          case '"':  Serial.print(F("\\\"")); break;
          case '\\': Serial.print(F("\\\\")); break;
          case '\n': Serial.print(F("\\n"));  break;
          case '\r': Serial.print(F("\\r"));  break;
          case '\t': Serial.print(F("\\t"));  break;
          default:
            if ((uint8_t)c < 0x20) {
              char buf[7];
              snprintf(buf, sizeof(buf), "\\u%04x", (uint8_t)c);
              Serial.print(buf);
            } else {
              Serial.print(c);
            }
        }
      }
    }
};

RingBufferLog logBuffer;

// =========================================================================
// LOG CENTRALIZADO — protótipos ANTES de Relogio
// =========================================================================

void logSerial(const String &nivel, const String &tag, const String &mensagem);
void registrarEvento(const String &nivel, const String &tag, const String &mensagem);
void enviarLogSite(const String &mensagem);

// =========================================================================
// MÓDULO: RELÓGIO (v7.1 — NTP + RTC + fallback millis)
// =========================================================================

class Relogio {
  public:
    bool begin() {
      if (rtc.begin()) {
        rtcDisponivel = true;
        fonteTempoAtual = FONTE_RTC;
        if (rtc.lostPower()) {
          Serial.println(F("[WARN] RTC perdeu energia. Aguardando sync NTP ou ajuste manual."));
        }
        logSerial("INFO", "RTC", "RTC DS3231 inicializado.");
        return true;
      }
      rtcDisponivel = false;
      fonteTempoAtual = FONTE_MILLIS;
      logSerial("WARN", "RTC", "RTC ausente. Usando millis() como fallback.");
      return false;
    }

    bool sincronizarNTP() {
      if (WiFi.status() != WL_CONNECTED) {
        logSerial("WARN", "NTP", "Sync NTP ignorado: Wi-Fi desconectado.");
        return false;
      }

      logSerial("INFO", "NTP", "Sincronizando com servidores NTP...");
      configTime(NTP_GMT_OFFSET_SEC, NTP_DAYLIGHT_OFFSET_SEC,
                 NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);

      struct tm timeinfo;
      unsigned long inicio = millis();
      bool ok = false;

      while (millis() - inicio < NTP_SYNC_TIMEOUT_MS) {
        if (getLocalTime(&timeinfo, 1000)) { ok = true; break; }
        delay(200);
      }

      if (!ok) {
        logSerial("WARN", "NTP", "Falha ao sincronizar (timeout " +
                  String(NTP_SYNC_TIMEOUT_MS / 1000) + "s). Mantendo fonte atual.");
        return false;
      }

      if (rtcDisponivel) {
        DateTime agoraNTP(
          timeinfo.tm_year + 1900,
          timeinfo.tm_mon + 1,
          timeinfo.tm_mday,
          timeinfo.tm_hour,
          timeinfo.tm_min,
          timeinfo.tm_sec
        );
        rtc.adjust(agoraNTP);
        logSerial("INFO", "NTP",
                  "RTC ajustado via NTP: " +
                  String(timeinfo.tm_mday) + "/" + String(timeinfo.tm_mon + 1) +
                  " " + String(timeinfo.tm_hour) + ":" +
                  String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));
        fonteTempoAtual = FONTE_RTC;
      } else {
        fonteTempoAtual = FONTE_NTP;
        logSerial("INFO", "NTP", "Sem RTC. Fonte de tempo = NTP (Wi-Fi).");
      }
      _ultimaSincronizacaoNTP = millis();
      return true;
    }

    void talvezRessincronizar() {
      if (WiFi.status() != WL_CONNECTED) return;
      if (_ultimaSincronizacaoNTP == 0) return;
      if (millis() - _ultimaSincronizacaoNTP < NTP_SYNC_INTERVAL_MS) return;
      logSerial("INFO", "NTP", "Ressincronizacao periodica NTP.");
      sincronizarNTP();
    }

    String timestamp() const {
      DateTime agora = _agora();
      if (agora.unixtime() > 0) {
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
      DateTime agora = _agora();
      if (agora.unixtime() > 0) {
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

    String fonteTempoStr() const {
      switch (fonteTempoAtual) {
        case FONTE_NTP:    return "NTP";
        case FONTE_RTC:    return "RTC";
        case FONTE_MILLIS: return "MILLIS";
        default:           return "UNKNOWN";
      }
    }

    bool ajustarManual(int ano, int mes, int dia, int hora, int min, int seg) {
      if (!rtcDisponivel) {
        logSerial("WARN", "RTC", "Ajuste manual ignorado: RTC ausente.");
        return false;
      }
      rtc.adjust(DateTime(ano, mes, dia, hora, min, seg));
      fonteTempoAtual = FONTE_RTC;
      _ultimaSincronizacaoNTP = millis();
      logSerial("INFO", "RTC",
                "RTC ajustado manualmente: " + String(ano) + "-" + String(mes) +
                "-" + String(dia) + " " + String(hora) + ":" +
                String(min) + ":" + String(seg));
      return true;
    }

  private:
    unsigned long _ultimaSincronizacaoNTP = 0;

    DateTime _agora() const {
      if (rtcDisponivel) {
        DateTime agora = rtc.now();
        if (agora.year() >= 2020) return agora;
      }
      if (fonteTempoAtual == FONTE_NTP && WiFi.status() == WL_CONNECTED) {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 0)) {
          return DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                          timeinfo.tm_mday, timeinfo.tm_hour,
                          timeinfo.tm_min, timeinfo.tm_sec);
        }
      }
      return DateTime((uint32_t)0);
    }
};

Relogio relogio;

// =========================================================================
// LOG CENTRALIZADO — implementação (usa relogio.timestamp())
// =========================================================================

/**
 * @brief Log na Serial E no ring buffer. Este é o ponto único de entrada
 *        para logs de eventos. Todo o código deve chamar isto (via
 *        registrarEvento) em vez de Serial.println direto.
 */
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

  // [v7.3] Alimenta o ring buffer (exceto se o buffer ainda não foi iniciado)
  // Mapeia o nível string para o enum numérico
  uint8_t n = NIVEL_INFO;
  if      (nivel == "WARN")  n = NIVEL_WARN;
  else if (nivel == "ALERT") n = NIVEL_ALERT;
  else if (nivel == "CRIT")  n = NIVEL_CRIT;

  // Formato compacto no buffer: "[TAG] mensagem"
  char buf[HIST_MSG_MAX];
  snprintf(buf, sizeof(buf), "[%s] %s", tag.c_str(), mensagem.c_str());
  logBuffer.push(n, buf);
}

// Wrapper semântico — quem chama registrarEvento está ciente de que é
// um evento de auditoria (não um log interno qualquer).
void registrarEvento(const String &nivel, const String &tag, const String &mensagem) {
  logSerial(nivel, tag, mensagem);
}

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
      if (millis() - _tempoInicio < 1500) { _estado = false; return; }
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
    bool          _estado = false;
    unsigned long _tempoUltimaMudanca = 0;
    unsigned long _tempoInicio = 0;
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
      _distancia   = -1.0f;
    }

    void update() {
      if (millis() - _tempoInicio < 1500) { _distancia = -1.0f; return; }
      float soma = 0.0f;
      int validas = 0;
      for (int i = 0; i < US_AMOSTRAS; i++) {
        float d = _medirBruto();
        if (d >= 2.0f && d < US_DIST_MAX_VALIDA) { soma += d; validas++; }
        delayMicroseconds(60);
      }
      _distancia = (validas > 0) ? (soma / validas) : -1.0f;
    }

    float distancia() const { return _distancia; }

  private:
    float _distancia = -1.0f;
    unsigned long _tempoInicio = 0;

    float _medirBruto() const {
      digitalWrite(Pins::TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(Pins::TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(Pins::TRIG, LOW);
      long duracao = pulseIn(Pins::ECHO, HIGH, 25000);
      float d = duracao * 0.0343f / 2.0f;
      return (d <= 0.0f) ? -1.0f : d;
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
      if (millis() - _tempoInicio < 1500) { _movimento = false; return; }
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

      if (_filtrado) { _tempoPressionado = millis(); return BOTAO_NENHUM; }

      unsigned long duracao = millis() - _tempoPressionado;
      if (millis() - _tempoUltimaAcao < DEBOUNCE_BOTAO_MS) return BOTAO_NENHUM;

      if (duracao >= DURACAO_CLIQUE_LONGO_MS) {
        _tempoUltimaAcao = millis();
        _cliques = 0;
        return BOTAO_LONGO;
      }

      if (duracao >= 50) { _cliques++; _tempoUltimoClique = millis(); }

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
// MÓDULO: BOTÃO DE PÂNICO (ISR)
// =========================================================================

class BotaoPanico {
  public:
    void begin() {
      pinMode(BOTAO_PANICO, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(BOTAO_PANICO), _isr, FALLING);
      _flagISR = false;
      _tempoUltimoDisparo = 0;
    }

    bool consumir() {
      if (millis() - _tempoUltimoDisparo < COOLDOWN_MS) {
        _flagISR = false;
        return false;
      }
      if (_flagISR) {
        _flagISR = false;
        _tempoUltimoDisparo = millis();
        return true;
      }
      return false;
    }

  private:
    static volatile bool _flagISR;
    unsigned long _tempoUltimoDisparo = 0;
    static constexpr unsigned long COOLDOWN_MS = 2000UL;

    static void IRAM_ATTR _isr();
};

volatile bool BotaoPanico::_flagISR = false;

void __attribute__((noinline)) IRAM_ATTR BotaoPanico::_isr() {
  _flagISR = true;
}

BotaoPanico botaoPanico;

// =========================================================================
// MÓDULO: RFID
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
      ev.valido = false;

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

    void forcarSilencio() {
      parar();
      _prioridadeAtual = 0;
      _tempoUltimoBip  = 0;
    }

    void beepCurto(int freq = 1200, int dur = 100) {
      if (_prioridadeAtual < 3) tone(Pins::BUZZER_E, freq, dur);
    }

    void beepErro() {
      if (_prioridadeAtual < 3) tone(Pins::BUZZER_D, 500, 200);
    }

    void beepSucesso() {
      if (_prioridadeAtual < 3) tone(Pins::BUZZER_E, 800, 120);
    }

    void tocar(EstadoSistema estado, bool bloqueioAtivo, EstadoAmbiental ambiente) {
      unsigned long agora = millis();

      int prioridade = 0;
      if (estado == EMERGENCIA_RACK || estado == INVASAO_SALA) {
        prioridade = 3;
      } else if (estado == AVISO_SALA || estado == APROXIMACAO_RACK || bloqueioAtivo) {
        prioridade = 2;
      } else if (ambiente == AMB_CRITICO || ambiente == AMB_ALERTA) {
        prioridade = 1;
      }

      if (prioridade != _prioridadeAtual) {
        parar();
        _tempoUltimoBip = 0;
      }
      _prioridadeAtual = prioridade;

      switch (prioridade) {
        case 3:
          if (estado == EMERGENCIA_RACK) {
            if (agora - _tempoUltimoBip >= 160) {
              static bool alt = false;
              tone(Pins::BUZZER_D, alt ? 900 : 1300, 140);
              alt = !alt;
              _tempoUltimoBip = agora;
            }
          } else if (estado == INVASAO_SALA) {
            if (agora - _tempoUltimoBip >= 25) {
              static int freq = 400, passo = 25;
              tone(Pins::BUZZER_D, freq, 20);
              freq += passo;
              if (freq >= 900 || freq <= 400) passo = -passo;
              _tempoUltimoBip = agora;
            }
          }
          break;

        case 2:
          if (bloqueioAtivo) {
            if (agora - _tempoUltimoBip >= 800) {
              tone(Pins::BUZZER_D, 500, 100);
              _tempoUltimoBip = agora;
            }
          } else if (estado == AVISO_SALA) {
            if (agora - _tempoUltimoBip >= 700) {
              tone(Pins::BUZZER_E, 523, 100);
              _tempoUltimoBip = agora;
            }
          } else if (estado == APROXIMACAO_RACK) {
            if (agora - _tempoUltimoBip >= 400) {
              tone(Pins::BUZZER_E, 750, 80);
              _tempoUltimoBip = agora;
            }
          }
          break;

        case 1:
          if (ambiente == AMB_CRITICO) {
            if (agora - _tempoUltimoBip >= 3000) {
              tone(Pins::BUZZER_D, 400, 200);
              _tempoUltimoBip = agora;
            }
          } else if (ambiente == AMB_ALERTA) {
            if (agora - _tempoUltimoBip >= 8000) {
              tone(Pins::BUZZER_E, 700, 150);
              _tempoUltimoBip = agora;
            }
          }
          break;

        case 0:
          parar();
          break;
      }
    }

  private:
    int _prioridadeAtual = 0;
    unsigned long _tempoUltimoBip = 0;
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
                       const String &uidDesarme, bool cartaoOk,
                       unsigned int contAberturas, unsigned int contInvasoes,
                       const String &uidRearme) {
      display.clearDisplay();
      String linha1, linha2;
      int icone = 1;

      switch (estado) {
        case SEGURO:
          linha1 = "Vigilante Armado"; linha2 = "Porta Fechada"; icone = 1;
          break;
        case ACESSO_RFID:
          linha1 = cartaoOk ? "Cartao Autorizado" : "Cartao Negado";
          linha2 = "UID: " + uidDesarme;
          icone  = cartaoOk ? 1 : 3;
          break;
        case AVISO_SALA:
          linha1 = "Aviso: Movimento"; linha2 = "Rack Fechado"; icone = 2;
          break;
        case APROXIMACAO_RACK:
          linha1 = "Aproximacao";
          if (s.distancia < 0.0f) linha2 = "Dist: N/A";
          else                    linha2 = "Dist: " + String(s.distancia, 1) + "cm";
          icone = 2;
          break;
        case INVASAO_SALA:
          linha1 = "Invasao na Sala";
          linha2 = "Mov+Luz | #" + String(contInvasoes);
          icone  = 3;
          break;
        case EMERGENCIA_RACK:
          linha1 = "Emergencia: Rack";
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
      display.print(relogio.timestamp().substring(6));
      display.print(" ");
      if (estadoAmbiental == AMB_CRITICO)      display.print("[!]");
      else if (estadoAmbiental == AMB_ALERTA)  display.print("[*]");
      else                                     display.print("[ ]");

      if (_ultimaTempValida > -100.0f) {
        display.print(String(_ultimaTempValida, 0));
        display.print("C/");
        display.print(String(_ultimaUmidValida, 0));
        display.print("%");
      } else {
        display.print("--C/--%");
      }

      display.setCursor(0, 60);
      display.print("UID:");
      String uidExibir = (modo == ARMADO)
                         ? (uidRearme.length() > 0 ? uidRearme : "-")
                         : uidDesarme;
      display.print(uidExibir);

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
// MÓDULO: DHT22
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
// MÓDULO: ALARME (v7.2 — persiste modo na NVS)
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
    bool          ultimoCartaoValido   = false;
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

    void dispararPanico(const String &origem) {
      if (modo == DESARMADO) {
        logSerial("WARN", "PANICO",
                  "Botao de panico ignorado: sistema DESARMADO (origem: " + origem + ").");
        enviarLogSite("Panico ignorado: sistema desarmado.");
        audio.beepErro();
        return;
      }
      if (estadoAtual == INVASAO_SALA) {
        logSerial("WARN", "PANICO", "Ja em INVASAO_SALA, ignorando novo disparo.");
        return;
      }

      estadoAnterior  = estadoAtual;
      estadoAtual     = INVASAO_SALA;
      estadoCandidato = INVASAO_SALA;
      tempoEstadoCandidato = millis();

      contadorInvasoes++;
      contadoresDirty = true;

      logSerial("CRIT", "PANICO",
                "BOTAO DE PANICO ACIONADO (origem: " + origem + ") | Invasao #" +
                String(contadorInvasoes));
      enviarLogSite("CRITICO: Botao de panico acionado! Invasao #" +
                    String(contadorInvasoes));
    }

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
    bool _primeiraLeitura     = true;

    void _logBordas(const LeiturasSensores &s) {
      if (_primeiraLeitura) {
        _estadoAnteriorPIR = s.movimento;
        _estadoAnteriorLDR = s.luzAcesa;
        _primeiraLeitura   = false;
        return;
      }

      if (s.movimento != _estadoAnteriorPIR) {
        if (modo == DESARMADO && s.movimento) {
          contadorMovimentosManutencao++;
          contadoresDirty = true;
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
          contadoresDirty = true;
          if (modo == ARMADO) {
            logSerial("ALERT", "PORTA", "Porta do rack aberta (total: " + String(contadorAberturasPorta) + ")");
            enviarLogSite("Porta do rack aberta");
          } else {
            logSerial("INFO", "PORTA", "Porta aberta em manutencao (total: " + String(contadorAberturasPorta) + ")");
            enviarLogSite("Porta aberta (manutencao)");
          }
        } else {
          logSerial("INFO", "PORTA", "Porta fechada");
          enviarLogSite("Porta do rack fechada");
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

      ultimoCartaoValido = valido;

      if (valido) {
        enviarLogSite("Cartao autorizado: " + ev.uid);
        contadorAcessosValidos++;
        contadoresDirty = true;
        logSerial("INFO", "RFID", "Cartao autorizado: " + ev.uid);
        _alternarModo(ev.uid, porta, audio);
      } else {
        logSerial("WARN", "RFID", "Cartao negado: " + ev.uid);
        enviarLogSite("Cartao negado: " + ev.uid);
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
        contadoresDirty    = true;
        motivoBloqueio     = "";
        prefs.begin("rack", false);
        prefs.putUInt("modo", (uint8_t)DESARMADO);
        prefs.end();
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
        contadoresDirty    = true;
        motivoBloqueio     = "Feche a porta!";
        tempoBloqueioAtivo = millis();

        logSerial("WARN", "VIGILANTE",
                  "Rearme bloqueado: porta aberta | " +
                  String(contadorRearmesBloqueados) + " bloqueio(s) ate agora.");
        logSerial("WARN", "VIGILANTE",
                  "Feche a porta e passe o cartao novamente para armar.");
        enviarLogSite("Rearme bloqueado: porta aberta");
        audio.beepErro();
        return false;
      }
      duracaoUltimoDesarme = (millis() - tempoInicioDesarme) / 1000UL;
      modo                = ARMADO;
      uidUltimoRearme     = uidUltimoDesarme;
      motivoBloqueio      = "";
      prefs.begin("rack", false);
      prefs.putUInt("modo", (uint8_t)ARMADO);
      prefs.end();
      logSerial("INFO", "VIGILANTE",
                "Rearmado | Duracao do desarme: " + String(duracaoUltimoDesarme) + "s");
      enviarLogSite("Sistema rearmado com sucesso");
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
      prefs.begin("rack", false);
      prefs.putUInt("modo", (uint8_t)ARMADO);
      prefs.end();
      logSerial("WARN", "VIGILANTE",
                "Rearme automatico por timeout | Duracao: " +
                String(duracaoUltimoDesarme) + "s");
      audio.beepCurto(1500, 200);
      audio.beepCurto(2000, 300);
      estadoAtual     = SEGURO;
      estadoAnterior  = SEGURO;
      estadoCandidato = SEGURO;
    }

    void _atualizarEstados(const LeiturasSensores &s, const EventoRFID &rfidEv) {
      EstadoSistema candidato;
      bool rfidRecente = rfidEv.novo || (millis() - ultimoUIDtempoRFID < 2500);

      if (s.portaAberta) {
        candidato = EMERGENCIA_RACK;
      }
      else if (rfidRecente) {
        candidato = ACESSO_RFID;
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
          contadoresDirty = true;
          logSerial("CRIT", "SEGURANCA", "Invasao fisica detectada no rack!");
        }
        if (estadoAtual == APROXIMACAO_RACK) {
          contadorAproximacoes++;
          contadoresDirty = true;
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
// MÓDULO: NVS
// =========================================================================

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

void carregarCartoes() {
  cartoesAutorizados.clear();

  auto adicionarSeNovo = [](const String &uid) {
    for (const String &c : cartoesAutorizados) if (c == uid) return;
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
  uint8_t modoSalvo = prefs.getUInt("modo", (uint8_t)ARMADO);
  alarme.modo = (modoSalvo == (uint8_t)DESARMADO) ? DESARMADO : ARMADO;
  prefs.end();
  logSerial("INFO", "NVS", "Configuracoes carregadas da flash.");
  logSerial("INFO", "NVS", "Modo restaurado: " + String(alarme.modo == ARMADO ? "ARMADO" : "DESARMADO"));
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
      _client.setBufferSize(512);
      _client.setKeepAlive(120);   // [v7.3] reduz rc=-3 em broker público

      _client.setCallback([this](char* topic, byte * payload, unsigned int length) {
        _tratarComando(topic, payload, length);
      });
    }

    void update() {
      if (WiFi.status() != WL_CONNECTED) {
        if (millis() - _ultimoTempoReconexao > MQTT_RETRY_MS) {
          _ultimoTempoReconexao = millis();
          WiFi.reconnect();
        }
        return;
      }

      if (!_client.connected()) {
        if (_estavaConectado) {
          logSerial("WARN", "MQTT", "Desconectado do broker (rc=" + String(_client.state()) + ").");
          _estavaConectado = false;
        }

        if (millis() - _ultimoTempoReconexao > MQTT_RETRY_MS) {
          _ultimoTempoReconexao = millis();
          _client.disconnect();
          String clientId = "RackESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
          Serial.print(F("[MQTT] Conectando ao broker... "));

          if (_client.connect(clientId.c_str(), nullptr, nullptr,
                              MQTT_TOPIC_LWT, 1, true, "{\"status\":\"offline\"}")) {
            Serial.println(F("OK"));
            _estavaConectado = true;
            _client.publish(MQTT_TOPIC_LWT, "{\"status\":\"online\"}", true);
            _client.subscribe(MQTT_TOPIC_CMD);
            _client.subscribe(MQTT_TOPIC_CMD_AMBIENTE);
            logSerial("INFO", "MQTT", String("Inscrito em: ") + MQTT_TOPIC_CMD
                      + " e " + MQTT_TOPIC_CMD_AMBIENTE);
            _precisaPublicarInicial = true;
          } else {
            Serial.print(F("Falha rc=")); Serial.println(_client.state());
          }
        }
        return;
      }

      _client.loop();

      if (_precisaPublicarInicial) {
        _precisaPublicarInicial = false;
        _publicar(true);
        _publicarAmbiente();
      }

      if (alarme.estadoAtual != _ultimoEstadoPublicado ||
          alarme.modo       != _ultimoModoPublicado ||
          porta.estaAberta() != _ultimoEstadoPortaPublicado) {
        _publicar();
      }

      if (millis() - _ultimoHeartbeat > INTERVALO_HEARTBEAT_MS) {
        _ultimoHeartbeat = millis();
        String hbPayload = "{\"ts\":\"" + relogio.timestampISO() +
                           "\",\"uptime_s\":" + String(millis() / 1000) +
                           ",\"time_source\":\"" + relogio.fonteTempoStr() + "\"}";
        _client.publish(MQTT_TOPIC_HB, hbPayload.c_str());
      }

      if (millis() - _ultimoAmbientePublicado > INTERVALO_AMBIENTE_MS) {
        _ultimoAmbientePublicado = millis();
        _publicarAmbiente();
      }
    }

    bool estaConectado() { return _client.connected(); }

    bool publicarLog(const String &payload) {
      return _client.publish(MQTT_TOPIC_LOGS, payload.c_str());
    }

  private:
    WiFiClient     _wifiClient;
    PubSubClient   _client{_wifiClient};
    unsigned long  _ultimoTempoReconexao = 0;
    unsigned long  _ultimoHeartbeat      = 0;
    const unsigned long INTERVALO_HEARTBEAT_MS = 30000UL;
    EstadoSistema  _ultimoEstadoPublicado = SEGURO;
    ModoVigilante  _ultimoModoPublicado   = ARMADO;

    bool           _estavaConectado       = false;
    bool           _precisaPublicarInicial = true;
    bool           _ultimoEstadoPortaPublicado = false;
    unsigned long  _ultimoStatusForcado   = 0;
    const unsigned long STATUS_RATE_LIMIT_MS = 10000UL;

    unsigned long _ultimoAmbientePublicado = 0;
    const unsigned long INTERVALO_AMBIENTE_MS = 30000UL;

    void _publicar(bool forcar = false) {
      if (!forcar && alarme.estadoAtual == _ultimoEstadoPublicado &&
          alarme.modo == _ultimoModoPublicado &&
          porta.estaAberta() == _ultimoEstadoPortaPublicado) {
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
      payload += "\",\"time_source\":\"";
      payload += relogio.fonteTempoStr();
      payload += "\"";
      payload += ",\"porta\":";
      payload += (porta.estaAberta() ? "true" : "false");
      payload += ",\"luz\":";
      payload += (ldr.estaAcesa() ? "true" : "false");
      float dist = ultrassonico.distancia();
      payload += ",\"dist_cm\":";
      if (dist < 0.0f) payload += "null";
      else             payload += String(dist, 1);
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

      payload += ",\"temp_c\":" + String(_ultimaTempValida, 1);
      payload += ",\"umid_pct\":" + String(_ultimaUmidValida, 1);
      payload += "}";

      Serial.print(F("[MQTT] Publicando em "));
      Serial.print(MQTT_TOPIC_STATUS);
      Serial.print(F(": "));
      Serial.println(payload);

      bool ok = _client.publish(MQTT_TOPIC_STATUS, payload.c_str(), true);
      if (!ok) {
        Serial.println(F("[ERRO] Falha ao publicar status! Buffer cheio?"));
        return;
      }

      _ultimoEstadoPublicado = alarme.estadoAtual;
      _ultimoModoPublicado   = alarme.modo;
      _ultimoEstadoPortaPublicado = porta.estaAberta();
    }

    void _publicarAmbiente() {
      String payload = "{";
      payload += "\"ts\":\"" + relogio.timestampISO() + "\"";
      payload += ",\"time_source\":\"" + relogio.fonteTempoStr() + "\"";
      payload += ",\"temp_c\":" + String(_ultimaTempValida, 1);
      payload += ",\"umid_pct\":" + String(_ultimaUmidValida, 1);
      payload += ",\"estado\":\"";
      payload += (estadoAmbiental == AMB_CRITICO ? "CRITICO" :
                  estadoAmbiental == AMB_ALERTA  ? "ALERTA"  : "OK");
      payload += "\",\"alerta_temp\":";
      payload += (alertaTempAtivo ? "true" : "false");
      payload += ",\"critico_temp\":";
      payload += (criticoTempAtivo ? "true" : "false");
      payload += ",\"alerta_umid\":";
      payload += (alertaUmidAtivo ? "true" : "false");
      payload += ",\"critico_umid\":";
      payload += (criticoUmidAtivo ? "true" : "false");
      payload += ",\"taxa_temp_alerta\":";
      payload += (taxaTempAlerta ? "true" : "false");
      payload += ",\"sensor_dht_ok\":";
      payload += (_sensorDHTPerdido ? "false" : "true");
      payload += ",\"limiares\":{\"t_alert\":" + String(limiarTempAlerta, 1) +
                 ",\"t_crit\":"  + String(limiarTempCritico, 1) +
                 ",\"u_alert\":" + String(limiarUmidAlerta, 1) +
                 ",\"u_crit\":"  + String(limiarUmidCritico, 1) + "}";
      payload += "}";

      bool ok = _client.publish(MQTT_TOPIC_AMBIENTE, payload.c_str(), true);
      if (!ok) {
        Serial.println(F("[WARN] Falha ao publicar ambiente (buffer cheio?)"));
      }
    }

    void _tratarComando(char* topic, byte* payload, unsigned int length) {
      String cmd;
      for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
      cmd.trim();

      String cmdOriginal = cmd;
      cmd.toUpperCase();

      logSerial("INFO", "MQTT_CMD", String("Recebido [") + topic + "]: " + cmd);

      if (String(topic) == MQTT_TOPIC_CMD_AMBIENTE) {
        if (cmd.startsWith("SET_TEMP_ALERTA ")) {
          float v = cmd.substring(16).toFloat();
          if (v >= 15 && v <= 50) {
            limiarTempAlerta = v;
            salvarConfig();
            logSerial("INFO", "MQTT_AMB", "Temp alerta: " + String(v, 1) + "C");
            _publicarAmbiente();
          } else logSerial("WARN", "MQTT_AMB", "Fora da faixa (15-50).");
        }
        else if (cmd.startsWith("SET_TEMP_CRIT ")) {
          float v = cmd.substring(14).toFloat();
          if (v >= 20 && v <= 60) {
            limiarTempCritico = v;
            salvarConfig();
            logSerial("INFO", "MQTT_AMB", "Temp critico: " + String(v, 1) + "C");
            _publicarAmbiente();
          } else logSerial("WARN", "MQTT_AMB", "Fora da faixa (20-60).");
        }
        else if (cmd.startsWith("SET_UMID_ALERTA ")) {
          float v = cmd.substring(16).toFloat();
          if (v >= 30 && v <= 90) {
            limiarUmidAlerta = v;
            salvarConfig();
            logSerial("INFO", "MQTT_AMB", "Umid alerta: " + String(v, 1) + "%");
            _publicarAmbiente();
          } else logSerial("WARN", "MQTT_AMB", "Fora da faixa (30-90).");
        }
        else if (cmd.startsWith("SET_UMID_CRIT ")) {
          float v = cmd.substring(14).toFloat();
          if (v >= 40 && v <= 100) {
            limiarUmidCritico = v;
            salvarConfig();
            logSerial("INFO", "MQTT_AMB", "Umid critico: " + String(v, 1) + "%");
            _publicarAmbiente();
          } else logSerial("WARN", "MQTT_AMB", "Fora da faixa (40-100).");
        }
        else if (cmd == "PANICO") {
          alarme.dispararPanico("MQTT");
        }
        else if (cmd == "STATUS_AMB") {
          _publicarAmbiente();
        }
        else {
          logSerial("WARN", "MQTT_AMB", "Comando ambiente desconhecido: " + cmd);
        }
        return;
      }

      if (cmd == "DESARMAR") {
        if (alarme.modo == ARMADO) {
          alarme.modo               = DESARMADO;
          alarme.tempoInicioDesarme = millis();
          alarme.uidUltimoDesarme   = "MQTT";
          alarme.aviso5minDisparado = false;
          alarme.contadorDesarmes++;
          contadoresDirty = true;
          prefs.begin("rack", false);
          prefs.putUInt("modo", (uint8_t)DESARMADO);
          prefs.end();
          logSerial("INFO", "MQTT_CMD", "Desarmado remoto executado.");
          enviarLogSite("Comando remoto: desarmar executado");
        } else {
          logSerial("WARN", "MQTT_CMD", "Ja desarmado, ignorando.");
          enviarLogSite("Comando remoto: desarmar (ja desarmado)");
        }
      }
      else if (cmd == "ARMAR") {
        if (alarme.modo == DESARMADO) {
          if (alarme.armarViaSerial(porta, audio)) {
            logSerial("INFO", "MQTT_CMD", "Armado remoto executado.");
            enviarLogSite("Comando remoto: armar executado");
          } else {
            logSerial("WARN", "MQTT_CMD", "Armar remoto bloqueado (porta aberta).");
            enviarLogSite("Comando remoto: armar bloqueado");
          }
        } else {
          logSerial("WARN", "MQTT_CMD", "Ja armado, ignorando.");
          enviarLogSite("Comando remoto: armar (ja armado)");
        }
      }
      else if (cmd == "STATUS") {
        if (millis() - _ultimoStatusForcado < STATUS_RATE_LIMIT_MS) {
          logSerial("WARN", "MQTT_CMD", "STATUS ignorado (rate limit).");
          return;
        }
        _ultimoStatusForcado = millis();
        logSerial("INFO", "MQTT_CMD", "Enviando STATUS sob demanda.");
        _publicar(true);
      }
      else if (cmd.startsWith("RESET")) {
        String token = cmdOriginal.substring(5);
        token.trim();

        if (token != MQTT_RESET_TOKEN) {
          logSerial("WARN", "MQTT_CMD", "RESET negado: token invalido.");
          enviarLogSite("Comando remoto: reset negado (token invalido)");
        } else {
          alarme.contadorAberturasPorta       = 0;
          alarme.contadorAproximacoes         = 0;
          alarme.contadorAcessosValidos       = 0;
          alarme.contadorMovimentosManutencao = 0;
          alarme.contadorDesarmes             = 0;
          alarme.resetContadoresPendente = false;
          salvarConfig();
          logSerial("INFO", "MQTT_CMD", "Reset remoto executado (seguranca preservada).");
          enviarLogSite("Comando remoto: reset operacional executado");
          _publicar(true);
        }
      }
      else if (cmd == "TESTE") {
        alarme.modoTestePendente = true;
        logSerial("INFO", "MQTT_CMD", "Modo teste remoto executado.");
        enviarLogSite("Comando remoto: teste iniciado");
      }
      else {
        logSerial("WARN", "MQTT_CMD", "Comando desconhecido: " + cmd);
      }
    }
};

NotificadorMQTT notificador;

// =========================================================================
// ALARME AMBIENTAL
// =========================================================================

void _verificarAmbiente(float temp, float umid) {
  if (temp == -999.0f || umid == -999.0f) {
    if (_ultimaLeituraValidaDHT > 0 &&
        (millis() - _ultimaLeituraValidaDHT) > TIMEOUT_DHT_MS) {
      if (!_sensorDHTPerdido) {
        _sensorDHTPerdido = true;
        alertaTempAtivo  = false;
        criticoTempAtivo = false;
        alertaUmidAtivo  = false;
        criticoUmidAtivo = false;
        taxaTempAlerta   = false;
        if (estadoAmbiental != AMB_OK) {
          estadoAmbiental = AMB_OK;
          audio.forcarSilencio();
        }
        logSerial("WARN", "AMBIENTE", "DHT22 sem leitura ha 10s. Estado ambiental resetado.");
        enviarLogSite("Sensor DHT22 sem leitura. Monitoramento ambiental pausado.");
      }
    }
    return;
  }

  _ultimaLeituraValidaDHT = millis();
  if (_sensorDHTPerdido) {
    _sensorDHTPerdido = false;
    logSerial("INFO", "AMBIENTE", "DHT22 voltou a responder. Retomando monitoramento.");
    enviarLogSite("Sensor DHT22 restabelecido.");
  }

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
    else if (taxaTempAlerta &&
             (temp < limiarTempAlerta || taxa < (TAXA_ALERTA_C_POR_MIN * 0.5f))) {
      taxaTempAlerta = false;
      logSerial("INFO", "AMBIENTE",
                "Taxa de subida normalizada: " + String(taxa, 2) + " C/min");
    }

    _tempAnteriorTaxa  = temp;
    _tempoAnteriorTaxa = agora;
  }

  if (taxaTempAlerta && temp < limiarTempAlerta) {
    taxaTempAlerta = false;
    logSerial("INFO", "AMBIENTE",
              "Taxa de subida resetada (temperatura segura: " + String(temp, 1) + "C).");
  }

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

    if (estadoAmbiental != AMB_OK && novoEstado == AMB_OK) {
      audio.forcarSilencio();
    }

    estadoAmbiental = novoEstado;
  }

#if ALARME_SONORO_AMBIENTAL
  bool segurancaCritica = (alarme.estadoAtual == EMERGENCIA_RACK ||
                           alarme.estadoAtual == INVASAO_SALA);
  if (!segurancaCritica) {
    static unsigned long _ultimoBipAmbiental = 0;
    unsigned long agoraBip = millis();

    if (estadoAmbiental == AMB_CRITICO && agoraBip - _ultimoBipAmbiental >= 3000) {
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
// HELPER JSON
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
    if (!notificador.publicarLog(payload)) {
      Serial.println(F("[WARN] Fila MQTT cheia, log descartado."));
    }
  }
}

// =========================================================================
// [v7.2] OTA — configuração e callbacks
// =========================================================================

void configurarOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setPort(OTA_PORT);

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    logSerial("INFO", "OTA", "Atualizacao iniciada (" + type + ").");
    enviarLogSite("OTA: atualizacao iniciada (" + type + ")");
    audio.forcarSilencio();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned long _ultimoLog = 0;
    if (millis() - _ultimoLog > 2000) {
      _ultimoLog = millis();
      int pct = (progress * 100) / total;
      logSerial("INFO", "OTA", "Progresso: " + String(pct) + "%");
    }
  });

  ArduinoOTA.onEnd([]() {
    logSerial("INFO", "OTA", "Atualizacao concluida. Reiniciando...");
    enviarLogSite("OTA: atualizacao concluida com sucesso");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    String motivo = "desconhecido";
    if (error == OTA_AUTH_ERROR)         motivo = "autenticacao";
    else if (error == OTA_BEGIN_ERROR)   motivo = "inicio";
    else if (error == OTA_CONNECT_ERROR) motivo = "conexao";
    else if (error == OTA_RECEIVE_ERROR) motivo = "recepcao";
    else if (error == OTA_END_ERROR)     motivo = "finalizacao";
    logSerial("CRIT", "OTA", "Erro na atualizacao: " + motivo);
    enviarLogSite("OTA: erro na atualizacao (" + motivo + ")");
  });

  ArduinoOTA.begin();
  logSerial("INFO", "OTA", "Servico OTA ativo em " + String(OTA_HOSTNAME) +
            ":" + String(OTA_PORT) + " (senha configurada).");
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
    Serial.print(F("  Temp/umid atuais:     T="));
    Serial.print(_ultimaTempValida, 1);
    Serial.print(F("C U=")); Serial.print(_ultimaUmidValida, 1); Serial.println(F("%"));
    Serial.print(F("  Fonte de tempo:       "));
    Serial.print(relogio.fonteTempoStr());
    Serial.print(F(" | Hora: "));
    Serial.println(relogio.timestamp());
    Serial.print(F("  Historico (buffer):   "));
    Serial.print(logBuffer.tamanho()); Serial.print(F("/"));
    Serial.println(HIST_TAMANHO);
    Serial.print(F("  WiFi:                 "));
    Serial.println(WiFi.status() == WL_CONNECTED ? "conectado" : "desconectado");
    Serial.print(F("  MQTT:                 "));
    Serial.println(notificador.estaConectado() ? "conectado" : "desconectado");
    Serial.print(F("  OTA:                  "));
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(OTA_HOSTNAME); Serial.print(":"); Serial.print(OTA_PORT);
      Serial.println(F(" (ativo)"));
    } else {
      Serial.println(F("(Wi-Fi offline)"));
    }
    Serial.println(F("-------------------------------------"));
  }
  else if (cmd == "OTA") {
    Serial.println(F("----------- STATUS OTA -----------"));
    Serial.print(F("  Hostname:  ")); Serial.println(OTA_HOSTNAME);
    Serial.print(F("  Porta:     ")); Serial.println(OTA_PORT);
    Serial.print(F("  Senha:     ")); Serial.println(OTA_PASSWORD);
    Serial.print(F("  IP:        "));
    if (WiFi.status() == WL_CONNECTED) Serial.println(WiFi.localIP());
    else                               Serial.println(F("Wi-Fi offline"));
    Serial.print(F("  Ativo:     "));
    Serial.println(WiFi.status() == WL_CONNECTED ? "sim" : "nao (Wi-Fi offline)");
    Serial.println(F("-----------------------------------"));
    Serial.println(F("  No Arduino IDE: Ferramentas > Porta > rede"));
    Serial.println(F("  Selecione o dispositivo e faca upload normalmente."));
  }
  else if (cmd == "HIST") {
    logBuffer.dumpSerial();
  }
  else if (cmd == "HIST_CLEAR") {
    logBuffer.clear();
    logSerial("INFO", "CMD", "Comando HIST_CLEAR executado.");
  }
  else if (cmd == "HIST_JSON") {
    logBuffer.dumpJSON();
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
      prefs.begin("rack", false);
      prefs.putUInt("modo", (uint8_t)DESARMADO);
      prefs.end();
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
  else if (cmd == "PANICO") {
    logSerial("INFO", "CMD", "Executando panico via Serial.");
    alarme.dispararPanico("SERIAL");
  }
  else if (cmd == "RESET") {
    alarme.contadorAberturasPorta       = 0;
    alarme.contadorAproximacoes         = 0;
    alarme.contadorAcessosValidos       = 0;
    alarme.contadorMovimentosManutencao = 0;
    alarme.contadorDesarmes             = 0;
    salvarConfig();
    logSerial("INFO", "CMD", "Contadores operacionais zerados (seguranca preservada).");
  }
  else if (cmd.startsWith("RTC ")) {
    int ano, mes, dia, hora, min, seg;
    if (sscanf(cmd.c_str(), "RTC %d-%d-%d %d:%d:%d",
               &ano, &mes, &dia, &hora, &min, &seg) == 6) {
      if (relogio.ajustarManual(ano, mes, dia, hora, min, seg)) {
        logSerial("INFO", "RTC", "Ajuste manual OK. Fonte: " + relogio.fonteTempoStr());
      }
    } else {
      logSerial("WARN", "CMD", "Formato: RTC YYYY-MM-DD HH:MM:SS");
    }
  }
  else if (cmd == "RTC") {
    logSerial("INFO", "RTC", "Hora atual: " + relogio.timestamp() +
              " | Fonte: " + relogio.fonteTempoStr());
  }
  else if (cmd == "NTP") {
    if (relogio.sincronizarNTP()) {
      logSerial("INFO", "NTP", "Sync manual OK. Fonte: " + relogio.fonteTempoStr());
    } else {
      logSerial("WARN", "NTP", "Sync manual falhou.");
    }
  }
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
    for (const String &card : cartoesAutorizados) Serial.println(card);
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
    Serial.println(F("  PANICO                  - Dispara panico manual"));
    Serial.println(F("  RTC                     - Mostra hora e fonte de tempo"));
    Serial.println(F("  RTC YYYY-MM-DD HH:MM:SS - Ajusta RTC manualmente"));
    Serial.println(F("  NTP                     - Forca sincronizacao NTP manual"));
    Serial.println(F("  OTA                     - Mostra status do servico OTA"));
    Serial.println(F("  HIST                    - Despeja historico de eventos (Serial)"));
    Serial.println(F("  HIST_JSON               - Despeja historico em JSON"));
    Serial.println(F("  HIST_CLEAR              - Limpa o historico de eventos"));
    Serial.println(F("  RESET                   - Zera contadores operacionais (preserva seguranca)"));
    Serial.println(F("  ADD_CARD <UID>          - Cadastra novo cartao na NVS"));
    Serial.println(F("  LIST_CARDS              - Lista cartoes autorizados"));
    Serial.println(F("  HELP                    - Lista comandos"));
    Serial.println(F("---------------------------------------------"));
    Serial.println(F("  BOTAO : curto=reset | longo=rearme | duplo=teste"));
    Serial.println(F("  PANICO: GPIO 26 (pushbutton para GND) - ISR, resposta <30ms"));
    Serial.println(F("  OTA   : Arduino IDE > Ferramentas > Porta > rede"));
    Serial.println(F("  HIST  : buffer circular de 100 eventos em RAM (perdido no reboot)"));
    Serial.println(F("  MQTT  : rack/status (pub) | rack/comando (sub seguranca)"));
    Serial.println(F("  MQTT  : rack/ambiente (pub retain 30s) | rack/comando/ambiente (sub)"));
    Serial.println(F("  MQTT  : rack/status/lwt | rack/logs | rack/heartbeat"));
    Serial.println(F("  MQTT_CMD seg: ARMAR | DESARMAR | STATUS | RESET <token> | TESTE"));
    Serial.println(F("  MQTT_CMD amb: SET_TEMP_ALERTA | SET_TEMP_CRIT | SET_UMID_ALERTA | SET_UMID_CRIT | PANICO | STATUS_AMB"));
    Serial.println(F("  NTP: pool.ntp.org | time.google.com | time.cloudflare.com"));
  }
  else {
    logSerial("WARN", "CMD", "Comando desconhecido. Digite HELP.");
  }
}

// =========================================================================
// MODO TESTE
// =========================================================================

void executarModoTeste() {
  logSerial("INFO", "TESTE", "=== INICIANDO AUTODIAGNOSTICO v3 ===");
  enviarLogSite("TESTE: Iniciando autodiagnostico...");

  int ok = 0, falhas = 0;
  String resumoFalhas = "";

  if (display.begin(SSD1306_SWITCHCAPVCC, ENDERECO_OLED)) {
    logSerial("INFO", "TESTE", "[OK] Display OLED (I2C) respondendo.");
    enviarLogSite("TESTE: OLED OK");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] Display OLED NAO responde.");
    enviarLogSite("TESTE: OLED FALHA");
    resumoFalhas += "OLED "; falhas++;
  }

  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20); display.println("MODO TESTE");
  display.setCursor(10, 40); display.println("Aguarde...");
  display.display();

  byte versao = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (versao != 0x00 && versao != 0xFF) {
    logSerial("INFO", "TESTE", "[OK] RFID MFRC522 (v0x" + String(versao, HEX) + ").");
    enviarLogSite("TESTE: RFID OK");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] RFID MFRC522 NAO responde.");
    enviarLogSite("TESTE: RFID FALHA");
    resumoFalhas += "RFID "; falhas++;
  }

  float d = ultrassonico.distancia();
  if (d > 0.0f && d < US_DIST_MAX_VALIDA) {
    logSerial("INFO", "TESTE", "[OK] Ultrassonico: " + String(d, 1) + " cm.");
    enviarLogSite("TESTE: Ultrassonico OK (" + String(d, 1) + " cm)");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] Ultrassonico: " + String(d, 1) + " cm.");
    enviarLogSite("TESTE: Ultrassonico FALHA");
    resumoFalhas += "US "; falhas++;
  }

  int ldrRaw = ldr.leituraBruta();
  logSerial("INFO", "TESTE", "[INFO] LDR: " + String(ldrRaw) + " (" + String(ldr.estaAcesa() ? "ACESA" : "APAGADA") + ").");
  enviarLogSite("TESTE: LDR " + String(ldr.estaAcesa() ? "ACESA" : "APAGADA"));
  ok++;

  logSerial("INFO", "TESTE", "[INFO] Porta: " + String(porta.estaAberta() ? "ABERTA" : "fechada") + ".");
  enviarLogSite("TESTE: Porta " + String(porta.estaAberta() ? "ABERTA" : "FECHADA"));
  ok++;

  logSerial("INFO", "TESTE", "[INFO] PIR: " + String(pir.detectaMovimento() ? "ATIVO" : "inativo") + ".");
  enviarLogSite("TESTE: PIR " + String(pir.detectaMovimento() ? "ATIVO" : "INATIVO"));
  ok++;

  float t = dht.temperatura();
  float u = dht.umidade();
  if (t > -50.0f && t < 80.0f) {
    logSerial("INFO", "TESTE", "[OK] DHT22: " + String(t, 1) + "C / " + String(u, 1) + "%.");
    enviarLogSite("TESTE: DHT22 OK (" + String(t, 1) + "C / " + String(u, 1) + "%)");
    ok++;
  } else {
    logSerial("CRIT", "TESTE", "[FALHA] DHT22 sem leitura valida.");
    enviarLogSite("TESTE: DHT22 FALHA");
    resumoFalhas += "DHT22 "; falhas++;
  }

  logSerial("INFO", "TESTE", "[INFO] Testando buzzers...");
  enviarLogSite("TESTE: Testando Buzzers...");
  tone(Pins::BUZZER_E, 500, 150);  delay(200);
  tone(Pins::BUZZER_E, 1000, 150); delay(200);
  tone(Pins::BUZZER_E, 2000, 150); delay(200);
  tone(Pins::BUZZER_D, 800, 300);  delay(350);
  ok++;

  logSerial("INFO", "TESTE", "[INFO] Testando LED RGB...");
  enviarLogSite("TESTE: Testando LED RGB...");
  led.definirCor(255, 0, 0);     delay(300);
  led.definirCor(0, 255, 0);     delay(300);
  led.definirCor(0, 0, 255);     delay(300);
  led.definirCor(255, 255, 255); delay(300);
  led.definirCor(0, 0, 0);
  ok++;

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
  botaoPanico.begin();

  leitorRFID.begin();
  audio.begin();

  dht.begin();

  logBuffer.begin();   // [v7.3] Inicializa o ring buffer ANTES do primeiro log

  carregarConfig();
  carregarCartoes();

  Serial.println();
  Serial.println(F("================================================="));
  Serial.println(F("  SISTEMA DE MONITORAMENTO DE RACK - v7.3"));
  Serial.println(F("  + Ring Buffer + HIST (auditoria em RAM)"));
  Serial.println(F("  + OTA (ArduinoOTA)"));
  Serial.println(F("  + NTP + RTC Sync + fallback millis"));
  Serial.println(F("  + Tópico ambiente dedicado"));
  Serial.println(F("  + Comandos de ambiente segregados"));
  Serial.println(F("  + Botão de pânico com ISR (GPIO 26)"));
  Serial.println(F("================================================="));
  logSerial("INFO", "SISTEMA", "Sistema inicializado. Modo: " + String(alarme.modo == ARMADO ? "ARMADO" : "DESARMADO"));
  logSerial("INFO", "RFID", "Cartao 1 Autorizado: " + CARTAO_AUTORIZADO_1);
  logSerial("INFO", "RFID", "Cartao 2 Autorizado: " + CARTAO_AUTORIZADO_2);
  logSerial("INFO", "RTC", relogio.disponivel() ? "RTC DS3231 ativo." : "RTC ausente (usando millis).");
  logSerial("INFO", "VIGILANTE", "Timeout de desarme: " + String(alarme.timeoutDesarme / 60000) + " min");
  logSerial("INFO", "VIGILANTE", "Politica: rearme BLOQUEADO se porta estiver aberta.");
  logSerial("INFO", "PANICO", "Botao de panico no GPIO 26 (ISR, pull-up interno).");
  logSerial("INFO", "OTA", "Hostname: " + String(OTA_HOSTNAME) + " | Porta: " + String(OTA_PORT));
  logSerial("INFO", "HIST", "Buffer de " + String(HIST_TAMANHO) + " eventos (" +
            String(sizeof(EventoLog) * HIST_TAMANHO) + " bytes em RAM).");
  logSerial("INFO", "FILTROS",
            "LDR: " + String(LDR_LIMIAR_APAGA) + "-" + String(LDR_LIMIAR_ACENDE) +
            " deb " + String(DEBOUNCE_LDR_MS) + "ms | US media: " + String(US_AMOSTRAS));
  logSerial("INFO", "FILTROS",
            "Botao: deb " + String(DEBOUNCE_BOTAO_MS) + "ms | duplo <" +
            String(JANELA_CLIQUE_DUPLO_MS) + "ms | longo >" + String(DURACAO_CLIQUE_LONGO_MS) + "ms");
  logSerial("INFO", "FILTROS", "Porta: deb " + String(DEBOUNCE_PORTA_MS) + "ms");
  logSerial("INFO", "SISTEMA", "Digite HELP no Serial para comandos disponiveis.");
  Serial.println(F("-------------------------------------------------"));

  alertaTempAtivo  = false;
  criticoTempAtivo = false;
  alertaUmidAtivo  = false;
  criticoUmidAtivo = false;
  taxaTempAlerta   = false;
  estadoAmbiental  = AMB_OK;
  _ultimaLeituraValidaDHT = 0;
  _sensorDHTPerdido = false;
  _ultimaTempValida = -999.0f;
  _ultimaUmidValida = -999.0f;

  notificador.begin();

  if (WiFi.status() == WL_CONNECTED) {
    configurarOTA();
  } else {
    logSerial("WARN", "OTA", "Wi-Fi offline no boot. OTA indisponivel ate reconectar.");
  }

  if (WiFi.status() == WL_CONNECTED) {
    delay(1000);
    if (relogio.sincronizarNTP()) {
      logSerial("INFO", "NTP", "Sincronizacao inicial OK. Fonte: " +
                relogio.fonteTempoStr());
    } else {
      logSerial("WARN", "NTP", "Sync inicial falhou. Fonte atual: " +
                relogio.fonteTempoStr());
    }
  } else {
    logSerial("WARN", "NTP", "Wi-Fi offline no boot. Fonte: " +
              relogio.fonteTempoStr() + " (tentara ressync no loop).");
  }
}

// =========================================================================
// LOOP PRINCIPAL
// =========================================================================
unsigned long tempoUltimoBip   = 0;
unsigned long tempoUltimoPisca = 0;
bool          estadoPiscaLED   = false;

unsigned long tempoUltimoSalvamento = 0;
constexpr unsigned long INTERVALO_SALVAMENTO_MS = 60000UL;

void loop() {
  // 0. OTA
  ArduinoOTA.handle();

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
  if (botaoPanico.consumir()) {
    alarme.dispararPanico("BOTAO_FISICO");
  }
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
  if (tempAtual != -999.0f) _ultimaTempValida = tempAtual;
  if (umidAtual != -999.0f) _ultimaUmidValida = umidAtual;
  _verificarAmbiente(tempAtual, umidAtual);

  // 6. Ações pendentes
  if (alarme.audioPararPendente) {
    alarme.audioPararPendente = false;
    audio.parar();
  }
  if (alarme.resetContadoresPendente) {
    alarme.resetContadoresPendente = false;
    alarme.contadorAberturasPorta       = 0;
    alarme.contadorAproximacoes         = 0;
    alarme.contadorAcessosValidos       = 0;
    alarme.contadorMovimentosManutencao = 0;
    alarme.contadorDesarmes             = 0;
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
  } else {
    oled.mostrarStatus(alarme.estadoAtual, alarme.modo, s,
                       alarme.uidUltimoDesarme, alarme.ultimoCartaoValido,
                       alarme.contadorAberturasPorta,
                       alarme.contadorInvasoes,
                       alarme.uidUltimoRearme);
  }

  // 8. Áudio com prioridade
  EstadoSistema estadoParaAudio = (alarme.modo == ARMADO) ? alarme.estadoAtual : SEGURO;
  bool bloqueioParaAudio = (alarme.modo == ARMADO) ? alarme.bloqueioAtivo() : false;
  audio.tocar(estadoParaAudio, bloqueioParaAudio, estadoAmbiental);

  // 9. LED
  led.atualizar(alarme.modo, alarme.estadoAtual,
                alarme.bloqueioAtivo(), tempoUltimoPisca, estadoPiscaLED);

  // 10. MQTT
  notificador.update();

  // 10.1 Ressincronização periódica NTP
  relogio.talvezRessincronizar();

  // 11. Salvamento periódico apenas se dirty
  if (contadoresDirty && millis() - tempoUltimoSalvamento > INTERVALO_SALVAMENTO_MS) {
    tempoUltimoSalvamento = millis();
    contadoresDirty = false;
    salvarConfig();
  }

  delay(20);
}