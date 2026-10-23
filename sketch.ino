/**
 * @file RackMonitor.ino
 * @brief Sistema de Monitoramento de Rack (ESP32).
 * 
 * Versão 6.4 - Modularizado, Wi-Fi/MQTT, NVS, LWT, watchdog e priorização de áudio.
 * 
 * Arquitetura:
 * Sensores -> Eventos -> Alarme (única fonte de verdade) -> Atuadores (Display, LED, Áudio) + MQTT (observador).
 */

// =========================================================================
// CHANGELOG
// =========================================================================
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

/**
 * @brief Arquitetura de Software e Regras de Ouro.
 * 
 * Sensores → struct LeiturasSensores + Eventos → Alarme (única fonte de
 * verdade) → Atuadores (render) + NotificadorMQTT (observador/publicador).
 * NVS é persistência passiva. loop() só orquestra.
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

// =========================================================================
// CONFIGURAÇÃO GLOBAL
// =========================================================================

/**
 * @namespace Pins
 * @brief Mapeamento de todos os GPIOs utilizados no projeto.
 */
namespace Pins {
constexpr uint8_t RFID_SS   = 5;   /**< Chip Select do MFRC522 */
constexpr uint8_t RFID_RST  = 4;   /**< Reset do MFRC522 */
constexpr uint8_t RFID_SCK  = 18;  /**< Clock SPI */
constexpr uint8_t RFID_MISO = 19;  /**< MISO SPI */
constexpr uint8_t RFID_MOSI = 23;  /**< MOSI SPI */

constexpr uint8_t PIR       = 27;  /**< Sensor de Movimento PIR */
constexpr uint8_t TRIG      = 17;  /**< Trigger do HC-SR04 */
constexpr uint8_t ECHO      = 16;  /**< Echo do HC-SR04 */
constexpr uint8_t LDR_AO    = 34;  /**< Leitura analógica do LDR (Input-only) */

constexpr uint8_t BOTAO     = 14;  /**< Pushbutton (INPUT_PULLUP) */
constexpr uint8_t SWITCH    = 25;  /**< Fim de curso da porta (INPUT_PULLDOWN) */

constexpr uint8_t BUZZER_E  = 15;  /**< Buzzer Esquerdo (Strapping) */
constexpr uint8_t BUZZER_D  = 2;   /**< Buzzer Direito (Strapping) */

constexpr uint8_t LED_R     = 32;  /**< LED RGB - Vermelho */
constexpr uint8_t LED_G     = 33;  /**< LED RGB - Verde */
constexpr uint8_t LED_B     = 0;   /**< LED RGB - Azul (Strapping) */
}

constexpr uint32_t PWM_FREQ       = 5000;  /**< Frequência do PWM para o LED RGB */
constexpr uint8_t  PWM_RESOLUTION = 8;     /**< Resolução do PWM (8 bits = 0-255) */
constexpr uint8_t  LARGURA_TELA   = 128;   /**< Largura do OLED */
constexpr uint8_t  ALTURA_TELA    = 64;    /**< Altura do OLED */
constexpr uint8_t  ENDERECO_OLED  = 0x3C;  /**< Endereço I2C do OLED */

// Filtros
constexpr int  LDR_LIMIAR_ACENDE = 2200;          /**< Limiar para considerar luz acesa */
constexpr int  LDR_LIMIAR_APAGA  = 1800;          /**< Limiar para considerar luz apagada */
constexpr unsigned long DEBOUNCE_LDR_MS = 800;    /**< Debounce do LDR */

constexpr int   US_AMOSTRAS        = 5;           /**< Número de amostras para média do ultrassônico */
constexpr float US_DIST_MAX_VALIDA = 400.0f;      /**< Distância máxima válida (cm) */

constexpr unsigned long DEBOUNCE_ESTADO_MS = 300; /**< Debounce para transição de estados */

constexpr unsigned long DEBOUNCE_BOTAO_MS       = 250;  /**< Debounce do botão */
constexpr unsigned long JANELA_CLIQUE_DUPLO_MS  = 400;  /**< Janela para clique duplo */
constexpr unsigned long DURACAO_CLIQUE_LONGO_MS = 3000; /**< Duração para clique longo */

constexpr unsigned long DEBOUNCE_PORTA_MS = 200;  /**< Debounce do switch da porta */

constexpr unsigned long DURACAO_AVISO_BLOQUEIO_MS = 4000; /**< Duração do aviso de bloqueio no display */

// Limites ambientais (ASHRAE + margem de segurança)
// Valores DEFAULT — podem ser sobrescritos pela NVS
constexpr float TEMP_ALERTA_C_DEFAULT    = 30.0f;  /**< Limiar de alerta de temperatura (°C) */
constexpr float TEMP_CRITICO_C_DEFAULT   = 35.0f;  /**< Limiar crítico de temperatura (°C) */
constexpr float UMID_ALERTA_PCT_DEFAULT  = 65.0f;  /**< Limiar de alerta de umidade (%) */
constexpr float UMID_CRITICO_PCT_DEFAULT = 80.0f;  /**< Limiar crítico de umidade (%) */

constexpr float TEMP_HISTERESE  = 2.0f;  /**< Histerese para temperatura */
constexpr float UMID_HISTERESE  = 5.0f;  /**< Histerese para umidade */

// Taxa de subida de temperatura
constexpr float TAXA_ALERTA_C_POR_MIN = 1.0f;  /**< Taxa de subida considerada crítica (°C/min) */
constexpr unsigned long INTERVALO_AMOSTRA_TAXA_MS = 60000UL; /**< Intervalo de amostragem da taxa (1 min) */

// Limiares efetivos (sobrescritos pela NVS no boot)
float limiarTempAlerta  = TEMP_ALERTA_C_DEFAULT;  /**< Limiar de alerta de temperatura atual */
float limiarTempCritico = TEMP_CRITICO_C_DEFAULT; /**< Limiar crítico de temperatura atual */
float limiarUmidAlerta  = UMID_ALERTA_PCT_DEFAULT;  /**< Limiar de alerta de umidade atual */
float limiarUmidCritico = UMID_CRITICO_PCT_DEFAULT; /**< Limiar crítico de umidade atual */

// Flags de estado ambiental
bool alertaTempAtivo  = false;  /**< Flag de alerta de temperatura ativo */
bool criticoTempAtivo = false;  /**< Flag de temperatura crítica ativa */
bool alertaUmidAtivo  = false;  /**< Flag de alerta de umidade ativo */
bool criticoUmidAtivo = false;  /**< Flag de umidade crítica ativa */
bool taxaTempAlerta   = false;  /**< Flag de taxa de subida de temperatura crítica */

// Watchdog do DHT22
unsigned long _ultimaLeituraValidaDHT = 0;  /**< Timestamp da última leitura válida do DHT22 */
bool          _sensorDHTPerdido       = false; /**< Flag indicando que o DHT22 foi perdido */
constexpr unsigned long TIMEOUT_DHT_MS = 10000UL; /**< Tempo para considerar o DHT22 perdido (10s) */

/**
 * @enum EstadoAmbiental
 * @brief Estado consolidado do ambiente (temperatura + umidade).
 */
enum EstadoAmbiental { AMB_OK, AMB_ALERTA, AMB_CRITICO };
EstadoAmbiental estadoAmbiental = AMB_OK;  /**< Estado ambiental atual */

// Buzzer ambiental opcional (prioridade respeitada)
#define ALARME_SONORO_AMBIENTAL 0

// Amostragem da taxa de subida (uso interno de _verificarAmbiente)
float         _tempAnteriorTaxa  = -999.0f; /**< Temperatura da amostra anterior */
unsigned long _tempoAnteriorTaxa = 0;       /**< Timestamp da amostra anterior */

// [4] Flag global de dirty para NVS (usada por Alarme e loop)
bool contadoresDirty = false;  /**< Flag que indica se há contadores não salvos na NVS */

const String CARTAO_AUTORIZADO_1 = "01020304";  /**< UID do cartão 1 autorizado */
const String CARTAO_AUTORIZADO_2 = "55667788";  /**< UID do cartão 2 autorizado */

constexpr unsigned long WARMUP_LDR_MS = 1500;  /**< Tempo de warmup do LDR */
constexpr unsigned long WARMUP_PIR_MS = 1500;  /**< Tempo de warmup do PIR */
constexpr unsigned long WARMUP_US_MS  = 1500;  /**< Tempo de warmup do ultrassônico */

std::vector<String> cartoesAutorizados; /**< Lista de cartões autorizados */

// =========================================================================
// CONFIGURAÇÃO DE REDE (Wi-Fi + MQTT)
// =========================================================================
const char* WIFI_SSID     = "Wokwi-GUEST";  /**< SSID da rede Wi-Fi */
const char* WIFI_PASSWORD = "";             /**< Senha da rede Wi-Fi */
const char* MQTT_BROKER   = "broker.emqx.io"; /**< Endereço do broker MQTT */
const int   MQTT_PORT     = 1883;           /**< Porta do broker MQTT */

const char* MQTT_TOPIC_STATUS = "rack_monitor_ABCD1234/status";  /**< Tópico de status */
const char* MQTT_TOPIC_LWT    = "rack_monitor_ABCD1234/status/lwt"; /**< Tópico de LWT */
const char* MQTT_TOPIC_CMD    = "rack_monitor_ABCD1234/comando";  /**< Tópico de comandos */
const char* MQTT_TOPIC_LOGS   = "rack_monitor_ABCD1234/logs";    /**< Tópico de logs */
const char* MQTT_TOPIC_HB     = "rack_monitor_ABCD1234/heartbeat"; /**< Tópico de heartbeat */

const unsigned long MQTT_RETRY_MS = 5000;  /**< Intervalo de retry do MQTT */

// [2] Token para comandos destrutivos. TROQUE ESTE VALOR!
const char* MQTT_RESET_TOKEN = "rack2026";  /**< Token para autorizar o RESET remoto */

// =========================================================================
// ENUMS E STRUCTS
// =========================================================================

/**
 * @enum ModoVigilante
 * @brief Modo de operação do sistema de segurança.
 */
enum ModoVigilante { ARMADO, DESARMADO };

/**
 * @enum EstadoSistema
 * @brief Estados possíveis do perímetro de segurança.
 */
enum EstadoSistema {
  SEGURO,
  AVISO_SALA,
  APROXIMACAO_RACK,
  EMERGENCIA_RACK,
  INVASAO_SALA,   /**< Reservado (não atribuído atualmente) */
  ACESSO_RFID
};

/**
 * @enum EventoBotao
 * @brief Tipos de eventos gerados pelo botão.
 */
enum EventoBotao { BOTAO_NENHUM, BOTAO_CURTO, BOTAO_DUPLO, BOTAO_LONGO };

/**
 * @struct EventoRFID
 * @brief Estrutura que representa um evento de leitura de RFID.
 */
struct EventoRFID {
  bool   novo   = false; /**< Indica se um novo cartão foi lido */
  bool   valido = false; /**< Indica se o cartão é válido (preenchido em Alarme) */
  String uid    = "";    /**< UID do cartão lido */
};

/**
 * @struct LeiturasSensores
 * @brief Snapshot das leituras atuais dos sensores.
 */
struct LeiturasSensores {
  bool  movimento   = false; /**< PIR detectou movimento */
  bool  luzAcesa    = false; /**< LDR detectou luz acesa */
  bool  portaAberta = false; /**< Switch da porta está aberto */
  float distancia   = 999.0f; /**< Distância medida pelo ultrassônico (cm) */
};

// =========================================================================
// PERIFÉRICOS
// =========================================================================
Adafruit_SSD1306 display(LARGURA_TELA, ALTURA_TELA, &Wire, -1); /**< Instância do display OLED */
RTC_DS3231       rtc; /**< Instância do RTC DS3231 */
MFRC522          rfid(Pins::RFID_SS, Pins::RFID_RST); /**< Instância do leitor RFID */

bool rtcDisponivel = false; /**< Flag indicando se o RTC está disponível */

// =========================================================================
// MÓDULO: RELÓGIO
// =========================================================================

/**
 * @class Relogio
 * @brief Gerencia o RTC DS3231 e fornece timestamps formatados.
 */
class Relogio {
  public:
    /**
     * @brief Inicializa o RTC.
     * @return true se o RTC foi inicializado com sucesso, false caso contrário.
     */
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

    /**
     * @brief Retorna o timestamp atual no formato "DD/MM HH:MM:SS".
     * @return String com o timestamp formatado.
     */
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

    /**
     * @brief Retorna o timestamp atual no formato ISO 8601 ("YYYY-MM-DDTHH:MM:SS").
     * @return String com o timestamp ISO.
     */
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

    /**
     * @brief Verifica se o RTC está disponível.
     * @return true se disponível, false caso contrário.
     */
    bool disponivel() const {
      return rtcDisponivel;
    }

    /**
     * @brief Retorna uma referência ao driver do RTC.
     * @return Referência ao objeto RTC_DS3231.
     */
    RTC_DS3231& driver()    {
      return rtc;
    }
};

Relogio relogio; /**< Instância global do relógio */

// =========================================================================
// LOG CENTRALIZADO
// =========================================================================

/**
 * @brief Função de log centralizada para a Serial.
 * @param nivel Nível do log (INFO, WARN, ALERT, CRIT).
 * @param tag Tag do módulo (ex: "PORTA", "RFID").
 * @param mensagem Mensagem a ser logada.
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
}

// Protótipo (implementação no final, após NotificadorMQTT)
void enviarLogSite(const String &mensagem);

// =========================================================================
// MÓDULO: LDR
// =========================================================================

/**
 * @class LDR
 * @brief Gerencia o sensor LDR com histerese e debounce.
 */
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

    bool estaAcesa() const {
      return _estado;
    }
    int  leituraBruta() const {
      return analogRead(Pins::LDR_AO);
    }

  private:
    bool          _estado             = false;
    unsigned long _tempoUltimaMudanca = 0;
    unsigned long _tempoInicio        = 0;
};

LDR ldr; /**< Instância global do LDR */

// =========================================================================
// MÓDULO: ULTRASSÔNICO
// =========================================================================

/**
 * @class Ultrassonico
 * @brief Gerencia o sensor HC-SR04 com média de amostras e tratamento de erro (-1).
 */
class Ultrassonico {
  public:
    void begin() {
      pinMode(Pins::TRIG, OUTPUT);
      pinMode(Pins::ECHO, INPUT);
      digitalWrite(Pins::TRIG, LOW);
      _tempoInicio = millis();
      _distancia   = -1.0f; // [CORREÇÃO] -1 significa "sem leitura"
    }

    void update() {
      if (millis() - _tempoInicio < 1500) {
        _distancia = -1.0f;
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
      // [CORREÇÃO] Se não houver nenhuma amostra válida, define como -1
      _distancia = (validas > 0) ? (soma / validas) : -1.0f;
    }

    float distancia() const {
      return _distancia;
    }

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
      return (d <= 0.0f) ? -1.0f : d; // [CORREÇÃO] -1 para erro
    }
};

Ultrassonico ultrassonico; /**< Instância global do ultrassônico */

// =========================================================================
// MÓDULO: PORTA
// =========================================================================

/**
 * @class Porta
 * @brief Gerencia o switch de porta com debounce.
 */
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

    bool estaAberta() const {
      return _estadoFiltrado;
    }

  private:
    bool          _estadoFiltrado     = false;
    bool          _brutoAnterior      = false;
    unsigned long _tempoUltimaMudanca = 0;
};

Porta porta; /**< Instância global da porta */

// =========================================================================
// MÓDULO: PIR
// =========================================================================

/**
 * @class PIR
 * @brief Gerencia o sensor de movimento PIR.
 */
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

    bool detectaMovimento() const {
      return _movimento;
    }

  private:
    bool          _movimento   = false;
    unsigned long _tempoInicio = 0;
};

PIR pir; /**< Instância global do PIR */

// =========================================================================
// MÓDULO: BOTÃO
// =========================================================================

/**
 * @class Botao
 * @brief Gerencia o botão físico com debounce, clique curto, duplo e longo.
 */
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

Botao botao; /**< Instância global do botão */

// =========================================================================
// MÓDULO: RFID  [5] validação removida do leitor
// =========================================================================

/**
 * @class LeitorRFID
 * @brief Gerencia a leitura de cartões RFID.
 */
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

LeitorRFID leitorRFID; /**< Instância global do leitor RFID */

// =========================================================================
// MÓDULO: LED RGB
// =========================================================================

/**
 * @class LedRGB
 * @brief Controla o LED RGB com PWM (ânodo comum).
 */
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

LedRGB led; /**< Instância global do LED RGB */

// =========================================================================
// MÓDULO: ÁUDIO (com prioridade)
// =========================================================================

/**
 * @class Audio
 * @brief Controla os buzzers com sistema de prioridade para evitar conflitos.
 */
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

    void beepCurto(int freq = 1200, int dur = 100) {
      // Beeps curtos (feedback) não são bloqueados por prioridade,
      // mas só tocam se não houver um alarme crítico ativo.
      if (_prioridadeAtual < 3) {
        tone(Pins::BUZZER_E, freq, dur);
      }
    }

    void beepErro() {
      if (_prioridadeAtual < 3) {
        tone(Pins::BUZZER_D, 500, 200); delay(250);
        tone(Pins::BUZZER_D, 400, 200);
      }
    }

    void beepSucesso() {
      if (_prioridadeAtual < 3) {
        tone(Pins::BUZZER_E, 800, 100);  delay(120);
        tone(Pins::BUZZER_E, 600, 150);
      }
    }

    // [ITEM 2] Nova função unificada de áudio com prioridade
    void tocar(EstadoSistema estado, bool bloqueioAtivo, EstadoAmbiental ambiente) {
      unsigned long agora = millis();

      // 1. Determinar a prioridade do som atual
      int prioridade = 0;
      if (estado == EMERGENCIA_RACK || estado == INVASAO_SALA) {
        prioridade = 3;
      } else if (estado == AVISO_SALA || estado == APROXIMACAO_RACK || bloqueioAtivo) {
        prioridade = 2;
      } else if (ambiente == AMB_CRITICO || ambiente == AMB_ALERTA) {
        prioridade = 1;
      }

      // 2. Se a prioridade caiu, paramos o som imediatamente
      if (prioridade < _prioridadeAtual) {
        parar();
      }
      _prioridadeAtual = prioridade;

      // 3. Toca o som conforme a prioridade
      switch (prioridade) {
        case 3: // CRÍTICO: Segurança
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

        case 2: // ALERTA: Segurança ou Bloqueio
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

        case 1: // ALERTA: Ambiental
          if (ambiente == AMB_CRITICO) {
            if (agora - _tempoUltimoBip >= 3000) {
              tone(Pins::BUZZER_D, 400, 200); delay(220);
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

        case 0: // NORMAL: Silêncio
          parar();
          break;
      }
    }

  private:
    int _prioridadeAtual = 0; /**< Prioridade atual do som */
    unsigned long _tempoUltimoBip = 0; /**< Timestamp do último bip */
};

Audio audio; /**< Instância global do áudio */

// =========================================================================
// MÓDULO: DISPLAY OLED
// =========================================================================

/**
 * @class DisplayOLED
 * @brief Gerencia o display OLED com ícones e textos padronizados.
 */
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
          linha1 = "Vigilante Armado"; linha2 = "Porta Fechada"; icone = 1;
          break;
        case ACESSO_RFID:
          linha1 = cartaoOk ? "Cartao Autorizado" : "Cartao Negado";
          linha2 = "UID: " + uid;
          icone  = cartaoOk ? 1 : 3;
          break;
        case AVISO_SALA:
          linha1 = "Aviso: Movimento"; linha2 = "Rack Fechado"; icone = 2;
          break;
        case APROXIMACAO_RACK:
          linha1 = "Aproximacao";
          if (s.distancia < 0.0f) {
            linha2 = "Dist: N/A";
          } else {
            linha2 = "Dist: " + String(s.distancia, 1) + "cm";
          }
          icone  = 2;
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

DisplayOLED oled; /**< Instância global do display OLED */

// =========================================================================
// MÓDULO: DHT22  [8] simplificado
// =========================================================================
#define DHT_PIN     13
#define DHT_TYPE    DHT22

/**
 * @class SensorDHT
 * @brief Gerencia o sensor DHT22 de temperatura e umidade.
 */
class SensorDHT {
  public:
    DHT dht;

    SensorDHT() : dht(DHT_PIN, DHT_TYPE) {}

    void begin() {
      dht.begin();
      logSerial("INFO", "DHT", "Sensor DHT22 inicializado no GPIO 13.");
    }

    /**
     * @brief Lê a temperatura atual.
     * @return Temperatura em °C ou -999.0f se falhar.
     */
    float temperatura() {
      float t = dht.readTemperature();
      return isnan(t) ? -999.0f : t;
    }

    /**
     * @brief Lê a umidade relativa atual.
     * @return Umidade em % ou -999.0f se falhar.
     */
    float umidade() {
      float u = dht.readHumidity();
      return isnan(u) ? -999.0f : u;
    }
};

SensorDHT dht; /**< Instância global do sensor DHT22 */

// =========================================================================
// MÓDULO: ALARME (máquina de estados)
// =========================================================================

/**
 * @class Alarme
 * @brief Máquina de estados central do sistema de segurança.
 * 
 * Processa eventos de sensores, RFID e botão para determinar o estado
 * de segurança (Seguro, Emergência, etc.) e disparar ações.
 */
class Alarme {
  public:
    ModoVigilante modo           = ARMADO; /**< Modo atual (ARMADO/DESARMADO) */
    EstadoSistema estadoAtual    = SEGURO; /**< Estado atual do perímetro */
    EstadoSistema estadoAnterior = SEGURO; /**< Estado anterior (para detectar transições) */
    EstadoSistema estadoCandidato      = SEGURO; /**< Estado candidato (debounce) */
    unsigned long tempoEstadoCandidato = 0;      /**< Timestamp do estado candidato */

    unsigned int  contadorDesarmes             = 0; /**< Total de desarmes */
    unsigned int  contadorRearmesBloqueados    = 0; /**< Total de rearmes bloqueados */
    unsigned int  contadorAberturasPorta       = 0; /**< Total de aberturas da porta */
    unsigned int  contadorInvasoes             = 0; /**< Total de invasões detectadas */
    unsigned int  contadorAproximacoes         = 0; /**< Total de aproximações detectadas */
    unsigned int  contadorAcessosValidos       = 0; /**< Total de acessos RFID válidos */
    unsigned int  contadorMovimentosManutencao = 0; /**< Movimentos em modo manutenção */

    unsigned long timeoutDesarme       = 30UL * 60UL * 1000UL; /**< Timeout para rearme automático */
    unsigned long tempoInicioDesarme   = 0;  /**< Timestamp do início do desarme */
    bool          aviso5minDisparado   = false; /**< Flag de aviso de 5 min */
    String        uidUltimoDesarme     = ""; /**< UID do último desarme */
    String        uidUltimoRearme      = ""; /**< UID do último rearme */
    bool          ultimoCartaoValido   = false; /**< Resultado do último cartão lido */
    unsigned long duracaoUltimoDesarme = 0;  /**< Duração do último desarme (s) */
    unsigned long ultimoUIDtempoRFID   = 0;  /**< Timestamp do último RFID lido */

    String        motivoBloqueio      = ""; /**< Motivo do bloqueio de rearme */
    unsigned long tempoBloqueioAtivo  = 0;  /**< Timestamp do início do bloqueio */

    bool resetContadoresPendente = false; /**< Flag de reset pendente */
    bool modoTestePendente       = false; /**< Flag de modo teste pendente */
    bool audioPararPendente      = false; /**< Flag de parar áudio pendente */

    float limiarDistancia = 50.0f; /**< Limiar de distância para aproximação (cm) */

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

    bool bloqueioAtivo() const {
      return motivoBloqueio.length() > 0;
    }

    bool armarViaSerial(Porta &porta, Audio &audio) {
      if (modo == ARMADO) return false;
      return _tentarArmar(porta, audio);
    }

    String nomeModo() const {
      return (modo == ARMADO) ? "ARMADO" : "DESARMADO";
    }
    String nomeEstadoAtual() const {
      return _nomeEstado(estadoAtual);
    }

  private:
    bool _estadoAnteriorPIR   = false; /**< Estado anterior do PIR */
    bool _estadoAnteriorLDR   = false; /**< Estado anterior do LDR */
    bool _portaEstadoAnterior = false; /**< Estado anterior da porta */
    bool _primeiraLeitura     = true;  /**< Flag de primeira leitura */

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
        if (card == ev.uid) {
          valido = true;
          break;
        }
      }

      ultimoCartaoValido = valido; // <-- NOVO: registra o resultado para o OLED

      if (valido) {
        enviarLogSite("Cartao autorizado: " + ev.uid);
        contadorAcessosValidos++;
        contadoresDirty = true;   // [4]
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
        motivoBloqueio     = "Feche a porta!"; // (No OLED pode manter o !, é curto)
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
      logSerial("WARN", "VIGILANTE",
                "Rearme automatico por timeout | Duracao: " +
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
          logSerial("CRIT", "SEGURANCA", "Invasao fisica detectada no rack!");
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

Alarme alarme; /**< Instância global do alarme */

// =========================================================================
// MÓDULO: NVS (Persistência em Flash)
// =========================================================================
Preferences prefs; /**< Instância global de Preferences */

/**
 * @brief Salva todas as configurações e contadores na NVS.
 */
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

/**
 * @brief Carrega os cartões autorizados da NVS, com deduplicação.
 */
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

/**
 * @brief Carrega as configurações e contadores da NVS.
 */
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

/**
 * @class NotificadorMQTT
 * @brief Gerencia a conexão Wi-Fi e MQTT, publicação de status e comandos remotos.
 */
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

      _client.setCallback([this](char* topic, byte * payload, unsigned int length) {
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

            // [CORREÇÃO ITEM 7] Apenas sinaliza que precisa publicar, sem publicar agora
            _precisaPublicarInicial = true;
          } else {
            Serial.print(F("Falha rc=")); Serial.println(_client.state());
          }
        }
        return;
      }

      _client.loop();

      // [CORREÇÃO ITEM 7] Publica o status real assim que o loop rodar pela primeira vez
      if (_precisaPublicarInicial) {
        _precisaPublicarInicial = false;
        _publicar(true);
      }

      // 3. Publica transição (cache atualizado dentro de _publicar [1])
      // [NOVO] Também publica se o estado físico da porta mudou
      if (alarme.estadoAtual != _ultimoEstadoPublicado ||
          alarme.modo       != _ultimoModoPublicado ||
          porta.estaAberta() != _ultimoEstadoPortaPublicado) { // <-- NOVO
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
    WiFiClient     _wifiClient; /**< Cliente Wi-Fi */
    PubSubClient   _client{_wifiClient}; /**< Cliente MQTT */
    unsigned long  _ultimoTempoReconexao = 0; /**< Timestamp da última tentativa de reconexão */
    unsigned long  _ultimoHeartbeat      = 0; /**< Timestamp do último heartbeat */
    const unsigned long INTERVALO_HEARTBEAT_MS = 30000UL; /**< Intervalo do heartbeat (30s) */
    EstadoSistema  _ultimoEstadoPublicado = SEGURO; /**< Cache do último estado publicado */
    ModoVigilante  _ultimoModoPublicado   = ARMADO; /**< Cache do último modo publicado */

    bool           _estavaConectado       = false; /**< Flag de conexão anterior */
    bool           _precisaPublicarInicial = true; /**< Flag de publicação inicial pendente */
    bool           _ultimoEstadoPortaPublicado = false; /**< Cache do último estado da porta publicado */
    unsigned long  _ultimoStatusForcado   = 0; /**< Timestamp do último STATUS forçado */
    const unsigned long STATUS_RATE_LIMIT_MS = 2000UL; /**< Rate limit do STATUS (2s) */

    void _publicar(bool forcar = false) {
      if (!forcar && alarme.estadoAtual == _ultimoEstadoPublicado &&
          alarme.modo == _ultimoModoPublicado &&
          porta.estaAberta() == _ultimoEstadoPortaPublicado) {  // ← ADICIONE ESTA LINHA
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
      // [CORREÇÃO] Envia null (JSON) se não houver leitura válida
      float dist = ultrassonico.distancia();
      payload += ",\"dist_cm\":";
      if (dist < 0.0f) {
        payload += "null";
      } else {
        payload += String(dist, 1);
      }
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

      // [CORREÇÃO] Verifica se o publish funcionou
      bool ok = _client.publish(MQTT_TOPIC_STATUS, payload.c_str(), true);
      if (!ok) {
        Serial.println(F("[ERRO] Falha ao publicar status! Buffer cheio?"));
        return; // <-- Não atualiza o cache, para tentar de novo no próximo ciclo
      }

      // [1] cache atualizado AQUI (qualquer publicação, forçada ou não)
      _ultimoEstadoPublicado = alarme.estadoAtual;
      _ultimoModoPublicado   = alarme.modo;
      _ultimoEstadoPortaPublicado = porta.estaAberta(); // <-- NOVO
    }

    void _tratarComando(char* topic, byte* payload, unsigned int length) {
      String cmd;
      for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];
      cmd.trim();

      // [CORREÇÃO ITEM 6] Salva o comando original antes de converter para maiúsculas
      String cmdOriginal = cmd;
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
        // cmd.toUpperCase(); // <-- Transforma "RESET rack2026" em "RESET RACK2026"
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
          // NÃO zera: contadorInvasoes, contadorRearmesBloqueados
          alarme.resetContadoresPendente = false;
          salvarConfig();
          logSerial("INFO", "MQTT_CMD", "Reset remoto executado (seguranca preservada).");
          enviarLogSite("Comando remoto: reset operacional executado");

          // [NOVO] Força a publicação imediata do novo estado para o dashboard atualizar
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

NotificadorMQTT notificador; /**< Instância global do notificador MQTT */

// =========================================================================
// ALARME AMBIENTAL (Temperatura + Umidade + taxa + crítico + consolidado)
// =========================================================================

/**
 * @brief Verifica os limiares de temperatura e umidade.
 * 
 * Atualiza as flags globais de alerta/crítico e o estado ambiental consolidado.
 * Possui um watchdog que reseta o estado se o DHT22 falhar por mais de 10s.
 * 
 * @param temp Temperatura atual em Celsius (ou -999 se falha).
 * @param umid Umidade relativa atual em porcentagem (ou -999 se falha).
 */
void _verificarAmbiente(float temp, float umid) {
  // [ITEM 1] Watchdog: se o DHT falhar por muito tempo, zera as flags
  if (temp == -999.0f || umid == -999.0f) {
    if (_ultimaLeituraValidaDHT > 0 &&
        (millis() - _ultimaLeituraValidaDHT) > TIMEOUT_DHT_MS) {
      // Sensor perdido há mais de 10s — resetar estado ambiental
      if (!_sensorDHTPerdido) {
        _sensorDHTPerdido = true;
        alertaTempAtivo  = false;
        criticoTempAtivo = false;
        alertaUmidAtivo  = false;
        criticoUmidAtivo = false;
        taxaTempAlerta   = false;
        estadoAmbiental  = AMB_OK;
        logSerial("WARN", "AMBIENTE", "DHT22 sem leitura ha 10s. Estado ambiental resetado.");
        enviarLogSite("Sensor DHT22 sem leitura. Monitoramento ambiental pausado.");
      }
    }
    return; // Sai da função (mas o watchdog acima já rodou)
  }

  // Leitura válida! Reseta o watchdog
  _ultimaLeituraValidaDHT = millis();
  if (_sensorDHTPerdido) {
    _sensorDHTPerdido = false;
    logSerial("INFO", "AMBIENTE", "DHT22 voltou a responder. Retomando monitoramento.");
    enviarLogSite("Sensor DHT22 restabelecido.");
  }

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

/**
 * @brief Escapa caracteres especiais de uma string para formato JSON.
 * @param in String de entrada.
 * @return String escapada.
 */
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

/**
 * @brief Envia uma mensagem de log para o tópico MQTT de logs.
 * @param mensagem Texto do log.
 */
void enviarLogSite(const String &mensagem) {
  if (notificador.estaConectado()) {
    String payload = "{\"ts\":\"" + relogio.timestamp() +
                     "\",\"msg\":\"" + jsonEscape(mensagem) + "\"}";
    notificador.publicarLog(payload);
    delay(50); // <-- Dá tempo do PubSubClient liberar o buffer
  }
}

// =========================================================================
// COMANDOS SERIAL
// =========================================================================
String bufferSerial = ""; /**< Buffer de comandos da Serial */

/**
 * @brief Processa um comando recebido via Serial Monitor.
 * @param cmd Comando em string.
 */
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
        if (c == novoUID) {
          jaExiste = true;
          break;
        }
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
// MODO TESTE (COM FEEDBACK DETALHADO)
// =========================================================================

/**
 * @brief Executa o autodiagnóstico completo do hardware.
 * 
 * Testa OLED, RFID, Ultrassônico, LDR, Porta, PIR, DHT22, Buzzers e LED.
 * Envia logs detalhados para o dashboard.
 */
void executarModoTeste() {
  logSerial("INFO", "TESTE", "=== INICIANDO AUTODIAGNOSTICO v3 ===");
  enviarLogSite("TESTE: Iniciando autodiagnostico...");

  int ok = 0, falhas = 0;
  String resumoFalhas = "";

  // 1. OLED
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

  // 2. RFID
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

  // 3. Ultrassonico
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

  // 4. LDR (Informativo)
  int ldrRaw = ldr.leituraBruta();
  logSerial("INFO", "TESTE", "[INFO] LDR: " + String(ldrRaw) + " (" + String(ldr.estaAcesa() ? "ACESA" : "APAGADA") + ").");
  enviarLogSite("TESTE: LDR " + String(ldr.estaAcesa() ? "ACESA" : "APAGADA"));
  ok++;

  // 5. Porta (Informativo)
  logSerial("INFO", "TESTE", "[INFO] Porta: " + String(porta.estaAberta() ? "ABERTA" : "fechada") + ".");
  enviarLogSite("TESTE: Porta " + String(porta.estaAberta() ? "ABERTA" : "FECHADA"));
  ok++;

  // 6. PIR (Informativo)
  logSerial("INFO", "TESTE", "[INFO] PIR: " + String(pir.detectaMovimento() ? "ATIVO" : "inativo") + ".");
  enviarLogSite("TESTE: PIR " + String(pir.detectaMovimento() ? "ATIVO" : "INATIVO"));
  ok++;

  // 7. DHT22
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

  // 8. Buzzers
  logSerial("INFO", "TESTE", "[INFO] Testando buzzers...");
  enviarLogSite("TESTE: Testando Buzzers...");
  tone(Pins::BUZZER_E, 500, 150);  delay(200);
  tone(Pins::BUZZER_E, 1000, 150); delay(200);
  tone(Pins::BUZZER_E, 2000, 150); delay(200);
  tone(Pins::BUZZER_D, 800, 300);  delay(350);
  ok++;

  // 9. LED
  logSerial("INFO", "TESTE", "[INFO] Testando LED RGB...");
  enviarLogSite("TESTE: Testando LED RGB...");
  led.definirCor(255, 0, 0);     delay(300);
  led.definirCor(0, 255, 0);     delay(300);
  led.definirCor(0, 0, 255);     delay(300);
  led.definirCor(255, 255, 255); delay(300);
  led.definirCor(0, 0, 0);
  ok++;

  // Resumo Final
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

/**
 * @brief Inicialização do sistema.
 */
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

  // [ITEM 1] Garantir que o estado ambiental comece limpo
  alertaTempAtivo  = false;
  criticoTempAtivo = false;
  alertaUmidAtivo  = false;
  criticoUmidAtivo = false;
  taxaTempAlerta   = false;
  estadoAmbiental  = AMB_OK;
  _ultimaLeituraValidaDHT = 0;
  _sensorDHTPerdido = false;

  notificador.begin();
}

// =========================================================================
// LOOP PRINCIPAL
// =========================================================================
unsigned long tempoUltimoBip   = 0; /**< Timestamp do último bip */
unsigned long tempoUltimoPisca = 0; /**< Timestamp do último pisca */
bool          estadoPiscaLED   = false; /**< Estado atual do pisca do LED */

// [4] salvamento só se dirty
unsigned long tempoUltimoSalvamento = 0; /**< Timestamp do último salvamento na NVS */
constexpr unsigned long INTERVALO_SALVAMENTO_MS = 60000UL; /**< Intervalo de salvamento na NVS (1 min) */

/**
 * @brief Loop principal do sistema.
 * 
 * Orquestra a leitura dos sensores, a máquina de estados do alarme,
 * a atualização dos atuadores e a comunicação MQTT.
 */
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
  } else {
    oled.mostrarStatus(alarme.estadoAtual, alarme.modo, s,
                       alarme.uidUltimoDesarme, alarme.ultimoCartaoValido,
                       alarme.contadorAberturasPorta,
                       alarme.contadorInvasoes);
  }

  // [ITEM 2] Controle unificado de áudio com prioridade
  // O modo DESARMADO não deve tocar alarme de segurança, mas mantém o ambiental
  EstadoSistema estadoParaAudio = (alarme.modo == ARMADO) ? alarme.estadoAtual : SEGURO;
  bool bloqueioParaAudio = (alarme.modo == ARMADO) ? alarme.bloqueioAtivo() : false;

  audio.tocar(estadoParaAudio, bloqueioParaAudio, estadoAmbiental);

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