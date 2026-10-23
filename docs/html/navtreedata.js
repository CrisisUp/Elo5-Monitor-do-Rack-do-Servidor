/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "My Project", "index.html", [
    [ "📋 RACK MONITOR — Sistema de Monitoramento de Rack com ESP32", "md__r_e_a_d_m_e.html", [
      [ "🎯 Visão Geral", "md__r_e_a_d_m_e.html#autotoc_md2", null ],
      [ "📦 Arquitetura Final", "md__r_e_a_d_m_e.html#autotoc_md3", null ],
      [ "🧩 Componentes do Sistema", "md__r_e_a_d_m_e.html#autotoc_md4", [
        [ "1. Código do ESP32 (sketch.ino)", "md__r_e_a_d_m_e.html#autotoc_md5", [
          [ "Arquitetura modular com classes", "md__r_e_a_d_m_e.html#autotoc_md6", null ]
        ] ]
      ] ],
      [ "Estados do sistema", "md__r_e_a_d_m_e.html#autotoc_md7", [
        [ "Modos", "md__r_e_a_d_m_e.html#autotoc_md8", null ],
        [ "Filtros implementados", "md__r_e_a_d_m_e.html#autotoc_md9", null ],
        [ "🌡️ Alarmes ambientais (v6.4)", "md__r_e_a_d_m_e.html#autotoc_md10", null ],
        [ "📡 Comunicação MQTT (v6.4)", "md__r_e_a_d_m_e.html#autotoc_md11", [
          [ "Tópicos (com prefixo configurável)", "md__r_e_a_d_m_e.html#autotoc_md12", null ],
          [ "Payload de status (exemplo)", "md__r_e_a_d_m_e.html#autotoc_md13", null ]
        ] ],
        [ "Features MQTT", "md__r_e_a_d_m_e.html#autotoc_md14", null ],
        [ "💾 Persistência NVS", "md__r_e_a_d_m_e.html#autotoc_md16", [
          [ "Salvos na flash do ESP32", "md__r_e_a_d_m_e.html#autotoc_md17", null ],
          [ "Comportamento", "md__r_e_a_d_m_e.html#autotoc_md18", null ]
        ] ],
        [ "🖥️ Dashboard Web (index.html)", "md__r_e_a_d_m_e.html#autotoc_md20", [
          [ "Design", "md__r_e_a_d_m_e.html#autotoc_md21", null ],
          [ "Estrutura visual", "md__r_e_a_d_m_e.html#autotoc_md22", null ],
          [ "Sistema de cores semântico", "md__r_e_a_d_m_e.html#autotoc_md23", null ],
          [ "Tecnologias", "md__r_e_a_d_m_e.html#autotoc_md24", null ],
          [ "Comportamento (v3.1)", "md__r_e_a_d_m_e.html#autotoc_md25", null ],
          [ "Correções aplicadas", "md__r_e_a_d_m_e.html#autotoc_md26", null ]
        ] ],
        [ "🧪 MQTTX (cliente MQTT gráfico)", "md__r_e_a_d_m_e.html#autotoc_md27", [
          [ "O que é", "md__r_e_a_d_m_e.html#autotoc_md28", null ],
          [ "Por que é útil", "md__r_e_a_d_m_e.html#autotoc_md29", null ],
          [ "Como usar (MQTTX Desktop — TCP 1883)", "md__r_e_a_d_m_e.html#autotoc_md30", null ],
          [ "MQTTX Web (WSS 8084)", "md__r_e_a_d_m_e.html#autotoc_md31", null ],
          [ "Conceitos MQTT aprendidos", "md__r_e_a_d_m_e.html#autotoc_md32", null ]
        ] ],
        [ "🖥️ Comandos Serial (firmware)", "md__r_e_a_d_m_e.html#autotoc_md34", [
          [ "Botão físico (multifunção)", "md__r_e_a_d_m_e.html#autotoc_md35", null ]
        ] ],
        [ "📁 Estrutura de Arquivos", "md__r_e_a_d_m_e.html#autotoc_md37", [
          [ "Bibliotecas necessárias (<span class=\"tt\">libraries.txt</span>)", "md__r_e_a_d_m_e.html#autotoc_md38", null ]
        ] ],
        [ "🎓 Princípios de Arquitetura Aplicados", "md__r_e_a_d_m_e.html#autotoc_md40", null ],
        [ "✅ O que está funcionando (v6.4 / v3.1)", "md__r_e_a_d_m_e.html#autotoc_md42", [
          [ "Firmware", "md__r_e_a_d_m_e.html#autotoc_md43", null ],
          [ "Dashboard", "md__r_e_a_d_m_e.html#autotoc_md44", null ]
        ] ],
        [ "⚠️ Limitações do ambiente gratuito", "md__r_e_a_d_m_e.html#autotoc_md46", null ],
        [ "🚀 Próximos passos sugeridos", "md__r_e_a_d_m_e.html#autotoc_md48", null ],
        [ "📌 Lembretes importantes", "md__r_e_a_d_m_e.html#autotoc_md50", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"index.html"
];

const SYNCONMSG = 'click to disable panel synchronization';
const SYNCOFFMSG = 'click to enable panel synchronization';
const LISTOFALLMEMBERS = 'List of all members';