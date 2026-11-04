# 🔌 Guia de Gravação no ESP32 (Docker)

Este documento explica como gravar o firmware no ESP32 quando o ambiente de compilação está rodando dentro do **Docker**. O processo varia conforme o sistema operacional, pois o Docker tem limitações diferentes para acessar portas USB.

## 📋 Resumo Rápido

| Sistema Operacional | Docker consegue acessar a USB? | Solução Recomendada |
| :--- | :--- | :--- |
| **Linux** | Sim, nativamente. | Use `--device=/dev/ttyUSB0` no `docker run`. |
| **macOS** | Não (limitação da VM). | Use o **PlatformIO nativo** no VS Code para o upload. |
| **Windows 11** | Parcialmente (via WSL 2). | Use o **PlatformIO nativo** (mais simples) ou configure o `usbipd-win`. |

---

## 🍎 macOS: A Limitação da VM

No macOS, o Docker Desktop roda dentro de uma máquina virtual (VM) que **não tem permissão para acessar diretamente os dispositivos USB** do seu Mac. Por isso, comandos como `--device=/dev/cu.usbserial-0001` não funcionam e retornam o erro `no such file or directory`.

### ✅ Solução Recomendada: PlatformIO Nativo

A forma mais prática e garantida de gravar no ESP32 no macOS é usar o **PlatformIO nativo** (fora do Docker). O Docker já cumpriu o papel de compilar e testar o código; o upload é uma operação simples que o PlatformIO nativo faz em segundos.

**Passo a passo:**

1. Abra o projeto no **VS Code** (o mesmo que você usa para o Docker).
2. Conecte o ESP32 via USB.
3. Na barra inferior azul, clique no ícone de **Upload** (seta para a direita).

O PlatformIO nativo vai compilar o sketch (se necessário) e gravar no ESP32, usando a porta USB diretamente.

### 🔧 Alternativa Avançada: Ponte Serial (ser2net)

Se você **realmente** precisar fazer o upload de dentro do Docker no macOS, é possível criar uma "ponte" serial usando o `ser2net`. Isso transforma a porta USB em um endereço de rede (TCP), que o container consegue acessar.

> **⚠️ Atenção:** Essa configuração é mais complexa e exige que o `ser2net` esteja rodando no Mac o tempo todo. Para a maioria dos casos, a solução nativa é mais simples.

---

## 🪟 Windows 11: WSL 2 e usbipd-win

No Windows 11, o Docker Desktop roda sobre o **WSL 2** (Windows Subsystem for Linux). É possível "passar" a porta USB para dentro do ambiente WSL (e, consequentemente, para o Docker) usando uma ferramenta chamada **`usbipd-win`**.

### ✅ Solução Recomendada: PlatformIO Nativo (*)

Assim como no macOS, a forma mais simples e indolor é usar o **PlatformIO nativo** no VS Code para o upload. No Windows, o PlatformIO acessa a porta `COM3` (ou similar) diretamente, sem precisar de configuração extra.

### 🔧 Alternativa Avançada: usbipd-win

Se você quiser fazer o upload de dentro do Docker, siga estes passos:

1. **Instale o `usbipd-win`** (no PowerShell como Administrador):

```powershell
winget install usbipd
```

Liste os dispositivos USB e anote o BUSID do seu ESP32 (geralmente aparece como "Silicon Labs CP210x" ou similar):

```powershell
usbipd list
```

Compartilhe o dispositivo:

```powershell
usbipd bind --busid <BUSID>
```

Anexe ao WSL:

```powershell
usbipd attach --wsl --busid <BUSID>
```

Verifique no WSL (abra o terminal do Ubuntu/WSL):

```bash
dmesg | tail
```

Você deve ver algo como cp210x converter now attached to ttyUSB0.

Use no Docker: Agora você pode mapear o dispositivo no docker-compose.yml ou no comando docker run:

```bash
docker run --rm -it \
    --device=/dev/ttyUSB0 \
    -v "$(pwd)":/app \
    -w /app \
    elo5-monitor \
    pio run -e esp32dev -t upload
```

> **💡 Dica:** No Windows, o caminho `$(pwd)` pode não funcionar no PowerShell. Use `${PWD}` ou o caminho absoluto da pasta.

## 🐧 Linux: Acesso Nativo

No Linux, o Docker tem acesso direto aos dispositivos USB. Basta mapear a porta correta no comando `docker run`.

## 1. **Descubra a porta serial:**

```bash
ls /dev/ttyUSB*
```

## ou

ls /dev/ttyACM*

## 2. **Rode o upload:**

```bash
docker run --rm -it \
    --device=/dev/ttyUSB0 \
    -v "$(pwd)":/app \
    -w /app \
    elo5-monitor \
    pio run -e esp32dev -t upload
```

⚠️ Permissões: Se você receber um erro de "permissão negada", adicione seu usuário ao grupo dialout:

```bash
sudo usermod -a -G dialout $USER
```

Depois, faça logout e login novamente.

## 📚 Referências

* [Documentação oficial do Docker](https://docs.docker.com/)
* [PlatformIO: Upload via linha de comando](https://docs.platformio.org/en/latest/core/userguide/cmd_run.html)
* [usbipd-win: Conectando dispositivos USB ao WSL](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

---

### 💡 O que esse README faz?

1. **Resume a situação:** Uma tabela mostra rapidamente o que funciona em cada sistema.
2. **Explica o "porquê":** Detalha a limitação do macOS e a solução do WSL 2 no Windows.
3. **Dá a solução mais simples primeiro:** Recomenda o PlatformIO nativo para macOS e Windows.
4. **Oferece alternativas avançadas:** Mostra como configurar o `ser2net` (macOS) e o `usbipd-win` (Windows) para quem quiser fazer o upload via Docker.
5. **Cobre o Linux:** Mostra que no Linux tudo funciona nativamente.

Salve esse arquivo como `README_DOCKER_HARDWARE.md` na raiz do projeto e adicione um link para ele no `README.md` principal. 🚀
