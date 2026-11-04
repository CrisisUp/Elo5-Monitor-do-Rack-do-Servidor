# =========================================
# STAGE 1: Build (Compilação)
# =========================================
FROM python:3.11-slim AS builder

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PLATFORMIO_CORE_DIR=/opt/platformio

# Instala dependências do sistema
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Instala o PlatformIO
RUN pip install --no-cache-dir platformio

WORKDIR /app

# Copia o projeto (o .dockerignore vai filtrar o que não presta)
COPY . .

# --- PREPARAÇÃO DO AMBIENTE PLATFORMIO ---
# 1. Cria a pasta src (se não existir)
RUN mkdir -p src

# 2. Move o sketch.ino para src/main.cpp (o PlatformIO compila .cpp)
RUN cp sketch.ino src/main.cpp

# 3. Cria o platformio.ini se ele não existir
#    (Ajuste a placa e as lib_deps conforme o seu projeto real)
RUN if [ ! -f platformio.ini ]; then \
    echo "[env:esp32dev]" > platformio.ini && \
    echo "platform = espressif32" >> platformio.ini && \
    echo "board = esp32dev" >> platformio.ini && \
    echo "framework = arduino" >> platformio.ini && \
    echo "monitor_speed = 115200" >> platformio.ini; \
    fi

# 4. Compila o firmware
#    (Se você tiver um platformio.ini próprio com as lib_deps, ele será usado)
RUN pio run -e esp32dev

# =========================================
# STAGE 2: Runtime (Imagem final leve)
# =========================================
FROM python:3.11-slim AS runtime

LABEL maintainer="Seu Nome"
LABEL description="Ambiente para compilar e gravar firmware ESP32 (sketch.ino)"

# Instala dependências mínimas de runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    && rm -rf /var/lib/apt/lists/*

# Instala o PlatformIO (necessário para upload/monitor)
RUN pip install --no-cache-dir platformio

# Cria usuário não-root
RUN useradd -m -u 1000 pio

WORKDIR /app

# Copia o binário compilado
COPY --from=builder /app/.pio/build/esp32dev/firmware.bin ./firmware.bin

# Copia o platformio.ini gerado e a pasta src (para permitir upload)
COPY --from=builder /app/platformio.ini ./
COPY --from=builder /app/src ./src

# Muda para o usuário não-root
USER pio

CMD ["bash"]