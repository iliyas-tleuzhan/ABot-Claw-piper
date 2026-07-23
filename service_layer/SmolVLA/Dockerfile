FROM nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG LEROBOT_COMMIT=9c82c39c7b541e9c5bd8340abb7c9d8803c98744

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.12 python3-pip python3.12-venv git curl ca-certificates build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 1
RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH
RUN python3 -m pip install --upgrade pip setuptools wheel
RUN python3 -m pip install --no-cache-dir torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu128

WORKDIR /opt
RUN git clone https://github.com/huggingface/lerobot.git && cd lerobot && git checkout ${LEROBOT_COMMIT}
RUN python3 -m pip install --no-cache-dir -e '/opt/lerobot[smolvla]'

WORKDIR /opt/smolvla
COPY service_layer/SmolVLA/requirements.txt ./requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY service_layer/SmolVLA/app.py ./app.py
COPY service_layer/SmolVLA/tests ./tests

RUN useradd -m appuser && chown -R appuser:appuser /opt/smolvla
USER appuser

ENV PYTHONUNBUFFERED=1
ENV HF_HOME=/data/huggingface
ENV HUGGINGFACE_HUB_CACHE=/data/huggingface
ENV SMOLVLA_PORT=8018
EXPOSE 8018
HEALTHCHECK --interval=30s --timeout=10s --retries=3 CMD curl -fsS http://127.0.0.1:8018/health || exit 1
CMD ["python3", "app.py"]
