FROM pytorch/pytorch:2.8.0-cuda12.8-cudnn9-runtime

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends curl && rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 app

WORKDIR /workspace/OpenVLA

COPY requirements.txt /tmp/requirements.txt
RUN pip install --upgrade pip && pip install -r /tmp/requirements.txt

COPY app.py /workspace/OpenVLA/app.py
RUN chown -R app:app /workspace/OpenVLA

USER app

EXPOSE 8018

HEALTHCHECK --interval=30s --timeout=10s --start-period=120s --retries=5 CMD curl -fsS http://127.0.0.1:8018/health || exit 1

CMD ["python", "app.py"]
