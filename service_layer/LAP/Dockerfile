FROM pytorch/pytorch:2.7.0-cuda12.8-cudnn9-runtime

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        git \
        git-lfs \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --no-cache-dir uv huggingface_hub

WORKDIR /workspace/lap

ENV PYTHONUNBUFFERED=1 \
    PORT=8016 \
    HF_HOME=/root/.cache/huggingface \
    OPENPI_DATA_HOME=/root/.cache/openpi \
    UV_LINK_MODE=copy

CMD ["bash", "-lc", "GIT_LFS_SKIP_SMUDGE=1 uv sync --group cuda && JAX_PLATFORMS=cuda uv run --group cuda scripts/serve_policy.py --env=LAP --port=${PORT}"]
