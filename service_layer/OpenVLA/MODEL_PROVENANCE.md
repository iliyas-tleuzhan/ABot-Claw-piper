# Model Provenance

- Model ID: `openvla/openvla-7b`
- Checkpoint revision: `47a0ec7fc4ec123775a391911046cf33cf9ed83f`
- Official repository: `https://github.com/openvla/openvla`
- Official source revision used for audit: `c8f03f48af692657d3060c19588038c7220e9af9`
- Transformers custom code files loaded from checkpoint:
  - `configuration_prismatic.py`
  - `processing_prismatic.py`
  - `modeling_prismatic.py`

Pinned runtime:

- PyTorch: `2.8.0`
- CUDA runtime: `12.8`
- Transformers: `4.40.1`
- tokenizers: `0.19.1`
- timm: `0.9.10`

FlashAttention is not required for this shadow service. The official examples mark it as optional.
