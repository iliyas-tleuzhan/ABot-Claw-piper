# OpenPI PiPER Policy Service

This service is the remote pi0.5 policy endpoint for the new default PiPER
manipulation path.

- Official OpenPI repository: `https://github.com/Physical-Intelligence/openpi.git`
- Pinned commit: `15a9616a00943ada6c20a0f158e3adb39df2ccac`
- Base checkpoint for fine-tuning: `gs://openpi-assets/checkpoints/pi05_base`
- Default port: `8017`
- Physical PiPER execution requires a PiPER-fine-tuned checkpoint with complete
  metadata and `piper_compatible: true`.

Public checkpoints are protocol/shadow-only. They must not command PiPER
hardware.

Start after a PiPER OpenPI config has been added to the official checkout:

```bash
OPENPI_ROOT=~/ABot-Claw-piper/openpi \
OPENPI_CONFIG=pi05_piper_single_arm \
OPENPI_CHECKPOINT=checkpoints/pi05_piper_single_arm/piper_cup_phase_v1/20000 \
./service_layer/OpenPI/start_openpi_piper_service.sh
```

Health:

```bash
python3 service_layer/OpenPI/check_openpi_piper_service.py --host 127.0.0.1 --port 8017
```

