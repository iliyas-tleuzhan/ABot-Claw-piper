# VLAC Policy Shadow Audit

Date: July 23, 2026

This audit covers the first shadow-only action-preview experiment for the already-downloaded VLAC-2B model. No PiPER command endpoint is exposed by the policy service, and every action response includes `execution_allowed: false`.

## Runtime

- Host: `iliyas@192.168.1.104`
- Repository: `/data/home/iliyas/ABot-Claw-piper`
- Existing critic container: `abot-vlac`
- Existing critic port: `8014`
- Policy-preview container: `abot-vlac-policy`
- Policy-preview port: `8016`
- Image used for policy preview: `abot-vlac:spm021`
- Model mount: `/data/home/iliyas/abot-data/vlac-model` mounted as `/model:ro`
- Hugging Face cache: `/data/home/iliyas/abot-data/huggingface`
- Policy GPU selection: physical GPU `0`, mapped inside the container as `cuda:0`
- Existing critic left running and not restarted.

## Existing Critic

`GET http://127.0.0.1:8014/health` returned healthy with `model_loaded: true` before and after starting policy preview.

The existing `main.py` provides the critic service only:

- `GET /health`
- `POST /critic`

The policy model was not added to this process to avoid disrupting the critic or increasing VRAM pressure in the existing service.

## Discovered Policy Interface

Source files inspected:

- `main.py`
- `README.md`
- `evo_vlac/examples/vla_example.py`
- `evo_vlac/utils/model_utils.py`
- `evo_vlac/utils/data_processing_vlm.py`

The bundled example uses:

- `GAC_model(tag="Policy")`
- one to three images
- a seven-value end-effector state
- a language task instruction

The example comments identify the expected raw state units as:

- XYZ in `0.001 mm`
- RPY in `0.001 degrees`

The implementation confirms that `GAC_model.format_state(..., gripper_format=False)` divides the first six values by `1000`, yielding prompt text in millimetres and degrees. The gripper convention is not fully verified.

Prompt action grammar discovered from `DataProcessor.action_format["songling"]`:

```text
{x: ...mm, y: ...mm, z: ...mm, roll: ... degrees, pitch: ... degrees, yaw: ... degrees, open: ...}
```

The action frame is not verified for the physical PiPER. It is recorded as `unknown_songling_end_effector_convention` and must not be executed.

## Policy-Preview API

`GET /health` returns model load, GPU, model path, shadow mode, and `execution_allowed: false`.

`GET /model-info` returns checkpoint, model name, expected camera count, state representation, action representation, units, action frame, and `execution_allowed: false`.

`POST /action-preview` accepts:

```json
{
  "images": ["<base64 image or data URI>"],
  "task_description": "Move toward the marked button",
  "end_effector_state": {
    "x_m": 0.0,
    "y_m": 0.0,
    "z_m": 0.0,
    "roll_rad": 0.0,
    "pitch_rad": 0.0,
    "yaw_rad": 0.0,
    "gripper_m": 0.0
  },
  "history": []
}
```

The response preserves the raw model output and includes parsed actions only when the VLAC grammar is recognized. Execution remains disabled.

## Saved-Image Shadow Results

Used a bundled example image and converted example state. All responses returned `execution_allowed: false`.

| Instruction | Raw output | Confidence | Latency |
| --- | --- | --- | --- |
| Move the arm to the left. | `{x: 91.6mm, y: 7.3mm, z: 9.4mm, roll: 4.4 degrees, pitch: -5.6 degrees, yaw: -8.1 degrees, open: 0.0}` | high | 997 ms |
| Move the arm to the right. | `{x: 89.8mm, y: 9.4mm, z: 8.8mm, roll: -2.1 degrees, pitch: -6.7 degrees, yaw: -6.9 degrees, open: 0.0}` | high | 345 ms |
| Move the arm upward. | `{x: 92.4mm, y: 8.0mm, z: 5.2mm, roll: 4.4 degrees, pitch: -6.2 degrees, yaw: -7.5 degrees, open: 0.0}` | high | 336 ms |
| Move the arm downward. | `{x: 90.2mm, y: 10.0mm, z: 8.7mm, roll: 3.8 degrees, pitch: -5.6 degrees, yaw: -6.5 degrees, open: 0.0}` | high | 335 ms |
| Move the arm forward. | `{x: 95.8mm, y: 6.9mm, z: 8.0mm, roll: 3.9 degrees, pitch: -5.8 degrees, yaw: -7.4 degrees, open: 0.0}` | high | 334 ms |
| Move the arm backward. | `{x: 90.5mm, y: 9.1mm, z: 9.5mm, roll: 3.6 degrees, pitch: -5.4 degrees, yaw: -8.3 degrees, open: 0.0}` | high | 335 ms |
| Open the gripper. | `{x: 94.1mm, y: 7.4mm, z: 10.6mm, roll: -2.8 degrees, pitch: -6.4 degrees, yaw: -1.6 degrees, open: 0.0}` | high | 332 ms |
| Close the gripper. | `{x: 85.2mm, y: 12.8mm, z: 10.1mm, roll: 3.2 degrees, pitch: -3.5 degrees, yaw: 1.1 degrees, open: 0.0}` | high | 334 ms |
| Move toward the marked button. | `{x: 85.7mm, y: 9.7mm, z: 8.2mm, roll: 4.1 degrees, pitch: -5.3 degrees, yaw: -7.6 degrees, open: 0.0}` | high | 338 ms |
| Press the marked button. | `{x: 104.6mm, y: 10.4mm, z: 12.4mm, roll: 4.2 degrees, pitch: -3.5 degrees, yaw: -4.4 degrees, open: 0.0}` | high | 333 ms |
| Retract from the button. | `{x: 95.6mm, y: 8.7mm, z: 8.0mm, roll: -3.4 degrees, pitch: -5.6 degrees, yaw: -6.7 degrees, open: 0.0}` | high | 333 ms |

Directional reasonableness is not established. The saved-image outputs are very similar across contradictory directional prompts and should not be considered execution-ready.

## Live Laptop Shadow Results

The laptop-side testbed client read a live RealSense color frame, `/end_pose`, and `/joint_states_single`, then sent one request for each instruction to port `8016`.

Live state used:

- `x_m`: `0.055984`
- `y_m`: `0.000667`
- `z_m`: `0.214169`
- `roll_rad`: `-0.23647466035271253`
- `pitch_rad`: `1.4782764231466787`
- `yaw_rad`: `-0.2255314459427081`
- `gripper_m`: `0.0`

| Instruction | Raw output | Parsed translation delta | Confidence |
| --- | --- | --- | --- |
| Move toward the marked button. | `{x: 0.1mm, y: 0.0mm, z: 0.0mm, roll: 0.0 degrees, pitch: 0.0 degrees, yaw: 0.0 degrees, open: 0.7}` | `[0.0001, 0.0, 0.0] m` | high |
| Press the marked button. | `{x: 0.0mm, y: 0.0mm, z: 0.0mm, roll: 0.0 degrees, pitch: 0.0 degrees, yaw: 0.0 degrees, open: 0.5}` | `[0.0, 0.0, 0.0] m` | high |
| Retract from the button. | `{x: 0.0mm, y: 0.0mm, z: 0.0mm, roll: 0.0 degrees, pitch: 0.0 degrees, yaw: 0.0 degrees, open: 0.4}` | `[0.0, 0.0, 0.0] m` | high |

Live-image directional reasonableness is not established. The action frame and gripper semantics remain unknown.

## Conclusion

VLAC-2B is usable as a shadow-only action generator endpoint for further logging and bakeoff comparisons. It should not be connected to PiPER execution until the embodiment, action frame, gripper convention, and directional behavior are validated against controlled data.
