# Robot Fleet Endpoints

## Piper

Use the clean Piper language-action server for the Piper arm currently connected on this laptop.

```bash
PIPER_BASE_URL=http://localhost:8891
```

Health check:

```bash
curl --noproxy '*' http://localhost:8891/health
```

State check:

```bash
curl --noproxy '*' http://localhost:8891/state
```

Notes:

- Do not use old port `8890`.
- Do not use ABot `/code/execute`.
- Do not use `/end_pose` for action control.
- Safe language-action endpoints are `POST /move_up`, `POST /move_down`, `POST /open_gripper`, and `POST /close_gripper`.
