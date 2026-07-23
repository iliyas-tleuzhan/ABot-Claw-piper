# Safety Boundary

This service is shadow-only.

It does not:

- publish robot commands
- call MoveIt services
- call 8891 movement endpoints
- import PiPER executors
- import CAN backends
- expose any execution endpoint

Every success and error response forces:

```json
{
  "execution_allowed": false
}
```
