# 07 · Troubleshooting by layer

1. **No port appears:** check USB data cable, power, hub, and board connection; rerun port discovery. Do not infer port identity from its position in a list.
2. **Wrong device or connection failure:** stop; re-identify each cable/arm, confirm the selected port and device type, and check for another program holding the serial port.
3. **Unexpected movement:** interrupt immediately and remove power using the normal disconnect. Recheck arm identity, calibration IDs, mount, and joint mapping before another attempt.
4. **Calibration issue:** verify correct arm and ID, inspect saved calibration metadata, and repeat the correct arm's procedure only if warranted. Never overwrite a known-good calibration casually.
5. **Camera missing:** run camera discovery, check permissions/cable, and identify the correct index by its preview or output.
6. **Record/train import error:** read the first missing-package message. The current known issue is missing `datasets`; do not diagnose it as a robot or serial problem.
7. **Dataset looks poor:** inspect episode boundaries, video synchronization/view, task labels, action continuity, and success rate before changing model settings.

Change one variable at a time and note the result. This turns debugging into evidence rather than guesswork.
