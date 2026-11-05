# SO-101 Learning Workspace

This workspace is organized as a small robotics course. Follow [Training/README.md](Training/README.md) in order; the goal is to understand each layer before asking the robot to move.

## Workspace map

| Folder | What it contains |
|---|---|
| `Training/` | Lessons, concepts, exercises, and runbooks |
| `SO-101 Print Parts/` | Leader/follower STLs, Prusa project and print files, camera mount |
| `Follower Print Parts/` | Extracted follower STL meshes and their inventory |
| `LeRobot Code (used by tasks)/` | The installed LeRobot source that the tasks invoke |

Open `~/Documents/SO-101-Workspace.code-workspace` in VS Code. The tasks use the existing Mac environment at `~/so101-env`; they do not create a second environment. A GitHub Codespace cannot access USB robot boards plugged into this Mac.

## Current setup

- Follower calibration ID: `michael_arm`
- Leader calibration ID: `michael_leader` (calibrate once before use)
- Calibration files: `~/.cache/huggingface/lerobot/calibration/`
- VS Code tasks: **Terminal → Run Task**
- The arms connect over USB serial; camera discovery is available when cameras are connected.

## First steps

1. Read [the learning path](Training/README.md).
2. Read [the system map](Training/01-system-map.md) before connecting anything.
3. Learn the port and calibration workflow in [02-setup-and-calibration.md](Training/02-setup-and-calibration.md).
4. Review safe motion and the control loop in [03-teleoperation.md](Training/03-teleoperation.md).

The shortcut task labels are there for convenience. Each lesson explains the ideas, inputs, and expected observations behind those shortcuts. Secure the follower arm and clear its full movement area before enabling torque or motion.
