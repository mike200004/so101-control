# SO-101 learning path

This is a hands-on sequence, not a list of commands to copy blindly. Read the lesson, predict what you expect to see, run only the exercise you intend, then write down what happened. No robot task starts automatically when VS Code opens.

## Course map

| Lesson | Topic | What you should be able to explain |
|---|---|---|
| [01](01-system-map.md) | System map | How the leader, follower, USB serial, LeRobot, and calibration fit together |
| [02](02-setup-and-calibration.md) | Setup | Port identity, calibration, IDs, and the existing Python environment |
| [03](03-teleoperation.md) | Teleoperation | The control loop, joint commands, feedback, and safe stopping |
| [04](04-data-and-recording.md) | Demonstrations | What a dataset records, episode quality, cameras, and current software readiness |
| [05](05-training-and-evaluation.md) | Learning policies | How demonstrations become a policy and how evaluation differs from teleoperation |
| [06](06-print-parts-and-hardware.md) | Hardware | What is in the print folders and how physical setup affects data |
| [07](07-troubleshooting.md) | Troubleshooting | A layered way to diagnose connection, motion, and data problems |

## Suggested rhythm

For each lesson: (1) read it, (2) explain the key idea in your own words, (3) do its exercise, and (4) record observations in `Learning-Log.md`. Start with observation-only tasks such as listing ports. Before any motion, check the follower mount, arm clearance, USB identity, and calibration. Keep a hand near the power switch and never reach into the moving workspace.

## Current readiness note

The local `lerobot-record` and `lerobot-train` entry points currently fail at import because the environment lacks the optional `datasets` package (`lerobot[dataset]`). Teleoperation is a separate entry point. The workspace has not installed anything or run a robot. Dataset collection and training lessons explain the concepts and prerequisites, but wait to run those stages until the dependency issue is deliberately resolved and checked against the installed LeRobot version.

## Use the VS Code tasks as learning instruments

The tasks are visible under **Terminal → Run Task**. `Find arm ports` is observational. `Calibrate leader` moves through a calibration procedure and writes calibration data. `Teleoperate leader → follower` enables live movement. Read [02](02-setup-and-calibration.md) and [03](03-teleoperation.md) before running the latter two.

## Terminal-first route

If you prefer the terminal, open Terminal.app and run:

```bash
cd ~/Documents/"SO-101 Control"
source ~/so101-env/bin/activate
pwd
which python
python --version
```

`pwd` should show the SO-101 Control folder, and `which python` should point inside `~/so101-env`. Then use the exact command sequences in lessons 02–04. Commands that can move an arm are clearly marked there. Close the terminal or run `deactivate` to leave the environment.
