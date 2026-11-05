# 02 · Setup and calibration

## Environment

This Mac already has a Python environment at `~/so101-env`. VS Code is configured to use it, and task commands call its `bin/` scripts directly. In a terminal, `source ~/so101-env/bin/activate` selects that environment for the current shell; `python --version` and `which python` let you confirm. Do not create a second environment just to follow a video that uses Conda: the important idea is isolation and a known package set, not the particular environment manager.

### Navigate to the project and check Python

Open Terminal.app, then run these one at a time:

```bash
cd ~/Documents/"SO-101 Control"
pwd
ls
source ~/so101-env/bin/activate
which python
python --version
```

`cd` changes the current directory. `pwd` prints it; `ls` lists the lesson files. The environment activation affects only this terminal session. You can also open the full multi-folder workspace from the command line if the `code` command is installed:

```bash
code ~/Documents/SO-101-Workspace.code-workspace
```

If `code` is not found, open VS Code and use **File → Open Workspace from File…**, then select `SO-101-Workspace.code-workspace` in Documents.

## Port discovery

Run **SO-101: Find arm ports** with one or both boards connected. A port is an operating-system name for a serial device, not a permanent identity. Unplugging, reconnecting, or using another USB socket can change it. Record which reported port belongs to which physical arm; do not guess based on ordering.

Terminal equivalent, after activating `so101-env`:

```bash
lerobot-find-port
```

This only discovers ports; it does not move the arms. If the command is not found, use its full path: `~/so101-env/bin/lerobot-find-port`.

## Calibration and IDs

Calibration tells the driver how raw motor positions relate to the arm's joint coordinates and safe ranges. The leader and follower each need their own calibration. IDs select the corresponding saved calibration: this workspace uses `michael_leader` for the leader and existing `michael_arm` for the follower. The files live under `~/.cache/huggingface/lerobot/calibration/`.

Run **SO-101: Calibrate leader** once with the leader port when it is securely placed and you are ready to follow the on-screen procedure. Calibration may require moving joints through ranges. Never force a joint against a stop. The task supplies the device type, port, and ID; it does not calibrate the follower. Preserve the existing follower calibration unless there is a reason to recalibrate it.

Terminal equivalent: replace the example port with the leader port you identified. This procedure can move/torque the leader, so prepare the arm and follow its prompts.

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYOUR_LEADER_PORT \
  --teleop.id=michael_leader
```

Do not paste the placeholder literally. To inspect the saved calibration folder in Finder from Terminal, run `open ~/.cache/huggingface/lerobot/calibration/`.

## Exercise

Run port discovery and make a tiny table in your learning log: device, port, how identified. Before calibrating, predict what information will be saved and where. Afterward, verify the expected calibration file exists without editing it.
