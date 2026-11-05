# 03 · Teleoperation

## What the loop does

The task runs LeRobot's `lerobot-teleoperate` program with an SO-101 leader and follower. At each cycle it reads the leader's joint state, builds an action for the follower, sends that action, and may display the values. This is direct human control. No task data is collected merely by teleoperating, and no policy is being trained.

The task asks for the follower and leader ports, then associates each arm with its calibration ID. The mapping is explicit in `.vscode/tasks.json`. `--display_data=true` requests state/action display; it does not mean camera video is enabled.

## Before motion

- Bolt or clamp the follower to a stable surface.
- Clear the complete swept volume, including the gripper and cables.
- Check both port identities and that the leader can move freely.
- Keep the emergency power disconnect reachable; keep people and loose objects clear.
- Start slowly, move a small amount, and stop if direction or joint motion looks wrong.

Run **SO-101: Teleoperate leader → follower** only after understanding the checks above. Stop with the terminal's interrupt key (Ctrl+C) if needed. Power down using the arm's normal procedure.

### Terminal command

Activate the environment and substitute both real ports discovered in lesson 02. This starts live follower motion. Check the physical safety list above before pressing Enter.

```bash
cd ~/Documents/"SO-101 Control"
source ~/so101-env/bin/activate
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodemYOUR_FOLLOWER_PORT \
  --robot.id=michael_arm \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYOUR_LEADER_PORT \
  --teleop.id=michael_leader \
  --display_data=true
```

Press **Ctrl+C** in that terminal to stop the loop. Keep the terminal visible so you can stop promptly. The VS Code task runs the same program and options, prompting for the two ports.

## Exercise

Before starting, predict which arm you will physically move and which arm will respond. Observe one joint at a time. Describe what you see in the displayed state/action values. Explain why the mapping is not object-aware and why a learned policy is a separate stage.
