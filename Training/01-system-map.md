# 01 · System map

## The two-arm arrangement

The **leader** is the input device: you move it by hand. The **follower** is the robot that reproduces that pose. Each arm has six bus servos and a control board. The Mac talks to each board over a separate USB serial connection.

```text
your hand → leader servos → leader board/USB → LeRobot teleop loop
                                               ↓ desired joint positions
environment ← follower arm ← follower board/USB ←
```

During teleoperation, LeRobot reads the leader's joint positions, maps the named joints to the follower, and sends target positions repeatedly. It is joint-space mirroring: it does not understand objects or task intent. The display option in the task exposes state/action data for inspection; it is not a learned policy.

## The software layers

1. **Hardware:** servos, arm structure, control boards, cables, mount, optional cameras.
2. **Transport:** USB serial ports such as `/dev/tty.usbmodem…`.
3. **Device drivers:** LeRobot's SO-101 leader and follower implementations.
4. **Calibration:** maps raw servo readings into usable joint positions and limits.
5. **Teleoperation:** repeatedly reads leader state and commands follower actions.
6. **Data:** demonstrations store observations, actions, task labels, and optional video.
7. **Policy:** a trained model predicts actions from observations; evaluation runs those predictions.

The code for the installed drivers is in the workspace folder **LeRobot Code (used by tasks)**. Start exploring at `teleoperators/so_leader/`, `robots/so_follower/`, and `scripts/lerobot_teleoperate.py`. Read configuration and data flow before changing library code.

## Exercise

Without connecting a robot, point to each layer in the workspace and explain: Which parts are physical? Which command discovers a serial port? Where do calibration files live? What does teleoperation copy, and what does it not know?
