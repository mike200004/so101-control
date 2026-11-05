# 04 · Demonstrations and recording

## From control to examples

A demonstration dataset is a time sequence of observations and actions grouped into episodes. An observation can include joint positions and camera frames; an action is the command sent to the follower. A task description gives context such as “pick up the block and place it in the bowl.” Teleoperation supplies the human-chosen actions. Good data has consistent camera views, a clearly defined start/end, successful examples, and enough variation to cover likely object positions and recoveries.

Recording is distinct from teleoperation: it stores data to disk (and may offer upload options depending on configuration). Check dataset destination and privacy before recording or uploading. Camera discovery is available from **SO-101: Find USB cameras**; the discovered index should be matched to the physical camera, and image size/FPS affect storage and training.

To discover cameras from Terminal, activate the environment and run:

```bash
cd ~/Documents/"SO-101 Control"
source ~/so101-env/bin/activate
lerobot-find-cameras opencv
```

This is a discovery command. To check whether the recording/training programs can start, the following commands only request help and should not operate the robot:

```bash
lerobot-record --help
lerobot-train --help
```

## Current software status

In this environment, invoking `lerobot-record --help` currently stops during import because `datasets` is missing; `lerobot-train --help` fails for the same reason. The error points to the `lerobot[dataset]` extra. No package was installed. Before recording, confirm the exact installed LeRobot version and its current CLI schema, then intentionally install the needed extra into `~/so101-env` if desired. Do not copy command lines from a different LeRobot release without checking its configuration names.

Useful read-only checks for the installed package version and missing dependency:

```bash
python -m pip show lerobot datasets
```

Once dependencies are available, use that version's help/config examples before constructing a recording command. The exact recording options are intentionally not guessed here because the help command currently fails before the CLI can show its accepted options. To install the optional extra later, the likely command is `python -m pip install 'lerobot[dataset]'`; this changes the existing environment and may update dependencies, so first review the package/version impact and then run it deliberately. It is not necessary for the earlier lessons.

## Exercise

Design a dataset on paper: one task, one sentence describing success, what the cameras should see, what counts as an episode, and three variations to record. Decide how you would label a failed or interrupted attempt. Do not record until the software prerequisite is resolved and the setup is physically ready.
