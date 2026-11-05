# 05 · Training and evaluation

## The learning pipeline

```text
human demonstrations → dataset inspection → policy training → held-out evaluation → careful deployment
```

Training adjusts a policy's parameters so that, given observations, its predicted actions resemble demonstrated actions. It does not teach abstract intent from a handful of examples automatically. Dataset consistency, camera placement, task definition, and success rate all matter. Evaluation uses the policy to act and measures task outcomes; teleoperation uses a human to act. Keep evaluation separate from collection so you can tell whether the model learned to generalize.

LeRobot includes policy families such as ACT, but valid options and configuration fields depend on the installed version and hardware. The dataset extra is currently absent here, so training commands cannot yet import successfully. Resolve that prerequisite and inspect the installed CLI/configuration before choosing a policy or launching a long run. Save checkpoints and record the dataset version, policy config, and evaluation results together.

## Exercise

Write down one success metric that can be judged consistently (for example, object ends inside a marked target area). List what must be held constant between demonstrations and what variation should be included. Explain why replaying training examples is not enough to show generalization.
