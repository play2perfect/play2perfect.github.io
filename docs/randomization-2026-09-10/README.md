# Local randomization experiment

This branch adds Randomize next to Reset for all five tasks. It returns the movable parts near their initial poses with independent uniform offsets of ±1 cm in X/Y and ±10 degrees of world-Z yaw. It preserves robot joint state and accumulated actuator targets, clears part velocities and policy recurrent state, and restarts task progress. During playback it continues running; while paused it stays paused.

Multi-Part Assembly uses the original one-way policy sequence: policy 1, then policy 2. The experimental dislodgement detector and automatic switch back to policy 1 were removed at Tyler’s request. No policy weights or physics parameters were changed by this experiment.

The JSON reports here were collected before pulling the newer public scenes. For the broader set of five sampled poses, the initial / 30-control-step / 100-control-step success counts were:

| Task | Initial | 30 steps | 100 steps |
| --- | --- | --- | --- |
| Screwing | 5/5 | 4/5 | 4/5 |
| Tight Insertion | 5/5 | 4/5 | 5/5 |
| Multi-Part | 0/5 | 0/5 | 0/5 |

These are small diagnostic samples, not reliable success-rate estimates. The newer scenes and two additional tasks were merged from public main at 8474cae. All five then passed browser loading and Randomize-button smoke checks; rollout results above do not validate those newer scenes. Tests also verified target preservation, recurrent reset, and invalidation of in-flight inference after teleporting.

Run the preview from this repository:

```sh
python3 -m http.server 8778 --bind 127.0.0.1
```

Open http://localhost:8778/interactive/.

Offline worker checks: `node scripts/check_randomization.mjs fabrica baseline` or `node scripts/check_randomization.mjs fabrica race`. These execute the actual WASM physics and exported policies in Node.

GitHub Pages was verified to deploy from main at the repository root. Publishing this experimental branch does not update the public site.
