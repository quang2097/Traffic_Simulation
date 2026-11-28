# Two-agent traffic-light policy controller

Open **RL controller** in `build/traffic_rl.exe`. This is an inference integration for future trained policies, not a trainer or pretrained model collection.

For camera-limited queue categories, phase ages, cumulative signal waits, held phases, final all-red **Tr**, randomized datasets and the live Python training environment, see [Training data and traffic-v2 policies](TRAINING_DATA.md). The updated application is `build/traffic_training_ui.exe`. The density/green-duration contract below remains available as legacy density-v1; traffic-v2 is selected explicitly in the policy sidecar.

## Configure the junction

Up to four physical signals belong to **two control groups**. Opposing approaches on the same axis are grouped automatically: horizontal OUT directions in group 1, vertical OUT directions in group 2. Change membership if the road geometry needs it. Every signal must be enabled and both groups must have a member. The group, rather than an individual light, is an RL agent.

Each physical signal has an upstream rectangle aligned with its controlled OUT direction. Default depth is 30 m; default width is its stop-line length. The UI shows rectangles, vehicle counts and densities on the map. Change depth/width before enabling control, using the same setup used in training.

Density is the sum of vehicle footprint area clipped to the rectangle divided by rectangle area, clamped to `[0,1]`. Only vehicles traveling in the signal's OUT direction count. IN-direction bodies do not contribute. Stationary vehicles retain their travel heading and count normally. This is an instantaneous footprint-area measure, not vehicle count, a rolling queue-detector percentage, or road-mask occupancy. The denominator includes the full rectangle, so fit it to the intended approach. Group density is total occupied area / total observation area across its signals, not the unweighted average of percentages. The simulator normally prevents overlapping bodies; overlapping diagnostic bodies can cause area to be counted twice before the clamp. Overlapping observation rectangles also sample vehicles independently.

Legacy density-v1 models receive exactly **two group densities**, ordered group 1 then group 2. The newer traffic-v2 contract and live training environment are documented in [Training data](TRAINING_DATA.md). `observeSignals` and `SignalTimingController` remain the density-v1 simulator-side API.

## Actions and synchronized timing

The policy chooses **two green durations**, each between **5 and 120 seconds**. Yellow is configured in the UI (1–10 s, default 3 s); all-red clearance is also configured there (0.5–10 s, default 1 s). A shared cycle is:

1. Group 1 green, then yellow, then all-red clearance.
2. Group 2 green, then yellow, then all-red clearance.

Each group's red time is derived from the other group's green/yellow and the two clearance periods. All signals in a group use the same timing and phase. IN/OUT direction, permission gates and detectors are preserved. Existing permissive red-turn rules still apply to vehicles, as they do with manual signals.

Enabling control first finishes a yellow guard and clearance. New green-duration actions are queued until the next common cycle boundary; they never truncate a current green/yellow interval. The decision interval is simulated time (default 5 s). The latest received action supplies the next cycle. Inference runs in a background worker, so UI/physics do not wait for it; if inference is slower than the decision interval, the last valid timings continue. Train against this delayed, cycle-boundary action behavior. The current controller is not a direct phase-switching action interface.

**Stop policy / restore manual timings** restores the original manual durations/offsets. A map/network/engine reset also stops policy timing. Load/inference errors stop policy control and restore manual timings. Editing lights is disabled during policy control. The loaded model can be enabled again after a reset; stale responses from an earlier control session are ignored.

## Policy files and model detection

### train (1).ipynb: embedded safety-mask weights

The newer notebook's `traffic_policy.pt` is supported by **Auto** and **DuelingDQN**. Its network adds a scalar `min_green_t` buffer and a tensor-based action mask to the earlier dueling model. The loader recognizes this signature and executes the exported mask directly: either group's yellow phase or positive Tr countdown permits only hold; an opposing switch is blocked until the active green reaches the model's minimum-green timer. Normalization and masking are not duplicated outside the model. Invalid negative/nonfinite minimum-green buffers are rejected for recognized TorchScript exports.

For `traffic_policy.onnx`, choose **DuelingDQN** (Auto also accepts the compatible joint DQN contract). The exported ONNX graph contains the same mask and normalization; `model_meta.json` is not required. The action indices and raw 28-feature ordering are unchanged. The earlier unmasked dueling and plain DQN exports remain supported.

Before enabling control, open **Training data → Hold / switch and final Tr** and match the notebook's configuration: **Tr 20 s, yellow 3 s, minimum green 5 s** in the supplied file. Tr and yellow durations are environment settings, not values encoded in this notebook's model metadata, so loading weights does not silently change the simulator's timers. Native signal timing still enforces the full yellow/all-red sequence independently of the policy output. Tests export the reference model from this notebook and check TorchScript/ONNX numerical parity, both groups' yellow/Tr interlocks, minimum-green boundaries, worker transport, and a full 20-second simulator clearance.

### train.ipynb: CQL followed by dueling Double-DQN

The supplied `train.ipynb` has a separate **DuelingDQN** implementation in **RL controller → Model family**. Load its exported **traffic_policy.pt** with **Auto** or **DuelingDQN**. Auto recognizes the notebook's exact default normalization buffers, 28 → 256 → 256 backbone (Linear/LayerNorm/SiLU), and separate 128-unit value/advantage heads. Inference executes the exported `V + A - mean(A)` computation directly; the app does not reconstruct or retrain the network.

The notebook's **traffic_policy.onnx** also works. Choose **DuelingDQN** to identify its training family explicitly. **Auto** accepts the shared `observation: [batch,28] → q_values: [batch,3]` contract as **DQN** because that tensor contract alone does not identify a dueling architecture; both selections use the same observation and action mapping. A standard `<weights-path>.json` sidecar or ONNX `traffic.algorithm` tag can explicitly identify `DuelingDQN` as well.

Both formats already contain normalization. Feed the native raw 28 traffic-v2 queue/phase/wait features unchanged. The notebook's sibling **model_meta.json** is informational and is not required for inference or applied a second time. Q-value index **0 requests group 0** (Group 1 in the UI), **1 requests group 1** (Group 2), and **2 holds**. Yellow, minimum green, maximum red and final all-red Tr remain handled by the shared native hold/request controller. Match the camera, queue cutoffs, group order and timing to the training data.

TorchScript needs `torch` and `numpy` in the selected interpreter; ONNX needs `onnxruntime` and `numpy`. The notebook is not executed by the loader. Compatibility checks use the reference network definition from the notebook and verify numerical agreement for exported TorchScript/ONNX, all three action choices, the C++ worker transport, and synchronized simulator lights. These checks validate inference integration rather than the performance of trained weights.

### Supplied Untitled0.ipynb training notebook

Load the notebook's **traffic_policy.pt** with **Model family: Auto**. The loader recognizes its normalization buffers and exact 28 → 128 → 128 → 3 network signature, selects DQN and traffic-v2 observations, and decodes Q-value indices as request group 0, request group 1, hold. No sidecar is needed. Normalization stays inside the model; the native 28 raw queue/phase/wait features are passed unchanged. Yellow, minimum green, maximum red and final all-red Tr remain enforced by the native hold controller. Set camera/group membership and timing in Training data to match your training run; group 0 in the notebook is Group 1 in the UI.

The notebook's ONNX contract is also recognized: one `observation` float input `[batch,28]` and one `q_values` output `[batch,3]`. Modern PyTorch exports may save weights in **traffic_policy.onnx.data**; keep it beside the ONNX file with that exact name. The Downloads export inspected on 2026-11-26 is missing that companion file; use its self-contained `.pt` instead.

The notebook trains partly in a surrogate environment whose phase transitions differ from the native safety controller. Successful loading verifies the inference contract, not traffic performance; evaluate the resulting policy in the native simulation. The notebook itself is not executed by the loader.

Supported inference formats:

- **ONNX `.onnx`**: graph and weights, executed on CPU by ONNX Runtime.
- **TorchScript `.pt`, `.pth`, `.torchscript`**: exported inference module loaded with `torch.jit.load` on CPU.

The model-family list includes **MAPPO, IPPO, QMIX, IDQN, DQN, DuelingDQN, MPLight, CoLight, PressLight, MA2C**. These names identify the policy's training family; they do not specify a unique neural architecture or checkpoint layout. No architecture is guessed or fabricated from the algorithm name. Export the trained inference actor/Q network, or a wrapper matching the contract below. A QMIX mixer is a training component; export the executable per-agent Q inference outputs, not just mixer weights.

Auto detection uses ONNX `traffic.algorithm` metadata, a `<weights-path>.json` sidecar's `algorithm`, or one unambiguous algorithm token in the filename (for example `ippo_actor.onnx`). If none identifies the family, the app opens **Choose policy model**. Explicit metadata conflicts are rejected. Manually choosing a family resolves identification, not input/output incompatibility.

Raw PyTorch `state_dict` files and arbitrary Python-pickled model objects are not loaded. They require the original training architecture and preprocessing; selecting MAPPO or another name alone cannot supply those. Export them from the training project first. Arbitrary framework-native HDF5/checkpoint directories are likewise not accepted.

No recurrent-state, critic-state or optimizer inputs are synthesized. The current contract is a stateless inference call; recurrent/graph implementations must export an appropriate trained adapter with exactly the supported inputs/outputs. Policies trained with queue lengths, phase states, larger traffic networks, different agent ordering, other density definitions or direct phase-switch actions require adaptation/retraining.

## Export contract

Default ONNX input:

```text
densities: float32 [1, 2] = [[group_1_density, group_2_density]]
```

Default output is one tensor of actual green seconds, shape `[2]`, `[1,2]`, or `[2,1]`. Output scaling belongs in the trained model/export wrapper. Nonfinite values and values outside `[5,120]` are rejected, not silently clamped. A sample input/output is validated on load before control can be enabled.

A sidecar changes names/layout/action decoding. For `junction.onnx`, save `junction.onnx.json`:

```json
{
  "schema": 1,
  "algorithm": "QMIX",
  "layout": "joint",
  "observation_input": "densities",
  "output": "q_values",
  "action_mode": "discrete_scores",
  "green_options": [5, 10, 15, 20, 30, 45, 60]
}
```

For `discrete_scores`, export logits/probabilities/Q values of shape `[2,K]` or `[1,2,K]`, with `K == len(green_options)`. Inference deterministically takes the argmax independently for each group and looks up the green duration. Supply the exact action table used in training. This is also suitable for discrete MAPPO/IPPO actors. `green_seconds` is the default action mode for an export that already returns durations.

Optional settings:

| Sidecar key | Behavior |
| --- | --- |
| `layout: "per_group"` | Densities and optional mask are `[2,1]` instead of `[1,2]`. |
| `mask_input: "active_groups"` | Extra float32 input matching the density shape. A configured enabled group is 1; absent is 0. Both must be present to enable the controller. |
| `adjacency_input: "adjacency"` | Extra float32 `[2,2]` input. Defaults to all ones (two mutually connected groups, including self links). |
| `adjacency: [[1,1],[1,1]]` | Explicit graph expected by an exported CoLight/graph adapter; inactive group rows/columns are masked. |
| `output: "name"` | Select one named ONNX output when the graph also exports critics, diagnostics, etc. |

TorchScript takes positional tensors in the order densities, optional mask, optional adjacency, and returns one tensor with the same output contract. It uses the same sidecar options except ONNX input/output names. Use a wrapper if the trained network returns tuples/distributions or requires a larger training state. The same shared network can run on two local observations via `per_group`, or an export can package both agents' separate weights into a joint two-output module.

For a trained actor that already follows this contract, an ONNX export in its training project can look like:

```python
actor.eval()
torch.onnx.export(
    actor, torch.zeros(1, 2), "junction.onnx",
    input_names=["densities"], output_names=["green_seconds"],
    opset_version=17,
)
# Or: torch.jit.trace(actor, torch.zeros(1, 2)).save("junction.pt")
```

The snippet exports an existing trained actor; it does not construct or train one. Use the model's actual dtype, normalization and action table during training/export.

## Runtime setup and validation

Use a Python version supported by the chosen inference runtime (the configured workspace environment uses Python 3.13):

```powershell
python3.13 -m venv .venv-rl313
.\.venv-rl313\Scripts\python.exe -m pip install -r requirements-rl.txt
# Optional TorchScript:
.\.venv-rl313\Scripts\python.exe -m pip install torch
```

Choose another interpreter under **RL controller → Load policy → Python runtime** if needed. CPU inference is isolated in a persistent subprocess. The app does not execute a shell command or unpickle raw checkpoints when loading weights. A worker failure, malformed response or 30-second timeout disables control with an error message.

For export/transport tests, additionally install `onnx`, then build:

```powershell
cmake --build build --target traffic_ui signal_learning_tests policy_runtime_tests sumo_ui_smoke -j 4
.\.venv-rl313\Scripts\python.exe tests/PolicyWorkerTests.py
ctest --test-dir build -R "signal_learning|policy_worker|editor_tools_and_manual" --output-on-failure
```

Tests generate small synthetic policies in temporary directories; these are inference fixtures, not trained traffic policies. Tests cover real exported ONNX execution for all family identifiers, model-selection fallback, action tables, shape/NaN failures, native process transport, and the UI-to-policy-to-synchronized-lights path. Timing tests check two-group exclusion, paired phases, clearance and manual restoration. Density tests check body clipping, area weighting and IN-direction exclusion.

References: [ONNX Runtime inference API](https://onnxruntime.ai/docs/api/python/api_summary.html), [PyTorch model saving/loading](https://docs.pytorch.org/tutorials/beginner/saving_loading_models.html).
