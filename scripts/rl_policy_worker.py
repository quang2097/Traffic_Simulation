"""Two-group policy inference. Protocol stdout is reserved for the native app."""
import argparse
import contextlib
import json
import math
import re
import sys
from pathlib import Path

ALGORITHMS = ("MAPPO", "IPPO", "QMIX", "IDQN", "MPLight", "CoLight", "PressLight", "MA2C", "DQN", "DuelingDQN")


def canonical(value):
    return next((name for name in ALGORITHMS if name.lower() == str(value).lower()), None)


def read_config(path):
    sidecar = Path(str(path) + ".json")
    config = json.loads(sidecar.read_text(encoding="utf-8")) if sidecar.exists() else {}
    if not isinstance(config, dict) or config.get("schema", 1) != 1:
        raise ValueError("Policy metadata must be a schema 1 JSON object")
    if "algorithm" in config and not canonical(config["algorithm"]):
        raise ValueError("Unknown algorithm in policy metadata")
    return config


def identify(path, config):
    if config.get("algorithm"):
        return canonical(config["algorithm"])
    words = re.split(r"[^a-z0-9]+", path.stem.lower())
    matches = [name for name in ALGORITHMS if name.lower() in words]
    return matches[0] if len(matches) == 1 else None


def notebook_network(state):
    """Recognize the supplied notebook's normalization and 28-128-128-3 Q net."""
    shapes = {"obs_mean": (28,), "obs_std": (28,),
              "net.0.weight": (128, 28), "net.0.bias": (128,),
              "net.1.weight": (128,), "net.1.bias": (128,),
              "net.3.weight": (128, 128), "net.3.bias": (128,),
              "net.4.weight": (128,), "net.4.bias": (128,),
              "net.6.weight": (3, 128), "net.6.bias": (3,)}
    return set(state) == set(shapes) and all(tuple(state[k].shape) == shape for k, shape in shapes.items())


def dueling_notebook_network(state):
    """Both train notebooks: dueling heads, optionally an embedded safety mask."""
    shapes = {"obs_mean": (28,), "obs_std": (28,),
              "backbone.0.weight": (256, 28), "backbone.0.bias": (256,),
              "backbone.1.weight": (256,), "backbone.1.bias": (256,),
              "backbone.3.weight": (256, 256), "backbone.3.bias": (256,),
              "backbone.4.weight": (256,), "backbone.4.bias": (256,),
              "val_head.0.weight": (128, 256), "val_head.0.bias": (128,),
              "val_head.2.weight": (1, 128), "val_head.2.bias": (1,),
              "adv_head.0.weight": (128, 256), "adv_head.0.bias": (128,),
              "adv_head.2.weight": (3, 128), "adv_head.2.bias": (3,)}
    if "min_green_t" in state:
        shapes["min_green_t"] = ()
    return set(state) == set(shapes) and all(tuple(state[k].shape) == shape for k, shape in shapes.items())


class Policy:
    def __init__(self, path, requested="Auto"):
        import numpy as np
        self.np = np
        self.path = Path(path).resolve(strict=True)
        self.config = read_config(self.path)
        detected = identify(self.path, self.config)
        requested = canonical(requested) if requested != "Auto" else None
        if requested and self.config.get("algorithm") and requested != detected:
            raise ValueError("Selected algorithm conflicts with the policy metadata")
        self.algorithm = requested or detected
        self.schema = self.config.get("observation_schema", "density_v1")
        if self.schema not in ("density_v1", "traffic_v2"):
            raise ValueError("observation_schema must be density_v1 or traffic_v2")
        self.training = self.schema == "traffic_v2"
        self.layout = self.config.get("layout", "joint")
        self.action_mode = self.config.get("action_mode", "hold_scores" if self.training else "green_seconds")
        if self.layout not in ("joint", "per_group"):
            raise ValueError("layout must be joint or per_group")
        allowed = ("hold_requests", "hold_scores", "joint_hold_scores") if self.training else ("green_seconds", "discrete_scores")
        if self.action_mode not in allowed:
            raise ValueError(f"action_mode for {self.schema} must be one of {allowed}")
        self.format = self.path.suffix.lower()
        if self.format == ".onnx":
            try:
                import onnxruntime as ort
            except ImportError as exc:
                raise ValueError("Install ONNX support in the selected Python: python -m pip install numpy onnxruntime") from exc
            options = ort.SessionOptions()
            options.intra_op_num_threads = options.inter_op_num_threads = 1
            self.model = ort.InferenceSession(str(self.path), sess_options=options, providers=["CPUExecutionProvider"])
            metadata = self.model.get_modelmeta().custom_metadata_map
            tagged = canonical(metadata.get("traffic.algorithm"))
            if metadata.get("traffic.algorithm") and not tagged:
                raise ValueError("Unknown traffic.algorithm in ONNX metadata")
            if tagged and requested and tagged != requested:
                raise ValueError("Selected algorithm conflicts with ONNX metadata")
            if tagged and self.config.get("algorithm") and tagged != detected:
                raise ValueError("Conflicting ONNX and sidecar algorithm metadata")
            self.algorithm = tagged or self.algorithm
            self.inputs = {node.name: node for node in self.model.get_inputs()}
            outputs = self.model.get_outputs()
            notebook_export = (list(self.inputs) == ["observation"] and
                               self.inputs["observation"].shape[1:] == [28] and
                               len(self.inputs["observation"].shape) == 2 and
                               len(outputs) == 1 and outputs[0].name == "q_values" and
                               len(outputs[0].shape) == 2 and outputs[0].shape[1] == 3)
            if notebook_export:
                # ONNX exposes the inference contract, not the training method.
                family = self.algorithm if self.algorithm in ("DQN", "DuelingDQN") else "DQN"
                self.use_notebook_contract(requested, family)
        elif self.format in (".pt", ".pth", ".torchscript"):
            # A raw state_dict does not identify its network. Never torch.load it.
            import zipfile
            if not zipfile.is_zipfile(self.path):
                raise ValueError("Raw weights require the original architecture; export an ONNX or TorchScript inference policy")
            with zipfile.ZipFile(self.path) as archive:
                if not any("/code/" in name for name in archive.namelist()):
                    raise ValueError("This is a raw state_dict checkpoint. Export the trained actor/Q network as ONNX or TorchScript")
            try:
                import torch
            except ImportError as exc:
                raise ValueError("Install TorchScript support in the selected Python: python -m pip install torch numpy") from exc
            self.torch = torch
            self.model = torch.jit.load(str(self.path), map_location="cpu").eval()
            state = self.model.state_dict()
            family = "DuelingDQN" if dueling_notebook_network(state) else "DQN" if notebook_network(state) else None
            if family:
                self.use_notebook_contract(requested, family)
                if not all(torch.isfinite(v).all() for v in state.values()) or not (state["obs_std"] > 0).all():
                    raise ValueError("Notebook policy has invalid weights or normalization buffers")
                if "min_green_t" in state and state["min_green_t"].item() < 0:
                    raise ValueError("Notebook policy has an invalid minimum-green timer")
        else:
            raise ValueError("Use an exported .onnx or TorchScript .pt/.pth/.torchscript policy")
        if not self.algorithm:
            raise LookupError("Choose the model family, then load the weights again")
        if self.action_mode == "discrete_scores":
            self.bins = self.config.get("green_options", [5, 10, 15, 20, 30, 45, 60])
            if not isinstance(self.bins, list) or not self.bins or any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) or not 5 <= v <= 120 for v in self.bins):
                raise ValueError("green_options must contain durations between 5 and 120 seconds")
        # Check the observation/action contract before exposing Enable in the UI.
        self.predict([0.0] * 28 if self.training else [0.25, 0.5], [1, 1])

    def use_notebook_contract(self, requested, family="DQN"):
        # These exports contain normalization already. Feed the native raw feature
        # order unchanged; action indices are group 0, group 1, hold.
        defaults = {"algorithm": family, "observation_schema": "traffic_v2",
                    "layout": "joint", "action_mode": "joint_hold_scores",
                    "observation_input": "observation", "output": "q_values"}
        if requested and requested != family:
            raise ValueError(f"Selected algorithm conflicts with the notebook {family} export")
        for key, value in defaults.items():
            actual = canonical(self.config[key]) if key == "algorithm" and key in self.config else self.config.get(key)
            if key in self.config and actual != value:
                raise ValueError(f"Policy metadata {key} conflicts with the notebook {family} export")
        if self.algorithm and self.algorithm != family:
            raise ValueError(f"Algorithm conflicts with the notebook {family} export")
        self.config = {**defaults, **self.config}
        self.algorithm = family
        self.schema = "traffic_v2"
        self.training = True
        self.layout = "joint"
        self.action_mode = "joint_hold_scores"

    def predict(self, densities, mask):
        np = self.np
        count = 28 if self.training else 2
        if len(densities) != count or len(mask) != 2 or any(not math.isfinite(v) for v in densities + mask) or any(not 0 <= v <= 1 for v in mask) or (not self.training and any(not 0 <= v <= 1 for v in densities)):
            raise ValueError(f"Expected {count} finite observation values and two active-group mask values")
        shape = (1, count) if self.layout == "joint" else (2, count // 2)
        observation = np.asarray(densities, dtype=np.float32).reshape(shape)
        args = [observation]
        feeds = {self.config.get("observation_input", "observations" if self.training else "densities"): observation}
        if self.config.get("mask_input"):
            active = np.asarray(mask, dtype=np.float32).reshape((1, 2) if self.layout == "joint" else (2, 1))
            feeds[self.config["mask_input"]] = active
            args.append(active)
        if self.config.get("adjacency_input"):
            adjacency = np.asarray(self.config.get("adjacency", [[1, 1], [1, 1]]), dtype=np.float32)
            if adjacency.shape != (2, 2) or not np.isfinite(adjacency).all():
                raise ValueError("adjacency must be a finite 2x2 matrix")
            adjacency = (adjacency * np.outer(mask, mask)).astype(np.float32, copy=False)
            feeds[self.config["adjacency_input"]] = adjacency
            args.append(adjacency)
        if self.format == ".onnx":
            if set(feeds) != set(self.inputs):
                raise ValueError(f"Policy expects inputs {list(self.inputs)}; configure observation_input, mask_input and adjacency_input in {self.path.name}.json")
            for name, value in feeds.items():
                node = self.inputs[name]
                if node.type != "tensor(float)" or len(node.shape) != value.ndim or any(isinstance(size, int) and size != actual for size, actual in zip(node.shape, value.shape)):
                    raise ValueError(f"Input {name} expects float32 {node.shape}; supplied {list(value.shape)}")
            names = [self.config["output"]] if self.config.get("output") else None
            outputs = self.model.run(names, feeds)
            if len(outputs) != 1:
                raise ValueError("Configure output in the sidecar to select the actor/Q output")
            result = np.asarray(outputs[0])
        else:
            with self.torch.inference_mode():
                output = self.model(*(self.torch.from_numpy(value) for value in args))
            if not isinstance(output, self.torch.Tensor):
                raise ValueError("Export a wrapper returning one tensor (manage recurrent state in the export wrapper)")
            result = output.detach().cpu().numpy()
        if not np.isfinite(result).all():
            raise ValueError("Policy returned nonfinite actions")
        if self.action_mode == "joint_hold_scores":
            if result.shape not in ((3,), (1, 3)):
                raise ValueError("Joint hold Q values must be [3] or [1,3]: group 0, group 1, hold")
            action = int(result.reshape(-1).argmax())
            return [int(action == 0), int(action == 1)]
        if self.action_mode == "hold_scores":
            if result.shape not in ((2, 2), (1, 2, 2)):
                raise ValueError("Hold policy scores must be [2,2] or [1,2,2]: hold/request per group")
            return result.reshape(2, 2).argmax(axis=1).tolist()
        if self.action_mode == "hold_requests":
            if result.shape not in ((2,), (1, 2), (2, 1)) or not np.isin(result, [0, 1]).all():
                raise ValueError("Hold requests must be two exact 0/1 actions")
            return result.reshape(-1).astype(int).tolist()
        if self.action_mode == "discrete_scores":
            if result.shape not in ((2, len(self.bins)), (1, 2, len(self.bins))):
                raise ValueError("Discrete policy output must be [2,K] or [1,2,K] logits/Q values")
            greens = [self.bins[int(index)] for index in result.reshape(2, -1).argmax(axis=1)]
        else:
            if result.shape not in ((2,), (1, 2), (2, 1)):
                raise ValueError("Green-duration output must have shape [2], [1,2], or [2,1]")
            greens = result.reshape(-1).tolist()
            if any(not 5 <= value <= 120 for value in greens):
                raise ValueError("Policy green durations must be between 5 and 120 seconds; export an appropriate output scaling")
        return [float(value) for value in greens]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("weights")
    parser.add_argument("--algorithm", default="Auto", choices=("Auto",) + ALGORITHMS)
    args = parser.parse_args()
    try:
        with contextlib.redirect_stdout(sys.stderr):
            policy = Policy(args.weights, args.algorithm)
        print("READY " + policy.algorithm + (" V2 HOLD" if policy.training else ""), flush=True)
    except LookupError as error:
        print("SELECT " + str(error), flush=True)
        return
    except Exception as error:
        print("ERROR " + str(error).replace("\n", " "), flush=True)
        return
    for line in sys.stdin:
        try:
            values = line.strip().split()
            if values == ["QUIT"]:
                return
            count = 28 if policy.training else 2
            if len(values) != count + 3 or values[0] != ("OBS2" if policy.training else "OBS"):
                raise ValueError("Observation protocol does not match the policy schema")
            with contextlib.redirect_stdout(sys.stderr):
                green = policy.predict(list(map(float, values[1:1+count])), list(map(float, values[1+count:])))
            print(("REQUEST " if policy.training else "ACTION ") + " ".join(map(str, green)), flush=True)
        except Exception as error:
            print("ERROR " + str(error).replace("\n", " "), flush=True)
            return


if __name__ == "__main__":
    main()
