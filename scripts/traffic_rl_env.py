"""Synchronous two-agent Native environment; no external gym dependency."""
import json
import subprocess
from pathlib import Path
import numpy as np


class TrafficLightEnv:
    """Observation (2,14), per-group rewards (2,), global action 0/1/2.

    0 requests group 1 green; 1 requests group 2 green; 2 holds the current
    allocation. Yellow, the final all-red Tr and maximum red override actions.
    reset/step use a live simulator, suitable for on-policy data collection.
    """
    def __init__(self, map_path, nodes_path, signals_path, output="build/training-data", **settings):
        root = Path(__file__).resolve().parents[1]
        binary = root / "build" / ("traffic_training.exe" if __import__("os").name == "nt" else "traffic_training")
        command = [str(binary), "--interactive", "--map", str(Path(map_path).resolve()), "--nodes", str(Path(nodes_path).resolve()), "--signals", str(Path(signals_path).resolve()), "--output", str(Path(output).resolve())]
        for key, value in settings.items():
            command.extend(["--" + key.replace("_", "-"), str(value)])
        self.process = subprocess.Popen(command, cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.last = None

    def _call(self, message):
        if self.process.poll() is not None:
            raise RuntimeError("Simulator exited: " + self.process.stderr.read())
        self.process.stdin.write(message + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("Simulator exited: " + self.process.stderr.read())
        result = json.loads(line)
        if "error" in result:
            raise RuntimeError(result["error"])
        self.last = result
        return result

    def reset(self, seed=42):
        result = self._call(f"RESET {int(seed)}")
        return np.asarray(result["observation"], dtype=np.float32), result

    def step(self, action):
        if self.last is None:
            raise RuntimeError("Reset the environment before stepping")
        agent_actions = None
        if isinstance(action, (list, tuple, np.ndarray)):
            # For independent agents: 0=hold, 1=request my group's green.
            requests = np.asarray(action)
            if requests.shape != (2,) or not np.isin(requests, [0, 1]).all():
                raise ValueError("Expected two hold/request actions")
            agent_actions = [int(value) for value in requests]
            if requests.sum() == 0:
                action = 2
            elif requests.sum() == 1:
                action = int(requests.argmax())
            else:
                phases = np.asarray(self.last["observation"])[:, 4]
                green = np.flatnonzero(phases == 2)
                action = 1 - int(green[0]) if len(green) else 0
        if not isinstance(action, (int, np.integer)) or action not in (0, 1, 2):
            raise ValueError("Action must be 0, 1, or 2")
        suffix = "" if agent_actions is None else " " + " ".join(map(str, agent_actions))
        result = self._call(f"STEP {action}" + suffix)
        return (np.asarray(result["observation"], dtype=np.float32), np.asarray(result["rewards"], dtype=np.float32), bool(result["done"]), False, result)

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write("QUIT\n")
            self.process.stdin.flush()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
        for pipe in (self.process.stdin, self.process.stdout, self.process.stderr):
            pipe.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
