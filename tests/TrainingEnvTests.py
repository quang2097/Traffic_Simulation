import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from traffic_rl_env import TrafficLightEnv


class TrainingEnvTests(unittest.TestCase):
    def test_partial_reset_and_close_finish_episode_boundaries(self):
        fixture = ROOT / "tests/fixtures/directional-signals"
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "partial"
            with TrafficLightEnv(fixture / "custom-map.png", fixture / "custom.nodes", fixture / "custom-map.png.signals", output,
                                 duration=20, decision_interval=5, max_red=10, tr=2, yellow=1, min_green=1,
                                 min_rate=.02, max_rate=.1) as env:
                env.reset(seed=71)
                env.step(2)
                env.reset(seed=72)
            records = [json.loads(line) for line in (output / "transitions.jsonl").read_text().splitlines()]
            self.assertEqual(len(records), 3)
            self.assertFalse(records[0]["done"])
            self.assertTrue(records[1]["done"] and records[2]["done"])
            self.assertEqual(len({row["session_id"] for row in records}), 2)

    def test_append_across_process_restarts(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "dataset"
            _, first = self.run_episode(output)
            combined = output / "transitions.jsonl"
            prefix = combined.read_bytes()
            original = (first / "transitions.jsonl").read_bytes()
            _, second = self.run_episode(output)
            self.assertNotEqual(first, second)
            self.assertTrue(combined.read_bytes().startswith(prefix))
            self.assertEqual((first / "transitions.jsonl").read_bytes(), original)
            rows = [json.loads(line) for line in combined.read_text().splitlines()]
            self.assertEqual(len(rows), 8)
            self.assertEqual(len({row["session_id"] for row in rows}), 2)
            self.assertTrue(rows[3]["done"] and rows[7]["done"])

    def run_episode(self, output):
        fixture = ROOT / "tests/fixtures/directional-signals"
        with TrafficLightEnv(fixture / "custom-map.png", fixture / "custom.nodes", fixture / "custom-map.png.signals", output,
                             duration=20, decision_interval=5, camera_range=40, max_red=10, tr=2, yellow=1,
                             min_green=1, min_rate=.02, max_rate=.1, max_vehicles=20) as env:
            observation, info = env.reset(seed=71)
            self.assertEqual(observation.shape, (2, 14))
            observations = [observation.copy()]
            done = False
            while not done:
                observation, rewards, done, truncated, info = env.step([1, 1])
                self.assertEqual(rewards.shape, (2,))
                self.assertTrue(np.isfinite(observation).all())
                self.assertFalse(truncated)
                self.assertFalse((observation[:, 4] == 2).all())
                observations.append(observation.copy())
            directory = Path(info["directory"])
        return observations, directory

    def test_online_rollout_exports_reproducible_transitions(self):
        with tempfile.TemporaryDirectory() as temporary:
            first, directory = self.run_episode(Path(temporary) / "first")
            second, other = self.run_episode(Path(temporary) / "second")
            for a, b in zip(first, second):
                np.testing.assert_array_equal(a, b)
            records = [json.loads(line) for line in (directory / "transitions.jsonl").read_text().splitlines()]
            self.assertEqual(len(records), 4)
            self.assertTrue(records[-1]["done"])
            self.assertTrue(all(record["agent_actions"] == [1, 1] for record in records))
            self.assertTrue(all(len(record["observation"]) == 28 and len(record["rewards"]) == 2 for record in records))
            self.assertEqual((directory / "spawn_rates.csv").read_text(), (other / "spawn_rates.csv").read_text())
            metadata = json.loads((directory / "metadata.json").read_text())
            self.assertEqual(metadata["camera_range_m"], 40)
            self.assertEqual(metadata["Tr_s"], 2)
            with (directory / "groups.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(rows)
            self.assertTrue(all(float(row["visible_queue_m"]) <= 40 for row in rows))
            self.assertTrue((directory / "map.png").exists() and (directory / "network.nodes").exists())


if __name__ == "__main__":
    unittest.main()
