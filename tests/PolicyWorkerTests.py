"""Real ONNX inference tests; use the project's RL Python environment."""
import json
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from rl_policy_worker import ALGORITHMS, Policy, identify, dueling_notebook_network


def make_policy(path, algorithm=None, shape=(1, 2), discrete=False, nonfinite=False):
    if discrete:
        output = np.array([[1, 4, 2], [9, 1, 2]], dtype=np.float32)
        nodes = [helper.make_node("Constant", [], ["actions"], value=numpy_helper.from_array(output))]
        out_shape = [2, 3]
    else:
        nodes = [helper.make_node("Mul", ["densities", "scale"], ["scaled"]), helper.make_node("Add", ["scaled", "base"], ["actions"])]
        out_shape = list(shape)
    graph = helper.make_graph(nodes, "test_policy", [helper.make_tensor_value_info("densities", TensorProto.FLOAT, list(shape))], [helper.make_tensor_value_info("actions", TensorProto.FLOAT, out_shape)],
                              [] if discrete else [numpy_helper.from_array(np.array(float("nan") if nonfinite else 50, dtype=np.float32), "scale"), numpy_helper.from_array(np.array(5, dtype=np.float32), "base")])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=9)
    if algorithm:
        helper.set_model_props(model, {"traffic.algorithm": algorithm})
    onnx.save(model, path)


class WorkerTests(unittest.TestCase):
    def dueling_model(self, masked=False):
        import torch
        filename = "masked_network.py" if masked else "network.py"
        spec = importlib.util.spec_from_file_location("train_dueling_fixture", ROOT / "tests/fixtures/train-dueling" / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        torch.manual_seed(42)
        return module.TrafficQNetwork(obs_mean=np.arange(28, dtype=np.float32),
                                     obs_std=np.arange(1, 29, dtype=np.float32)).eval()

    def test_masked_train_notebook_exports_and_interlocks(self):
        import torch
        model = self.dueling_model(masked=True)
        self.assertTrue(dueling_notebook_network(model.state_dict()))
        with torch.no_grad():
            model.adv_head[2].weight.zero_()
            model.adv_head[2].bias.copy_(torch.tensor([0., 10., 5.]))
        path = self.path.with_suffix(".pt")
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        torch.onnx.export(model, torch.zeros(1, 28), str(self.path), input_names=["observation"],
                          output_names=["q_values"], dynamic_axes={"observation": {0: "batch_size"}, "q_values": {0: "batch_size"}},
                          opset_version=17, dynamo=False)
        observations = np.zeros((8, 28), dtype=np.float32)
        observations[:, 4] = 2  # Group 0 green; prefer switching to group 1.
        observations[:, 5] = 6
        observations[:, 7] = observations[:, 21] = -1
        observations[1, 4] = 1  # Group 0 yellow.
        observations[2, 18] = 1  # Group 1 yellow.
        observations[3, 7] = 20  # Group 0 Tr.
        observations[4, 21] = 20  # Group 1 Tr.
        observations[5, 5] = 4.99  # Minimum green has not elapsed.
        observations[6, 5] = 5  # Exact minimum-green boundary.
        observations[7, 7] = 0  # Finished Tr no longer locks actions.
        expected_actions = [1, 2, 2, 2, 2, 2, 1, 1]
        with torch.inference_mode():
            expected_q = model(torch.from_numpy(observations)).numpy()
        self.assertEqual(expected_q.argmax(axis=1).tolist(), expected_actions)
        for exported, family in ((path, "DuelingDQN"), (self.path, "DQN")):
            policy = Policy(exported)
            self.assertEqual(policy.algorithm, family)
            self.assertTrue(policy.training)
            self.assertEqual(Policy(exported, "DuelingDQN").algorithm, "DuelingDQN")
            for obs, action in zip(observations, expected_actions):
                self.assertEqual(policy.predict(obs.tolist(), [1, 1]), [int(action == 0), int(action == 1)])
            if exported.suffix == ".onnx":
                actual = policy.model.run(None, {"observation": observations})[0]
            else:
                with torch.inference_mode():
                    actual = policy.model(torch.from_numpy(observations)).numpy()
            np.testing.assert_allclose(actual, expected_q, rtol=1e-4, atol=1e-5)
        # Test the opposite minimum-green direction too.
        with torch.no_grad():
            model.adv_head[2].bias.copy_(torch.tensor([10., 0., 5.]))
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        obs = observations[0].copy();obs[4] = 0;obs[18] = 2;obs[19] = 4.99
        self.assertEqual(Policy(path).predict(obs.tolist(), [1, 1]), [0, 0])
        obs[19] = 5
        self.assertEqual(Policy(path).predict(obs.tolist(), [1, 1]), [1, 0])

    def test_masked_policy_native_transport_and_lights(self):
        import torch
        model = self.dueling_model(masked=True)
        with torch.no_grad():
            model.adv_head[2].weight.zero_()
            model.adv_head[2].bias.copy_(torch.tensor([0., 10., 5.]))
        path = self.path.with_suffix(".pt")
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        runner = ROOT / "build/policy_runtime_tests.exe"
        if runner.exists():
            result = subprocess.run([str(runner), sys.executable, str(ROOT / "scripts/rl_policy_worker.py"),
                                     str(path), "v2", "DuelingDQN", "0", "1"], capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        runner = ROOT / "build/sumo_ui_smoke.exe"
        if runner.exists():
            result = subprocess.run([str(runner), "policy-masked", sys.executable, str(path)], capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        with torch.no_grad():
            model.min_green_t.fill_(-1)
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        with self.assertRaisesRegex(ValueError, "minimum-green"):
            Policy(path)

    def test_train_notebook_dueling_exports(self):
        import torch
        model = self.dueling_model()
        self.assertTrue(dueling_notebook_network(model.state_dict()))
        path = self.path.with_suffix(".pt")
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        torch.onnx.export(model, torch.zeros(1, 28), str(self.path), input_names=["observation"],
                          output_names=["q_values"], dynamic_axes={"observation": {0: "batch_size"}, "q_values": {0: "batch_size"}},
                          opset_version=17, dynamo=False)
        # This notebook companion is informational; normalization is in the graph.
        (self.path.parent / "model_meta.json").write_text(json.dumps({"obs_dim": 28, "act_dim": 3,
            "obs_mean": model.obs_mean.tolist(), "obs_std": model.obs_std.tolist()}))
        observations = np.asarray([np.linspace(0, 80, 28), np.zeros(28), np.linspace(-1, 3, 28)], dtype=np.float32)
        with torch.inference_mode():
            expected = model(torch.from_numpy(observations)).numpy()
        for exported, family in ((path, "DuelingDQN"), (self.path, "DQN")):
            policy = Policy(exported)
            self.assertEqual(policy.algorithm, family)
            self.assertTrue(policy.training)
            self.assertEqual(Policy(exported, "DuelingDQN").algorithm, "DuelingDQN")
            for observation, q in zip(observations, expected):
                action = int(q.argmax())
                self.assertEqual(policy.predict(observation.tolist(), [1, 1]), [int(action == 0), int(action == 1)])
            if exported.suffix == ".onnx":
                actual = policy.model.run(None, {"observation": observations})[0]
            else:
                with torch.inference_mode():
                    actual = policy.model(torch.from_numpy(observations)).numpy()
            np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=1e-5)
            with self.assertRaisesRegex(ValueError, "conflicts"):
                Policy(exported, "IPPO")
            native = [0.0] * 28
            for index in (4, 7, 18, 21):
                native[index] = -1.0
            runner = ROOT / "build/policy_runtime_tests.exe"
            if runner.exists():
                requests = policy.predict(native, [1, 1])
                result = subprocess.run([str(runner), sys.executable, str(ROOT / "scripts/rl_policy_worker.py"),
                                         str(exported), "v2", family, *map(str, requests)], capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_dueling_all_three_actions_and_ui(self):
        import torch
        model = self.dueling_model()
        path = self.path.with_suffix(".pt")
        for action, expected in ((0, [1, 0]), (2, [0, 0]), (1, [0, 1])):
            with torch.no_grad():
                model.adv_head[2].weight.zero_()
                model.adv_head[2].bias.fill_(-5)
                model.adv_head[2].bias[action] = 5
            torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
            policy = Policy(path)
            self.assertEqual(policy.predict([0.0] * 28, [1, 1]), expected)
        message = "OBS2 " + " ".join(["0"] * 28 + ["1", "1"]) + "\nQUIT\n"
        result = subprocess.run([sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(path)],
                                input=message, text=True, capture_output=True, timeout=30)
        self.assertEqual(result.stdout.splitlines(), ["READY DuelingDQN V2 HOLD", "REQUEST 0 1"])
        runner = ROOT / "build/sumo_ui_smoke.exe"
        if runner.exists():
            result = subprocess.run([str(runner), "policy", sys.executable, str(path)], capture_output=True, text=True, timeout=35)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_dueling_normalization_and_contract_conflicts(self):
        import torch
        model = self.dueling_model()
        path = self.path.with_suffix(".pt")
        with torch.no_grad():
            model.obs_std[0] = 0
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        with self.assertRaisesRegex(ValueError, "normalization"):
            Policy(path)
        with torch.no_grad():
            model.obs_std.fill_(1)
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        Path(str(path) + ".json").write_text(json.dumps({"observation_schema": "density_v1"}))
        with self.assertRaisesRegex(ValueError, "conflicts"):
            Policy(path)
        malformed = dict(model.state_dict())
        malformed["adv_head.2.weight"] = torch.zeros(4, 128)
        self.assertFalse(dueling_notebook_network(malformed))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "policy.onnx"

    def tearDown(self):
        self.temp.cleanup()

    def test_all_families_auto_detect_and_run(self):
        for algorithm in ALGORITHMS:
            make_policy(self.path, algorithm)
            policy = Policy(self.path)
            self.assertEqual(policy.algorithm, algorithm)
            self.assertEqual(policy.predict([.2, .8], [1, 1]), [15, 45])

    def test_dqn_detection_is_distinct_from_idqn(self):
        self.assertEqual(identify(Path("dqn_policy.onnx"), {}), "DQN")
        self.assertEqual(identify(Path("idqn_policy.onnx"), {}), "IDQN")
        make_policy(self.path)
        self.assertEqual(Policy(self.path, "DQN").algorithm, "DQN")

    def test_dqn_joint_hold_q_values(self):
        Path(str(self.path) + ".json").write_text(json.dumps({"algorithm": "DQN", "observation_schema": "traffic_v2", "action_mode": "joint_hold_scores"}))
        for scores, expected in [([9, 1, 0], [1, 0]), ([0, 9, 1], [0, 1]), ([0, 1, 9], [0, 0])]:
            graph = helper.make_graph([helper.make_node("Constant", [], ["q_values"], value=numpy_helper.from_array(np.asarray([scores], dtype=np.float32)))], "dqn",
                                      [helper.make_tensor_value_info("observations", TensorProto.FLOAT, [1, 28])],
                                      [helper.make_tensor_value_info("q_values", TensorProto.FLOAT, [1, 3])])
            onnx.save(helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=9), self.path)
            policy = Policy(self.path)
            self.assertEqual(policy.algorithm, "DQN")
            self.assertEqual(policy.predict([0.0] * 28, [1, 1]), expected)

    def test_notebook_exports_without_sidecar(self):
        import torch
        class TrafficQNetwork(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.register_buffer("obs_mean", torch.arange(28, dtype=torch.float32))
                self.register_buffer("obs_std", torch.arange(1, 29, dtype=torch.float32))
                self.net = torch.nn.Sequential(
                    torch.nn.Linear(28, 128), torch.nn.LayerNorm(128), torch.nn.SiLU(),
                    torch.nn.Linear(128, 128), torch.nn.LayerNorm(128), torch.nn.SiLU(),
                    torch.nn.Linear(128, 3))

            def forward(self, x):
                return self.net((x - self.obs_mean) / self.obs_std)

        torch.manual_seed(42)
        model = TrafficQNetwork().eval()
        observation = np.linspace(0, 80, 28, dtype=np.float32)
        path = self.path.with_suffix(".pt")
        torch.jit.trace(model, torch.zeros(1, 28)).save(str(path))
        torch.onnx.export(model, torch.zeros(1, 28), str(self.path),
                          input_names=["observation"], output_names=["q_values"],
                          opset_version=17, dynamo=False)
        with torch.inference_mode():
            expected_q = model(torch.from_numpy(observation.reshape(1, 28))).numpy()
        for exported in (path, self.path):
            policy = Policy(exported)
            self.assertEqual(policy.algorithm, "DQN")
            self.assertTrue(policy.training)
            action = int(expected_q.argmax())
            self.assertEqual(policy.predict(observation.tolist(), [1, 1]), [int(action == 0), int(action == 1)])
            if exported.suffix == ".onnx":
                actual_q = policy.model.run(None, {"observation": observation.reshape(1, 28)})[0]
            else:
                with torch.inference_mode():
                    actual_q = policy.model(torch.from_numpy(observation.reshape(1, 28))).numpy()
            np.testing.assert_allclose(actual_q, expected_q, rtol=1e-4, atol=1e-5)
            with self.assertRaisesRegex(ValueError, "conflicts"):
                Policy(exported, "IPPO")
            message = "OBS2 " + " ".join(map(str, observation.tolist() + [1, 1])) + "\nQUIT\n"
            result = subprocess.run([sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(exported)],
                                    input=message, text=True, capture_output=True, timeout=20)
            self.assertEqual(result.stdout.splitlines(), ["READY DQN V2 HOLD", "REQUEST " + " ".join(map(str, policy.predict(observation.tolist(), [1, 1])))])
            runner = ROOT / "build/policy_runtime_tests.exe"
            if runner.exists():
                # Default snapshot has unknown phases and inactive Tr (-1).
                native_observation = [0.0] * 28
                for index in (4, 7, 18, 21):
                    native_observation[index] = -1.0
                expected = policy.predict(native_observation, [1, 1])
                result = subprocess.run([str(runner), sys.executable, str(ROOT / "scripts/rl_policy_worker.py"),
                                         str(exported), "v2", "DQN", *map(str, expected)],
                                        capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_unknown_requires_selection_then_loads(self):
        make_policy(self.path)
        with self.assertRaises(LookupError):
            Policy(self.path)
        self.assertEqual(Policy(self.path, "IPPO").predict([0, 1], [1, 1]), [5, 55])

    def test_explicit_metadata_conflict_is_rejected(self):
        make_policy(self.path, "QMIX")
        with self.assertRaisesRegex(ValueError, "conflicts"):
            Policy(self.path, "MAPPO")

    def test_wrong_input_shape_rejected_before_ready(self):
        make_policy(self.path, "IPPO", shape=(1, 4))
        with self.assertRaisesRegex(ValueError, "expects float32"):
            Policy(self.path)

    def test_nonfinite_actions_rejected(self):
        make_policy(self.path, "IPPO", nonfinite=True)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            Policy(self.path)

    def test_q_values_decode_using_the_training_action_table(self):
        make_policy(self.path, "QMIX", discrete=True)
        Path(str(self.path) + ".json").write_text(json.dumps({"action_mode": "discrete_scores", "green_options": [7, 17, 37]}))
        self.assertEqual(Policy(self.path).predict([.1, .9], [1, 1]), [17, 7])

    def test_per_group_layout(self):
        make_policy(self.path, "IDQN", shape=(2, 1))
        Path(str(self.path) + ".json").write_text(json.dumps({"layout": "per_group"}))
        self.assertEqual(Policy(self.path).predict([.2, .8], [1, 1]), [15, 45])

    def test_raw_state_dict_gives_export_guidance(self):
        raw = self.path.with_suffix(".pth")
        raw.write_bytes(b"not a scripted network")
        with self.assertRaisesRegex(ValueError, "original architecture"):
            Policy(raw, "MAPPO")

    def test_real_torchscript_export(self):
        try:
            import torch
        except ImportError:
            self.skipTest("Install torch for the optional TorchScript check")
        class Actor(torch.nn.Module):
            def forward(self, densities):
                return 5 + densities * 50
        path = self.path.with_suffix(".pt")
        torch.jit.trace(Actor().eval(), torch.zeros(1, 2)).save(str(path))
        Path(str(path) + ".json").write_text(json.dumps({"algorithm": "IPPO"}))
        self.assertEqual(Policy(path).predict([.2, .8], [1, 1]), [15, 45])
        raw = self.path.with_suffix(".pth")
        torch.save(torch.nn.Linear(2, 2).state_dict(), raw)
        with self.assertRaisesRegex(ValueError, "state_dict"):
            Policy(raw, "IPPO")

    def test_graph_mask_and_adjacency_inputs(self):
        graph = helper.make_graph([
            helper.make_node("Mul", ["densities", "active"], ["masked"]),
            helper.make_node("ReduceSum", ["graph"], ["summed"], keepdims=0),
            helper.make_node("Mul", ["summed", "zero"], ["graph_zero"]),
            helper.make_node("Mul", ["masked", "scale"], ["scaled"]),
            helper.make_node("Add", ["scaled", "base"], ["greens"]),
            helper.make_node("Add", ["greens", "graph_zero"], ["actions"]),
        ], "graph_policy", [helper.make_tensor_value_info("densities", TensorProto.FLOAT, [1, 2]), helper.make_tensor_value_info("active", TensorProto.FLOAT, [1, 2]), helper.make_tensor_value_info("graph", TensorProto.FLOAT, [2, 2])],
            [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 2])],
            [numpy_helper.from_array(np.array(value, dtype=np.float32), name) for name, value in [("zero", 0), ("scale", 50), ("base", 5)]])
        model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=9)
        onnx.save(model, self.path)
        Path(str(self.path) + ".json").write_text(json.dumps({"algorithm": "CoLight", "mask_input": "active", "adjacency_input": "graph"}))
        self.assertEqual(Policy(self.path).predict([.2, .8], [1, 1]), [15, 45])

    def test_queue_wait_hold_contract(self):
        scores = np.array([[3, 1], [0, 4]], dtype=np.float32)
        graph = helper.make_graph([helper.make_node("Constant", [], ["requests"], value=numpy_helper.from_array(scores))], "hold_policy",
                                  [helper.make_tensor_value_info("observations", TensorProto.FLOAT, [1, 28])],
                                  [helper.make_tensor_value_info("requests", TensorProto.FLOAT, [2, 2])])
        model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=9)
        helper.set_model_props(model, {"traffic.algorithm": "MAPPO"})
        onnx.save(model, self.path)
        Path(str(self.path) + ".json").write_text(json.dumps({"observation_schema": "traffic_v2", "action_mode": "hold_scores"}))
        policy = Policy(self.path)
        self.assertEqual(policy.predict([0.0] * 28, [1, 1]), [0, 1])
        message = "OBS2 " + " ".join(["0"] * 28 + ["1", "1"]) + "\nQUIT\n"
        result = subprocess.run([sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(self.path)], input=message, text=True, capture_output=True, timeout=15)
        self.assertEqual(result.stdout.splitlines(), ["READY MAPPO V2 HOLD", "REQUEST 0 1"])
        runner = ROOT / "build/policy_runtime_tests.exe"
        if runner.exists():
            result = subprocess.run([str(runner), sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(self.path), "v2"], capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_actual_worker_protocol(self):
        make_policy(self.path, "CoLight")
        result = subprocess.run([sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(self.path)], input="OBS 0.2 0.8 1 1\nQUIT\n", text=True, capture_output=True, timeout=15)
        self.assertEqual(result.stdout.splitlines(), ["READY CoLight", "ACTION 15.0 45.0"])

    def test_bad_observations_terminate_with_error(self):
        make_policy(self.path, "MA2C")
        result = subprocess.run([sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(self.path)], input="OBS nan 0 1 1\n", text=True, capture_output=True, timeout=15)
        self.assertTrue(result.stdout.splitlines()[1].startswith("ERROR "))

    def test_native_transport(self):
        runner = ROOT / "build/policy_runtime_tests.exe"
        if not runner.exists():
            self.skipTest("Build policy_runtime_tests for the native transport check")
        make_policy(self.path, "MAPPO")
        result = subprocess.run([str(runner), sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(self.path)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_full_ui_to_worker_to_synchronized_lights(self):
        runner = ROOT / "build/sumo_ui_smoke.exe"
        if not runner.exists():
            self.skipTest("Build sumo_ui_smoke for the policy UI integration check")
        make_policy(self.path, "MAPPO")
        result = subprocess.run([str(runner), "policy", sys.executable, str(self.path)], capture_output=True, text=True, timeout=25)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_unknown_worker_requests_model_selection(self):
        make_policy(self.path)
        result = subprocess.run([sys.executable, str(ROOT / "scripts/rl_policy_worker.py"), str(self.path)], text=True, capture_output=True, timeout=15)
        self.assertTrue(result.stdout.startswith("SELECT "), result.stdout)

    def test_unknown_model_opens_ui_selection_dialog(self):
        runner = ROOT / "build/sumo_ui_smoke.exe"
        if not runner.exists():
            self.skipTest("Build sumo_ui_smoke for the selection dialog check")
        make_policy(self.path)
        result = subprocess.run([str(runner), "policy-select", sys.executable, str(self.path)], capture_output=True, text=True, timeout=25)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)


if __name__ == "__main__":
    unittest.main()
