import importlib.util
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_submission(path):
    spec = importlib.util.spec_from_file_location("candidate", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_generated_submission_has_callable_control():
    path = ROOT / "submissions" / "no_other_cars" / "team_controller.py"
    subprocess.run(["python", "scripts/build_submission.py", "--mode", "no_other_cars"], cwd=ROOT, check=True)
    module = load_submission(path)
    assert callable(module.control)

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    steering, speed = module.control(image, image, 0.0)
    assert isinstance(float(steering), float)
    assert isinstance(float(speed), float)


def test_generated_submission_keeps_module_states_isolated():
    """The single-file build must not merge similarly named module state."""

    path = ROOT / "submissions" / "with_other_cars" / "team_controller.py"
    subprocess.run(
        ["python", "scripts/build_submission.py", "--mode", "with_other_cars"],
        cwd=ROOT,
        check=True,
    )
    module = load_submission(path)

    assert type(module._PERCEPTION_STATE).__name__ == "PerceptionState"
    assert type(module._ESTIMATOR_STATE).__name__ == "EstimatorState"
    assert type(module._POLICY_STATE).__name__ == "PolicyState"

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = (35, 120, 35)
    image[180:, 180:460, :] = (82, 74, 64)
    image[220:, 185:192, :] = 255
    image[220:, 448:455, :] = 255

    steering, speed = module.control(image, image, 0.0)

    assert -1.0 <= float(steering) <= 1.0
    assert 0.0 < float(speed) <= 1.0


def test_local_control_has_callable_interface():
    from controller.team_controller_local import control

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    steering, speed = control(image, image, 0.0)
    assert isinstance(float(steering), float)
    assert isinstance(float(speed), float)
