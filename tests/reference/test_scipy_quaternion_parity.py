#!/usr/bin/env python3
"""Compare the compiled attitude C ABI with SciPy's independent rotations."""

from __future__ import annotations

import argparse
import ctypes
from pathlib import Path
import sys
import warnings

try:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        import numpy as np
        from scipy.spatial.transform import Rotation, Slerp
except (ImportError, ValueError) as exc:
    print(f"SKIP: SciPy parity dependencies are unavailable: {exc}")
    raise SystemExit(77)


Double4 = ctypes.c_double * 4
Double3 = ctypes.c_double * 3
Double9 = ctypes.c_double * 9


def _as_double4(values: np.ndarray) -> Double4:
    return Double4(*(float(value) for value in values))


def _as_double3(values: np.ndarray) -> Double3:
    return Double3(*(float(value) for value in values))


def _array(values: ctypes.Array[ctypes.c_double]) -> np.ndarray:
    return np.ctypeslib.as_array(values).copy()


def _matrix_distance(actual: np.ndarray, expected: np.ndarray) -> float:
    return float(np.max(np.abs(actual - expected)))


class AttitudeABI:
    """Typed access to the public quaternion ABI."""

    def __init__(self, library: Path) -> None:
        self.lib = ctypes.CDLL(str(library))

        self.lib.quaternion_to_dcm.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.quaternion_to_dcm.restype = None

        self.lib.quaternion_multiply.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.quaternion_multiply.restype = None

        self.lib.quaternion_inverse.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.quaternion_inverse.restype = ctypes.c_int

        self.lib.quaternion_relative.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.quaternion_relative.restype = ctypes.c_int

        self.lib.quaternion_rotate_vector.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.quaternion_rotate_vector.restype = None

        self.lib.quaternion_slerp.argtypes = [
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.quaternion_slerp.restype = None

    def dcm(self, quaternion_wxyz: np.ndarray) -> np.ndarray:
        q = _as_double4(quaternion_wxyz)
        matrix = Double9()
        self.lib.quaternion_to_dcm(q, matrix)
        return _array(matrix).reshape(3, 3)

    def multiply(self, left_wxyz: np.ndarray, right_wxyz: np.ndarray) -> np.ndarray:
        left = _as_double4(left_wxyz)
        right = _as_double4(right_wxyz)
        output = Double4()
        self.lib.quaternion_multiply(left, right, output)
        return _array(output)

    def inverse(self, quaternion_wxyz: np.ndarray) -> np.ndarray:
        q = _as_double4(quaternion_wxyz)
        output = Double4()
        if self.lib.quaternion_inverse(q, output) != 1:
            raise AssertionError("C quaternion_inverse rejected a valid unit quaternion")
        return _array(output)

    def relative(self, current_wxyz: np.ndarray, target_wxyz: np.ndarray) -> np.ndarray:
        current = _as_double4(current_wxyz)
        target = _as_double4(target_wxyz)
        output = Double4()
        if self.lib.quaternion_relative(current, target, output) != 1:
            raise AssertionError("C quaternion_relative rejected valid unit quaternions")
        return _array(output)

    def rotate(self, quaternion_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
        q = _as_double4(quaternion_wxyz)
        source = _as_double3(vector)
        output = Double3()
        self.lib.quaternion_rotate_vector(q, source, output)
        return _array(output)

    def slerp(self, start_wxyz: np.ndarray, end_wxyz: np.ndarray, t: float) -> np.ndarray:
        start = _as_double4(start_wxyz)
        end = _as_double4(end_wxyz)
        output = Double4()
        self.lib.quaternion_slerp(start, end, t, output)
        return _array(output)


def _scipy_rotation(quaternion_wxyz: np.ndarray) -> Rotation:
    return Rotation.from_quat(quaternion_wxyz, scalar_first=True)


def _wxyz(rotation: Rotation) -> np.ndarray:
    return np.asarray(rotation.as_quat(scalar_first=True), dtype=np.float64)


def _assert_rotation_equal(
    label: str,
    actual_wxyz: np.ndarray,
    expected: Rotation,
    tolerance: float,
) -> float:
    error = _matrix_distance(_scipy_rotation(actual_wxyz).as_matrix(), expected.as_matrix())
    if error > tolerance:
        raise AssertionError(f"{label}: rotation matrix error {error:.3e} > {tolerance:.3e}")
    return error


def run(library: Path, samples: int, seed: int, tolerance: float) -> None:
    if not library.is_file():
        raise FileNotFoundError(f"shared attitude library not found: {library}")

    abi = AttitudeABI(library)
    rng = np.random.default_rng(seed)
    rotations = Rotation.random(samples * 2, random_state=rng)
    quaternions = rotations.as_quat(scalar_first=True)
    vectors = rng.normal(size=(samples, 3))
    interpolation_points = (0.0, 0.125, 0.5, 0.875, 1.0)

    maxima = {
        "dcm": 0.0,
        "rotate": 0.0,
        "multiply": 0.0,
        "inverse": 0.0,
        "relative": 0.0,
        "slerp": 0.0,
    }

    for index in range(samples):
        left_rotation = rotations[index]
        right_rotation = rotations[samples + index]
        left_q = quaternions[index]
        right_q = quaternions[samples + index]

        maxima["dcm"] = max(
            maxima["dcm"],
            _matrix_distance(abi.dcm(left_q), left_rotation.as_matrix()),
        )

        actual_vector = abi.rotate(left_q, vectors[index])
        expected_vector = left_rotation.apply(vectors[index])
        maxima["rotate"] = max(
            maxima["rotate"],
            float(np.max(np.abs(actual_vector - expected_vector))),
        )

        maxima["multiply"] = max(
            maxima["multiply"],
            _assert_rotation_equal(
                "Hamilton composition",
                abi.multiply(left_q, right_q),
                left_rotation * right_rotation,
                tolerance,
            ),
        )

        maxima["inverse"] = max(
            maxima["inverse"],
            _assert_rotation_equal(
                "inverse",
                abi.inverse(left_q),
                left_rotation.inv(),
                tolerance,
            ),
        )

        maxima["relative"] = max(
            maxima["relative"],
            _assert_rotation_equal(
                "current-to-target relative rotation",
                abi.relative(left_q, right_q),
                right_rotation * left_rotation.inv(),
                tolerance,
            ),
        )

        scipy_slerp = Slerp([0.0, 1.0], Rotation.concatenate([left_rotation, right_rotation]))
        for t in interpolation_points:
            maxima["slerp"] = max(
                maxima["slerp"],
                _assert_rotation_equal(
                    f"SLERP t={t}",
                    abi.slerp(left_q, right_q, t),
                    scipy_slerp([t])[0],
                    tolerance,
                ),
            )

    failed = {name: error for name, error in maxima.items() if error > tolerance}
    print(f"SciPy quaternion parity: samples={samples} seed={seed} tolerance={tolerance:.1e}")
    for name, error in maxima.items():
        print(f"  {name:10s} max_abs={error:.3e}")
    if failed:
        raise AssertionError(f"parity failures: {failed}")
    print("PASS: compiled C quaternion ABI matches SciPy Rotation")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--seed", type=int, default=20260712)
    parser.add_argument("--tolerance", type=float, default=2.0e-12)
    args = parser.parse_args()
    run(args.library.resolve(), args.samples, args.seed, args.tolerance)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
