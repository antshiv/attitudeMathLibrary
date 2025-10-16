# C-AML Attitude Math Library written in C

## Overview

The **Attitude Math Library** is a C-based mathematical library designed for attitude control applications, providing robust support for quaternion operations, Euler angle conversions, direction cosine matrices (DCMs), and vector math. This library serves as a foundation for building attitude control systems in drones and robotics.

---

## Features

- **Quaternion Operations**:
  - Normalize, multiply, and invert quaternions.
  - Convert between quaternions, Euler angles, and DCMs.
  - Rotate vectors with both optimized and fully explicit formulations.
  - Convert quaternions to axis-angle form and interpolate with SLERP.
- **Euler Angles**:
  - Convert Euler angles to/from DCMs.
  - Convert Euler angles to/from quaternions.
- **Direction Cosine Matrices (DCM)**:
  - Verify orthonormality with `dcm_is_orthonormal`.
  - Apply transformations to vectors.
- **Vector Operations**:
  - Compute addition, subtraction, dot products, and cross products.
  - Normalize vectors.
  - Calculate vector magnitude.
- **Utility Functions**:
  - Convert degrees to radians and vice versa.
  - Wrap angles to a specified range.

---

## Purpose

The library is designed to:

1. Provide a reliable mathematical foundation for attitude control calculations.
2. Support developers and researchers in implementing and testing drone or robotic control systems.

---

## Getting Started

### Prerequisites

To use this library, you need:

- A C compiler (e.g., GCC or Clang)
- A Linux or Windows development environment

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/attitude-math-library.git
   cd attitude-math-library
   ```

2. Build the library:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. Link the library to your project. Include the header files and link the compiled library binary during the build process.

### File Structure

- `src/`: Contains source files for the library.
- `include/`: Header files for the library.
- `CMakeLists.txt`: Build configuration.
- `README.md`: Documentation (this file).

---

## Usage

### Library Functions

Recent updates expose additional helpers:

- `quaternion_inverse`, `quaternion_to_axis_angle`, `quaternion_rotate_vector`, and `quaternion_rotate_vector_explicit` are now part of the public API.
- `dcm_is_orthonormal` can be used to sanity-check direction cosine matrices before they enter control loops.
- Use `quaternion_set_explicit_debug(int enabled)` to toggle verbose tracing inside `quaternion_rotate_vector_explicit` when teaching or debugging the q⊗v⊗q* sequence.

#### Quaternions
- Convert quaternion to DCM:
  ```c
  double q[4] = {1.0, 0.0, 0.0, 0.0};
  double dcm[3][3];
  quaternion_to_dcm(q, dcm);
  ```
- Normalize a quaternion:
  ```c
  quaternion_normalize(q);
  ```

#### Euler Angles
- Convert Euler angles to DCM:
  ```c
  EulerAngles e = {roll, pitch, yaw, EULER_ZYX};
  euler_to_dcm(&e, dcm);
  ```

#### Vector Operations
- Compute cross product:
  ```c
  double a[3] = {1.0, 0.0, 0.0};
  double b[3] = {0.0, 1.0, 0.0};
  double out[3];
  vector3_cross(a, b, out);
  ```

---

## Building and Running Tests

All executables in `tests/` are picked up automatically by CMake.

```bash
cmake -S . -B build
cmake --build build
```

- Run the entire suite: `ctest --test-dir build --output-on-failure`
- List tests without executing: `ctest --test-dir build -N`
- Run a specific binary, e.g. `./build/test_quaternion_rotate`

### Educational demos

- `test_quaternion_rotate` compares the optimized and explicit rotation helpers to guard against regressions.
- `test_quaternion_rotate_explicit_demo` enables `quaternion_set_explicit_debug(1)` so you can watch each step in the explicit q⊗v⊗q* computation:
  ```bash
  ./build/test_quaternion_rotate_explicit_demo
  ```
  This is useful for walkthroughs or debugging orientation pipelines.

---

## SEO Keywords

- Drone attitude control
- Quaternion math library
- Direction cosine matrices
- Euler angles to quaternion conversion
- C-based attitude library

---

## Contributions

Contributions are welcome! Feel free to fork the repository, create a new branch, and submit a pull request. For major changes, open an issue to discuss your proposed modifications first.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## Contact

For questions, feedback, or collaboration, reach out to us at [anthony.shivakumar@antshiv.com].
