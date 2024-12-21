# C-AML Attitude Math Library written in C

## Overview

The **Attitude Math Library** is a C-based mathematical library designed for attitude control applications, providing robust support for quaternion operations, Euler angle conversions, direction cosine matrices (DCMs), and vector math. This library serves as a foundation for building attitude control systems in drones and robotics.

---

## Features

- **Quaternion Operations**:
  - Normalize and multiply quaternions.
  - Convert between quaternions and Euler angles.
  - Convert between quaternions and DCMs.
- **Euler Angles**:
  - Convert Euler angles to/from DCMs.
  - Convert Euler angles to/from quaternions.
- **Direction Cosine Matrices (DCM)**:
  - Verify orthonormality.
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


