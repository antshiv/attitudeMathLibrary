# Quaternion Operations Library

This library provides efficient implementations for quaternion-based rotations and attitude operations. It includes both direct axis-angle rotations and quaternion-based methods, optimized for different use cases.

## Core Functions

### Quaternion Normalization

```c
void quaternion_normalize(double q[4])
```

Ensures quaternion maintains unit length (magnitude = 1). This is super important because:

- Unit quaternions guarantee valid rotations
- Prevent drift/scaling issues in consecutive operations
- Maintain numerical stability

The normalization check uses two thresholds:

```c
if (norm > 1e-6 && fabs(norm - 1.0) > 1e-6)
```

- First check (`norm > 1e-6`): Prevents division by zero
- Second check (`fabs(norm - 1.0) > 1e-6`): Only normalize if significantly different from 1.0
- Uses 1e-6 threshold to balance precision vs performance

### Rotation Methods

#### 1. Axis-Angle Rotation

```c
void axis_angle_rotate(const double axis[3], double angle,
                      const double v_in[3], double v_out[3])
```

Direct implementation of Rodrigues' rotation formula:

```
v' = v*cos(θ) + (k×v)*sin(θ) + k(k·v)(1-cos(θ))
```

where:

- k is rotation axis (must be unit vector)
- θ is rotation angle
- × is cross product
- · is dot product

Best used when:

- Working directly with angular velocities
- Need single vector rotation
- Want most efficient computation

#### 2. Optimized Quaternion Rotation

```c
void quaternion_rotate_vector(const double q[4],
                            const double v_in[3], double v_out[3])
```

Converts quaternion to rotation matrix internally for efficient vector rotation.

- Most efficient for multiple vectors
- Requires normalized quaternion input
- Uses optimized form instead of full q⊗v⊗q\*

#### 3. Explicit Quaternion Rotation

```c
void quaternion_rotate_vector_explicit(const double q[4],
                                     const double v_in[3], double v_out[3])
```

Full quaternion multiplication implementation (q⊗v⊗q\*). Useful for:

- Understanding quaternion math
- Debugging rotation issues
- Teaching/learning quaternion operations

## Performance Considerations

Choose your rotation method based on your needs:

- Single rotation → Use `axis_angle_rotate`
- Multiple rotations with same quaternion → Use `quaternion_rotate_vector`
- Learning/debugging → Use `quaternion_rotate_vector_explicit`

## Usage Example

```c
// Initialize a rotation quaternion (45° around Z-axis)
double q[4] = {0.9238795, 0, 0, 0.3826834};  // cos(θ/2), 0, 0, sin(θ/2)
quaternion_normalize(q);  // Always normalize!

// Vector to rotate
double v_in[3] = {1, 0, 0};
double v_out[3];

// Rotate using optimized method
quaternion_rotate_vector(q, v_in, v_out);
```

## Important Notes

1. Always normalize quaternions before use:

   - Prevents rotation scaling errors
   - Maintains proper rotation representation
   - Critical for numerical stability

2. Input vectors don't need normalization unless specifically required

3. Choose appropriate method based on:
   - Number of vectors to rotate
   - Whether you have quaternion or axis-angle
   - Debug/learning needs

## Debug Features

When compiled with DEBUG flag, includes additional checks:

```c
#ifdef DEBUG
if (fabs(norm - 1.0) > 1e-6) {
    printf("Quaternion is not normalized: norm = %f\n", norm);
}
#endif
```
