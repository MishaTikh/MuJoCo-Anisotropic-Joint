# CMakeLists.txt
#
# This script builds the C plugin into a shared library that MuJoCo can load.

# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(MuJoCoAnisotropicJoint)

# Find the MuJoCo package. This requires that the user has installed the
# MuJoCo SDK and that CMake can find it. The user must provide the path
# to the installation via the -DMUJOCO_PATH flag.
find_package(mujoco REQUIRED)

# Define the shared library target
# This will create `anisotropic_joint.dll` on Windows or `libanisotropic_joint.so` on Linux/macOS.
add_library(anisotropic_joint SHARED anisotropic_joint.c)

# Explicitly add the MuJoCo include directory to the target's include paths.
# This ensures the compiler can find <mujoco/mujoco.h>.
target_include_directories(anisotropic_joint PRIVATE "${MUJOCO_PATH}/include")

# Link our plugin against the MuJoCo library. This is necessary because our
# C code calls MuJoCo functions (e.g., mju_rotVecQuat).
target_link_libraries(anisotropic_joint PRIVATE mujoco::mujoco)