# MuJoCo Plugin: Advanced Anisotropic Ball Joint (Cosserat Rod Model)

This repository contains a high-performance C-based engine plugin for MuJoCo that implements an advanced, anisotropic ball joint. It is designed for high-fidelity simulations of complex flexible bodies, such as medical catheters or soft robots, where physically-accurate bending and twisting behaviors are required.

## 1. Overview

This plugin extends MuJoCo's physics capabilities by providing a custom force model for any `ball` joint in an MJCF model. It replaces the joint's default stiffness with a custom model based on the **constitutive laws of Cosserat rod theory**. This allows for a more physically-grounded simulation where stiffness is defined by material properties (like `EI` and `GJ`) rather than abstract joint spring coefficients.

A key feature of this model is the ability to define an **intrinsic curvature**, allowing parts of the model to have a naturally curved or twisted resting shape.

The plugin is implemented as a self-contained C library that is loaded by MuJoCo at runtime, ensuring native performance and stability.

## 2. Features

* **Cosserat Rod-Based Stiffness**: The torque at the joint is calculated using the physically-based formula `τ = (B/L) * (κ - κ⁰)`, where `B` is the material stiffness matrix, `L` is the segment length, `κ` is the current curvature, and `κ⁰` is the intrinsic curvature.
* **Material-Based Parameters**: Instead of defining spring constants in `Nm/rad`, you define the actual engineering properties of your flexible rod:
    * Bending Stiffness (`EI`) in `Nm²`
    * Torsional Stiffness (`GJ`) in `Nm²`
* **Intrinsic Curvature**: Model objects that are naturally curved or twisted by specifying a non-zero resting curvature (`κ⁰`). This is essential for modeling pre-bent catheters or complex biological structures.
* **Decoupled Damping**: Damping is **not** handled by the plugin. This allows you to use MuJoCo's highly stable native joint damping on the `ball` joint itself, providing a clean separation between stiffness and damping forces.
* **High Performance**: The C code is optimized to cache joint information on initialization, ensuring minimal overhead during the simulation loop.

## 3. Prerequisites

To compile and use this plugin, you will need:

* **MuJoCo SDK**: Version 3.1.4 or newer.
* **C/C++ Compiler**:
    * On Windows: Microsoft Visual C++ (MSVC), available with Visual Studio or VS Build Tools.
    * On Linux/macOS: GCC or Clang.
* **Python**: With the `mujoco` package installed (`pip install mujoco`).

## 4. Compilation

The plugin must be compiled into a shared library (`.dll` on Windows, `.so` on Linux).

### On Windows (Direct Command)

1.  Open a **Developer Command Prompt for VS**.
2.  Navigate to the directory containing `anisotropic_joint.c`.
3.  Run the following command, replacing the path with the correct location of your MuJoCo SDK:
    ```shell
    cl.exe /LD anisotropic_joint.c /I"C:\path\to\your\mujoco-3.3.4-windows-x86_64\include" /link /LIBPATH:"C:\path\to\your\mujoco-3.3.4-windows-x86_64\lib" mujoco.lib
    ```

## 5. Usage in MJCF Models

To use the plugin, you must declare it in the `<extension>` section and then attach it as a `<plugin>` actuator targeting a specific `ball` joint.

### 5.1. Configuration Attributes

The plugin is configured via key-value pairs. All parameters are optional and default to `0`.

| Attribute        | Description                                                                                             | Units   |
| :--------------- | :------------------------------------------------------------------------------------------------------ | :------ |
| `B11`, `B22`     | Bending stiffness of the material about the local x and y axes, respectively. This is the **EI** value.   | Nm²     |
| `B33`            | Torsional stiffness of the material about the local z-axis. This is the **GJ** value.                   | Nm²     |
| `k0_x`, `k0_y`, `k0_z` | Components of the intrinsic (resting) curvature vector **κ⁰**. Defines the stress-free shape.         | rad     |
| `segment_length` | The length of the discrete body segment (**L_seg**) that this joint connects. **This is critical** for correct physics. | m       |

### 5.2. Example

```xml
<mujoco model="example">
  <extension>
    <plugin plugin="user.joint.anisotropic.advanced"/>
  </extension>

  <worldbody>
    ...
    <body name="segment_1" pos="0 0 0">
      <joint name="J_Seg1" type="ball" damping="5e-5"/> <geom type="capsule" size="0.01 0.05"/>
      <body name="segment_2" pos="0 0 0.1">
        ...
      </body>
    </body>
    ...
  </worldbody>

  <actuator>
    <plugin joint="J_Seg1" plugin="user.joint.anisotropic.advanced">
      <config key="B11" value="0.001"/>            <config key="B22" value="0.001"/>            <config key="B33" value="0.0001"/>           <config key="k0_x" value="0.0"/>             <config key="k0_y" value="0.0"/>
      <config key="k0_z" value="0.0"/>
      <config key="segment_length" value="0.1"/>  </plugin>
  </actuator>
</mujoco>

## 6. Python Integration
Integrating the plugin into a Python application involves two steps: loading the compiled library and generating the correct XML.

### 6.1. Loading the Plugin Library
Before you can load a model that uses the plugin, you must first load the compiled C library into memory. This should be done once at the start of your application.

Python

import mujoco
import os

# Path to the compiled library
plugin_name = 'anisotropic_joint.dll' if os.name == 'nt' else 'libanisotropic_joint.so'

# Assume the DLL is in a directory named 'anisotropic_joint' next to your script
plugin_path = os.path.join(os.getcwd(), 'anisotropic_joint', plugin_name)

# Load the plugin library
try:
    mujoco.mj_loadPluginLibrary(plugin_path)
    print(f"Custom C plugin loaded successfully from '{plugin_path}'.")
except Exception as e:
    print(f"FATAL: Could not load the custom anisotropic joint plugin.\nError: {e}")
    # It is recommended to exit if the plugin cannot be loaded
    # as any model using it will fail to compile.

# Now it is safe to load any model that uses 'user.joint.anisotropic.advanced'
# model = mujoco.MjModel.from_xml_string(...)

### 6.2. Generating the Model
Your Python code that generates the MJCF model must be updated to:

Set a damping value on the <joint> tag itself. This should be calculated based on the critical damping for the joint.

Add a <plugin> actuator for each joint.

Populate the <config> keys with the correct physical parameters (B11, B33, segment_length, etc.) for each specific joint, as shown in the example above.
