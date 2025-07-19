# MuJoCo Plugin: Advanced Anisotropic Ball Joint

This repository contains a C-based engine plugin for MuJoCo that implements an advanced, anisotropic ball joint. It is designed for high-fidelity simulations of complex flexible bodies, such as medical catheters, robotic arms, or other objects where bending and twisting behaviors are not uniform.

## 1. Overview

This plugin extends MuJoCo's physics capabilities by providing a custom joint type that can be attached to any `ball` joint in an MJCF model. It replaces the joint's default physics with a custom force model that includes several advanced features, allowing for more realistic and nuanced simulations.

## 2. Features

* **Anisotropic Bending**: The joint can have different stiffness and damping for bending around its local X and Y axes.
* **Symmetric Bending by Default**: If only general bending parameters are provided, they are applied to both X and Y axes for simple, symmetric behavior.
* **Independent Torsion**: The joint has separate parameters for twisting (torsion) around its local Z-axis.
* **Physically-Accurate Stiffness**: The stiffness model uses the joint's true rotation vector, making it physically correct even for large-angle rotations.
* **Non-Linear Stiffness Hardening**: An optional cubic stiffness term (`k_bend_cubic`) can be added to make the joint significantly stiffer as it bends further from its resting state.
* **Numerically-Stable Coulomb Friction**: An optional dry friction term (`d_coulomb`) is implemented using a smoothed `tanh` function to model material hysteresis without causing solver instability or chatter.
* **High Performance**: The C code is optimized by caching joint IDs on initialization and using fast math approximations, making it suitable for simulations with many joints.

## 3. Prerequisites

To use this plugin, you will need:

* **Python**: With the `mujoco` package installed (`pip install mujoco`).

## 4. Usage in MJCF Models

To use the plugin, you must first declare it in the `<extension>` section and then attach it as a `<plugin>` actuator targeting a specific `ball` joint.

### 5.1. Configuration Attributes

The plugin is configured via key-value pairs. All parameters are optional and default to `0`.

| Attribute          | Description                                                                                             | Units    |
| :----------------- | :------------------------------------------------------------------------------------------------------ | :------- |
| `k_bend`           | **Base** linear stiffness for bending. Used for both X and Y axes if `k_bend_x`/`k_bend_y` are not set.    | Nm/rad   |
| `d_bend`           | **Base** viscous damping for bending. Used for both X and Y axes if `d_bend_x`/`d_bend_y` are not set.    | Nms/rad  |
| `k_bend_x`         | **Specific** linear stiffness for bending around the local X-axis. Overrides `k_bend`.                    | Nm/rad   |
| `d_bend_x`         | **Specific** viscous damping for bending around the local X-axis. Overrides `d_bend`.                     | Nms/rad  |
| `k_bend_y`         | **Specific** linear stiffness for bending around the local Y-axis. Overrides `k_bend`.                    | Nm/rad   |
| `d_bend_y`         | **Specific** viscous damping for bending around the local Y-axis. Overrides `d_bend`.                     | Nms/rad  |
| `k_tor`            | Linear stiffness for torsion (twisting) around the local Z-axis.                                        | Nm/rad   |
| `d_tor`            | Viscous damping for torsion around the local Z-axis.                                                    | Nms/rad  |
| `k_bend_cubic`     | **Optional** non-linear hardening factor. Adds a force proportional to `angle³`.                          | Nm/rad³  |
| `d_coulomb`        | **Optional** constant dry friction force that opposes motion at any velocity.                             | Nm       |
| `friction_vel_tol` | **Optional** velocity tolerance for smoothing the Coulomb friction. Default: `1e-4`.                      | m/s      |

### 5.2. Example

```xml
<mujoco model="example">
  <extension>
    <!-- 1. Declare the plugin -->
    <plugin plugin="user.joint.anisotropic.advanced"/>
  </extension>

  <worldbody>
    ...
    <body name="my_body" ...>
      <joint name="my_joint" type="ball"/>
    </body>
    ...
  </worldbody>

  <actuator>
    <!-- 2. Attach the plugin to the joint as an actuator -->
    <plugin joint="my_joint" plugin="user.joint.anisotropic.advanced">
      <!-- 3. Configure the parameters -->
      <config key="k_bend_x" value="10.0"/>
      <config key="d_bend_x" value="0.5"/>
      <config key="k_bend_y" value="25.0"/> 
      <config key="d_bend_y" value="0.8"/>
      <config key="k_tor" value="5.0"/>
      <config key="d_tor" value="0.2"/>
    </plugin>
  </actuator>
</mujoco>
```

## 6. Python Integration

### 6.1. Loading the Plugin

Before loading any model that uses this plugin, you must first load the compiled C library.

```python
import mujoco
import os

# Path to the compiled library
plugin_name = 'anisotropic_joint.dll' if os.name == 'nt' else 'libanisotropic_joint.so'
plugin_path = os.path.join(os.path.dirname(__file__), plugin_name)

# Load the plugin
try:
    mujoco.mj_loadPluginLibrary(plugin_path)
    print("Custom C plugin loaded successfully.")
except Exception as e:
    print(f"Error loading plugin: {e}")

# Now it is safe to load the model
# model = mujoco.MjModel.from_xml_path(...)
```

### 6.2. MjSpec Integration

I am currently unable to get it to run using MjSpec, it appears it may be a low level bug within MjSpec itself as of MuJoCo 3.3.4
