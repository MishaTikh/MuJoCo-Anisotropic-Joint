# test_plugin.py
#
# This script performs a headless or visual test of the ADVANCED anisotropic joint C plugin.
# It now generates a longer, more damped pendulum to better test stability.

import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import argparse

# =================================================================================================
#  1. Define the MJCF Model as a String
# =================================================================================================
def generate_anisotropic_test_xml(num_bodies=10):
    """
    Generates an MJCF model for a multi-link pendulum to showcase anisotropic behavior.
    Each joint is much stiffer in the Y direction than the X direction.
    """
    
    # Start with the first joint rotated to initiate the swing
    qpos_key_list = ["0.7071 0 0.7071 0"]  # 90-degree rotation around Y-axis
    for _ in range(num_bodies - 1):
        qpos_key_list.append("1 0 0 0") # The rest are at default orientation
    qpos_key = " ".join(qpos_key_list)

    # Generate the chain of bodies and actuators recursively
    body_xml = ""
    actuator_xml = ""
    for i in range(1, num_bodies + 1):
        joint_name = f"joint{i}"
        body_name = f"body{i}"
        
        # Make the last link a different color
        rgba = "0.2 0.2 1 1" if i == num_bodies else "0.2 1 0.2 1"
        
        body_xml += f"""
        <body name="{body_name}" pos="0 0 -0.5">
          <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" rgba="{rgba}"/>
          <joint name="{joint_name}" type="ball" pos="0 0 0"/>
        """
        
        actuator_xml += f"""
        <plugin joint="{joint_name}" plugin="user.joint.anisotropic.advanced">
            <!-- Make bending much stiffer in Y than in X -->
            <config key="k_bend_x" value="10.0"/>
            <config key="d_bend_x" value="0.5"/>
            <config key="k_bend_y" value="100.0"/> 
            <config key="d_bend_y" value="2.0"/>
            <config key="k_tor" value="10.0"/>
            <config key="d_tor" value="0.5"/>
        </plugin>
        """

    # Add closing body tags
    body_xml += "</body>" * num_bodies

    # Combine into the full XML string
    xml = f"""
<mujoco model="anisotropic_chain">
  <keyframe>
    <key name="initial_pose" qpos="{qpos_key.strip()}"/>
  </keyframe>

  <extension>
    <plugin plugin="user.joint.anisotropic.advanced"/>
  </extension>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <body name="body0" pos="0 0 1.0">
      <geom type="sphere" size="0.05" rgba="1 0.2 0.2 1"/>
      {body_xml}
    </body>
  </worldbody>

  <actuator>
    {actuator_xml}
  </actuator>
</mujoco>
"""
    return xml

MJCF_XML_STRING = generate_anisotropic_test_xml(10)

# =================================================================================================
#  2. Test Functions
# =================================================================================================

def load_plugin_and_model():
    """Loads the C plugin and the MJCF model, returning them."""
    print("--- Custom Anisotropic Joint Plugin Test ---")

    # Load the compiled C plugin library
    plugin_name = 'anisotropic_joint.dll' if os.name == 'nt' else 'libanisotropic_joint.so'
    plugin_path = os.path.join(os.path.dirname(__file__), plugin_name)

    if not os.path.exists(plugin_path):
        print(f"ERROR: Compiled plugin not found at '{plugin_path}'")
        print("Please compile the C code first.")
        return None, None

    try:
        print(f"Loading C plugin from: {plugin_path}")
        mujoco.mj_loadPluginLibrary(plugin_path)
        print("Plugin library loaded successfully.")
    except Exception as e:
        print(f"ERROR: Failed to load plugin library: {e}")
        return None, None

    # Load the model and create data
    try:
        print("Loading MJCF model from string...")
        model = mujoco.MjModel.from_xml_string(MJCF_XML_STRING)
        data = mujoco.MjData(model)
        print("Model and data created successfully.")
        return model, data
    except Exception as e:
        print(f"ERROR: Failed to compile model: {e}")
        return None, None

def run_headless_test(model, data):
    """Runs the simulation without visualization and logs output."""
    print("\nRunning HEADLESS simulation...")
    duration = 5.0
    framerate = 60
    n_steps = int(duration * framerate)
    joint1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint1')

    for i in range(n_steps):
        try:
            mujoco.mj_step(model, data)
            if i % 30 == 0:
                qvel1 = data.qvel[model.jnt_dofadr[joint1_id]:model.jnt_dofadr[joint1_id]+3]
                print(f"Step {i:4d} | J1 Vel: [{qvel1[0]:.3f}, {qvel1[1]:.3f}, {qvel1[2]:.3f}]")
                if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                    print("\nERROR: Simulation is unstable!")
                    break
        except mujoco.FatalError as e:
            print(f"\nERROR: MuJoCo fatal error at step {i}: {e}")
            break
    
    print("\nHeadless simulation finished.")

def run_visual_test(model, data):
    """Launches the interactive MuJoCo viewer to run the simulation."""
    print("\nRunning VISUAL simulation...")
    print("Close the viewer window to exit.")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            start = time.time()
            while viewer.is_running():
                step_start = time.time()
                mujoco.mj_step(model, data)
                viewer.sync()
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        print("\nVisual simulation finished.")
    except Exception as e:
        print(f"ERROR: Failed to launch viewer: {e}")

# =================================================================================================
#  3. Main Execution Block
# =================================================================================================

def main():
    """Parses command-line arguments and runs the appropriate test."""
    parser = argparse.ArgumentParser(description="Test script for the custom anisotropic joint plugin.")
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='If set, run the simulation in the interactive viewer.'
    )
    args = parser.parse_args()

    model, data = load_plugin_and_model()
    if not model or not data:
        return

    # Set the initial state to the keyframe to start the pendulum swinging
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'initial_pose')
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        print("Set initial state from keyframe 'initial_pose'.")

    if args.visualize:
        run_visual_test(model, data)
    else:
        run_headless_test(model, data)

if __name__ == "__main__":
    main()