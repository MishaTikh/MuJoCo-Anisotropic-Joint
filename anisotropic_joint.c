// anisotropic_joint.c
//
// A high-performance, advanced MuJoCo Engine Plugin that implements an
// anisotropic ball joint with non-linear physics.
//
// Compatible with modern MuJoCo versions (3.1.4+).

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

// =================================================================================================
//  Plugin data structures
// =================================================================================================

// Struct to hold all configurable parameters and cached data.
typedef struct {
  // Final computed parameters used in physics calculations
  mjtNum k_bend_x, d_bend_x;
  mjtNum k_bend_y, d_bend_y;
  mjtNum k_tor, d_tor;
  mjtNum k_bend_cubic;
  mjtNum d_coulomb;

  // Pre-calculated inverse for performance
  mjtNum friction_vel_tol_inv; 

  // Cached data for performance
  int joint_id;
  int joint_qposadr;
  int joint_dofadr;
  int actuator_id;
} AnisotropicAttrs;


// =================================================================================================
//  Plugin callbacks
// =================================================================================================

// Helper function to safely parse a config attribute.
static void get_attribute(const char** value, const char* name, const mjModel* m, int instance) {
  *value = mj_getPluginConfig(m, instance, name);
}

// Reads all attributes from the MJCF model and applies the default/override logic.
static void read_attributes(const mjModel* m, int instance, AnisotropicAttrs* attrs) {
  const char *value;
  double k_bend = -1, d_bend = -1;
  double k_bend_x = -1, d_bend_x = -1;
  double k_bend_y = -1, d_bend_y = -1;
  double friction_vel_tol = 1e-4;

  // Set defaults
  memset(attrs, 0, sizeof(AnisotropicAttrs));

  // Read all provided attributes using the helper
  get_attribute(&value, "k_bend", m, instance); if (value) sscanf(value, "%lf", &k_bend);
  get_attribute(&value, "d_bend", m, instance); if (value) sscanf(value, "%lf", &d_bend);
  get_attribute(&value, "k_bend_x", m, instance); if (value) sscanf(value, "%lf", &k_bend_x);
  get_attribute(&value, "d_bend_x", m, instance); if (value) sscanf(value, "%lf", &d_bend_x);
  get_attribute(&value, "k_bend_y", m, instance); if (value) sscanf(value, "%lf", &k_bend_y);
  get_attribute(&value, "d_bend_y", m, instance); if (value) sscanf(value, "%lf", &d_bend_y);
  get_attribute(&value, "k_tor", m, instance); if (value) sscanf(value, "%lf", &attrs->k_tor);
  get_attribute(&value, "d_tor", m, instance); if (value) sscanf(value, "%lf", &attrs->d_tor);
  get_attribute(&value, "k_bend_cubic", m, instance); if (value) sscanf(value, "%lf", &attrs->k_bend_cubic);
  get_attribute(&value, "d_coulomb", m, instance); if (value) sscanf(value, "%lf", &attrs->d_coulomb);
  get_attribute(&value, "friction_vel_tol", m, instance); if (value) sscanf(value, "%lf", &friction_vel_tol);

  // Apply logic: specific values (x/y) override general values.
  attrs->k_bend_x = (k_bend_x >= 0) ? k_bend_x : ((k_bend >= 0) ? k_bend : 0);
  attrs->k_bend_y = (k_bend_y >= 0) ? k_bend_y : ((k_bend >= 0) ? k_bend : 0);
  attrs->d_bend_x = (d_bend_x >= 0) ? d_bend_x : ((d_bend >= 0) ? d_bend : 0);
  attrs->d_bend_y = (d_bend_y >= 0) ? d_bend_y : ((d_bend >= 0) ? d_bend : 0);

  attrs->friction_vel_tol_inv = 1.0 / (friction_vel_tol > 1e-9 ? friction_vel_tol : 1e-9);
}

// `init` callback: called when mjData is created.
int anisotropic_init(const mjModel* m, mjData* d, int instance) {
  AnisotropicAttrs* attrs = (AnisotropicAttrs*)malloc(sizeof(AnisotropicAttrs));
  if (!attrs) {
    mju_error("Could not allocate memory for plugin attributes.");
    return -1;
  }
  read_attributes(m, instance, attrs);

  // Find the actuator this plugin instance is attached to, then find the joint.
  attrs->joint_id = -1;
  attrs->actuator_id = -1;
  for (int i = 0; i < m->nu; ++i) {
    if (m->actuator_plugin[i] == instance) {
      if (m->actuator_trntype[i] == mjTRN_JOINT) {
        int jnt_id = m->actuator_trnid[i*2]; // First element of trnid for joint
        if (m->jnt_type[jnt_id] == mjJNT_BALL) {
          attrs->joint_id = jnt_id;
          attrs->actuator_id = i;
          attrs->joint_qposadr = m->jnt_qposadr[jnt_id];
          attrs->joint_dofadr = m->jnt_dofadr[jnt_id];
          break;
        }
      }
    }
  }

  d->plugin_data[instance] = (uintptr_t)attrs;
  return 0;
}

// `destroy` callback: called when mjData is freed.
void anisotropic_destroy(mjData* d, int instance) {
  free((void*)d->plugin_data[instance]);
  d->plugin_data[instance] = 0;
}

// `reset` callback: called during mj_resetData.
void anisotropic_reset(const mjModel* m, double* plugin_state, void* plugin_data, int instance) {
  // This plugin is stateless, so there is nothing to do.
}

// `nstate` callback: returns the size of the plugin's state in mjData.
// This plugin is stateless, so we return 0. This is a REQUIRED callback.
int anisotropic_nstate(const mjModel* m, int instance) {
  return 0;
}

// Fast tanh approximation using a Pade approximant.
static inline mjtNum fast_tanh(mjtNum x) {
    mjtNum x2 = x * x;
    return x * (27 + x2) / (27 + 9 * x2);
}

// `compute` callback: called during each physics step.
void anisotropic_compute(const mjModel* m, mjData* d, int instance, int capability_bit) {
  if (!(capability_bit & mjPLUGIN_ACTUATOR)) {
    return;
  }

  const AnisotropicAttrs* attrs = (const AnisotropicAttrs*)d->plugin_data[instance];
  if (!attrs || attrs->joint_id == -1) {
    return;
  }
  
  const mjtNum* quat = d->qpos + attrs->joint_qposadr;
  const mjtNum* ang_vel = d->qvel + attrs->joint_dofadr;

  mjtNum local_ang_vel[3];
  mju_rotVecQuat(local_ang_vel, ang_vel, quat);
  
  // Stiffness model using quaternion components (small-angle approximation).
  const mjtNum qx = quat[1];
  const mjtNum qy = quat[2];
  const mjtNum qz = quat[3];
  mjtNum local_stiffness_torque[3] = {
    -qx * (attrs->k_bend_x + attrs->k_bend_cubic * qx * qx),
    -qy * (attrs->k_bend_y + attrs->k_bend_cubic * qy * qy),
    -qz * attrs->k_tor
  };
  mju_scl(local_stiffness_torque, local_stiffness_torque, 2.0, 3);

  mjtNum local_damping_torque[3] = {
    -attrs->d_bend_x * local_ang_vel[0] - attrs->d_coulomb * fast_tanh(local_ang_vel[0] * attrs->friction_vel_tol_inv),
    -attrs->d_bend_y * local_ang_vel[1] - attrs->d_coulomb * fast_tanh(local_ang_vel[1] * attrs->friction_vel_tol_inv),
    -attrs->d_tor * local_ang_vel[2]    - attrs->d_coulomb * fast_tanh(local_ang_vel[2] * attrs->friction_vel_tol_inv)
  };

  mjtNum local_total_torque[3];
  mju_add(local_total_torque, local_damping_torque, local_stiffness_torque, 3);
  
  mjtNum inv_quat[4];
  mju_negQuat(inv_quat, quat);
  mjtNum global_torque[3];
  mju_rotVecQuat(global_torque, local_total_torque, inv_quat);

  // Add the computed torque directly to the joint's DoFs in qfrc_actuator.
  mju_addTo(d->qfrc_actuator + attrs->joint_dofadr, global_torque, 3);
}

// =================================================================================================
//  Plugin registration
// =================================================================================================

const char* const attributes[] = {
  "k_bend", "d_bend",
  "k_bend_x", "d_bend_x",
  "k_bend_y", "d_bend_y",
  "k_tor", "d_tor",
  "k_bend_cubic", "d_coulomb", "friction_vel_tol"
};

mjpPlugin anisotropic_joint_plugin = {
  .name = "user.joint.anisotropic.advanced",
  .nattribute = sizeof(attributes) / sizeof(attributes[0]),
  .attributes = attributes,
  .capabilityflags = mjPLUGIN_ACTUATOR,
  .needstage = mjSTAGE_POS,
  .nstate = anisotropic_nstate, // Assign the nstate callback
  .init = anisotropic_init,
  .destroy = anisotropic_destroy,
  .reset = anisotropic_reset,
  .compute = anisotropic_compute,
};

void mjplugins_register(void) {
  mjp_registerPlugin(&anisotropic_joint_plugin);
}

#ifdef _WIN32
  #include <windows.h>
  BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    if (ul_reason_for_call == DLL_PROCESS_ATTACH) {
      mjplugins_register();
    }
    return TRUE;
  }
#else
  __attribute__((constructor))
  void constructor_library(void) {
    mjplugins_register();
  }
#endif