// Copyright 2024. All Rights Reserved.
//
// This file implements a custom MuJoCo plugin for applying anisotropic stiffness
// based on a simplified Cosserat rod theory, written in C.
// This version is adapted to use the actuator-based plugin mechanism for
// compatibility with older MuJoCo API patterns.

#include <stdlib.h>
#include <string.h>
#include <stdint.h> // For uintptr_t

#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>

// Structure to hold the plugin's state and parameters
typedef struct {
    // Stiffness matrix diagonal elements [Nm^2]
    mjtNum B11;  // Bending stiffness about local x-axis
    mjtNum B22;  // Bending stiffness about local y-axis
    mjtNum B33;  // Torsional stiffness about local z-axis (twist)

    // Intrinsic curvature vector (k0) [rad/m]
    mjtNum k0[3];

    // BUG FIX: Add segment_length to the state
    mjtNum segment_length;

    // Cached data for performance
    int joint_id;
    int joint_qposadr;
    int joint_dofadr;
    int actuator_id;
} AnisotropicJointState;

// Function to safely parse a numerical attribute from the XML
static mjtNum get_numeric_attribute(const char* value) {
    if (!value) {
        return 0.0;
    }
    return strtod(value, NULL);
}

// init callback: This is called when mjData is created.
static int AnisotropicJointInit(const mjModel* m, mjData* d, int plugin_id) {
    // Allocate and initialize the plugin state
    AnisotropicJointState* state = (AnisotropicJointState*)malloc(sizeof(AnisotropicJointState));
    if (!state) {
        mju_error("Could not allocate memory for plugin state.");
        return -1;
    }
    memset(state, 0, sizeof(AnisotropicJointState));

    // --- Read configuration attributes from the MJCF model ---
    const char* B11_char = mj_getPluginConfig(m, plugin_id, "B11");
    const char* B22_char = mj_getPluginConfig(m, plugin_id, "B22");
    const char* B33_char = mj_getPluginConfig(m, plugin_id, "B33");
    const char* k0_x_char = mj_getPluginConfig(m, plugin_id, "k0_x");
    const char* k0_y_char = mj_getPluginConfig(m, plugin_id, "k0_y");
    const char* k0_z_char = mj_getPluginConfig(m, plugin_id, "k0_z");
    // BUG FIX: Read the new segment_length attribute
    const char* segment_length_char = mj_getPluginConfig(m, plugin_id, "segment_length");

    state->B11 = get_numeric_attribute(B11_char);
    state->B22 = get_numeric_attribute(B22_char);
    state->B33 = get_numeric_attribute(B33_char);
    state->k0[0] = get_numeric_attribute(k0_x_char);
    state->k0[1] = get_numeric_attribute(k0_y_char);
    state->k0[2] = get_numeric_attribute(k0_z_char);
    // BUG FIX: Store segment_length
    state->segment_length = get_numeric_attribute(segment_length_char);


    // --- Find the associated joint by searching through actuators ---
    state->joint_id = -1;
    for (int i = 0; i < m->nu; ++i) {
        if (m->actuator_plugin[i] == plugin_id) {
            if (m->actuator_trntype[i] == mjTRN_JOINT) {
                int jnt_id = m->actuator_trnid[i*2];
                if (jnt_id >= 0 && m->jnt_type[jnt_id] == mjJNT_BALL) {
                    state->joint_id = jnt_id;
                    state->actuator_id = i;
                    state->joint_qposadr = m->jnt_qposadr[jnt_id];
                    state->joint_dofadr = m->jnt_dofadr[jnt_id];
                    // Store the state pointer in mjData's plugin_data field
                    d->plugin_data[plugin_id] = (uintptr_t)state;
                    return 0; // Success
                }
            }
        }
    }

    mju_warning("Anisotropic joint plugin: Could not find unique target ball joint for this instance.");
    free(state);
    return -1; // Failure
}

// destroy callback: called when mjData is freed.
static void AnisotropicJointDestroy(mjData* d, int plugin_id) {
    if (d->plugin_data[plugin_id]) {
        free((void*)d->plugin_data[plugin_id]);
        d->plugin_data[plugin_id] = 0;
    }
}

// compute callback: called at each simulation step
static void AnisotropicJointCompute(const mjModel* m, mjData* d, int plugin_id, int capability_bit) {
    if (!(capability_bit & mjPLUGIN_ACTUATOR)) {
        return;
    }

    AnisotropicJointState* state = (AnisotropicJointState*)d->plugin_data[plugin_id];
    if (!state || state->joint_id == -1) {
        return;
    }

    const mjtNum* quat = d->qpos + state->joint_qposadr;

    mjtNum current_kappa[3];
    mju_quat2Vel(current_kappa, quat, 1.0);

    // BUG FIX: Calculate joint stiffness (k = B/L)
    mjtNum inv_len = (state->segment_length > 1e-9) ? 1.0 / state->segment_length : 0.0;
    mjtNum k_bend_x = state->B11 * inv_len;
    mjtNum k_bend_y = state->B22 * inv_len;
    mjtNum k_tor = state->B33 * inv_len;

    mjtNum kappa_error[3];
    mju_sub3(kappa_error, current_kappa, state->k0);

    mjtNum torque[3];
    // BUG FIX: Use the calculated joint stiffness
    torque[0] = k_bend_x * kappa_error[0];
    torque[1] = k_bend_y * kappa_error[1];
    torque[2] = k_tor * kappa_error[2];

    int dof_adr = state->joint_dofadr;
    d->qfrc_passive[dof_adr + 0] -= torque[0];
    d->qfrc_passive[dof_adr + 1] -= torque[1];
    d->qfrc_passive[dof_adr + 2] -= torque[2];

    d->ctrl[state->actuator_id] = 0;
}

// reset callback
static void AnisotropicJointReset(const mjModel* m, double* plugin_state, void* plugin_data, int plugin_id) {
    // This plugin is stateless in mjData.plugin_state, so nothing to do.
}

// nstate callback
static int AnisotropicJointNstate(const mjModel* m, int instance) {
    return 0;
}

// --- Plugin Registration ---
// BUG FIX: Add "segment_length" to the list of recognized attributes
const char* const attributes[] = {"B11", "B22", "B33", "k0_x", "k0_y", "k0_z", "segment_length"};

mjpPlugin anisotropic_joint_plugin = {
    .name = "user.joint.anisotropic.advanced",
    .nattribute = sizeof(attributes) / sizeof(attributes[0]),
    .attributes = attributes,
    .capabilityflags = mjPLUGIN_ACTUATOR,
    .needstage = mjSTAGE_POS,
    .nstate = AnisotropicJointNstate,
    .init = AnisotropicJointInit,
    .destroy = AnisotropicJointDestroy,
    .reset = AnisotropicJointReset,
    .compute = AnisotropicJointCompute,
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
