#ifndef Constants_wipper_h
#define Constants_wipper_h

const double G  = 9.81;
const double PI = 3.14159265359;

// ====================================================================================================================================
// 1. GEOMETRY (From CAD)
// ====================================================================================================================================
    double r_main_pivot_to_cg  = 0.3825;
    double r_pivot_to_hinge    = 0.175;
    double r_hinge_to_cw_cg    = 0.6916;
    double r_main_tip          = 0.9625;
    double sling_length        = 0.9;

// ====================================================================================================================================
// 2. CAD MASS & INERTIA INPUTS
// ====================================================================================================================================
    double m_arm               = 0.167;
    double m_cw                = 1.016;
    double m_p                 = 0.023;
    double m_total_system_mass = m_arm + m_cw + m_p;

    double I_main_arm          = 0.000005859 + m_arm * r_main_pivot_to_cg * r_main_pivot_to_cg;
    double I_cw_arm_total      = 0.03467282  + m_cw  * r_hinge_to_cw_cg  * r_hinge_to_cw_cg;
    double I_projectile_tip    = m_p * r_main_tip * r_main_tip;
    double I_fused             = I_main_arm + I_cw_arm_total + I_projectile_tip;

// ====================================================================================================================================
// 3. ANGLES
// ====================================================================================================================================
    double theta_main_start        = 45  * (PI / 180.0);
    double theta_cw_rel_start      =  4.12   * (PI / 180.0);
    double release_arm_angle_deg   = 90.0;
    double sling_release_angle_deg = 43.55;

    // Sling absolute angle at release: ball launches at 90deg to sling,
    // so sling points at (43.55 - 90) = -46.45 deg from horizontal
    double sling_release_abs_angle = (sling_release_angle_deg - 90.0) * (PI / 180.0);

    double mu_poplar   = 0.35;
    double pin_radius  = 0.00635;

// ====================================================================================================================================
// 4. STATE TRANSITION THRESHOLDS
// ====================================================================================================================================
    double fused_to_parametric_threshold = -90.0 * (PI / 180.0);
    double target_theta                  = (release_arm_angle_deg - 360.0) * (PI / 180.0);

// ====================================================================================================================================
// 5. STATE LOGIC
// ====================================================================================================================================
    enum SimuStates { fused_motion, parametric_motion, launch, data_collection };
    bool running      = true;
    SimuStates current_state = fused_motion;

// ====================================================================================================================================
// 6. SIMULATION PARAMETERS
// ====================================================================================================================================
    double omega_main  = 0.0;
    double dt          = 0.0001;
    int    iterations  = 0;

#endif