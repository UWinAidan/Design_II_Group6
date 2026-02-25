#ifndef Constants_wipper_h
#define Constants_wipper_h

const double G = 9.81;
const double PI = 3.14159265359;

// ==========================================================
    // 1. GEOMETRY (From CAD)
    // ==========================================================
    double r_main_pivot_to_cg = 0.3825; // Distance from main pivot to main arm CG (m)
    double r_pivot_to_hinge = 0.175; // Distance from main pivot to CW arm hinge (m)
    double r_hinge_to_cw_cg = 0.6916; // Distance from hinge to center of CW (m)
    double r_main_tip = 0.9625;       // Distance from main pivot to sling pin (m)
    double sling_length = 0.9;     // Length of the rope (m)

// ==========================================================
    // 2. CAD MASS & INERTIA INPUTS (Directly from your model)
    // ==========================================================
    double m_arm = 0.167;                          // Mass of the main arm (kg)
    double m_cw = 1.016;                           // Mass of cw (kg)
    double m_p = 0.023;                            // Mass of projectile (kg)
    double m_total_arms = m_arm + m_cw + m_p;      // Combined mass of all wood (for friction)

    // Enter the "Moment of Inertia" about the Z-axis (pivot axis)
    double I_main_arm = 0.000005859 + m_arm * r_main_pivot_to_cg * r_main_pivot_to_cg ; // CAD: Main arm + sling hardware (kg*m^2)
    double I_cw_arm_total = 0.03467282 + m_cw * r_hinge_to_cw_cg * r_hinge_to_cw_cg;     // CAD: Secondary arm + CW cylinder (kg*m^2)
    double I_projectile = 0.0;                                                          // Calculated below based on position

    // ==========================================================
    // 3. STARTING ANGLES (Estimated from your image)
    // ==========================================================
    // 0 rad is horizontal. Positive is counter-clockwise.
    double theta_main = 77.865 * (PI / 180.0);   // Main arm is pointed high up
    // double theta_cw_rel = 81.985 * (PI / 180.0); // CW arm folded back from main
    double theta_cw_rel = (81.985-77.865) * (PI / 180.0); // CW arm folded back from main

    double release_angle_deg = 43.55; // Angle of projectile release above horizontal (degrees)
    double mu_poplar = 0.35;       // Friction: Poplar on Poplar
    double pin_radius = 0.00635;     // Radius of the pivot pin (m)
    
    // ==========================================================
    // 4. SIMULATION LOGIC
    // ==========================================================
    double omega_main = 0.0;
    double dt = 0.0005;
    double target_theta = (PI / 2.0) + (release_angle_deg * PI / 180.0);
#endif