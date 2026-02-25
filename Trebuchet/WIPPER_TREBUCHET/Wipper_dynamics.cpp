#include <iostream>
#include <cmath>
#include <iomanip>
#include "Constants_WIPPER.h"

using namespace std;

int main() {
    int iterations = 0;
    // 1. Calculate the target in radians
double target_rad = (PI / 2.0) + (release_angle_deg * PI / 180.0);

// 2. Check if we are already past it
if (theta_main >= target_rad) {
    cout << "Arm already past release point!" << endl;
    return 0;
}

while (theta_main < target_rad) {
    iterations++;
    
    // Safety: If it takes more than 5 seconds of "simulated time"
    if (iterations * dt > 5.0) { 
        cout << "Simulation timed out: Arm moving too slow or wrong way." << endl;
        break;
    }

    double theta_cw_abs = theta_main + theta_cw_rel;
    
    // TORQUE CALCULATION
    double torque_cw = m_cw * G * (r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs));
    double r_p_eff = r_main_tip + sling_length;
    double torque_p = m_p * G * r_p_eff * cos(theta_main);
    
    // Note: Friction must oppose the direction of motion
    double friction_mag = mu_poplar * (m_cw + m_p + m_total_arms) * G * pin_radius;
    double torque_friction = (omega_main > 0) ? -friction_mag : (omega_main < 0 ? friction_mag : 0);

    double I_total = I_main_arm + I_cw_arm_total + (m_p * pow(r_p_eff, 2));

    double net_torque = torque_cw - torque_p + torque_friction;
    double alpha = net_torque / I_total;

    // Numerical Integration (Euler)
    omega_main += alpha * dt;
    theta_main += omega_main * dt;

    // The "Whip" effect
    theta_cw_rel -= (omega_main * 0.2) * dt;
}


// int main() {

//         int iterations = 0;

//     while (theta_main < target_theta) {
//         iterations++;
//     if (iterations > 1000000) { // Safety break to prevent infinite hang
//         cout << "Error: Simulation timed out. Check torque/angle logic." << endl;
//         return 1;
//     }

//         // Current absolute angle of the CW arm
//         double theta_cw_abs = theta_main + theta_cw_rel;

//         // Torques (T = r * F * cos(theta))
//         // The CW pulls down, creating torque around the main pivot
//         double torque_cw = m_cw * G * (r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs));
//         double r_p_eff = r_main_tip + sling_length;
//         double torque_p = m_p * G * r_p_eff * cos(theta_main);

//         // Friction Torque (Resists motion)
//         double total_load = (m_cw + m_p + m_total_arms) * G;
//         double torque_friction = mu_poplar * total_load * pin_radius;

//         // Dynamic Inertia of Projectile (changes with arm position)
//         I_projectile = m_p * pow(r_p_eff, 2);
//         double I_total = I_main_arm + I_cw_arm_total + I_projectile;

//         // Acceleration
//         double net_torque = torque_cw - torque_p;
//         if (omega_main >= 0) net_torque -= torque_friction;
//         else net_torque += torque_friction;

//         double alpha = net_torque / I_total;
//         omega_main += alpha * dt;
//         theta_main += omega_main * dt;

//         // Simulate the "Whip": The CW arm unfolds as the main arm drops
//         // This is a simplification of the secondary rotation
//         theta_cw_rel -= (omega_main * 0.2) * dt; 

//         if (theta_main < -2/PI) break; 
//     }

    // ==========================================================
    // 5. OUTPUT
    // ==========================================================
    double v_release = omega_main * (r_main_tip + sling_length);
    double vx = v_release * cos(release_angle_deg * PI / 180.0);
    double vy = v_release * sin(release_angle_deg * PI / 180.0);
    double range = (vx * (vy + sqrt(vy * vy + 2 * G * 0))) / G;

    cout << fixed << setprecision(2);
    cout << "--- CAD-Based Whipper Results ---" << endl;
    cout << "Starting Main Angle:  " << (theta_main * 180 / PI) << " deg" << endl;
    cout << "Final Angular Vel:    " << omega_main << " rad/s" << endl;
    cout << "Release Velocity:     " << v_release << " m/s" << endl;
    cout << "Estimated Range:      " << range << " m" << endl;
    cout << "---------------------------------" << endl;

    return 0;
}