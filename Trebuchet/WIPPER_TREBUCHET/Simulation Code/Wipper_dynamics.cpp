#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <windows.h>
#include "Constants_WIPPER.h"

using namespace std;

int main() {
    double theta_main = theta_main_start; // Start at +77.865°
    double theta_cw_rel = theta_cw_rel_start;
    
    // Store simulation data for analysis
    vector<double> time_data, angle_data, velocity_data;
    
    // Calculate starting torques (using CCW-positive convention)
    double r_p_eff = r_main_tip + sling_length;
    double theta_cw_abs_start = theta_main + theta_cw_rel;
    
    double horizontal_dist_cw_start = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs_start);
    double horizontal_dist_p_start = r_p_eff * cos(theta_main);

    // Torques are negative for clockwise rotation (our desired direction)
    double torque_cw_gravity_start = -m_cw * G * horizontal_dist_cw_start;
    double torque_p_gravity_start = -m_p * G * horizontal_dist_p_start;
    
    double net_initial_torque = torque_cw_gravity_start + torque_p_gravity_start; // Sum of negative torques
    double t_fric_magnitude = mu_poplar * m_total_system_mass * G * pin_radius;

    cout << "=== TREBUCHET SIMULATION STARTING ===" << endl;
    cout << "Starting Main Angle:  " << (theta_main * 180/PI) << " degrees" << endl;
    cout << "Target Release Angle: " << (target_theta * 180/PI) << " degrees" << endl;
    cout << "Initial CW Torque (driving): " << torque_cw_gravity_start << " Nm" << endl;
    cout << "Initial Projectile Torque (resisting): " << torque_p_gravity_start << " Nm" << endl;
    cout << "Net Initial Torque: " << net_initial_torque << " Nm" << endl;
    cout << "Friction Torque Magnitude: " << t_fric_magnitude << " Nm" << endl;
    
    if (abs(net_initial_torque) < t_fric_magnitude) {
        cout << "!!! STALL DETECTED !!!" << endl;
        cout << "The Counterweight is not heavy enough to overcome friction." << endl;
        return 1; 
    }

    cout << "Starting simulation (clockwise rotation)..." << endl;
    
    // Simulation loop - continue until we reach target angle
    while (theta_main > target_theta && iterations < 3000000) {
        iterations++;
        
        double theta_cw_abs = theta_main + theta_cw_rel;
        
        double horizontal_dist_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs);
        double horizontal_dist_p = r_p_eff * cos(theta_main);

        double torque_cw_gravity = -m_cw * G * horizontal_dist_cw;
        double torque_p_gravity = -m_p * G * horizontal_dist_p;
        
        double I_total = I_main_arm + I_cw_arm_total + (m_p * pow(r_p_eff, 2));

        // Net torque calculation
        double net_torque = torque_cw_gravity + torque_p_gravity;
        
        // Apply friction: always opposes motion. If omega_main < 0 (CW), friction adds positive torque.
        if (omega_main < 0) { // Moving clockwise
            net_torque += t_fric_magnitude;
        } else if (omega_main > 0) { // Moving counter-clockwise
            net_torque -= t_fric_magnitude;
        } else { // omega_main == 0, check static friction
            if (abs(net_torque) < t_fric_magnitude) {
                net_torque = 0; // Stalled
            } else {
                // Break static friction
                net_torque = (net_torque < 0) ? net_torque + t_fric_magnitude : net_torque - t_fric_magnitude;
            }
        }

        double alpha = net_torque / I_total;
        omega_main += alpha * dt;
        theta_main += omega_main * dt;

        // The "Whip" - Secondary arm unfolding during the swing
        // Ensure theta_cw_rel decreases (unfolds) as arm swings down
        if (omega_main < 0 && theta_main < theta_main_start && theta_main > target_theta) { 
            theta_cw_rel -= abs(omega_main) * 0.05 * dt; // Adjust factor (0.05) as needed
            // Clamp theta_cw_rel to prevent it from going too far
            if (theta_cw_rel < -PI/2) theta_cw_rel = -PI/2; // Example lower bound
            if (theta_cw_rel > PI/2) theta_cw_rel = PI/2; // Example upper bound
        }
        
        // Store data every 5000 iterations for analysis
        if (iterations % 5000 == 0) {
            time_data.push_back(iterations * dt);
            angle_data.push_back(theta_main * 180.0 / PI);
            velocity_data.push_back(omega_main);
        }
        
        // Debug output every 100000 iterations
        if (iterations % 100000 == 0) {
            cout << "Iteration " << iterations << ": angle = " << (theta_main * 180/PI) 
                 << " deg, omega = " << omega_main << " rad/s" << endl;
        }
        
        // Safety check for stalled simulation
        if (iterations > 100000 && abs(omega_main) < 0.001) { // Reduced threshold for stall
            cout << "Simulation appears stalled. Final angle: " << (theta_main * 180/PI) << " degrees" << endl;
            break;
        }
    }

    cout << "Simulation completed after " << iterations << " iterations" << endl;
    
    // Calculate final results
    double v_release = abs(omega_main) * (r_main_tip + sling_length); // Use absolute angular velocity
    double vx = v_release * cos(sling_release_angle_deg * PI / 180.0);
    double vy = v_release * sin(sling_release_angle_deg * PI / 180.0);
    double flight_time = (vy + sqrt(vy * vy + 2 * G * 1.5)) / G; // Assuming 1.5m release height
    double range = vx * flight_time;

    cout << fixed << setprecision(3);
    cout << "\n=== CAD-Based Whipper Results ===" << endl;
    cout << "Starting Angle:       " << (theta_main_start * 180 / PI) << " deg" << endl;
    cout << "Final Main Angle:     " << (theta_main * 180 / PI) << " deg" << endl;
    cout << "Final Angular Vel:    " << omega_main << " rad/s" << endl;
    cout << "Release Velocity:     " << v_release << " m/s" << endl;
    cout << "Horizontal Velocity:  " << vx << " m/s" << endl;
    cout << "Vertical Velocity:    " << vy << " m/s" << endl;
    cout << "Estimated Range:      " << range << " m" << endl;
    cout << "Simulation Time:      " << (iterations * dt) << " s" << endl;
    cout << "=================================" << endl;

    return 0;
}
