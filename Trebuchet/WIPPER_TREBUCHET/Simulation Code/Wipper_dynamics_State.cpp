#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <windows.h>
#include "Constants_WIPPER.h"

using namespace std;

int main() {

    vector<double> time_data, angle_data, velocity_data; // Store simulation data for analysis

    double theta_main = theta_main_start;
    double theta_cw_rel = theta_cw_rel_start;

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

    while(running) {
    //=================================================================================================
    // state selector
    //=================================================================================================
        if (iterations == 0) {
            current_state = fused_motion;
            cout << "selected state: fused_motion" << endl;
        }else if (abs_cw_angle > -90) {
            current_state = paremetric_motion;
            cout << "selected state: paremetric_motion" << endl;
        }else if (torque_cw_pv==0) {
            current_state = launch;
            cout << "selected state: stall_stop" << endl;
        }else if (Sling_angle== launch_anlge ) {
            current_state = data_collection;
            cout << "selected state: data_collection" << endl;
        }else {
            running = false;
            cout << "No state selected. Ending simulation." << endl;
        }
    //=================================================================================================
    //State run
    //=================================================================================================
        switch(current_state) {
            case fused_motion:
            {
            cout << "Running fused motion state"<<endl;
            
                break;
            }
            case paremetric_motion:
            {
            cout << "Running paremetric motion state"<<endl;
                break;
            }    
            case launch:
            {
            cout << "Running launch state"<<endl;
                break;
            }
            case data_collection:
            {
            cout << "Running data collection state"<<endl;
                break;
            }
        }
    }
}