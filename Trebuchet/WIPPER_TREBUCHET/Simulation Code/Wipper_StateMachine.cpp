#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include "Constants_WIPPER.h"

using namespace std;

// ============================================================
//  STATE MACHINE DEFINITION
//  States derived from the 6-phase diagram (sketch page 2):
//
//  STATE 0 - INIT        : Setup, stall-check, compute constants
//  STATE 1 - EARLY_SWING : θ = 77.865° → 45°   | CW arm locked at β=4.12°
//  STATE 2 - MID_SWING   : θ = 45°    → 0°     | CW arm begins to unfold
//  STATE 3 - LATE_SWING  : θ = 0°     → -90°   | CW arm actively unfolding
//  STATE 4 - DEEP_SWING  : θ = -90°   → -180°  | Arm swings below pivot
//  STATE 5 - FINAL_ARC   : θ = -180°  → -270°  | Sling whips outward
//  STATE 6 - RELEASE     : θ ≤ -270° (≡ +90°)  | Projectile launched
//  STATE 7 - DONE        : Post-flight ballistics & results
//  STATE 8 - ERROR       : Stall or runaway detected
// ============================================================

enum SimState {
    STATE_INIT        = 0,
    STATE_EARLY_SWING = 1,
    STATE_MID_SWING   = 2,
    STATE_LATE_SWING  = 3,
    STATE_DEEP_SWING  = 4,
    STATE_FINAL_ARC   = 5,
    STATE_RELEASE     = 6,
    STATE_DONE        = 7,
    STATE_ERROR       = 8
};

// Helper: readable state name for logging
const char* stateName(SimState s) {
    switch (s) {
        case STATE_INIT:        return "INIT";
        case STATE_EARLY_SWING: return "EARLY_SWING";
        case STATE_MID_SWING:   return "MID_SWING";
        case STATE_LATE_SWING:  return "LATE_SWING";
        case STATE_DEEP_SWING:  return "DEEP_SWING";
        case STATE_FINAL_ARC:   return "FINAL_ARC";
        case STATE_RELEASE:     return "RELEASE";
        case STATE_DONE:        return "DONE";
        case STATE_ERROR:       return "ERROR";
        default:                return "UNKNOWN";
    }
}

// Angle thresholds for state transitions (radians)
const double DEG2RAD = PI / 180.0;
const double THRESH_1_2 =  45.0 * DEG2RAD;    // EARLY → MID
const double THRESH_2_3 =   0.0 * DEG2RAD;    // MID   → LATE
const double THRESH_3_4 = -90.0 * DEG2RAD;    // LATE  → DEEP
const double THRESH_4_5 =-180.0 * DEG2RAD;    // DEEP  → FINAL
// target_theta (from Constants) = -270° = release point

int main() {

    // ── Live simulation variables ──────────────────────────────
    double theta_main  = theta_main_start;
    double theta_cw_rel = theta_cw_rel_start;
    double r_p_eff     = r_main_tip + sling_length;
    double t_fric_magnitude = mu_poplar * m_total_system_mass * G * pin_radius;

    // Results
    double v_release = 0, vx = 0, vy = 0, range = 0, flight_time = 0;

    // Data log
    vector<double> time_data, angle_data, velocity_data;

    // State machine
    SimState state = STATE_INIT;
    bool running   = true;

    cout << fixed << setprecision(4);
    cout << "=== WHIPPER TREBUCHET — STATE MACHINE SIMULATION ===" << endl;

    while (running) {

        switch (state) {

        // ══════════════════════════════════════════════════════
        case STATE_INIT:
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 0: INIT]" << endl;
            cout << "  Starting Main Angle  : " << (theta_main * 180/PI) << " deg" << endl;
            cout << "  Target Release Angle : " << (target_theta * 180/PI) << " deg" << endl;
            cout << "  Friction Torque Mag  : " << t_fric_magnitude << " Nm" << endl;

            // Compute initial torques to check for stall
            double theta_cw_abs_start = theta_main + theta_cw_rel;
            double h_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs_start);
            double h_p  = r_p_eff * cos(theta_main);

            double torque_cw  = -m_cw * G * h_cw;
            double torque_p   = -m_p  * G * h_p;
            double net_torque = torque_cw + torque_p;

            cout << "  Initial CW Torque    : " << torque_cw  << " Nm" << endl;
            cout << "  Initial Proj Torque  : " << torque_p   << " Nm" << endl;
            cout << "  Net Initial Torque   : " << net_torque << " Nm" << endl;

            if (abs(net_torque) < t_fric_magnitude) {
                cout << "  !!! STALL — Counterweight insufficient to overcome friction !!!" << endl;
                state = STATE_ERROR;
            } else {
                cout << "  Stall check PASSED — beginning swing." << endl;
                state = STATE_EARLY_SWING;
            }
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_EARLY_SWING:
        // Phase 1 → 2 in sketch: arm falls from 77.865° → 45°
        // CW arm stays locked at β = 4.12° (no unfolding yet)
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 1: EARLY_SWING]  (θ: " << (theta_main*180/PI)
                 << "° → 45°, CW locked)" << endl;

            while (theta_main > THRESH_1_2 && iterations < 3000000) {
                iterations++;

                double theta_cw_abs = theta_main + theta_cw_rel;
                double h_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs);
                double h_p  = r_p_eff * cos(theta_main);

                double torque_cw  = -m_cw * G * h_cw;
                double torque_p   = -m_p  * G * h_p;
                double net_torque = torque_cw + torque_p;

                double I_total = I_main_arm + I_cw_arm_total + m_p * r_p_eff * r_p_eff;

                // Friction opposes motion
                if      (omega_main < 0) net_torque += t_fric_magnitude;
                else if (omega_main > 0) net_torque -= t_fric_magnitude;
                else {
                    if (abs(net_torque) < t_fric_magnitude) net_torque = 0;
                    else net_torque += (net_torque < 0) ? t_fric_magnitude : -t_fric_magnitude;
                }

                double alpha = net_torque / I_total;
                omega_main += alpha * dt;
                theta_main += omega_main * dt;

                // theta_cw_rel stays constant in this state (locked)

                if (iterations % 5000 == 0) {
                    time_data.push_back(iterations * dt);
                    angle_data.push_back(theta_main * 180.0 / PI);
                    velocity_data.push_back(omega_main);
                }
                if (iterations % 100000 == 0)
                    cout << "    iter=" << iterations << "  θ=" << (theta_main*180/PI)
                         << "°  ω=" << omega_main << " rad/s" << endl;

                if (iterations > 100000 && abs(omega_main) < 0.001) { state = STATE_ERROR; break; }
            }

            if (state != STATE_ERROR) {
                cout << "  → Exiting EARLY_SWING at θ=" << (theta_main*180/PI)
                     << "°  ω=" << omega_main << " rad/s" << endl;
                state = STATE_MID_SWING;
            }
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_MID_SWING:
        // Phase 2 → 3 in sketch: arm falls from 45° → 0° (horizontal)
        // CW arm starts gently unfolding
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 2: MID_SWING]  (θ: " << (theta_main*180/PI)
                 << "° → 0°, CW begins unfolding)" << endl;

            while (theta_main > THRESH_2_3 && iterations < 3000000) {
                iterations++;

                double theta_cw_abs = theta_main + theta_cw_rel;
                double h_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs);
                double h_p  = r_p_eff * cos(theta_main);

                double torque_cw  = -m_cw * G * h_cw;
                double torque_p   = -m_p  * G * h_p;
                double net_torque = torque_cw + torque_p;

                double I_total = I_main_arm + I_cw_arm_total + m_p * r_p_eff * r_p_eff;

                if      (omega_main < 0) net_torque += t_fric_magnitude;
                else if (omega_main > 0) net_torque -= t_fric_magnitude;
                else {
                    if (abs(net_torque) < t_fric_magnitude) net_torque = 0;
                    else net_torque += (net_torque < 0) ? t_fric_magnitude : -t_fric_magnitude;
                }

                double alpha = net_torque / I_total;
                omega_main += alpha * dt;
                theta_main += omega_main * dt;

                // CW arm begins to unfold slowly (factor 0.025 — half the full unfold rate)
                if (omega_main < 0) {
                    theta_cw_rel -= abs(omega_main) * 0.025 * dt;
                    theta_cw_rel = max(theta_cw_rel, -PI/2.0);
                }

                if (iterations % 5000 == 0) {
                    time_data.push_back(iterations * dt);
                    angle_data.push_back(theta_main * 180.0 / PI);
                    velocity_data.push_back(omega_main);
                }
                if (iterations % 100000 == 0)
                    cout << "    iter=" << iterations << "  θ=" << (theta_main*180/PI)
                         << "°  ω=" << omega_main << "  β=" << (theta_cw_rel*180/PI) << "°" << endl;

                if (iterations > 100000 && abs(omega_main) < 0.001) { state = STATE_ERROR; break; }
            }

            if (state != STATE_ERROR) {
                cout << "  → Exiting MID_SWING at θ=" << (theta_main*180/PI)
                     << "°  ω=" << omega_main << " rad/s" << endl;
                state = STATE_LATE_SWING;
            }
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_LATE_SWING:
        // Phase 3 → 4 in sketch: arm swings 0° → -90°
        // CW arm actively unfolding at full rate
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 3: LATE_SWING]  (θ: " << (theta_main*180/PI)
                 << "° → -90°, CW actively unfolding)" << endl;

            while (theta_main > THRESH_3_4 && iterations < 3000000) {
                iterations++;

                double theta_cw_abs = theta_main + theta_cw_rel;
                double h_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs);
                double h_p  = r_p_eff * cos(theta_main);

                double torque_cw  = -m_cw * G * h_cw;
                double torque_p   = -m_p  * G * h_p;
                double net_torque = torque_cw + torque_p;

                double I_total = I_main_arm + I_cw_arm_total + m_p * r_p_eff * r_p_eff;

                if      (omega_main < 0) net_torque += t_fric_magnitude;
                else if (omega_main > 0) net_torque -= t_fric_magnitude;
                else {
                    if (abs(net_torque) < t_fric_magnitude) net_torque = 0;
                    else net_torque += (net_torque < 0) ? t_fric_magnitude : -t_fric_magnitude;
                }

                double alpha = net_torque / I_total;
                omega_main += alpha * dt;
                theta_main += omega_main * dt;

                // Full unfold rate
                if (omega_main < 0) {
                    theta_cw_rel -= abs(omega_main) * 0.05 * dt;
                    theta_cw_rel = max(theta_cw_rel, -PI/2.0);
                }

                if (iterations % 5000 == 0) {
                    time_data.push_back(iterations * dt);
                    angle_data.push_back(theta_main * 180.0 / PI);
                    velocity_data.push_back(omega_main);
                }
                if (iterations % 100000 == 0)
                    cout << "    iter=" << iterations << "  θ=" << (theta_main*180/PI)
                         << "°  ω=" << omega_main << "  β=" << (theta_cw_rel*180/PI) << "°" << endl;

                if (iterations > 100000 && abs(omega_main) < 0.001) { state = STATE_ERROR; break; }
            }

            if (state != STATE_ERROR) {
                cout << "  → Exiting LATE_SWING at θ=" << (theta_main*180/PI)
                     << "°  ω=" << omega_main << " rad/s" << endl;
                state = STATE_DEEP_SWING;
            }
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_DEEP_SWING:
        // Phase 4 → 5 in sketch: arm swings -90° → -180°
        // Arm below pivot level, sling beginning to whip
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 4: DEEP_SWING]  (θ: " << (theta_main*180/PI)
                 << "° → -180°, sling whipping)" << endl;

            while (theta_main > THRESH_4_5 && iterations < 3000000) {
                iterations++;

                double theta_cw_abs = theta_main + theta_cw_rel;
                double h_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs);
                double h_p  = r_p_eff * cos(theta_main);

                double torque_cw  = -m_cw * G * h_cw;
                double torque_p   = -m_p  * G * h_p;
                double net_torque = torque_cw + torque_p;

                double I_total = I_main_arm + I_cw_arm_total + m_p * r_p_eff * r_p_eff;

                if      (omega_main < 0) net_torque += t_fric_magnitude;
                else if (omega_main > 0) net_torque -= t_fric_magnitude;
                else {
                    if (abs(net_torque) < t_fric_magnitude) net_torque = 0;
                    else net_torque += (net_torque < 0) ? t_fric_magnitude : -t_fric_magnitude;
                }

                double alpha = net_torque / I_total;
                omega_main += alpha * dt;
                theta_main += omega_main * dt;

                // Clamp CW arm — already mostly unfolded
                if (omega_main < 0 && theta_cw_rel > -PI/2.0) {
                    theta_cw_rel -= abs(omega_main) * 0.05 * dt;
                    theta_cw_rel = max(theta_cw_rel, -PI/2.0);
                }

                if (iterations % 5000 == 0) {
                    time_data.push_back(iterations * dt);
                    angle_data.push_back(theta_main * 180.0 / PI);
                    velocity_data.push_back(omega_main);
                }
                if (iterations % 100000 == 0)
                    cout << "    iter=" << iterations << "  θ=" << (theta_main*180/PI)
                         << "°  ω=" << omega_main << endl;

                if (iterations > 100000 && abs(omega_main) < 0.001) { state = STATE_ERROR; break; }
            }

            if (state != STATE_ERROR) {
                cout << "  → Exiting DEEP_SWING at θ=" << (theta_main*180/PI)
                     << "°  ω=" << omega_main << " rad/s" << endl;
                state = STATE_FINAL_ARC;
            }
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_FINAL_ARC:
        // Phase 5 → 6 in sketch: arm swings -180° → -270°
        // Sling fully extended, maximum whip velocity building
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 5: FINAL_ARC]  (θ: " << (theta_main*180/PI)
                 << "° → -270°, sling fully extended)" << endl;

            while (theta_main > target_theta && iterations < 3000000) {
                iterations++;

                double theta_cw_abs = theta_main + theta_cw_rel;
                double h_cw = r_pivot_to_hinge * cos(theta_main) + r_hinge_to_cw_cg * cos(theta_cw_abs);
                double h_p  = r_p_eff * cos(theta_main);

                double torque_cw  = -m_cw * G * h_cw;
                double torque_p   = -m_p  * G * h_p;
                double net_torque = torque_cw + torque_p;

                double I_total = I_main_arm + I_cw_arm_total + m_p * r_p_eff * r_p_eff;

                if      (omega_main < 0) net_torque += t_fric_magnitude;
                else if (omega_main > 0) net_torque -= t_fric_magnitude;
                else {
                    if (abs(net_torque) < t_fric_magnitude) net_torque = 0;
                    else net_torque += (net_torque < 0) ? t_fric_magnitude : -t_fric_magnitude;
                }

                double alpha = net_torque / I_total;
                omega_main += alpha * dt;
                theta_main += omega_main * dt;

                if (iterations % 5000 == 0) {
                    time_data.push_back(iterations * dt);
                    angle_data.push_back(theta_main * 180.0 / PI);
                    velocity_data.push_back(omega_main);
                }
                if (iterations % 100000 == 0)
                    cout << "    iter=" << iterations << "  θ=" << (theta_main*180/PI)
                         << "°  ω=" << omega_main << endl;

                if (iterations > 100000 && abs(omega_main) < 0.001) { state = STATE_ERROR; break; }
            }

            if (state != STATE_ERROR) {
                cout << "  → Reached release point at θ=" << (theta_main*180/PI)
                     << "°  ω=" << omega_main << " rad/s" << endl;
                state = STATE_RELEASE;
            }
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_RELEASE:
        // Phase 6 in sketch: Projectile leaves sling at 43.55°
        // Compute release velocity components
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 6: RELEASE]  (sling angle = " << sling_release_angle_deg << "°)" << endl;

            v_release   = abs(omega_main) * r_p_eff;
            vx          = v_release * cos(sling_release_angle_deg * DEG2RAD);
            vy          = v_release * sin(sling_release_angle_deg * DEG2RAD);

            cout << "  Final Angular Vel   : " << omega_main   << " rad/s" << endl;
            cout << "  Release Velocity    : " << v_release    << " m/s"   << endl;
            cout << "  Horizontal Velocity : " << vx           << " m/s"   << endl;
            cout << "  Vertical Velocity   : " << vy           << " m/s"   << endl;

            state = STATE_DONE;
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_DONE:
        // Ballistic flight from 1.5 m release height
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE 7: DONE — Ballistics]" << endl;

            double h0     = 1.5;  // release height (m)
            flight_time   = (vy + sqrt(vy*vy + 2.0*G*h0)) / G;
            range         = vx * flight_time;

            cout << "\n╔══════════════════════════════════════╗" << endl;
            cout << "║     CAD-BASED WHIPPER RESULTS        ║" << endl;
            cout << "╠══════════════════════════════════════╣" << endl;
            cout << "║  Starting Angle     : " << setw(8) << (theta_main_start * 180/PI) << " deg     ║" << endl;
            cout << "║  Final Main Angle   : " << setw(8) << (theta_main       * 180/PI) << " deg     ║" << endl;
            cout << "║  Final Angular Vel  : " << setw(8) << omega_main                  << " rad/s   ║" << endl;
            cout << "║  Release Velocity   : " << setw(8) << v_release                   << " m/s     ║" << endl;
            cout << "║  Horizontal Vel     : " << setw(8) << vx                           << " m/s     ║" << endl;
            cout << "║  Vertical Vel       : " << setw(8) << vy                           << " m/s     ║" << endl;
            cout << "║  Flight Time        : " << setw(8) << flight_time                  << " s       ║" << endl;
            cout << "║  Estimated Range    : " << setw(8) << range                        << " m       ║" << endl;
            cout << "║  Simulation Time    : " << setw(8) << (iterations * dt)            << " s       ║" << endl;
            cout << "║  Total Iterations   : " << setw(8) << iterations                   << "         ║" << endl;
            cout << "╚══════════════════════════════════════╝" << endl;

            running = false;
            break;
        }

        // ══════════════════════════════════════════════════════
        case STATE_ERROR:
        // ══════════════════════════════════════════════════════
        {
            cout << "\n[STATE ERROR]" << endl;
            cout << "  Simulation halted. Angle at stop: " << (theta_main * 180/PI) << " deg" << endl;
            cout << "  Angular velocity at stop: " << omega_main << " rad/s" << endl;
            cout << "  Iterations completed: " << iterations << endl;
            running = false;
            break;
        }

        default:
            cout << "Unknown state — aborting." << endl;
            running = false;
            break;

        } // end switch
    } // end while

    return (state == STATE_ERROR) ? 1 : 0;
}
