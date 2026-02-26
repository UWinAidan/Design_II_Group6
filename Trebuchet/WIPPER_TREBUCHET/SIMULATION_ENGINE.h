#ifndef SIMULATION_ENGINE_H
#define SIMULATION_ENGINE_H

#include <cmath>
#include <vector>

class TrebuchetSimulation {
public:
    // Constants
    static constexpr double G = 9.81;
    static constexpr double PI = 3.14159265359;

    // Geometry
    double r_main_pivot_to_cg = 0.3825;
    double r_pivot_to_hinge = 0.175;
    double r_hinge_to_cw_cg = 0.6916;
    double r_main_tip = 0.9625;
    double sling_length = 0.9;

    // Masses
    double m_arm = 0.167;
    double m_cw = 1.016;
    double m_p = 0.023;

    // Inertia
    double I_main_arm;
    double I_cw_arm_total;

    // Starting angles (radians)
    double theta_main_start = 77.865 * (PI / 180.0);
    double theta_cw_rel_start = 4.12 * (PI / 180.0);

    // Release parameters
    double release_arm_angle_deg = 90.0;
    double sling_release_angle_deg = 43.55;

    // Friction
    double mu_poplar = 0.35;
    double pin_radius = 0.00635;

    // Simulation state
    double theta_main;
    double theta_cw_rel;
    double omega_main = 0.0;
    double dt = 0.0001;
    double target_theta;

    int iterations = 0;
    bool is_released = false;
    bool has_stalled = false;

    // Results
    double v_release = 0.0;
    double vx = 0.0, vy = 0.0;
    double range = 0.0;

    // Projectile trajectory
    struct ProjectileState {
        double x, y;  // Position relative to launch point
        double vx, vy; // Velocity
        double time;
    };
    std::vector<ProjectileState> trajectory;

    TrebuchetSimulation() {
        InitializeConstants();
    }

    void InitializeConstants() {
        I_main_arm = 0.000005859 + m_arm * r_main_pivot_to_cg * r_main_pivot_to_cg;
        I_cw_arm_total = 0.03467282 + m_cw * r_hinge_to_cw_cg * r_hinge_to_cw_cg;

        theta_main = theta_main_start;
        theta_cw_rel = theta_cw_rel_start;
        target_theta = (release_arm_angle_deg - 360) * (PI / 180.0);
    }

    void Step() {
        if (is_released || has_stalled) return;

        double theta_cw_abs = theta_main + theta_cw_rel;
        double r_p_eff = r_main_tip + sling_length;

        double horizontal_dist_cw = r_pivot_to_hinge * cos(theta_main) + 
                                     r_hinge_to_cw_cg * cos(theta_cw_abs);
        double horizontal_dist_p = r_p_eff * cos(theta_main);

        double torque_cw_gravity = -m_cw * G * horizontal_dist_cw;
        double torque_p_gravity = -m_p * G * horizontal_dist_p;

        double m_total = m_arm + m_cw + m_p;
        double t_fric_magnitude = mu_poplar * m_total * G * pin_radius;
        double I_total = I_main_arm + I_cw_arm_total + (m_p * r_p_eff * r_p_eff);

        double net_torque = torque_cw_gravity + torque_p_gravity;

        // Friction model
        if (omega_main < 0) {
            net_torque += t_fric_magnitude;
        } else if (omega_main > 0) {
            net_torque -= t_fric_magnitude;
        } else {
            if (fabs(net_torque) < t_fric_magnitude) {
                net_torque = 0;
                has_stalled = true;
                return;
            } else {
                net_torque = (net_torque < 0) ? net_torque + t_fric_magnitude : 
                                                 net_torque - t_fric_magnitude;
            }
        }

        double alpha = net_torque / I_total;
        omega_main += alpha * dt;
        theta_main += omega_main * dt;

        // Secondary arm unfolding
        if (omega_main < 0 && theta_main < theta_main_start && theta_main > target_theta) {
            theta_cw_rel -= fabs(omega_main) * 0.05 * dt;
            if (theta_cw_rel < -PI/2) theta_cw_rel = -PI/2;
            if (theta_cw_rel > PI/2) theta_cw_rel = PI/2;
        }

        iterations++;

        // Check release condition
        if (theta_main <= target_theta && !is_released) {
            ReleaseProjectile();
        }
    }

    void ReleaseProjectile() {
        is_released = true;
        double r_p_eff = r_main_tip + sling_length;
        v_release = fabs(omega_main) * r_p_eff;
        vx = v_release * cos(sling_release_angle_deg * PI / 180.0);
        vy = v_release * sin(sling_release_angle_deg * PI / 180.0);

        // Calculate trajectory
        double release_height = 1.5;
        double t_max = (vy + sqrt(vy * vy + 2.0 * G * release_height)) / G;
        int steps = 200;
        for (int i = 0; i <= steps; i++) {
            double t = (t_max / steps) * i;
            ProjectileState state;
            state.time = t;
            state.x = vx * t;
            state.y = release_height + vy * t - 0.5 * G * t * t;
            state.vx = vx;
            state.vy = vy - G * t;
            trajectory.push_back(state);
            if (state.y < 0) break;
        }
        range = vx * t_max;
    }

    bool IsSimulationRunning() const {
        return !is_released && !has_stalled && iterations < 3000000;
    }
};

#endif
