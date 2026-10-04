// ====================================================================================================================================
// Whipper trebuchet launch simulation, v3
//
// What changed from Wipper_dynamics_State.cpp (v2):
//   1. The equations are derived again from the real geometry. The counterweight hinge is on the short end of the arm,
//      opposite the tip (v2 had it on the tip side).
//   2. The velocity (Coriolis) terms now match the kinetic energy, so the model conserves energy. v2 gained about 43 J
//      during the swing from a system that only starts with about 16 J.
//   3. The start matches the machine: both arms cocked just past the balance point, the ball resting on the arm near
//      the pivot with the sling folded back along the arm.
//   4. The counterweight arm rests on the main arm and lifts off on its own when the forces say so (v2 switched at a fixed angle).
//   5. The ball leaves the arm, falls until the sling goes tight, and the snap of the sling going tight loses a little energy.
//   6. Release is the loop slipping off the pin: it happens at a set angle between the sling and the arm.
//   7. The arm's moment of inertia is about the swing axis (see Constants_WIPPER_v3.h).
//   8. Air drag on the ball in flight.
//   9. Air drag on the machine itself while it swings (main arm, counterweight, and the ball + pouch + cords on the sling).
//      This is the biggest loss: about 2.6 J, against about 0.4 J for pin friction.
//  10. Pin friction follows the real load on each pin at every step, instead of the plain weights.
//
// Angles are absolute, measured from horizontal, counter-clockwise positive. The throw is toward +x, so the arm turns
// clockwise and the angles go down.
//      th = main arm (pivot -> tip)      ph = counterweight arm (hinge -> counterweight)      ps = sling (tip -> ball)
//
// Build:  g++ -O2 -o trebuchet_v3 Wipper_dynamics_v3.cpp
// ====================================================================================================================================
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include "Constants_WIPPER_v3.h"

using namespace std;

// ====================================================================================================================================
// Lumped constants
// ====================================================================================================================================
static const double I1  = I_arm_cg + m_arm * r_main_pivot_to_cg * r_main_pivot_to_cg;   // arm about the pivot
static const double I2  = I_cw_cg  + m_cw  * r_hinge_to_cw_cg  * r_hinge_to_cw_cg;      // counterweight about the hinge
static const double C12 = -m_cw * r_pivot_to_hinge * r_hinge_to_cw_cg;                  // arm <-> counterweight coupling (minus: hinge is behind the pivot)
static const double C13 =  m_p  * r_main_tip * sling_length;                            // arm <-> ball coupling
// Friction torque at each pin = mu * (the force the pin is carrying right now) * pin radius.
// They start at the plain weights and are updated every step from the loads worked out in main().
static double TAU_PIVOT = mu_poplar * (m_arm + m_cw + m_p) * G * pin_radius;            // friction torque at the main pivot
static double TAU_HINGE = mu_poplar * m_cw * G * pin_radius;                            // friction torque at the hinge
// Air drag on the machine
static const double K_ARM   = 0.5 * air_density * arm_drag_coeff * arm_face_width
                              * (pow(r_main_tip, 4) + pow(r_pivot_to_hinge, 4)) / 4.0;  // arm: torque = K_ARM * speed^2
static const double K_CW    = 0.5 * air_density * cw_drag_area;                         // counterweight: force = K_CW * speed^2
static const double K_SLING = 0.5 * air_density * sling_drag_area;                      // ball + pouch + cords: force = K_SLING * speed^2

enum BallState { on_arm, falling, on_sling, flight };
static const char* ballName[] = { "resting on the arm", "falling (sling slack)", "on the sling", "in flight" };

double sign(double x) { return (x > 0.0) - (x < 0.0); }

bool solve2x2(const double A[2][2], const double b[2], double x[2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabs(det) < 1e-14) return false;
    x[0] = ( A[1][1]*b[0] - A[0][1]*b[1]) / det;
    x[1] = (-A[1][0]*b[0] + A[0][0]*b[1]) / det;
    return true;
}

bool solve3x3(const double A[3][3], const double b[3], double x[3]) {
    double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
               - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
               + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    if (fabs(det) < 1e-14) return false;
    for (int col = 0; col < 3; col++) {
        double Ac[3][3];
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) Ac[r][c] = (c == col) ? b[r] : A[r][c];
        x[col] = (Ac[0][0]*(Ac[1][1]*Ac[2][2] - Ac[1][2]*Ac[2][1])
                - Ac[0][1]*(Ac[1][0]*Ac[2][2] - Ac[1][2]*Ac[2][0])
                + Ac[0][2]*(Ac[1][0]*Ac[2][1] - Ac[1][1]*Ac[2][0])) / det;
    }
    return true;
}

// ====================================================================================================================================
// Equations of motion, from the Lagrangian L = T - V
//
//   T = 0.5*M11*th'^2 + 0.5*I2*ph'^2 + 0.5*m_p*L^2*ps'^2 + C12*cos(th-ph)*th'*ph' + C13*cos(th-ps)*th'*ps'
//   V = G*( m_arm*r_cg*sin(th) + m_cw*(-r_hinge*sin(th) + r_cw*sin(ph)) + m_p*(r_tip*sin(th) + L*sin(ps)) )
//
// which gives  M * [th'' ph'' ps''] = f, with
//   f1 = -C12*sin(th-ph)*ph'^2 - C13*sin(th-ps)*ps'^2 - dV/dth  (+ friction)
//   f2 = +C12*sin(th-ph)*th'^2                         - dV/dph  (+ friction)
//   f3 = +C13*sin(th-ps)*th'^2                         - dV/dps
// ====================================================================================================================================
struct State { double th, ph, ps, w1, w2, w3; };

// Arm + counterweight. ballOnArm = the ball is riding on the arm as a small extra mass near the pivot.
// opening = the counterweight arm is resting on the main arm: hinge friction then resists it starting to swing away.
void armCw(const State& s, bool ballOnArm, double M[2][2], double f[2], bool opening = false) {
    double rb2 = ball_along_arm*ball_along_arm + ball_above_arm*ball_above_arm;
    double c   = C12 * cos(s.th - s.ph);
    M[0][0] = I1 + m_cw*r_pivot_to_hinge*r_pivot_to_hinge + (ballOnArm ? m_p*rb2 : 0.0);
    M[0][1] = c;   M[1][0] = c;   M[1][1] = I2;
    f[0] = -C12*sin(s.th - s.ph)*s.w2*s.w2 - G*(m_arm*r_main_pivot_to_cg - m_cw*r_pivot_to_hinge)*cos(s.th);
    if (ballOnArm) f[0] += -G*m_p*(ball_along_arm*cos(s.th) - ball_above_arm*sin(s.th));
    f[1] =  C12*sin(s.th - s.ph)*s.w1*s.w1 - G*m_cw*r_hinge_to_cw_cg*cos(s.ph);
    double slip = opening ? 1.0 : sign(s.w2 - s.w1);
    f[0] += -TAU_PIVOT*sign(s.w1) + TAU_HINGE*slip;
    f[1] += -TAU_HINGE*slip;
}

// Arm + counterweight + ball on the sling.
void armCwSling(const State& s, double M[3][3], double f[3], bool opening = false) {
    double a = C12 * cos(s.th - s.ph), b = C13 * cos(s.th - s.ps);
    M[0][0] = I1 + m_cw*r_pivot_to_hinge*r_pivot_to_hinge + m_p*r_main_tip*r_main_tip;
    M[0][1] = a;   M[0][2] = b;
    M[1][0] = a;   M[1][1] = I2;   M[1][2] = 0.0;
    M[2][0] = b;   M[2][1] = 0.0;  M[2][2] = m_p*sling_length*sling_length;
    f[0] = -C12*sin(s.th - s.ph)*s.w2*s.w2 - C13*sin(s.th - s.ps)*s.w3*s.w3
           - G*(m_arm*r_main_pivot_to_cg - m_cw*r_pivot_to_hinge + m_p*r_main_tip)*cos(s.th);
    f[1] =  C12*sin(s.th - s.ph)*s.w1*s.w1 - G*m_cw*r_hinge_to_cw_cg*cos(s.ph);
    f[2] =  C13*sin(s.th - s.ps)*s.w1*s.w1 - G*m_p*sling_length*cos(s.ps);
    double slip = opening ? 1.0 : sign(s.w2 - s.w1);
    f[0] += -TAU_PIVOT*sign(s.w1) + TAU_HINGE*slip;
    f[1] += -TAU_HINGE*slip;
}

// Air drag on the machine, as a push on each of the three angles (q[0] main arm, q[1] counterweight arm, q[2] sling).
void airDrag(const State& s, bool onSling, double q[3]) {
    q[0] = -K_ARM * s.w1 * fabs(s.w1);                                               // main arm, both sides of the pivot
    // counterweight: how its centre moves with th and with ph, then its speed
    double ax = r_pivot_to_hinge*sin(s.th), ay = -r_pivot_to_hinge*cos(s.th);
    double cx = -r_hinge_to_cw_cg*sin(s.ph), cy = r_hinge_to_cw_cg*cos(s.ph);
    double vx = ax*s.w1 + cx*s.w2, vy = ay*s.w1 + cy*s.w2, sp = sqrt(vx*vx + vy*vy);
    q[0] += -K_CW*sp*(vx*ax + vy*ay);
    q[1]  = -K_CW*sp*(vx*cx + vy*cy);
    q[2]  = 0.0;
    if (onSling) {                                                                   // ball + pouch + cords
        double gx = -r_main_tip*sin(s.th), gy = r_main_tip*cos(s.th), hx = -sling_length*sin(s.ps), hy = sling_length*cos(s.ps);
        double ux = gx*s.w1 + hx*s.w3, uy = gy*s.w1 + hy*s.w3, su = sqrt(ux*ux + uy*uy);
        q[0] += -K_SLING*su*(ux*gx + uy*gy);
        q[2]  = -K_SLING*su*(ux*hx + uy*hy);
    }
}

// Angular accelerations for the current situation.
// cwOnArm = the counterweight arm is resting against the main arm, so the two turn together.
// Sets cwLiftsOff when the counterweight arm would swing away from the main arm on its own (it has to beat the
// hinge friction to do that, which is what stops it rattling on and off the arm).
bool accelerations(const State& s, BallState ball, bool cwOnArm, double a[3], bool& cwLiftsOff) {
    cwLiftsOff = false;
    if (ball == on_sling) {
        double M[3][3], f[3], fr[3];
        armCwSling(s, M, f, cwOnArm);
        double q[3];  airDrag(s, true, q);  f[0] += q[0];  f[1] += q[1];  f[2] += q[2];
        if (!solve3x3(M, f, fr)) return false;
        if (cwOnArm && fr[1] - fr[0] <= 0.0) {          // still pressed together: th'' = ph''
            double Mr[2][2] = { { M[0][0] + 2.0*M[0][1] + M[1][1], M[0][2] + M[1][2] }, { M[0][2] + M[1][2], M[2][2] } };
            double br[2] = { f[0] + f[1], f[2] }, x[2];
            if (!solve2x2(Mr, br, x)) return false;
            a[0] = x[0]; a[1] = x[0]; a[2] = x[1];
        } else {
            cwLiftsOff = cwOnArm;
            a[0] = fr[0]; a[1] = fr[1]; a[2] = fr[2];
        }
    } else {
        double M[2][2], f[2], fr[2];
        armCw(s, ball == on_arm, M, f, cwOnArm);
        double q[3];  airDrag(s, false, q);  f[0] += q[0];  f[1] += q[1];
        if (!solve2x2(M, f, fr)) return false;
        if (cwOnArm && fr[1] - fr[0] <= 0.0) {
            a[0] = a[1] = (f[0] + f[1]) / (M[0][0] + 2.0*M[0][1] + M[1][1]);
        } else {
            cwLiftsOff = cwOnArm;
            a[0] = fr[0]; a[1] = fr[1];
        }
        a[2] = 0.0;
    }
    return true;
}

// Horizontal position of the cocked assembly's centre of mass (times its mass). Zero at the balance angle.
double cockedMomentArm(double th) {
    return m_arm*r_main_pivot_to_cg*cos(th)
         + m_cw*(-r_pivot_to_hinge*cos(th) + r_hinge_to_cw_cg*cos(th + theta_cw_rel_start))
         + m_p*(ball_along_arm*cos(th) - ball_above_arm*sin(th));
}

double balanceAngle() {
    double lo = 60.0*DEG, hi = 120.0*DEG;
    for (int i = 0; i < 60; i++) {
        double mid = 0.5*(lo + hi);
        if (cockedMomentArm(mid) > 0.0) lo = mid; else hi = mid;
    }
    return 0.5*(lo + hi);
}

// Mechanical energy of everything that is moving (ball position/velocity passed in separately).
double energy(const State& s, double by, double bvx, double bvy) {
    double M[2][2], f[2];
    armCw(s, false, M, f);
    double T = 0.5*M[0][0]*s.w1*s.w1 + M[0][1]*s.w1*s.w2 + 0.5*M[1][1]*s.w2*s.w2 + 0.5*m_p*(bvx*bvx + bvy*bvy);
    double V = G*(m_arm*r_main_pivot_to_cg*sin(s.th) + m_cw*(-r_pivot_to_hinge*sin(s.th) + r_hinge_to_cw_cg*sin(s.ph)) + m_p*by);
    return T + V;
}

// Ball flight from the release point to the floor. Returns the horizontal distance from the pivot.
double flyBall(double x, double y, double vx, double vy, bool drag, double& maxHeight) {
    double k = drag ? 0.5*air_density*ball_drag_coeff*PI*ball_diameter*ball_diameter/4.0 : 0.0;
    double h = 0.0002;
    maxHeight = y;
    while (y > 0.0) {
        double sp = sqrt(vx*vx + vy*vy);
        vx += (-k*sp*vx/m_p) * h;
        vy += (-G - k*sp*vy/m_p) * h;
        x  += vx*h;   y += vy*h;
        maxHeight = max(maxHeight, y);
    }
    return x;
}

int main() {
    // ---- start position ----
    double th_balance = balanceAngle();
    State s;
    s.th = th_balance - start_past_balance_deg*DEG;
    s.ph = s.th + theta_cw_rel_start;
    s.ps = 0.0;  s.w1 = s.w2 = s.w3 = 0.0;

    bool      cwOnArm = true, cwReported = false;
    BallState ball    = on_arm;
    double bx = 0, by = 0, bvx = 0, bvy = 0;      // ball position (from the pivot) and velocity
    double t = 0.0;

    // ball's resting spot, to start the energy book-keeping
    bx = ball_along_arm*cos(s.th) - ball_above_arm*sin(s.th);
    by = ball_along_arm*sin(s.th) + ball_above_arm*cos(s.th);
    const double E_start = energy(s, by, 0.0, 0.0);

    double tautLoss = 0.0, airLoss = 0.0;
    double maxPivotForce = 0.0, maxHingeForce = 0.0, maxSlingTension = 0.0;
    double launch_x = 0, launch_y = 0, launch_vx = 0, launch_vy = 0, E_release = 0;

    cout << fixed << setprecision(3);
    cout << "=== WHIPPER TREBUCHET SIMULATION v3 ===" << endl;
    cout << "Balance angle        : " << th_balance/DEG << " deg" << endl;
    cout << "Arm start angle      : " << s.th/DEG << " deg  (" << start_past_balance_deg << " deg past balance)" << endl;
    cout << "Arm inertia at pivot : " << I1 << " kg*m^2" << endl;
    cout << "CW inertia at hinge  : " << I2 << " kg*m^2" << endl;
    cout << "Release angle        : " << release_angle_deg << " deg between sling and arm" << endl << endl;

    while (t < t_max && ball != flight) {
        // ---- accelerations for this step ----
        double a[3];
        bool lifts = false;
        if (!accelerations(s, ball, cwOnArm, a, lifts)) { cout << "Singular matrix at t=" << t << endl; return 1; }
        if (lifts) {
            cwOnArm = false;
            if (!cwReported) cout << "t=" << t << " s  counterweight arm swings away from the main arm (arm at " << s.th/DEG << " deg)" << endl;
            cwReported = true;
        }

        // ---- has the ball left the arm? (the arm's surface is dropping away faster than the ball can follow) ----
        if (ball == on_arm) {
            double rx = ball_along_arm*cos(s.th) - ball_above_arm*sin(s.th);
            double ry = ball_along_arm*sin(s.th) + ball_above_arm*cos(s.th);
            double nx = -sin(s.th), ny = cos(s.th);                         // out of the arm's top face
            double ax = a[0]*(-ry) - s.w1*s.w1*rx, ay = a[0]*rx - s.w1*s.w1*ry;
            if (m_p*(ax*nx + ay*ny + G*ny) < 0.0) {
                ball = falling;  bx = rx;  by = ry;  bvx = -s.w1*ry;  bvy = s.w1*rx;
                cout << "t=" << t << " s  ball leaves the arm (arm at " << s.th/DEG << " deg)" << endl;
                continue;
            }
            bx = rx; by = ry; bvx = -s.w1*ry; bvy = s.w1*rx;
        }

        // ---- loads on the pins (for the frame design, and they set the friction for the next step) ----
        {
            double aax = r_main_pivot_to_cg*(-a[0]*sin(s.th) - s.w1*s.w1*cos(s.th));
            double aay = r_main_pivot_to_cg*( a[0]*cos(s.th) - s.w1*s.w1*sin(s.th));
            double acx = -r_pivot_to_hinge*(-a[0]*sin(s.th) - s.w1*s.w1*cos(s.th)) + r_hinge_to_cw_cg*(-a[1]*sin(s.ph) - s.w2*s.w2*cos(s.ph));
            double acy = -r_pivot_to_hinge*( a[0]*cos(s.th) - s.w1*s.w1*sin(s.th)) + r_hinge_to_cw_cg*( a[1]*cos(s.ph) - s.w2*s.w2*sin(s.ph));
            double Fx = m_arm*aax + m_cw*acx, Fy = m_arm*(aay + G) + m_cw*(acy + G);
            double hingeForce = m_cw*sqrt(acx*acx + (acy + G)*(acy + G));
            maxHingeForce = max(maxHingeForce, hingeForce);
            if (ball == on_sling) {
                double apx = r_main_tip*(-a[0]*sin(s.th) - s.w1*s.w1*cos(s.th)) + sling_length*(-a[2]*sin(s.ps) - s.w3*s.w3*cos(s.ps));
                double apy = r_main_tip*( a[0]*cos(s.th) - s.w1*s.w1*sin(s.th)) + sling_length*( a[2]*cos(s.ps) - s.w3*s.w3*sin(s.ps));
                Fx += m_p*apx;  Fy += m_p*(apy + G);
                maxSlingTension = max(maxSlingTension, m_p*sqrt(apx*apx + (apy + G)*(apy + G)));
            }
            double pivotForce = sqrt(Fx*Fx + Fy*Fy);
            maxPivotForce = max(maxPivotForce, pivotForce);
            TAU_PIVOT = mu_poplar * pivotForce * pin_radius;
            TAU_HINGE = mu_poplar * hingeForce * pin_radius;
        }

        // ---- step forward (semi-implicit Euler: speeds first, then angles) ----
        { double q[3];  airDrag(s, ball == on_sling, q);  airLoss += -(q[0]*s.w1 + q[1]*s.w2 + q[2]*s.w3)*dt; }
        s.w1 += a[0]*dt;  s.w2 += a[1]*dt;  s.w3 += a[2]*dt;
        s.th += s.w1*dt;  s.ph += s.w2*dt;  s.ps += s.w3*dt;
        t += dt;

        // ---- the counterweight arm swings back onto the main arm: they stick together again ----
        if (!cwOnArm && s.ph - s.th <= theta_cw_rel_start && s.w2 - s.w1 < 0.0) {
            double M[2][2], f[2];
            armCw(s, ball == on_arm, M, f);
            double w = (M[0][0]*s.w1 + M[0][1]*(s.w1 + s.w2) + M[1][1]*s.w2) / (M[0][0] + 2.0*M[0][1] + M[1][1]);
            if (ball == on_sling) {   // share the momentum with the ball too
                double M3[3][3], f3[3];
                armCwSling(s, M3, f3);
                double Mr[2][2] = { { M3[0][0] + 2.0*M3[0][1] + M3[1][1], M3[0][2] }, { M3[0][2], M3[2][2] } };
                double pr[2] = { (M3[0][0] + M3[0][1])*s.w1 + (M3[0][1] + M3[1][1])*s.w2 + M3[0][2]*s.w3, M3[0][2]*s.w1 + M3[2][2]*s.w3 }, x[2];
                solve2x2(Mr, pr, x);
                w = x[0];  s.w3 = x[1];
            }
            s.w1 = s.w2 = w;  s.ph = s.th + theta_cw_rel_start;  cwOnArm = true;
        }

        // ---- ball falling with the sling slack ----
        if (ball == falling) {
            bvy -= G*dt;  bx += bvx*dt;  by += bvy*dt;
            double tx = r_main_tip*cos(s.th), ty = r_main_tip*sin(s.th);
            double dx = bx - tx, dy = by - ty;
            if (sqrt(dx*dx + dy*dy) >= sling_length) {
                // The sling snaps tight. The ball can no longer move along the sling's length, so that part of its motion
                // is lost. The new speeds are the ones that keep as much of the old momentum as the tight sling allows.
                double E_before = energy(s, by, bvx, bvy);
                s.ps = atan2(dy, dx);
                double M[2][2], f[2];
                armCw(s, false, M, f);
                double gx = -r_main_tip*sin(s.th), gy = r_main_tip*cos(s.th);            // how the ball moves with th
                double hx = -sling_length*sin(s.ps), hy = sling_length*cos(s.ps);        // how the ball moves with ps
                double A[3][3] = { { M[0][0] + m_p*(gx*gx + gy*gy), M[0][1], m_p*(gx*hx + gy*hy) },
                                   { M[1][0],                       M[1][1], 0.0                 },
                                   { m_p*(gx*hx + gy*hy),           0.0,     m_p*(hx*hx + hy*hy) } };
                double b[3] = { M[0][0]*s.w1 + M[0][1]*s.w2 + m_p*(gx*bvx + gy*bvy),
                                M[1][0]*s.w1 + M[1][1]*s.w2,
                                m_p*(hx*bvx + hy*bvy) }, q[3];
                solve3x3(A, b, q);
                s.w1 = q[0];  s.w2 = q[1];  s.w3 = q[2];
                ball = on_sling;
                bx = tx + sling_length*cos(s.ps);  by = ty + sling_length*sin(s.ps);
                bvx = gx*s.w1 + hx*s.w3;           bvy = gy*s.w1 + hy*s.w3;
                tautLoss = E_before - energy(s, by, bvx, bvy);
                cout << "t=" << t << " s  sling goes tight (arm at " << s.th/DEG << " deg), " << tautLoss << " J lost in the snap" << endl;
            }
        }

        // ---- ball on the sling: has the loop slipped off the pin? ----
        if (ball == on_sling) {
            bx  = r_main_tip*cos(s.th) + sling_length*cos(s.ps);
            by  = r_main_tip*sin(s.th) + sling_length*sin(s.ps);
            bvx = -r_main_tip*sin(s.th)*s.w1 - sling_length*sin(s.ps)*s.w3;
            bvy =  r_main_tip*cos(s.th)*s.w1 + sling_length*cos(s.ps)*s.w3;
            double rel = fmod(s.ps - s.th + PI, 2.0*PI);
            if (rel < 0.0) rel += 2.0*PI;
            rel -= PI;                                                    // sling angle from the arm line, -180..180 deg
            if (rel <= release_angle_deg*DEG && rel > -35.0*DEG && bvx > 0.0 && bvy > 0.0) {
                ball = flight;
                launch_x = bx;  launch_y = by + pivot_height;  launch_vx = bvx;  launch_vy = bvy;
                E_release = energy(s, by, bvx, bvy);
                cout << "t=" << t << " s  PROJECTILE RELEASED (arm at " << s.th/DEG << " deg)" << endl;
            }
        }
    }

    if (ball != flight) {
        cout << endl << "The ball was never released. It ended " << ballName[ball] << ". Check the start and release angles." << endl;
        return 1;
    }

    // ---- results ----
    double speed = sqrt(launch_vx*launch_vx + launch_vy*launch_vy);
    double angle = atan2(launch_vy, launch_vx)/DEG;
    double h_drag, h_vac;
    double range_drag = flyBall(launch_x, launch_y, launch_vx, launch_vy, true,  h_drag);
    double range_vac  = flyBall(launch_x, launch_y, launch_vx, launch_vy, false, h_vac);
    double ke_ball = 0.5*m_p*speed*speed;

    cout << endl;
    cout << "=====================================================" << endl;
    cout << "         TREBUCHET SIMULATION RESULTS (v3)           " << endl;
    cout << "=====================================================" << endl;
    cout << setprecision(2);
    cout << "PROJECTILE" << endl;
    cout << "  Release time     : " << setprecision(3) << t << " s" << setprecision(2) << endl;
    cout << "  Launch position  : (" << launch_x << ", " << launch_y << ") m  (x from the pivot, y above the floor)" << endl;
    cout << "  Launch speed     : " << speed << " m/s" << endl;
    cout << "  Launch angle     : " << angle << " deg above horizontal" << endl;
    cout << "  Range, with drag : " << range_drag << " m  (" << range_drag*3.28084 << " ft),  max height " << h_drag << " m" << endl;
    cout << "  Range, no drag   : " << range_vac  << " m  (" << range_vac*3.28084  << " ft),  max height " << h_vac  << " m" << endl;
    cout << endl;
    cout << "ENERGY" << endl;
    cout << "  Ball at release  : " << ke_ball << " J of kinetic energy" << endl;
    cout << "  Lost when the sling snapped tight : " << tautLoss << " J" << endl;
    cout << "  Lost to air drag on the machine   : " << airLoss << " J  (arm, counterweight, sling)" << endl;
    cout << "  Lost to pivot + hinge friction    : " << (E_start - E_release - tautLoss - airLoss) << " J" << endl;
    cout << "  (start " << E_start << " J, at release " << E_release << " J, measured from the pivot height)" << endl;
    cout << endl;
    cout << "PEAK LOADS (up to release)" << endl;
    cout << "  Main pivot       : " << maxPivotForce   << " N" << endl;
    cout << "  CW hinge         : " << maxHingeForce   << " N" << endl;
    cout << "  Sling tension    : " << maxSlingTension << " N" << endl;
    cout << "=====================================================" << endl;
    return 0;
}
