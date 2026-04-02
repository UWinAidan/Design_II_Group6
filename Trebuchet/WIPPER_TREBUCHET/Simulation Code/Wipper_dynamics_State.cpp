#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include "Constants_WIPPER.h"

using namespace std;

// ====================================================================================================================================
// Lagrangian constants
// ====================================================================================================================================
static double s_IA, s_IB, s_IC, s_mD, s_mE;
static double s_tau_fric_pivot, s_tau_fric_hinge;

// ====================================================================================================================================
// Helpers
// ====================================================================================================================================
double sign(double x) {
    if (x > 0.0) return  1.0;
    if (x < 0.0) return -1.0;
    return 0.0;
}

bool solve3x3(double A[3][3], double b[3], double x[3]) {
    double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
               - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
               + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    if (fabs(det) < 1e-12) return false;
    double inv = 1.0 / det;
    for (int col = 0; col < 3; col++) {
        double Ac[3][3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                Ac[r][c] = (c == col) ? b[r] : A[r][c];
        double d = Ac[0][0]*(Ac[1][1]*Ac[2][2] - Ac[1][2]*Ac[2][1])
                 - Ac[0][1]*(Ac[1][0]*Ac[2][2] - Ac[1][2]*Ac[2][0])
                 + Ac[0][2]*(Ac[1][0]*Ac[2][1] - Ac[1][1]*Ac[2][0]);
        x[col] = d * inv;
    }
    return true;
}

bool solve2x2(double A[2][2], double b[2], double x[2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabs(det) < 1e-12) return false;
    double inv = 1.0 / det;
    x[0] = ( A[1][1]*b[0] - A[0][1]*b[1]) * inv;
    x[1] = (-A[1][0]*b[0] + A[0][0]*b[1]) * inv;
    return true;
}

// ====================================================================================================================================
// Adaptive dt: keep angular step below MAX_DTHETA radians per step
// ====================================================================================================================================
static const double MAX_DTHETA = 0.001;    // ~0.057 deg per step
static const double DT_MIN     = 1e-8;
static const double DT_MAX     = 0.0001;

double adaptiveDt(double omega) {
    double om = fabs(omega);
    if (om < 1e-6) return DT_MAX;
    double h = MAX_DTHETA / om;
    if (h < DT_MIN) h = DT_MIN;
    if (h > DT_MAX) h = DT_MAX;
    return h;
}

// ====================================================================================================================================
// 3-DOF accelerations  — CORRECTED Coriolis terms
//
// EOM derived from Lagrangian L = T - V:
//
// T = 0.5*IA*dq1^2 + 0.5*IB*(dq1+dq2)^2 + 0.5*IC*(dq1+dq3)^2
//   + mD*cos(q2)*dq1*(dq1+dq2) + mE*cos(q3)*dq1*(dq1+dq3)
//
// EOM1 RHS: mD*sin(q2)*dq2*(2*dq1+dq2) + mE*sin(q3)*dq3*(2*dq1+dq3)
// EOM2 RHS: mD*sin(q2)*dq1*(dq1+2*dq2)    <-- was dq1^2, now FIXED
// EOM3 RHS: mE*sin(q3)*dq1*(dq1+2*dq3)    <-- was dq1^2, now FIXED
// ====================================================================================================================================
bool computeAccel3(double q1, double q2, double q3,
                   double dq1, double dq2, double dq3,
                   double ddq[3], bool with_friction)
{
    double c2    = cos(q2),  s2 = sin(q2);
    double c3    = cos(q3),  s3 = sin(q3);
    double q2abs = q1 + q2,  q3abs = q1 + q3;

    double A[3][3] = {};
    A[0][0] = s_IA + s_IB + s_IC + 2.0*s_mD*c2 + 2.0*s_mE*c3;
    A[0][1] = s_IB + s_mD*c2;   A[1][0] = A[0][1];
    A[0][2] = s_IC + s_mE*c3;   A[2][0] = A[0][2];
    A[1][1] = s_IB;
    A[1][2] = 0.0;               A[2][1] = 0.0;
    A[2][2] = s_IC;

    // ---- CORRECTED Coriolis/centrifugal terms ----
    double cor1 = s_mD*s2*dq2*(2.0*dq1 + dq2)
                + s_mE*s3*dq3*(2.0*dq1 + dq3);
    double cor2 = s_mD*s2*dq1*(dq1 + 2.0*dq2);   // FIXED (was: mD*s2*dq1^2)
    double cor3 = s_mE*s3*dq1*(dq1 + 2.0*dq3);   // FIXED (was: mE*s3*dq1^2)

    double grav1 = -G*(m_arm*r_main_pivot_to_cg*cos(q1)
                     + m_cw*(r_pivot_to_hinge*cos(q1) + r_hinge_to_cw_cg*cos(q2abs))
                     + m_p *(r_main_tip*cos(q1)        + sling_length*cos(q3abs)));
    double grav2 = -G*m_cw*r_hinge_to_cw_cg*cos(q2abs);
    double grav3 = -G*m_p*sling_length*cos(q3abs);

    double Q1 = with_friction ? -sign(dq1)*s_tau_fric_pivot : 0.0;
    double Q2 = with_friction ? -sign(dq2)*s_tau_fric_hinge : 0.0;

    double b[3] = { cor1+grav1+Q1, cor2+grav2+Q2, cor3+grav3 };
    return solve3x3(A, b, ddq);
}

// ====================================================================================================================================
// 2-DOF accelerations (post-release, no projectile)
// EOM2 RHS correction also applied here:
// cor1 = mD*sin(q2)*dq2*(2*dq1+dq2)
// cor2 = mD*sin(q2)*dq1*(dq1+2*dq2)   FIXED
// ====================================================================================================================================
bool computeAccel2(double q1, double q2,
                   double dq1, double dq2,
                   double ddq[2], bool with_friction,
                   double tau_fric_piv_free)
{
    double c2    = cos(q2),  s2 = sin(q2);
    double q2abs = q1 + q2;
    double IA_free = I_main_arm + m_cw*r_pivot_to_hinge*r_pivot_to_hinge;

    double A[2][2] = {};
    A[0][0] = IA_free + s_IB + 2.0*s_mD*c2;
    A[0][1] = s_IB + s_mD*c2;   A[1][0] = A[0][1];
    A[1][1] = s_IB;

    double cor1 = s_mD*s2*dq2*(2.0*dq1 + dq2);   // FIXED
    double cor2 = s_mD*s2*dq1*(dq1 + 2.0*dq2);   // FIXED

    double grav1 = -G*(m_arm*r_main_pivot_to_cg*cos(q1)
                     + m_cw*(r_pivot_to_hinge*cos(q1) + r_hinge_to_cw_cg*cos(q2abs)));
    double grav2 = -G*m_cw*r_hinge_to_cw_cg*cos(q2abs);

    double Q1 = with_friction ? -sign(dq1)*tau_fric_piv_free : 0.0;
    double Q2 = with_friction ? -sign(dq2)*s_tau_fric_hinge  : 0.0;

    double b[2] = { cor1+grav1+Q1, cor2+grav2+Q2 };
    return solve2x2(A, b, ddq);
}

// ====================================================================================================================================
// RK4 integrators
// ====================================================================================================================================
bool rk4_3dof(double state[6], double h, bool with_friction)
{
    auto deriv = [&](const double s[6], double ds[6]) -> bool {
        ds[0] = s[3]; ds[1] = s[4]; ds[2] = s[5];
        double ddq[3] = {};
        if (!computeAccel3(s[0],s[1],s[2],s[3],s[4],s[5],ddq,with_friction)) return false;
        ds[3] = ddq[0]; ds[4] = ddq[1]; ds[5] = ddq[2];
        return true;
    };
    double k1[6]={}, k2[6]={}, k3[6]={}, k4[6]={};
    double s2[6]={}, s3[6]={}, s4[6]={};
    if (!deriv(state,k1)) return false;
    for (int i=0;i<6;i++) s2[i]=state[i]+0.5*h*k1[i];
    if (!deriv(s2,k2))   return false;
    for (int i=0;i<6;i++) s3[i]=state[i]+0.5*h*k2[i];
    if (!deriv(s3,k3))   return false;
    for (int i=0;i<6;i++) s4[i]=state[i]+h*k3[i];
    if (!deriv(s4,k4))   return false;
    for (int i=0;i<6;i++) state[i]+=(h/6.0)*(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i]);
    return true;
}

bool rk4_2dof(double state[4], double h, bool with_friction, double tau_fric_piv_free)
{
    auto deriv = [&](const double s[4], double ds[4]) -> bool {
        ds[0] = s[2]; ds[1] = s[3];
        double ddq[2] = {};
        if (!computeAccel2(s[0],s[1],s[2],s[3],ddq,with_friction,tau_fric_piv_free)) return false;
        ds[2] = ddq[0]; ds[3] = ddq[1];
        return true;
    };
    double k1[4]={}, k2[4]={}, k3[4]={}, k4[4]={};
    double s2[4]={}, s3[4]={}, s4[4]={};
    if (!deriv(state,k1)) return false;
    for (int i=0;i<4;i++) s2[i]=state[i]+0.5*h*k1[i];
    if (!deriv(s2,k2))   return false;
    for (int i=0;i<4;i++) s3[i]=state[i]+0.5*h*k2[i];
    if (!deriv(s3,k3))   return false;
    for (int i=0;i<4;i++) s4[i]=state[i]+h*k3[i];
    if (!deriv(s4,k4))   return false;
    for (int i=0;i<4;i++) state[i]+=(h/6.0)*(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i]);
    return true;
}

bool rk4_fused(double state[2], double h)
{
    auto deriv = [&](const double s[2], double ds[2]) -> void {
        ds[0] = s[1];
        double tA = -m_arm*G*r_main_pivot_to_cg*cos(s[0]);
        double tC = -m_cw*G*(r_pivot_to_hinge*cos(s[0])
                            +r_hinge_to_cw_cg*cos(s[0]+theta_cw_rel_start));
        double tP = -m_p*G*r_main_tip*cos(s[0]);
        double tF = -sign(s[1])*s_tau_fric_pivot;
        ds[1] = (tA+tC+tP+tF)/I_fused;
    };
    double k1[2]={}, k2[2]={}, k3[2]={}, k4[2]={};
    double s2[2]={}, s3[2]={}, s4[2]={};
    deriv(state,k1);
    for (int i=0;i<2;i++) s2[i]=state[i]+0.5*h*k1[i];
    deriv(s2,k2);
    for (int i=0;i<2;i++) s3[i]=state[i]+0.5*h*k2[i];
    deriv(s3,k3);
    for (int i=0;i<2;i++) s4[i]=state[i]+h*k3[i];
    deriv(s4,k4);
    for (int i=0;i<2;i++) state[i]+=(h/6.0)*(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i]);
    return true;
}

// ====================================================================================================================================
// Event bisection: land 3-DOF state exactly at q1 == event_angle
// ====================================================================================================================================
double stepToEvent3(double state[6], double event_angle,
                    bool with_friction, double tol=1e-7)
{
    double s_before[6];
    for (int i=0;i<6;i++) s_before[i]=state[i];

    double h_lo=0.0, h_hi=adaptiveDt(state[3]);

    // Confirm crossing happens within h_hi
    double s_test[6];
    for (int i=0;i<6;i++) s_test[i]=s_before[i];
    rk4_3dof(s_test,h_hi,with_friction);
    if (s_test[0] > event_angle) {
        // Crossing not within one step — just take the step normally
        for (int i=0;i<6;i++) state[i]=s_test[i];
        return h_hi;
    }

    // Bisect
    for (int iter=0;iter<80;iter++) {
        double h_mid=0.5*(h_lo+h_hi);
        for (int i=0;i<6;i++) s_test[i]=s_before[i];
        rk4_3dof(s_test,h_mid,with_friction);
        if (s_test[0] > event_angle) h_lo=h_mid;
        else                          h_hi=h_mid;
        if (h_hi-h_lo < tol*1e-4) break;
    }

    double h_final=h_lo;
    for (int i=0;i<6;i++) state[i]=s_before[i];
    rk4_3dof(state,h_final,with_friction);
    return h_final;
}

// ====================================================================================================================================
// Reaction forces
// ====================================================================================================================================
void computeReactionForces(
    double q1,   double q2,   double q3abs,
    double dq1,  double dq2,  double dq3abs,
    double ddq1, double ddq2, double ddq3abs,
    bool   sling_attached,
    double& pivot_Fx, double& pivot_Fy,
    double& hinge_Fx, double& hinge_Fy)
{
    double q2abs   = q1  + q2;
    double dq2abs  = dq1 + dq2;
    double ddq2abs = ddq1 + ddq2;

    double ax_cw = -r_pivot_to_hinge*sin(q1)*ddq1
                   -r_pivot_to_hinge*cos(q1)*dq1*dq1
                   -r_hinge_to_cw_cg*sin(q2abs)*ddq2abs
                   -r_hinge_to_cw_cg*cos(q2abs)*dq2abs*dq2abs;
    double ay_cw =  r_pivot_to_hinge*cos(q1)*ddq1
                   -r_pivot_to_hinge*sin(q1)*dq1*dq1
                   +r_hinge_to_cw_cg*cos(q2abs)*ddq2abs
                   -r_hinge_to_cw_cg*sin(q2abs)*dq2abs*dq2abs;
    hinge_Fx = m_cw * ax_cw;
    hinge_Fy = m_cw * (ay_cw + G);

    double ax_arm = r_main_pivot_to_cg*(-ddq1*sin(q1) - dq1*dq1*cos(q1));
    double ay_arm = r_main_pivot_to_cg*( ddq1*cos(q1) - dq1*dq1*sin(q1));

    double ax_p=0.0, ay_p=0.0;
    if (sling_attached) {
        ax_p = -r_main_tip*sin(q1)*ddq1
               -r_main_tip*cos(q1)*dq1*dq1
               -sling_length*sin(q3abs)*ddq3abs
               -sling_length*cos(q3abs)*dq3abs*dq3abs;
        ay_p =  r_main_tip*cos(q1)*ddq1
               -r_main_tip*sin(q1)*dq1*dq1
               +sling_length*cos(q3abs)*ddq3abs
               -sling_length*sin(q3abs)*dq3abs*dq3abs;
    }
    double mp_eff = sling_attached ? m_p : 0.0;
    pivot_Fx = m_arm*ax_arm + mp_eff*ax_p + hinge_Fx;
    pivot_Fy = m_arm*ay_arm + mp_eff*ay_p + (m_arm+mp_eff)*G + hinge_Fy;
}

// ====================================================================================================================================
int main() {
// ====================================================================================================================================

    s_IA = I_main_arm
         + m_cw*r_pivot_to_hinge*r_pivot_to_hinge
         + m_p *r_main_tip*r_main_tip;
    s_IB = I_cw_arm_total;
    s_IC = m_p *sling_length*sling_length;
    s_mD = m_cw*r_pivot_to_hinge*r_hinge_to_cw_cg;
    s_mE = m_p *r_main_tip*sling_length;

    s_tau_fric_pivot           = mu_poplar*m_total_system_mass*G*pin_radius;
    s_tau_fric_hinge           = mu_poplar*m_cw*G*pin_radius;
    const double tau_fric_piv_free = mu_poplar*(m_arm+m_cw)*G*pin_radius;

    double sf[2] = { theta_main_start, omega_main };
    double s3[6] = { theta_main_start, theta_cw_rel_start, 0.0, omega_main, 0.0, 0.0 };
    double s2[4] = {};

    double x_p=0.0,y_p=0.0,vx_p=0.0,vy_p=0.0;
    bool   phase_b_active=false;

    double max_pivot_force=0.0,max_hinge_force=0.0;
    double max_pivot_Fx=0.0,max_pivot_Fy=0.0;
    double max_hinge_Fx=0.0,max_hinge_Fy=0.0;

    double launch_vx=0.0,launch_vy=0.0,launch_x=0.0,launch_y=0.0;
    double max_height=0.0;
    double t=0.0;

    // Stall check
    {
        double q1=theta_main_start, q2abs=theta_main_start+theta_cw_rel_start;
        double net=-m_arm*G*r_main_pivot_to_cg*cos(q1)
                  -m_cw*G*(r_pivot_to_hinge*cos(q1)+r_hinge_to_cw_cg*cos(q2abs))
                  -m_p *G*r_main_tip*cos(q1);

        cout<<"=== TREBUCHET SIMULATION STARTING ==="<<endl;
        cout<<fixed<<setprecision(5);
        cout<<"theta_main  start  : "<<(theta_main_start*180/PI)       <<" deg"   <<endl;
        cout<<"theta_cw    start  : "<<(q2abs*180/PI)                  <<" deg"   <<endl;
        cout<<"I_fused            : "<<I_fused                         <<" kg*m^2"<<endl;
        cout<<"IA / IB / IC       : "<<s_IA<<" / "<<s_IB<<" / "<<s_IC <<" kg*m^2"<<endl;
        cout<<"Net initial torque : "<<net                              <<" Nm"    <<endl;
        cout<<"Friction (pivot)   : "<<s_tau_fric_pivot                <<" Nm"    <<endl;
        cout<<"Sling release abs  : "<<(sling_release_abs_angle*180/PI)<<" deg"   <<endl;
        cout<<"Target theta       : "<<(target_theta*180/PI)           <<" deg"   <<endl;
        cout<<"Max dtheta/step    : "<<(MAX_DTHETA*180/PI)             <<" deg"   <<endl;
        if (fabs(net)<=s_tau_fric_pivot){
            cout<<"\n!!! STALL DETECTED. Aborting."<<endl; return 1;
        }
        cout<<"\nSimulation starting...\n"<<endl;
    }

    // ================================================================
    // MAIN LOOP
    // ================================================================
    while (running) {

        double q1=s3[0], q2=s3[1], abs_cw=q1+q2;

        // STATE SELECTOR
        if (iterations==0) {
            current_state=fused_motion;
            cout<<"State: fused_motion"<<endl;
        }
        else if (current_state==fused_motion && abs_cw<=fused_to_parametric_threshold) {
            current_state=parametric_motion;
            s3[0]=sf[0]; s3[3]=sf[1];
            s3[1]=theta_cw_rel_start; s3[2]=0.0; s3[4]=0.0; s3[5]=0.0;
            cout<<"State: parametric_motion  t="<<t
                <<"s  theta_main="<<(sf[0]*180/PI)<<" deg"
                <<"  omega="<<sf[1]<<" rad/s"<<endl;
        }
        else if ((current_state==parametric_motion||current_state==launch)
                 && !phase_b_active && s3[0]<=target_theta) {
            if (current_state!=launch) {
                current_state=launch;
                cout<<"State: launch (Phase A)  t="<<t
                    <<"s  theta_main="<<(s3[0]*180/PI)<<" deg"
                    <<"  omega="<<s3[3]<<" rad/s"<<endl;
            }
        }
        else if (current_state==launch && phase_b_active && y_p<=0.0) {
            current_state=data_collection;
        }

        // STATE EXECUTION
        switch (current_state) {

            case fused_motion:
            {
                double tA=-m_arm*G*r_main_pivot_to_cg*cos(sf[0]);
                double tC=-m_cw*G*(r_pivot_to_hinge*cos(sf[0])
                                  +r_hinge_to_cw_cg*cos(sf[0]+theta_cw_rel_start));
                double tP=-m_p*G*r_main_tip*cos(sf[0]);
                double tF=-sign(sf[1])*s_tau_fric_pivot;
                double alpha=(tA+tC+tP+tF)/I_fused;

                double pFx,pFy,hFx,hFy;
                computeReactionForces(sf[0],theta_cw_rel_start,sf[0]+theta_cw_rel_start,
                                      sf[1],0.0,sf[1],alpha,alpha,alpha,
                                      true,pFx,pFy,hFx,hFy);
                double pF=sqrt(pFx*pFx+pFy*pFy),hF=sqrt(hFx*hFx+hFy*hFy);
                if(pF>max_pivot_force){max_pivot_force=pF;max_pivot_Fx=pFx;max_pivot_Fy=pFy;}
                if(hF>max_hinge_force){max_hinge_force=hF;max_hinge_Fx=hFx;max_hinge_Fy=hFy;}

                rk4_fused(sf,DT_MAX);
                t+=DT_MAX;
                s3[0]=sf[0]; s3[3]=sf[1];
                break;
            }

            case parametric_motion:
            case launch:
            {
                if (!phase_b_active) {
                    // Use bisection near the parametric->launch crossing
                    double h;
                    if (current_state==parametric_motion && s3[0]>target_theta
                        && s3[0]+fabs(s3[3])*DT_MAX*10 <= target_theta) {
                        h=stepToEvent3(s3,target_theta,true);
                    } else {
                        h=adaptiveDt(s3[3]);
                        if (!rk4_3dof(s3,h,true)){
                            cout<<"WARNING: Singular 3x3 at t="<<t<<endl; break;
                        }
                    }
                    t+=h;

                    // Reaction forces
                    double ddq[3]={};
                    computeAccel3(s3[0],s3[1],s3[2],s3[3],s3[4],s3[5],ddq,true);
                    double q3abs=s3[0]+s3[2], dq3abs=s3[3]+s3[5], ddq3abs=ddq[0]+ddq[2];
                    double pFx,pFy,hFx,hFy;
                    computeReactionForces(s3[0],s3[1],q3abs,s3[3],s3[4],dq3abs,
                                          ddq[0],ddq[1],ddq3abs,true,pFx,pFy,hFx,hFy);
                    double pF=sqrt(pFx*pFx+pFy*pFy),hF=sqrt(hFx*hFx+hFy*hFy);
                    if(pF>max_pivot_force){max_pivot_force=pF;max_pivot_Fx=pFx;max_pivot_Fy=pFy;}
                    if(hF>max_hinge_force){max_hinge_force=hF;max_hinge_Fx=hFx;max_hinge_Fy=hFy;}

                    // Sling release check
                    if (current_state==launch) {
                        double q3abs_new=s3[0]+s3[2];
                        if (q3abs_new<=sling_release_abs_angle) {
                            double dq3ar=s3[3]+s3[5];
                            launch_x= r_main_tip*cos(s3[0])+sling_length*cos(q3abs_new);
                            launch_y= r_main_tip*sin(s3[0])+sling_length*sin(q3abs_new);
                            launch_vx=-r_main_tip*sin(s3[0])*s3[3]
                                      -sling_length*sin(q3abs_new)*dq3ar;
                            launch_vy= r_main_tip*cos(s3[0])*s3[3]
                                      +sling_length*cos(q3abs_new)*dq3ar;
                            x_p=launch_x; y_p=launch_y;
                            vx_p=launch_vx; vy_p=launch_vy;
                            phase_b_active=true;
                            s2[0]=s3[0];s2[1]=s3[1];s2[2]=s3[3];s2[3]=s3[4];

                            double spd=sqrt(launch_vx*launch_vx+launch_vy*launch_vy);
                            double ang=atan2(launch_vy,launch_vx)*180/PI;
                            cout<<"\n--- PROJECTILE RELEASED ---"<<endl;
                            cout<<fixed<<setprecision(4);
                            cout<<"  t        = "<<t<<" s"<<endl;
                            cout<<"  Position = ("<<launch_x<<", "<<launch_y<<") m"<<endl;
                            cout<<"  Velocity = ("<<launch_vx<<", "<<launch_vy<<") m/s"<<endl;
                            cout<<"  Speed    = "<<spd<<" m/s"<<endl;
                            cout<<"  Angle    = "<<ang<<" deg"<<endl;
                        }
                    }
                }
                else {
                    // Phase B: free-flight + 2-DOF arm
                    double h=adaptiveDt(s2[2]);
                    vy_p-=G*h; x_p+=vx_p*h; y_p+=vy_p*h;
                    if(y_p>max_height) max_height=y_p;
                    if(!rk4_2dof(s2,h,true,tau_fric_piv_free)){
                        cout<<"WARNING: Singular 2x2 at t="<<t<<endl; break;
                    }
                    t+=h;

                    double ddq2[2]={};
                    computeAccel2(s2[0],s2[1],s2[2],s2[3],ddq2,true,tau_fric_piv_free);
                    double pFx,pFy,hFx,hFy;
                    computeReactionForces(s2[0],s2[1],0.0,s2[2],s2[3],0.0,
                                          ddq2[0],ddq2[1],0.0,false,pFx,pFy,hFx,hFy);
                    double pF=sqrt(pFx*pFx+pFy*pFy),hF=sqrt(hFx*hFx+hFy*hFy);
                    if(pF>max_pivot_force){max_pivot_force=pF;max_pivot_Fx=pFx;max_pivot_Fy=pFy;}
                    if(hF>max_hinge_force){max_hinge_force=hF;max_hinge_Fx=hFx;max_hinge_Fy=hFy;}
                }
                break;
            }

            case data_collection:
            {
                double x_land=x_p-vx_p*(y_p/vy_p);
                double spd=sqrt(launch_vx*launch_vx+launch_vy*launch_vy);
                double ang=atan2(launch_vy,launch_vx)*180/PI;

                cout<<"\n====================================================="<<endl;
                cout<<"         TREBUCHET SIMULATION RESULTS                "<<endl;
                cout<<"====================================================="<<endl;
                cout<<fixed<<setprecision(4);
                cout<<"PROJECTILE"<<endl;
                cout<<"  Launch position  : ("<<launch_x<<", "<<launch_y<<") m"<<endl;
                cout<<"  Launch velocity  : ("<<launch_vx<<", "<<launch_vy<<") m/s"<<endl;
                cout<<"  Launch speed     : "<<spd<<" m/s"<<endl;
                cout<<"  Launch angle     : "<<ang<<" deg above horizontal"<<endl;
                cout<<"  Max height       : "<<max_height<<" m"<<endl;
                cout<<"  Range (y=0)      : "<<x_land<<" m"<<endl;
                cout<<endl;
                cout<<"STRUCTURAL LOADS (peak over full simulation)"<<endl;
                cout<<"  Max pivot force  : "<<max_pivot_force
                    <<" N  (Fx="<<max_pivot_Fx<<", Fy="<<max_pivot_Fy<<")"<<endl;
                cout<<"  Max hinge force  : "<<max_hinge_force
                    <<" N  (Fx="<<max_hinge_Fx<<", Fy="<<max_hinge_Fy<<")"<<endl;
                cout<<"====================================================="<<endl;
                running=false;
                break;
            }
        }

        iterations++;
        if (iterations>100000000){ cout<<"Max iterations reached."<<endl; running=false; }
    }
    return 0;
}