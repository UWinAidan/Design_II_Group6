#ifndef Constants_wipper_v3_h
#define Constants_wipper_v3_h

// Inputs for Wipper_dynamics_v3.cpp.  Lengths in m, masses in kg, inertias in kg*m^2, angles in rad unless they say deg.

const double G  = 9.81;
const double PI = 3.14159265358979323846;
const double DEG = PI / 180.0;

// ====================================================================================================================================
// 1. GEOMETRY (from CAD)
// ====================================================================================================================================
    const double r_main_pivot_to_cg  = 0.3825;   // pivot -> arm CG, toward the tip
    const double r_pivot_to_hinge    = 0.175;    // pivot -> counterweight hinge. The hinge is on the SHORT end, opposite the tip
    const double r_hinge_to_cw_cg    = 0.6916;   // hinge -> counterweight CG
    const double r_main_tip          = 0.9625;   // pivot -> release pin at the tip
    const double sling_length        = 0.9;
    const double pivot_height        = 0.976;    // pivot above the floor

    // where the ball sits before launch: on the arm near the pivot, with the sling folded back along the arm
    const double ball_along_arm      = 0.100;    // from the pivot toward the tip
    const double ball_above_arm      = 0.029;    // above the arm's centre line

// ====================================================================================================================================
// 2. MASS & INERTIA (from CAD)
// ====================================================================================================================================
    const double m_arm               = 0.167;
    const double m_cw                = 1.016;
    const double m_p                 = 0.023;    // squash ball

    // Arm about its own CG, about the SWING axis (the axis parallel to the pivot dowel).
    // v2 had 0.000005859 here, which is the size of the moment about the arm's long axis. Measured off the CAD solid
    // with a uniform density, the swing-axis value is 0.0176.  CONFIRM IN INVENTOR (iProperties > Physical).
    const double I_arm_cg            = 0.01762;
    const double I_cw_cg             = 0.03467282;   // counterweight assembly about its own CG

// ====================================================================================================================================
// 3. START POSITION
// ====================================================================================================================================
    // The counterweight arm rests against the main arm, this far behind it (angle between the main arm and the line
    // from the hinge to the counterweight's CG). The CAD gives 4.12 deg. Measured off the slow-motion launch video the
    // built machine sits at about 7 to 9 deg, and 9 deg makes the counterweight follow the path it takes in the video
    // (it stops about 40 deg past straight down instead of swinging up to horizontal). The whip is very sensitive to
    // this angle, so MEASURE IT ON THE MACHINE if you can.
    const double theta_cw_rel_start      = 9.0 * DEG;
    // Both arms start as high as possible: just past the balance point, on the side that swings the right way.
    // The balance angle is worked out from the masses above; this is how far past it the arm is cocked.
    const double start_past_balance_deg  = 3.0;

// ====================================================================================================================================
// 4. RELEASE
// ====================================================================================================================================
    // The sling loop slips off the pin at the tip. This is the angle between the sling and the arm's centre line
    // at the moment it lets go (0 = sling straight out past the tip). It is set by the pin angle and how easily the loop slides.
    const double release_angle_deg       = 30.0;

// ====================================================================================================================================
// 5. LOSSES
// ====================================================================================================================================
    // Pin friction: torque = mu * (the force the pin is carrying at that moment) * pin radius. During the whip the pins
    // carry three to four times the plain weight, so this is worked out every step from the real loads.
    const double mu_poplar           = 0.35;     // wood on the steel pivot pins
    const double pin_radius          = 0.00635;
    const double ball_diameter       = 0.040;
    const double ball_drag_coeff     = 0.50;     // a sphere at these speeds
    const double air_density         = 1.20;

    // Air drag on the machine itself. The arm and counterweight numbers come from the CAD sizes. The sling number is
    // the one that was TUNED so the throw lands at the measured 115 ft: it stands for the ball, the pouch and the cords
    // while they are whipping round (the bare ball on its own would be 0.0006).
    const double arm_face_width      = 0.010;    // the arm's leading edge: it is 10 mm thick
    const double arm_drag_coeff      = 1.5;      // a rectangular edge
    const double cw_drag_area        = 0.024;    // counterweight, drag coefficient x frontal area (m^2): cans + plates 0.146 x 0.132 m, plus the rails
    const double sling_drag_area     = 0.002;    // ball + pouch + cords on the sling, drag coefficient x area (m^2)

// ====================================================================================================================================
// 6. SIMULATION PARAMETERS
// ====================================================================================================================================
    const double dt      = 0.00001;
    const double t_max   = 5.0;

#endif
