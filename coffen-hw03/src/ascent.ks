// Sarah Coffen

// ascent table
set v_term_kerbin_est to LIST(
    0,          100,
    100,        101,
    1000,       109,
    10000,      240
).

// Run Ascent Profile
function run_ascent_profile {
    set throttle_val to 1.
    print "Running Ascent Profile...".
    SET steering_val TO r(0,0,0)*up.
    UNTIL SHIP:ALTITUDE > 10000 {
        SET steering_val TO r(0,0,0)*up.
    }.

    // Original heading value format taken from reddit user u/space_is_hard
    // https://www.reddit.com/r/Kos/comments/3ijayc/recommended_ascent_curves/
    // Modified for different function curve.
    set gt_start_altitude to 10000.
    set gt_end_altitude to 100000.
    UNTIL SHIP:APOAPSIS > gt_end_altitude {
        set heading_val to (90 - (90 * (MAX(0, SHIP:ALTITUDE - gt_start_altitude) / gt_end_altitude)^(1.0/3.0))).
        SET steering_val to HEADING(90, heading_val).
    }.
    set steering_val to prograde.

    set throttle_val to 0.
    print "Completed Ascent Profile.".
}.

// return total thrust force
function force_thrust {
    parameter twr. // thrust-weight-ratio
    parameter mass_val. // mass
    parameter grav_val. // gravity value

    return twr * mass_val * grav_val.
}.

// return effective thrust force
function force_thrust_eff {
    parameter twr. // thrust-weight-ratio
    parameter pitch_ang. // engine pitch angle
    parameter mass_val. // mass
    parameter grav_val. // gravity value

    local thr_force_total is force_thrust(twr, pitch_ang, mass_val, grav_val).
    return thr_force_total * cos(pitch_ang).
}.

// effective TWR
function twr_eff {
    parameter twr. // thrust-weight-ratio
    parameter pitch_ang. // engine pitch angle

    return twr * cos(pitch_ang).
}.