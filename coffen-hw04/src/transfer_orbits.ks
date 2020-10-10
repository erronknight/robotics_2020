// Functions for transfer orbit calcs

function get_semimajor_axis {
    parameter r_a.
    parameter r_b.
    local a_tx is (r_a + r_b) / 2.

    return a_tx.
}.

// Hohmann Transfer

function get_velocity_pt {
    parameter GM.
    parameter r_a.

    local v_A is sqrt(GM/r_a).
    return v_A.
}.

// Returns orbital velocity value from vis_viva equation.
function vis_viva_eq {
    parameter r_a. // radius of orbit
    parameter a_tx. // semi-major axis
    parameter mu. // standard grav param

    return sqrt(mu * ((2 / r_a) - (1 / a_tx))).
}.

function hht_velo_change_A {
    parameter r_a.
    parameter r_b.
    parameter GM.

    local a_tx is get_semimajor_axis(r_a, r_b).

    local v_tx_a is vis_viva_eq(r_a, a_tx, GM).
    local v_ia is get_velocity_pt(r_a, GM).
    local d_v_a is v_tx_a - v_ia.

    local v_tx_b is vis_viva_eq(r_b, a_tx, GM).
    local v_fb is get_velocity_pt(r_b, GM).
    local d_v_b is v_fb - v_tx_b.

    return d_v_a.
}.

function hht_velo_change_B {
    parameter r_a.
    parameter r_b.
    parameter GM.

    local a_tx is get_semimajor_axis(r_a, r_b).

    local v_tx_b is vis_viva_eq(r_b, a_tx, GM).
    local v_fb is get_velocity_pt(r_b, GM).
    local d_v_b is v_fb - v_tx_b.

    return d_v_b.
}.

function hht_velo_change_total {
    parameter r_a.
    parameter r_b.
    parameter GM.

    local d_v_a is hht_velo_change_A(r_a, r_b, GM).
    local d_v_b is hht_velo_change_B(r_a, r_b, GM).
    local d_v_t is d_v_a + d_v_b.
}.

// One Tangent Burn Calcs

// for getting transfer ellipse eccentricity
function get_eccentricity {
    parameter r_a.
    parameter a_tx.

    local ecc is 1 - (r_a / a_tx).
    return ecc.
}.

// get the true anomaly at the second burn
function get_true_anomaly_burn_2 {
    parameter r_a.
    parameter r_b.
    parameter a_tx.
    
    local ecc is get_eccentricity(r_a, a_tx).
    local true_a is arccos((((a_tx * (1 - (ecc * ecc))) / r_b) - 1) / ecc).
    return true_a.
}.

// get the flight path angle at the second burn
function get_flight_path_angle_burn_2 {
    parameter true_a.
    parameter ecc.
    
    local phi is arctan((ecc * sin(true_a)) / (1 + (ecc * cos(true_a)))).
    return phi.
}.

function q_fl_ang_at_b {
    parameter r_a.
    parameter r_b.

    local a_tx is get_semimajor_axis(r_a, r_b).
    local ec is get_eccentricity(r_a, a_tx).
    local t_a is get_true_anomaly_burn_2(r_a, r_b, a_tx).
    local fl_ang is get_flight_path_angle_burn_2(t_a, ec).

    return fl_ang.
}.

function velo_change_burn_2 {
    parameter r_a.
    parameter r_b.
    parameter GM.

    local a_tx is get_semimajor_axis(r_a, r_b, GM).
    local v_tx_b is vis_viva_eq(r_b, a_tx, GM).
    local v_fb is get_velocity_pt(r_b, GM).

    local ecc is get_eccentricity(r_a, a_tx).
    local true_a is get_true_anomaly_burn_2(r_a, r_b, a_tx).
    local phi is get_flight_path_angle_burn_2(true_a, ecc).

    local d_v_b is sqrt((v_tx_b * v_tx_b) + (v_fb * v_fb) - (2 * v_tx_b * v_fb * cos(phi))).

    return d_v_b.
}.

function get_eccentric_anomaly {
    parameter true_a.
    parameter ecc.

    local ecc_a is arccos((ecc + cos(true_a) / (1 + (ecc * cos(true_a))))).
    return ecc_a.
}.

// time of flight
// Expects Eccentric anomaly to be in radians,
function get_time_of_flight {
    parameter ecc.
    parameter ecc_a.
    parameter a_tx.
    parameter GM.

    local tof is ((ecc_a - (ecc * sin(ecc_a))) * sqrt((a_tx ^ 3) / GM)).
    return tof.
}.

// Takes radius of desired orbit
function orb_velo_at_rad {
    parameter rad_v.
    parameter mu.

    local orb_velo is vis_viva_eq(rad_v, rad_v, mu).

    return orb_velo.
}.

function circ_delta_velo {
    parameter orb.
    parameter rad_v.
    parameter mu.
    
    local ov_f is orb_velo_at_rad(rad_v, mu).
    local ov_i is orb:velocity:orbit:mag.
    local delt_v is ov_f - ov_i.

    return delt_v.
}.

// delta v change for changing inclination (at ascending node)
// normal to orbit plane for node.
function inc_change_delta_v {
    parameter velo.
    parameter inc_change.

    local d_v_inc is 2 * velo * sin(inc_change / 2).
    return d_v_inc.
}.