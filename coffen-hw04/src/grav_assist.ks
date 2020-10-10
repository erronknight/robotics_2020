
RUNPATH("0:/transfer_orbits.ks").

// Assume Coplanar
// need to do inclination adjust if not coplanar
// know planet_velo, planet_flight_path
// know spacecraft velo_init, flight_path_ang, d_miss_dist

// planet and spacecraft velo is <>:velocity:orbit

function get_velo_x {
    parameter velo_mag.
    parameter fp_ang.
    return velo_mag * cos(fp_ang).
}.

function get_velo_y {
    parameter velo_mag.
    parameter fp_ang.
    return velo_mag * sin(fp_ang).
}.

// returns relative spacecraft velocity
// alt: vsc - vp (vector math)
// Vs/p
function sc_rel_velo {
    parameter vp.
    parameter vp_flang.
    parameter vsc.
    parameter vsc_flang.

    local v_rel_x is get_velo_x(vsc, vsc_flang) - get_velo_x(vp, vp_flang).
    local v_rel_y is get_velo_y(vsc, vsc_flang) - get_velo_y(vp, vp_flang).

    local v_rel_mag is sqrt((v_rel_x ^ 2) + (v_rel_y ^ 2)).
    return v_rel_mag.
}.

function get_theta_init {
    parameter vp.
    parameter vp_flang.
    parameter vsc.
    parameter vsc_flang.

    local v_rel_x is get_velo_x(vsc, vsc_flang) - get_velo_x(vp, vp_flang).
    local v_rel_y is get_velo_y(vsc, vsc_flang) - get_velo_y(vp, vp_flang).

    local theta_i is arctan(v_rel_y / v_rel_x).
    return theta_i.
}.

// user hyperbolic eccentricity.
function turn_ang {
    parameter d_miss_dist. // -1 (trailing body) or 1 (leading body).
    parameter ecc. // eccentricity 

    local d_sign is 0.
    if (d_miss_dist < 0) {
        set d_sign to -1.
    } else {
        set d_sign to 1.
    }.

    local turn_ang is (d_sign * 2 * arcsin(1 / ecc)).
    return turn_ang.
}.

function get_theta_fin {
    parameter turn_ang.
    parameter theta_i.

    local theta_f is theta_i + turn_ang.
    return theta_f.
}.

// returns relative spacecraft velocity (final angle)
// alt: vsc - vp (vector math)
// Vs/p
function sc_velo_fin {
    parameter vp.
    parameter vp_flang.
    parameter vsp.
    parameter theta_f.

    local v_sf_x is get_velo_x(vsp, theta_f) + get_velo_x(vp, vp_flang).
    local v_sf_y is get_velo_y(vsp, theta_f) + get_velo_y(vp, vp_flang).

    local v_sf_mag is sqrt((v_sf_x ^ 2) + (v_sf_x ^ 2)).
    return v_sf_mag.
}.

function sc_fl_fin {
    parameter vp.
    parameter vp_flang.
    parameter vsp.
    parameter theta_f.

    local v_sf_x is get_velo_x(vsp, theta_f) + get_velo_x(vp, vp_flang).
    local v_sf_y is get_velo_y(vsp, theta_f) + get_velo_y(vp, vp_flang).

    local sfl_fin is arctan(v_sf_y, v_sf_x).
    return sfl_fin.
}.

// hyperbolic descriptor elements (semimajor axis + impact param + eccentricity)

// impact parameter
function h_b {
    parameter d_miss_dist.
    parameter theta_i.

    local b_imp is d * sin(theta_i).
    return b_imp.
}.

// hyperbolic semimajoraxis
function h_a_tx {
    parameter GM.
    parameter v_sp.

    local a_tx is -1 * (GM / (v_sp ^ 2)).
    return a_tx.
}.

// hyperbolic eccentricity
function h_ecc {
    parameter b_imp.
    parameter a_tx.

    local ecc is sqrt(1 + ((b_imp ^ 2) / (a_tx ^ 2))).
    return ecc.
}.

// get turn angle from initially known values.
function get_turn_ang {
    parameter vp.
    parameter vp_flang.
    parameter vsc.
    parameter vsc_flang.
    parameter d_miss_dist.

    local vsp is sc_rel_velo(vp, vp_flang, vsc, vsc_flang).
    local theta_i is get_theta_init(vp, vp_flang, vsc, vsc_flang).
    local ha_tx is h_a_tx(GM, vsp).
    local b_imp is h_b(d_miss_dist, theta_i).
    local ecc is h_ecc(b_imp, ha_tx).
    local def_ang is turn_ang(d_miss_dist, ecc).
}.