// Logistical for handling moons
clearscreen.

RUNPATH("0:/transfer_orbits.ks").
RUNPATH("0:/grav_assist.ks").
RUNPATH("0:/angles.ks").
RUNPATH("0:/node_functions.ks").

set moon_list to list().
set node_list to list().
set visited to list().
set latest_orbit to ship:orbit.

when ship:orbit:body = Mun THEN {
    visited:add(MUN).
}.

when ship:orbit:body = Minmus THEN {
    visited:add(Minmus).
}.

function get_moons {
    parameter b.
    return b:orbitingchildren:copy.
}.

// determine which target to go to next.
// check first if moonlist:empty is false.
function get_next_target {
    parameter b. // main body (Kerbin)
    parameter shporbit. // ship orbit before aiming at next target.

    local nx_t is moon_list[0].
    local ind_sel is 0.
    local ind_va is 0.
    local pt_wt_win is 2. // set so first entry has good values.
    local pt_wt_tmp is 0.

    local d_inc is abs(moon_list[0]:orbit:inclination - shporbit:inclination).
    local d_sma is abs(moon_list[0]:orbit:semimajoraxis - shporbit:semimajoraxis).

    if moon_list:length > 1 {
        for moon in moon_list {
            if (d_inc < abs(moon:orbit:inclination - shporbit:inclination)) {
                set pt_wt_tmp to pt_wt_tmp + 1.
            }.

            if (d_sma < abs(moon:orbit:semimajoraxis - shporbit:semimajoraxis)) {
                set pt_wt_tmp to pt_wt_tmp + 1.
            }.

            if pt_wt_tmp > pt_wt_win {
                set ind_sel to ind_va.
                set pt_wt_win to pt_wt_tmp.
                set d_inc to abs(moon:orbit:inclination - shporbit:inclination).
                set d_sma to abs(moon:orbit:semimajoraxis - shporbit:semimajoraxis).
            }.

            set ind_va to ind_va + 1.
            set pt_wt_tmp to 0.
        }.

        set nx_t to moon_list[ind_sel]:name.
        moon_list:remove(ind_sel).
    } else {
        set nx_t to moon_list[ind_sel]:name.
        set moon_list to list().
    }.

    return body(nx_t).
}.

function build_node {
    parameter m_b. // major body
    parameter b.
    parameter b_orb_offset.
    parameter shporb.
    parameter GM.
    parameter begin_time.
    parameter tx_mult.
    parameter extra_v.

    print "TARGET NODE".

    local b_orbit is b:orbit.

    local moon_ang is absang(b_orbit).
    local ship_ang is absang(shporb).

    // burn angle
    // assume circular orbit, fixed degrees/second
    local burn_ang is absang(b_orbit) + 180.
    local d_theta is ang_diff(burn_ang, ship_ang).
    local orbit_rate is shporb:period / 360.
    local burn_time is d_theta * orbit_rate.

    local r_b is tx_mult * b_orbit:semimajoraxis.
    local delt_v_a is delta_v_major_to_moon_transfer(m_b, r_b, shporb).
    local nd is node(begin_time + burn_time, 0, 0, delt_v_a + extra_v).

    add nd.
    print nd.
    local offset_ang is -1 * (2* nd:orbit:semiminoraxis - (1.4 * b:SOIRADIUS)) / (b_orbit:semimajoraxis * constant:pi) * 360.
    set offset_ang to offset_ang + b_orb_offset.
    set nd:eta to correct_node_eta_moon(b, nd, offset_ang, shporb, tx_mult).
    node_list:add(nd).
    return nd:eta.
}.

// assumes all maneuver delta_v goes to prograde burn
function delta_v_major_to_moon_transfer {
    parameter b. //major body
    parameter target_ap.
    parameter orb.

    local Vo is vis_viva_eq(orb:semimajoraxis, orb:semimajoraxis, b:MU).
    local Vf is vis_viva_eq(orb:semimajoraxis, (target_ap + orb:semimajoraxis + b:radius) / 2, b:MU).

    return (Vf-Vo).
}.

function correct_node_eta_moon {
    parameter b.
    parameter nd_obj.
    parameter offset_ang.
    parameter shorb.
    parameter tx_mult.

    local b_orbit_rate is 360 / b:orbit:period.
    local b_predict is (b_orbit_rate * ((1/tx_mult) * nd_obj:orbit:period / 2)) + offset_ang.
    local orb_rate is shorb:period / 360.
    local burn_time_adjust is orb_rate*b_predict.

    return nd_obj:eta + burn_time_adjust.
}.

// set latest orbit
// return eta to final orbit patch
function go_to_resultant_orbit_eta {
    parameter nx_t.

    local eta_t is 0.

    if hasnode {
        set shorb_t to nextnode:orbit.
    } else {
        set shorb_t to ship:orbit.
    }.

    if shorb_t:hasnextpatch {
        until shorb_t:hasnextpatch and shorb_t:nextpatch:transition = "FINAL" {
            if shorb_t:hasnextpatch and shorb_t:nextpatch:transition = "ENCOUNTER" {
                if not visited:contains(nx_t) {
                    visited:add(nx_t).
                }.
            }.
            if shorb_t:hasnextpatch {
                set shorb_t to shorb_t:nextpatch.
                if shorb_t:hasnextpatch {
                    set eta_t to shorb_t:nextpatcheta.
                }.
            }.
        }.
        if shorb_t:hasnextpatch {
            set eta_t to shorb_t:nextpatcheta.
            set shorb_t to shorb_t:nextpatch.
        }.
    }.

    // print shorb_t:transition.
    set latest_orbit to shorb_t.
    return eta_t.
}.

// for periapsis to trueanomaly.
function get_orbit_tof {
    parameter orb.
    parameter GM.

    local ecc_a is true2ecc_anomaly(orb:trueanomaly, orb:eccentricity).
    get_time_of_flight(orb:eccentricity, ecc_a, orb:semimajoraxis, GM).
}.

// apoapsis to trueanomaly
function get_tof {
    parameter orb.
    parameter GM.

    local ta_tof is get_orbit_tof(orb, GM).
    local T_half is orb:period / 2.

    return T_half - ta_tof.
}.

// for periapsis to trueanomaly.
function get_orbit_tof_ta {
    parameter orb.
    parameter orb_ta.
    parameter GM.

    local ecc_a is true2ecc_anomaly(orb_ta, orb:eccentricity).
    get_time_of_flight(orb:eccentricity, ecc_a, orb:semimajoraxis, GM).
}.

// apoapsis to trueanomaly
function get_tof_ta {
    parameter orb.
    parameter orb_ta.
    parameter GM.

    local ta_tof is get_orbit_tof_ta(orb, orb_ta, GM).
    local T_half is orb:period / 2.

    return T_half - ta_tof.
}.

function build_circ_node {
    parameter b.
    parameter orb.
    parameter GM.
    parameter t_v.

    print "CIRCLE NODE".

    local Vo is vis_viva_eq(orb:apoapsis, orb:semimajoraxis, GM).
    local Vf is vis_viva_eq(orb:apoapsis, orb:apoapsis, GM).
    local delta_v is Vf - Vo.

    local d_t is 0.
    set d_t to t_v.
    local T_half is orb:period / 2.
    local t_a_oop is orb:trueanomaly.

    until orb:trueanomaly < 180 {
        set d_t to d_t + T_half.
        set t_a_oop to mod(t_a_oop + 180, 360).
    }.
    set d_t to d_t + get_tof_ta(orb, t_a_oop, GM) - 10.
    local circ_nd is node(d_t, 0, 0, delta_v).

    add circ_nd.
    print circ_nd.
}.

function build_inc_node {
    parameter b.
    parameter nx_t.
    parameter orb.
    parameter GM.
    parameter t_v.
    parameter t_off.
    parameter sign_v.

    print "INCLINATION NODE".

    local delt_v is inc_change_delta_v(orb:velocity:orbit:mag, abs(nx_t:orbit:inclination - orb:inclination)).
    local nd is node(t_v + t_off*orb:period, 0, sign_v * delt_v * 4, 0).

    add nd.
    print nd.
}.

function det_orbit_offset {
    parameter b.
    parameter t_v.

    local b_orbit_rate is 360 / b:orbit:period.
    local b_predict is t_v * b_orbit_rate.
    return mod(absang(b:orbit) + b_predict, 360).
}.

function make_plan {
    parameter b. // major body

    set moon_list to get_moons(b).
    set node_list to list().
    set visited to list().

    local nx_t is b.
    local time_v is time:seconds.

    local get_next is true.

    local tx_mult is 1.9.

    local nxt_orbit is nx_t:orbit.
    local nxt_st_orbit is nx_t:orbit.
    local t_off is 0.23.

    until moon_list:empty {

        if latest_orbit:eccentricity > 0.1 {
            build_circ_node(b, latest_orbit, b:mu, time_v).
            set time_v to go_to_resultant_orbit_eta(nx_t).

            build_inc_node(b, nx_t, latest_orbit, b:mu, time_v, t_off, -1).
            set time_v to go_to_resultant_orbit_eta(nx_t).
        }.

        set nx_t to get_next_target(b, latest_orbit).
        set nxt_st_orbit to nx_t:orbit.

        set tx_mult to 1.9.
        
        local pff is 0.
        local addit_v is 0.
        if nx_t = Minmus {
            set tx_mult to 1.5.
            set pff to det_orbit_offset(nx_t, time_v).
            set time_v to time_v + latest_orbit:period.
            set addit_v to -42.
        }.
        build_node(b, nx_t, pff, latest_orbit, b:mu, time_v, tx_mult, addit_v).
        set time_v to go_to_resultant_orbit_eta(nx_t).

        if moon_list:length > 1 {

            local circ_dvi is vis_viva_eq(latest_orbit:apoapsis, latest_orbit:semimajoraxis, kerbin:mu).
            local circ_dvf is vis_viva_eq(latest_orbit:apoapsis, latest_orbit:apoapsis + kerbin:radius, kerbin:mu).
            local nd_circ is node(time_v + latest_orbit:period * 0.3, 0, 0, circ_dvf-circ_dvi).

            add nd_circ.
            print nd_circ.
            
            set time_v to go_to_resultant_orbit_eta(nx_t).

            build_inc_node(b, nx_t, latest_orbit, b:mu, time_v, t_off, 1).
            set time_v to go_to_resultant_orbit_eta(nx_t).
        }.

        if visited:contains(nx_t) {
            set get_next to true.
        }.
    }.

    print allnodes.
}.


