// Logistical for handling moons
clearscreen.

run transfer_orbits.
run grav_assist.

set moon_list to list().
set node_list to list().
set node_idx to 0.
set target_aim to true.
set latest_orbit to ship:orbit.
set visited to list().

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
    // local d_ang is abs()

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

function build_circ_node {
    parameter b.
    parameter shporb.
    parameter GM.
    parameter old_time.

    local m_an is true2mean_anomaly(shporb:trueanomaly, shporb:eccentricity).
    local tof is (90 - m_an) * sqrt(shporb:semimajoraxis ^ 3 / GM).
    local pT2 is shporb:period / 2.
    print "TRUN".
    print shporb:trueanomaly.
    print "MAN".
    print m_an.
    print "TOF".
    print tof.

    local delt_v_a is 0.

    if shporb:eccentricity > 0.01 {
    //     set delta_v_a to delta_v_major_to_moon_transfer_non_circ_ap_burn(kerbin, b:orbit:semimajoraxis, shporb).
    // } else {
        set delt_v_a to delta_v_circ_burn(kerbin, shporb).
        set target_aim to true.
    }.

    local t is node(old_time + tof + pT2, 0, 0, delt_v_a).
    add t.
    set t:eta to old_time + tof + pT2.
    print t.
    set node_idx to node_idx + 1.
    node_list:add(t).
    return old_time + tof + pT2.
}.

function build_node {
    parameter m_b. // major body
    parameter b.
    parameter shorb.
    parameter GM.
    parameter begin_time.
    parameter tx_mult.

    local b_orbit is orbitat(b, begin_time).
    set shporb to orbitat(ship, begin_time).

    local moon_ang is absang(b_orbit).
    local ship_ang is absang(shporb).

    // burn angle
    // assume circular orbit, fixed degrees/second
    local burn_ang is absang(b_orbit) + 180.
    local d_theta is ang_diff(burn_ang, ship_ang).
    local orbit_rate is shporb:period / 360.
    local burn_time is d_theta * orbit_rate.

    local r_b is 0.
    local delt_v_a is 0.

    // local nd is node(begin_time, 0, 0, 0).

    set r_b to tx_mult * b_orbit:semimajoraxis.
    set delt_v_a to delta_v_major_to_moon_transfer(m_b, r_b, shporb).
    local nd is node(begin_time + burn_time, 0, 0, delt_v_a).

    // print "OLD".
    // print (begin_time + burn_time).
    // print missiontime.

    add nd.

    print nd.
    // set node_idx to node_idx + 1.
    // node_list:add(nd).

    local offset_ang is -1 * (2* nd:orbit:semiminoraxis - (1.4 * b:SOIRADIUS)) / (b_orbit:semimajoraxis * constant:pi) * 360.
    set nd:eta to correct_node_eta_moon(b, nd, offset_ang, shporb, tx_mult).
    // print nd:eta.
    node_list:add(nd).
    return nd:eta.
}.

// assumes all maneuver delta_v goes to prograde burn
function delta_v_major_to_moon_transfer_non_circ_ap_burn {
    parameter b. //major body
    parameter target_ap.
    parameter orb.

    local Vo is vis_viva_eq(orb:apoapsis, orb:semimajoraxis, b:MU).
    local Vf is vis_viva_eq(orb:semimajoraxis, (target_ap + orb:semimajoraxis + b:radius) / 2, b:MU).

    return (Vf-Vo).
}.

function delta_v_circ_burn {
    parameter b. //major body
    parameter orb.

    local Vo is vis_viva_eq(orb:apoapsis, orb:semimajoraxis, b:MU).
    local Vf is vis_viva_eq(orb:apoapsis, orb:apoapsis, b:MU).

    return (Vf-Vo).
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
    // return nd_obj:eta.
}.

function make_plan_2 {
    parameter b. // major body

    set moon_list to get_moons(b).
    // print moon_list.
    set node_list to list().
    local num_moons is moon_list:length.
    set visited to list().
    local get_next is true.
    local d_t is 0. // hold time change values.

    local plan_time is missiontime + 60. // elasped maneuver time
    set latest_orbit to orbitat(ship, plan_time).
    print "PLAN TIME".
    print plan_time.

    until moon_list:empty and (visited:length = num_moons) {
        if get_next {
            set nx_t to get_next_target(b, latest_orbit).
        }.

        set d_t to single_target_act(b, nx_t, latest_orbit, plan_time).
        set plan_time to d_t.
        // print plan_time.
        // eta to next patch include
        if moon_list:empty {
            set get_next to false.
        }.
    }.
}.

function change_inclination {

    local time_v is 0.

    return time_v.
}.

function circl {
    parameter world_time.
    parameter nx_t_orb.

    local shorb is OrbitAt(ship, world_time).
    local nx_t_orb is OrbitAt(nx_t, world_time).
}.

// function target_body {
//     set time_trace to build_node(kerbin, nx_t, shorb_t, b:MU, time_trace, tx_mult).
// }.

// set latest orbit
// return eta to final orbit patch
function go_to_resultant_orbit_eta {
    parameter nx_t.

    set shorb_t to latest_orbit.

    local eta_t is 0.
    if shorb_t:hasnextpatch {
        until shorb_t:nextpatch:transition = "FINAL" {
            if shorb_t:nextpatch:transition = "ENCOUNTER" {
                if not visited:contains(nx_t) {
                    visited:add(nx_t).
                }.
            }.
            set shorb_t to shorb_t:nextpatch.
            set eta_t to shorb_t:nextpatcheta.
        }.
    }.
    print shorb_t:transition.

    set latest_orbit to shorb_t.
    return eta_t.
}.

function single_target_act {
    parameter b.
    parameter nx_t.
    parameter ship_orb.
    parameter plan_time.

    // local state_val is "circle". // circle, inc_change, target
    // local done is false.
    // local d_t is 0.s

    // inc_change
    // if abs(latest_orbit:inclination - nx_t:orbit:inclination) > 3 {
    //     // set d_t to change_inclination().
    // }.

    // circle

    // target
    set d_t to build_node(b, nx_t, latest_orbit, b:MU, plan_time, 1.9).
    print d_t.
    print go_to_resultant_orbit_eta(nx_t).
    return d_t.
    // circle

}.

make_plan(kerbin).

function make_plan {
    parameter b. // major body

    print "HERE".

    set moon_list to get_moons(b).
    set node_idx to 0.
    set node_list to list().
    local left_to_visit is moon_list:length.
    local visited is list().

    local nx_t is b.
    local shorb_t is SHIP:ORBIT.

    local tx_mult is 1.

    local get_next is true.

    set time_trace to missiontime.
    until moon_list:empty {
        if get_next {
            set nx_t to get_next_target(b, shorb_t).
            set get_next to false.
        }.

        if hasnode {
            set shorb_t to nextnode:orbit.
        }.

        if moon_list:empty {
            set tx_mult to 1.
        } else {
            set tx_mult to 1.9.
        }.  

        if target_aim {
            set time_trace to build_node(kerbin, nx_t, shorb_t, b:MU, time_trace, tx_mult).
            set target_aim to false.
        } else {
            set time_trace to build_circ_node(nx_t, shorb_t, b:MU, time_trace).
        }.

        // print time_trace.
        if shorb_t:hasnextpatch {
            until shorb_t:nextpatch:transition = "FINAL" {
                if shorb_t:nextpatch:transition = "ENCOUNTER" {
                    if not visited:contains(nx_t) {
                        visited:add(nx_t).
                        set left_to_visit to left_to_visit - 1.
                        set get_next to true.
                    }.
                }.
                set shorb_t to shorb_t:nextpatch.
            }.
        }.
        print shorb_t:transition.

        // if NOT (left_to_visit = 0) and moon_list:empty {
        //     moon_list:add(nx_t).
        // }.
    }.
}.