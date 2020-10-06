// Sarah Coffen

run angles.

// Declare constants
set mu_kerbin to 3.5316 * (10 ^ 12). // m3/s2 standard gravitational parameter mu = GM
set eq_rad_kerbin to 600000. // equatorial radius.
set mu_mun to 6.5138398* (10 ^10). // m3/s2 standard gravitational parameter mu = GM
set eq_rad_mun to 200000.
set mun_orbital_velo to 600. // orbital velocity m/s

// Decays the orbit to a more circular, lower Munar orbit
function orbital_decay_to_orbit {
    parameter ves.
    // local avg_orb_velo is (velocityat(ves, eta:apoapsis):orbit:mag + velocityat(ves, eta:periapsis):orbit:mag) / 2.
    local apo_velo is vis_viva_eq(ship:orbit:apoapsis, ship:orbit:semimajoraxis, mu_mun).
    local peri_velo is vis_viva_eq(ship:orbit:periapsis, ship:orbit:semimajoraxis, mu_mun).
    local avg_orb_velo is (apo_velo + peri_velo) / 2.
    local delta_v is -1 * abs(avg_orb_velo - mun_orbital_velo).
    // local delta_v is -100.

    local decay_node is node(time:seconds + eta:periapsis, 0, 0, delta_v).
    add decay_node.
}.

// Decays the orbit to intercept the surface of the Mun
function orbital_decay_to_land {
    set WARPMODE to "RAILS".
    warpto(time:seconds + eta:apoapsis - 40).
    set warp to 2.
    wait until eta:apoapsis - 1.
    set warp to 1.
    lock STEERING to srfretrograde.
    wait 1.
    set throt_val to 0.
    lock throttle to throt_val.
    until alt:periapsis < -150000 {
        set throt_val to 1.
        wait 0.0001.
    }.
    set throt_val to 0.
}.

// Returns orbital velocity value from vis_viva equation.
function vis_viva_eq {
    parameter r. // radius of orbit
    parameter a. // semi-major axis
    parameter mu. // standard grav param

    return sqrt(mu * ((2 / r) - (1 / a))).
}.

// assumed node at periapsis
function delta_v_mun_capture {
    parameter target_pe.

    local Vo is SHIP:VELOCITY:Orbit:MAG.
    local Vf is vis_viva_eq(ship:orbit:periapsis, (ship:orbit:periapsis + target_pe) / 2, mu_mun).

    return (Vf-Vo).
}.

// get retrograde delta_v, assumed node at periapsis
function delta_v_shrink_orbit {
    parameter target_pe.

    local Vo is vis_viva_eq(ship:orbit:periapsis, ship:orbit:semimajoraxis, mu_mun).
    local Vf is vis_viva_eq(ship:orbit:periapsis, (ship:orbit:periapsis + target_pe) / 2, mu_mun).

    return (Vf-Vo).
}.

// assumes all manuever delta_v goes to prograde burn
function delta_v_kerbin_to_mun_transfer {
    parameter target_ap.
    parameter orb.

    local Vo is vis_viva_eq(orb:semimajoraxis, orb:semimajoraxis, mu_kerbin).
    local Vf is vis_viva_eq(orb:semimajoraxis, (target_ap + orb:semimajoraxis + eq_rad_kerbin) / 2, mu_kerbin).

    return (Vf-Vo).
}.

// create manuever node for Low Kerbin Orbit to Mun interception
function node_LKO_to_MUN {
    parameter offset_a.
    local mun_ang is absang(mun:orbit).
    local ship_ang is absang(ship:orbit).

    // burn angle
    // assume circular orbit, fixed degrees/second
    local burn_ang is absang(mun:orbit) + 180.
    local d_theta is ang_diff(burn_ang, ship_ang).
    local orbit_rate is ship:orbit:period / 360.
    local burn_time is d_theta * orbit_rate.

    // calculate delta_v
    local delta_v is delta_v_kerbin_to_mun_transfer(mun:orbit:apoapsis, ship:orbit).

    // create node for LKO to MUN transfer.
    local LKO_MUN_node is node(time:seconds + burn_time, 0, 0, delta_v).
    add LKO_MUN_node.

    // correct the node_eta via offset and manuever travel time.
    set LKO_MUN_node:eta to correct_node_eta_MUN(nextnode, offset_a).
}.

// correct the manuever eta time for MUN interception
function correct_node_eta_MUN {
    parameter node_obj.
    parameter offset_ang.
    local mun_orbit_rate is 360 / Mun:orbit:period.
    local mun_predict is (mun_orbit_rate * (node_obj:orbit:period / 2)) + offset_ang.
    local orb_rate is ship:orbit:period / 360.
    local burn_time_adjust is orb_rate*mun_predict.

    return node_obj:eta + burn_time_adjust.
}.

// turn towards the direction of the deltaV vector
function aim_node_burnvector {
    lock STEERING to nextnode:deltav.
    wait until vdot(ship:facing:forevector, nextnode:deltav) > 0.
}.

// warp to just before manuever node eta.
function warp_to_node {
    if HASNODE {
        warpto(time:seconds + nextnode:eta - 40).
    }.
}.