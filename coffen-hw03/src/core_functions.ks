//Sarah Coffen

run node_functions.
run landing.
run angles.

// Setup Ship Env for mission
function core_settings {
    //Settings and locks
    CLEARSCREEN.
    SAS OFF.
    RCS ON.
    LIGHTS ON.

    SET WARPMODE TO "PHYSICS".
    SET WARP TO 0.

    set steering_val to R(0,0,-90) + HEADING(90,90).
    set throttle_val to 0.

    LOCK THROTTLE to throttle_val.
    LOCK STEERING to steering_val.
}.

// Run this after call to launch
function gen_staging_control {
    // separate on stages
    WHEN (MAXTHRUST = 0 or (stage:number > 3 and (stage:liquidfuel < 0.01 and STAGE:solidfuel < 0.01))) THEN {
        PRINT "Staging".
        STAGE.
        wait 1.
        PRESERVE.
    }.
}.

set rails_warp_vals to LIST(
    0,	1,
    1,	5,
    2,	10,
    3,	50,
    4,	100,
    5,	1000,
    6,	10000,
    7,	100000
).

// tiered warp
function tiered_warp {
    parameter stop_at.
    set WARPMODE to "RAILS".
    set WARP to 0.
}.

// run the ascent profile code.
function do_ascent_to_orbit {
    print "== ASCENT ==".

    run ascent.
    run circle.
    
    run_ascent_profile().

    set warp to 1.
    WAIT UNTIL eta:apoapsis-10.
    set warp to 0.

    circularize().
    if (orbit:eccentricity > .1) {
        print "!!! FAILED TO CIRCULARIZE !!!".
    } ELSE {
        print "Completed Ascent.".
    }.
}.

// launch the rocket off the pad.
function launch {
    print "== LAUNCH ==".
    set throttle_val to 1.
    STAGE.
}.

// Landing behavior
function landing {
    print "== LAND ==".

    orbital_decay_to_land().
    wait 5.
    set warpmode to "RAILS".
    warpto(time:seconds + (3 * eta:periapsis / 4)).
    set warp to 4.
    burn_start().

    print "== Mission Completed ==".
}.