//Sarah Coffen

// Initially based off the quickstart script.

//Settings and locks
CLEARSCREEN.
SAS OFF.
RCS ON.
SET WARPMODE TO "PHYSICS".
SET WARP TO 0.
set logging to true.

SET THROTTLE_VAL TO 1.0. // 1.0 is the max, 0.0 is idle.
LOCK THROTTLE TO THROTTLE_VAL.

SET STEER_VAL TO r(0,0,0)*up.
LOCK STEERING TO STEER_VAL.

// separate on stages
WHEN MAXTHRUST = 0 THEN {
    PRINT "Staging".
    STAGE.
    PRESERVE.
}.

// remove solid rocket boosters
WHEN STAGE:solidfuel <0.01 THEN {
    wait 1.
    STAGE.
}.

// log values to terminal
WHEN logging THEN {
    print "Apoapsis= "+ROUND(SHIP:APOAPSIS, 0) at (0,10).
    print "Periapsis= "+ROUND(SHIP:PERIAPSIS, 0) at (0,11).
    print "1.1 * Periapsis = "+ROUND(SHIP:PERIAPSIS * 1.1, 0) at (0,12).

    print "Orbit Velocity= "+ROUND(ship:velocity:orbit:mag,0) at (0,14).

    wait 1.
    PRESERVE.
}.

// try to save a little fuel in reduced atmosphere
WHEN SHIP:ALTITUDE > 35000 AND SHIP:ALTITUDE < 55000 THEN {
    set THROTTLE_VAL to MIN(1, (1/((ALTITUDE - 30000) / 5000)) + 0.7).
    PRESERVE.
}.

// Original heading value format taken from reddit user u/space_is_hard
// https://www.reddit.com/r/Kos/comments/3ijayc/recommended_ascent_curves/
// Modified for different function curve.
// Aiming for slightly less than 2296 m/s orbital velocity
set gt_start_altitude to 8500.
set gt_end_altitude to 70700.
UNTIL SHIP:APOAPSIS > 70700 {
    set heading_val to (90 - (90 * (MAX(0, SHIP:ALTITUDE - gt_start_altitude) / gt_end_altitude)^(1.0/3.0))).
    SET steer_val to HEADING(90, heading_val).
}.

// set throttle to zero.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
SET THROTTLE_VAL TO 0.0.

// speed through traveling to apoapsis.
SET WARP TO 1.
WAIT UNTIL SHIP:ALTITUDE > 70000.
SET WARP TO 0.

// The circularize function was taken and modified from:
// Giacomo Rizzi @gufoe on Github
// https://gist.github.com/gufoe/ffea36be6d05b435927f
// https://www.reddit.com/r/Kos/comments/495a35/automated_nearperfect_circular_orbit_manouver/

Function circularize {
    print "Waiting apoapsis".
    lock steering to heading(90,0). // Look at east (90), zero degrees above the horizon
    wait eta:apoapsis-.1. // Wait to reach apoapsis
    lock throttle to 1. // Full power

    set oldEcc to orbit:eccentricity.
    until ((oldEcc < orbit:eccentricity) OR (SHIP:APOAPSIS <= (SHIP:PERIAPSIS * 1.1) AND SHIP:PERIAPSIS > 70000)) { // Exists when the eccentricity stop dropping
        set oldEcc to orbit:eccentricity.
        
        set power to 1.
        if (orbit:eccentricity < .1) OR (SHIP:APOAPSIS <= (SHIP:PERIAPSIS * 1.1) AND SHIP:PERIAPSIS > 70000) {
            // Lower the power when eccentricity < 0.1
            set power to max(.02, orbit:eccentricity*10).
        }
        
        // Radius is altitude plus planet radius
        set radius to altitude+orbit:body:radius.
        
        // Gravitational force
        set gForce to constant:G*mass*orbit:body:mass/radius^2.
        
        // Centripetal force
        set cForce to mass*ship:velocity:orbit:mag^2/radius.
        
        // Set total force
        set totalForce to gForce - cForce.
        
        // Current stage ended?
        until (maxThrust > 0) {
            stage.
        }
        set thrust to power*maxThrust.
        
        // Check if the thrust is enough to keep the v. speed at ~0m/s
        if (thrust^2-totalForce^2 < 0) {
            print "The vessel hasn't enough thrust to reach a circular orbit.".
            break.
        }
        
        // The angle above the horizon is the angle 
        set angle to arctan(totalForce/sqrt(thrust^2-totalForce^2)).
        
        // Adjust new values for throttle and steering
        lock throttle to power.
        lock steering to heading(90,angle).
        
        // Print stats
        clearscreen.
        print "Attraction:  "+gForce.
        print "Centripetal: "+cForce.
        
        // Wait one tenth of a second
        wait .1.
    }

    // Shut down engines
    lock throttle to 0.
    print "Orbit reached, eccentricity: "+orbit:eccentricity.
}.

// in case it doesn't reach correct altitude.
Function inc_orbit {
    lock steering to heading(90,0). // Look at east (90), zero degrees above the horizon
    wait eta:apoapsis+.5. // Wait to reach just after apoapsis
    lock throttle to 1. // Full power
    wait 0.5.
}.

// check circularity
UNTIL (SHIP:APOAPSIS <= (SHIP:PERIAPSIS * 1.1) AND SHIP:PERIAPSIS > 70000) {
    circularize().
    if (SHIP:APOAPSIS > (SHIP:PERIAPSIS * 1.1) OR SHIP:PERIAPSIS < 70000) {
        inc_orbit().
    } ELSE {
        set logging to false.
    }.
}.

print "=== DONE ===".
print "Apoapsis= "+ROUND(SHIP:APOAPSIS, 0) at (0,10).
print "Periapsis= "+ROUND(SHIP:PERIAPSIS, 0) at (0,11).
print "1.1 * Periapsis = "+ROUND(SHIP:PERIAPSIS * 1.1, 0) at (0,12).

print "Orbit Velocity= "+ROUND(ship:velocity:orbit:mag,0) at (0,14).
print "=== DONE ===" at (0, 16).