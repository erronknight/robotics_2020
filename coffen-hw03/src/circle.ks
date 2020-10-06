// The circularize function was taken and modified from:
// Giacomo Rizzi @gufoe on Github
// https://gist.github.com/gufoe/ffea36be6d05b435927f
// https://www.reddit.com/r/Kos/comments/495a35/automated_nearperfect_circular_orbit_manouver/

function circularize {
    print "Waiting for apoapsis...".
    lock steering to heading(90,0). // Look at east (90), zero degrees above the horizon
    warpto(time:seconds + eta:apoapsis - 30).
    lock steering to heading(90,0).
    wait eta:apoapsis-.1. // Wait to reach apoapsis
    lock throttle to 1. // Full power

    set oldEcc to orbit:eccentricity.
    until (oldEcc < orbit:eccentricity) { // Exists when the eccentricity stop dropping
        set oldEcc to orbit:eccentricity.

        set power to 1.
        if (orbit:eccentricity < .1) {
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

// additional code for circularizing orbit through node creation.
function circ_node {
    parameter nd.
    local delta_v is 0.

    set nd to node(time:seconds + eta:periapsis, 0, 0, delta_v).

    until nd:orbit:eccentricity < 0.1 {
        if nd:orbit:eccentricity < 0.1 {
            return delta_v.
        }.
        set delta_v to delta_v + 1.
        set nd to node(time:seconds + eta:periapsis, 0, 0, delta_v).
    }.
}.