// Sarah Coffen

//Function to Land in a Hover Slam.

// return the max acceleration for the ship.
function get_ship_max_acc {
    lock acc to availableThrust/Ship:mass.
    return acc.
}.

// determine the throttle value as a function of altitude.
function get_thrt_val {
    lock velo to SHIP:VELOCITY:SURFACE:MAG.
    if velo < 50 {
        return 0.2.
    } else if velo < 150 {
        return 0.3.
    } else if velo < 300 {
        return 0.5.
    } else {
        return 1.0.
    }.
}.

// return the surface altitude, using the radar value when accessible
// for higher accuracy.
function surface_alt {
    local radar_srf is alt:radar.
    local alt_srf is ship:altitude.
    
    if abs(radar_srf - alt_srf) < 10 {
        return radar_srf.
    } else {
        return alt_srf.
    }.
}.

// start the final descent burn to slow to 3 m/s
function burn_start {
    set warp to 0.
    lock ship_velo to SHIP:VELOCITY:SURFACE:MAG.
    lock burn_time to 0.75 * (ship_velo/get_ship_max_acc()).
    lock vert_acc to availableThrust * cos(gen_pitch()) / ship:mass.

    lock steering to SRFRETROGRADE.

    set WARPMODE to "PHYSICS".
    set warp to 4.
    wait until alt:radar < 11000.
    set warp to 0.

    when surface_alt < ship_velo * burn_time then {
        lock THROTTLE to thrt.
        lock thrt to get_thrt_val().
        // wait burn_time.
        RCS OFF.
        wait until ship_velo < 3.
        set thrt to 0.
        GEAR OFF.
        GEAR ON.
        maintain_burn().
    }.
    wait until surface_alt < ship_velo * burn_time.
}.

// maintain the final burn speed after the main descent burn.
function maintain_burn {
    RCS OFF.
    lock THROTTLE to thrt.
    until surface_alt < 1 {
        if ship:VERTICALSPEED < 0 and abs(ship:VERTICALSPEED) > 3 {
            set thrt to 0.1.
        } else {
            set thrt to 0.
        }.
    }.
    set thrt to 0.
}.
