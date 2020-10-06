// Sarah Coffen

run node_functions.
run exec_node.

// Warp along transfer from kerbin to munar orbit
function warp_along_transfer {
    parameter how_fast.
    set WARPMODE to "RAILS".
    set WARP TO how_fast.
    wait until (ship:periapsis < 20000 or eta:periapsis < 70).
    set WARP TO 0.
}.

// Mun Orbit Capture node created
function mun_capture_node {
    local delta_v is delta_v_mun_capture(200000).

    local mun_capt_node is node(time:seconds + (eta:periapsis - 1), 0, 0, delta_v).
    add mun_capt_node.
}.

function mun_orbit_closer_node {
    local delta_v3 is -100.

    local mun_shrink_obt_node is node(time:seconds + (eta:periapsis - 1), 0, 0, delta_v3).
    add mun_shrink_obt_node.

    warp_along_transfer(3).
}.

// Builds behavior for transfer from Kerbin orbit to Mun orbit.
function transfer_to_mun {
    node_LKO_to_MUN(-20).
    aim_node_burnvector().
    warpto(time:seconds + nextnode:eta - 60).
    execute_node().

    warp_along_transfer(5).

    mun_capture_node().
    aim_node_burnvector().
    warpto(time:seconds + nextnode:eta - 60).
    execute_node().

    orbital_decay_to_orbit(ship).
    aim_node_burnvector().
    warpto(time:seconds + nextnode:eta - 60).
    execute_node().
}.