// testing file

// COPYPATH("0:/test.ks", "").
// COPYPATH("0:/transfer_orbits.ks", "").
// // COPYPATH("0:/grav_assist.ks", "").
// COPYPATH("0:/new_moon_enum.ks", "").
// COPYPATH("0:/angles.ks", "").
COPYPATH("0:/exec_node.ks", "").

RUNPATH("0:/transfer_orbits.ks").
RUNPATH("0:/grav_assist.ks").
RUNPATH("0:/new_moon_enum.ks").
RUNPATH("0:/angles.ks").

RCS ON.
SAS OFF.

// turn towards the direction of the deltaV vector
function aim_node_burnvector {
    lock STEERING to nextnode:deltav.
    wait until vdot(ship:facing:forevector, nextnode:deltav) > 0.
}.

set WARPMODE to "RAILS".

make_plan(kerbin).

warpto(time:seconds + nextnode:eta - 90).
// set WARP TO 3.
// wait until ().
// set WARP TO 0.

aim_node_burnvector().
RUNPATH("exec_node.ks").

warpto(time:seconds + nextnode:eta - 90).

aim_node_burnvector().
RUNPATH("exec_node.ks").

warpto(time:seconds + nextnode:eta - 90).

aim_node_burnvector().
RUNPATH("exec_node.ks").

warpto(time:seconds + nextnode:eta - 90).

aim_node_burnvector().
RUNPATH("exec_node.ks").

set WARP to 6.