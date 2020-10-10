

// returns pitch angle between zenith and nose of ship
function gen_pitch {
    return 90 - vectorangle(ship:up:forevector, ship:facing:forevector).
}.

// return the mean motion given the semimajor axis. (angular speed)
function mean_motion {
    parameter sma. // semi-major axis.
    parameter mu. // std grav parameter.

    return sqrt(mu / (sma^3)).
}.

function get_soi_angle {
  parameter b.
  local soi_angle is (b:soiradius) / (b:orbit:semimajoraxis * constant:pi).

  return soi_angle.
}.

// calculates time for chane in mean anomaly given mean motion.
function find_transfer_time_delta {
    parameter mean_an. // mean_anomaly change
    parameter mean_mot. // mean_motion 

    return (mean_an / mean_mot).
}.

// converts true anomaly to mean anomaly
function true2mean_anomaly {
    parameter ta. // true anomaly
    parameter ecc. // eccentricity

    local ecc_an is true2ecc_anomaly(ta, ecc).
    local mean_an is ecc2mean_anomaly(ecc_an, ecc).
    return mean_an.
}.

// converts eccentric anomaly to true anomaly
function ecc2mean_anomaly {
    parameter ecc_an. // eccentricity anomaly
    parameter ecc. // eccentricity

    local mean_an is (ecc_an - (ecc * sin(ecc_an))).
    return mean_an.
}.

// ---Following Functions from Nat Tuck's Class Demo---

// converts true anomaly to eccentric anomaly
function true2ecc_anomaly {
    parameter ta. // true anomaly
    parameter ecc. // eccentricity
    local y is sqrt(1-(ecc*ecc))*sin(ta).
    local x is ecc+cos(ta).
    return arctan2(y, x).
}.

// given an orbit, show angle from
// vernal equinox to body
// assume an equatorial orbit (inc = 0)
function absang {
  parameter obt. // orbit
  return mod(obt:lan +    // moves with AN
             obt:argumentofperiapsis + // move with AN
             obt:trueanomaly, 360). 
}.

// normalize angle
function norm {
    parameter ang.
    return mod(ang, 360).
}.

// angle difference
function ang_diff {
  parameter a0. // future angle
  parameter a1. // current angle
  set a0 to norm(a0).
  set a1 to norm(a1).
  if (a0 > a1) {
    return a0 - a1.
  }
  else {
    return a0 - a1 + 360.
  }
}.
