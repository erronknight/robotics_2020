// Sarah Coffen

// main Munar Mission control file.

execute_mission().

// Run the Mission to the Mun.
function execute_mission {
    print "== STARTING ==".

    run core_functions.
    run lko_mun_transfer.

    core_settings().
    launch().
    gen_staging_control().
    do_ascent_to_orbit().

    transfer_to_mun().

    landing().
}.

