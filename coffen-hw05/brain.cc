
#include <iostream>
#include <math.h>

#include "robot.hh"

#include <time.h>
#include <stdlib.h>
#include <bits/stdc++.h>

// if running OBS, you will need to change things marked OBS

using std::cout;
using std::endl;

#define PI      3.14159265

#define NORTH   0
#define NE      1
#define EAST    2
#define SE      3
#define SOUTH   4
#define SW      5
#define WEST    6
#define NW      7

#define LEFT_TURN   90
#define RIGHT_TURN  -90
#define TURN_180    180

#define WALL    1
#define NO_WALL 0

#define S_READY             40
#define S_TURN_LEFT         44
#define S_TURN_RIGHT        45
#define S_DO_180            46
#define S_WF_RT             47
#define S_WF_LF             48
#define S_TR_DOOR_RT        49
#define S_TR_DOOR_LF        50

#define FORWARD     1
#define REVERSE     2

time_t last_cmd = 0;
double last_vel_0 = 0.0;
double last_vel_1 = 0.0;

double total_dist = 0.0;

time_t cmd_time = 0;
// time_t cmd_time = time(NULL);

int state = S_WF_RT;
int state_subtask = 0; // just an extra flag
int start_dir = 0;

int wall_buf[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int turns_buf[10];

int program_mode = FORWARD;
// int program_mode = REVERSE;

double
dist_trav(time_t new_time) {
    if (last_cmd == 0) {
        return 0;
    } else {
        double avg_velo = (last_vel_0 + last_vel_1) / 2;
        double time_diff = new_time - last_cmd;
        double dist = avg_velo * time_diff;

        total_dist = total_dist + dist;
        cout << "total_dist: " << total_dist << endl;
        return dist;
    }
}

int
get_dir(double thet) {
    if (abs(thet) <= (PI / 16)) {
        return NORTH;
    } else if (abs(thet) >= (15 * PI / 16)) {
        return SOUTH;
    } else if ((thet >= (7 * PI / 16)) && (thet <= (9 * PI / 16))) {
        return WEST;
    } else if ((thet <= (-7 * PI / 16)) && (thet >= (-9 * PI / 16))) {
        return EAST;
    } else if ((thet > (PI / 16)) && (thet < (7 * PI / 16))) {
        return NW;
    } else if ((thet > (9 * PI / 16)) && (thet < (15 * PI / 16))) {
        return SW;
    } else if ((thet < (-1 * PI / 16)) && (thet > (-7 * PI / 16))) {
        return NE;
    } else {
        return SE;
    }
}

void
update_turns_buf(int turn_val) {
    int length = 10;
    for (int i = 1; i < length; i++) {
        turns_buf[i-1] = turns_buf[i];
    }
    turns_buf[length - 1] = turn_val;
}

void
reset_wall_buf() {
    memset(wall_buf, WALL, sizeof(wall_buf));
    return;
}

void
reset_turns_buf() {
    memset(turns_buf, 0, sizeof(turns_buf));
    return;
}

void
state_change(Robot* robot) {
    start_dir = get_dir(robot->pos_t);
    reset_wall_buf();
    return;
}

void
turn_left(Robot* robot) {
    if (get_dir(robot->pos_t) == ((start_dir + 6) % 8)) {
        robot->set_vel(0.0, 0.0);
        // state = S_READY;
        state_change(robot);
        update_turns_buf(LEFT_TURN);
    } else {
        robot->set_vel(-2.0, 2.0);
    }
    return;
}

void
turn_right(Robot* robot) {
    if (get_dir(robot->pos_t) == ((start_dir + 2) % 8)) {
        robot->set_vel(0.0, 0.0);
        // state = S_READY;
        state_change(robot);
        update_turns_buf(RIGHT_TURN);
    } else {
        robot->set_vel(2.0, -2.0);
    }
    return;
}

void
turn_to(Robot* robot, int direction) {
    cout << "TURN TO CORRECTION " << direction << endl;
    if (get_dir(robot->pos_t) == direction) {
        robot->set_vel(0.0, 0.0);
        // state = S_READY;
        state_change(robot);
    } else {
        if (direction > 4) {
            robot->set_vel(-2.0, 2.0);
        } else {
            robot->set_vel(2.0, -2.0);
        }
    }
    return;
}

void
do_180(Robot* robot) {
    if (get_dir(robot->pos_t) == ((start_dir + 4) % 8)) {
        robot->set_vel(0.0, 0.0);
        // state = S_READY;
        state_change(robot);
        update_turns_buf(TURN_180);
    } else {
        robot->set_vel(2.0, -2.0);
    }
    return;
}

void
update_wall_buf(int wall_val) {
    int length = 10;
    for (int i = 1; i < length; i++) {
        wall_buf[i-1] = wall_buf[i];
    }
    wall_buf[length - 1] = wall_val;

    // for (int j = 0; j < length; j++) {
    //     cout << wall_buf[j];
    // }
    // cout << endl;
}

void
wall_follow_rt_side(Robot* robot) {
    if (robot->range < 1.0) {
        robot->set_vel(-2.0, 2.0);
        return;
    }

    if (robot->range < 1.7) {
        robot->set_vel(2.0, 2.0);
    }
    else {
        robot->set_vel(2.0, -2.0);
    }
    return;
}

void
wall_follow_lf_side(Robot* robot) {
    if (robot->range < 1.0) {
        robot->set_vel(2.0, -2.0);
        return;
    }

    if (robot->range < 1.7) {
        robot->set_vel(2.0, 2.0);
    }
    else {
        robot->set_vel(-2.0, 2.0);
    }
    return;
}

void
wall_follow_lf_side_out(Robot* robot) {
    if (robot->range < 1.5) {
        robot->set_vel(2.0, -2.0);
        return;
    }

    if (robot->range < 2.0) {
        robot->set_vel(2.0, 2.0);
    }
    else {
        robot->set_vel(-2.0, 2.0);
    }
    return;
}

bool
detect_doorway() {
    int length = 10;
    int not_wall_vals = 0;
    for (int i = 0; i < length; i++) {
        if (wall_buf[i] == NO_WALL) {
            not_wall_vals++;
        }
    }
    return (not_wall_vals > 6);
}

void
traverse_door_rt(Robot* robot) {
    if (state_subtask == 0) {
        // drive forward a little bit
        // OBS = 8
        // otherwise = 6
        if (time(NULL) - cmd_time > 6) {
            robot->set_vel(0.0, 0.0);
            state_subtask = 1;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 1) {
        // turn right (to NORTH)
        turn_right(robot);
        if (get_dir(robot->pos_t) == NORTH) {
            state_subtask = 2;
            last_cmd = time(NULL);
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else if (state_subtask == 2) {
        // run until wall < 0.7
        // if (get_dir(robot->pos_t) == NORTH) {
            cout << dist_trav(time(NULL));
            last_cmd = time(NULL);
            last_vel_0 = 2.0;
            last_vel_1 = 2.0;
        // }

        if (robot->range < 0.7) {
            robot->set_vel(0.0, 0.0);
            state_subtask = 3;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 3) {
        // turn right to EAST
        turn_right(robot);
        if (get_dir(robot->pos_t) == EAST) {
            state_subtask = 4;
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else {
        // begin wall follow left
        robot->set_vel(0.0, 0.0);
        state = S_WF_LF;
        return;
    }
    return;
}

void
traverse_door_lf(Robot* robot) {
    if (state_subtask == 0) {
        // drive forward a little bit
        // OBS = 8
        // otherwise = 6
        if (time(NULL) - cmd_time > 6) {
            robot->set_vel(0.0, 0.0);
            state_subtask = 1;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 1) {
        // turn left (to NORTH)
        turn_left(robot);
        if (get_dir(robot->pos_t) == NORTH) {
            state_subtask = 2;
            last_cmd = time(NULL);
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else if (state_subtask == 2) {
        // run until wall < 0.7
        // if (get_dir(robot->pos_t) == NORTH) {
            cout << dist_trav(time(NULL));
            last_cmd = time(NULL);
            last_vel_0 = 2.0;
            last_vel_1 = 2.0;
        // }

        if (robot->range < 0.7) {
            robot->set_vel(0.0, 0.0);
            state_subtask = 3;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 3) {
        // turn right to WEST
        turn_right(robot);
        if (get_dir(robot->pos_t) == WEST) {
            state_subtask = 4;
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else {
        // begin wall follow left
        robot->set_vel(0.0, 0.0);
        state = S_WF_RT;
        return;
    }
    return;
}

void
reverse_do(Robot* robot) {
    if (robot->range < 1.7) {
        update_wall_buf(WALL);
    } else {
        update_wall_buf(NO_WALL);
    }

    if (state_subtask == 0) {
        // wall follow a little bit
        if (time(NULL) - cmd_time > 20) {
            robot->set_vel(0.0, 0.0);
            state_subtask = 1;
            return;
        }
        wall_follow_rt_side(robot);
    } else if (state_subtask == 1) {
        // turn left (to SOUTH)
        turn_left(robot);
        if (get_dir(robot->pos_t) == SOUTH) {
            state_subtask = 2;
            last_cmd = time(NULL);
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else if (state_subtask == 2) {
        // run until wall < 0.7
        // if (get_dir(robot->pos_t) == NORTH) {
            total_dist = total_dist - (2 * dist_trav(time(NULL)));
            last_cmd = time(NULL);
            last_vel_0 = 2.0;
            last_vel_1 = 2.0;
        // }

        if (robot->range < 0.7) {
            robot->set_vel(0.0, 0.0);
            state_subtask = 3;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 3) {
        // turn right to WEST
        turn_right(robot);
        if (get_dir(robot->pos_t) == WEST) {
            state_subtask = 10;
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else if (state_subtask == 4) {
        // turn left (to start_dir)
        turn_left(robot);
        if (get_dir(robot->pos_t) == SOUTH) {
            cmd_time = time(NULL);
            state_subtask = 6;
            last_cmd = time(NULL);
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else if (state_subtask == 5) {
        // drive forward a little bit
        // OBS = 6
        // otherwise = 5
        if (time(NULL) - cmd_time > 5) {
            robot->set_vel(0.0, 0.0);
            start_dir = ((get_dir(robot->pos_t) + 6) % 8);
            state_subtask = 4;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 6) {
        // drive forward a little bit
        // OBS = 6
        // otherwise = 5
        if (time(NULL) - cmd_time > 5) {
            robot->set_vel(0.0, 0.0);
            start_dir = ((get_dir(robot->pos_t) + 6) % 8);
            state_subtask = 7;
            return;
        }
        robot->set_vel(2.0, 2.0);
    } else if (state_subtask == 7) {
        // turn left (to start_dir)
        turn_left(robot);
        if (get_dir(robot->pos_t) == EAST) {
            cmd_time = time(NULL);
            state_subtask = 9;
            last_cmd = time(NULL);
            robot->set_vel(0.0, 0.0);
            return;
        }
    } else if (state_subtask == 9) {
        wall_follow_lf_side_out(robot);
        // if (detect_doorway()) {
        //     cmd_time = time(NULL);
        //     state_subtask = 5;
        // }
        return;
    } else {
        wall_follow_lf_side(robot);
        if (detect_doorway()) {
            cmd_time = time(NULL);
            state_subtask = 5;
        }
        return;
    }
}

bool
validate_PL(Robot* robot) {
    if (robot->range < 1.7) {
        update_wall_buf(WALL);
    } else {
        update_wall_buf(NO_WALL);
    }

    // OBS = 275
    // otherwise = 240
    if ((total_dist > 240) && ((state == S_WF_RT) || (state == S_WF_LF))) {
        state_subtask = 0;
        cmd_time = time(NULL);
        program_mode = REVERSE;
        return false;
    }

    if (state == S_WF_RT) {
        if (get_dir(robot->pos_t) != WEST && get_dir(robot->pos_t) != NW && get_dir(robot->pos_t) != SW) {
            turn_to(robot, WEST);
            return false;
        } else if (detect_doorway()) {
            cout << "DOORWAY DETECTED" << endl;
            turn_to(robot, WEST);
            if (get_dir(robot->pos_t) == WEST) {
                state = S_TR_DOOR_RT;
                state_subtask = 0;
                cmd_time = time(NULL);
                return true;
            }
            return false;
        }
        if ((robot->range < 0.8) && (get_dir(robot->pos_t) == WEST)) {
            turn_to(robot, EAST);
            state = S_WF_LF;
            update_turns_buf(TURN_180);
            return true;
        }
    } else if (state == S_WF_LF) {
        if (get_dir(robot->pos_t) != EAST && get_dir(robot->pos_t) != NE && get_dir(robot->pos_t) != SE) {
            turn_to(robot, EAST);
            return false;
        } else if (detect_doorway()) {
            cout << "DOORWAY DETECTED" << endl;
            turn_to(robot, EAST);
            if (get_dir(robot->pos_t) == EAST) {
                state = S_TR_DOOR_LF;
                state_subtask = 0;
                cmd_time = time(NULL);
                return true;
            }
            return false;
        }
        if ((robot->range < 0.8) && (get_dir(robot->pos_t) == EAST)) {
            turn_to(robot, WEST);
            state = S_WF_RT;
            update_turns_buf(TURN_180);
            return true;
        }
    } else {
        return true;
    }
    return true;
}

// state_subtask = 0;
// cmd_time = time(NULL);
// program_mode = REVERSE;

void
callback(Robot* robot)
{
    time_t seconds;
    seconds = time(NULL);
    // dist_trav(seconds);

    cout << "time " << seconds << endl;
    cout << "start_dir " << start_dir << endl;
    cout << "get_dir " << get_dir(robot->pos_t) << endl;
    cout << "range " << robot->range << endl;
    cout << "state " << state << endl;
    cout << "dist north " << total_dist << endl;
    cout << "program mode " << program_mode << endl;
    cout << "state_subtask " << state_subtask << endl;

    if (robot->at_goal()) {
        robot->set_vel(0.0, 0.0);
        return;
    }

    if (program_mode == FORWARD) {
        if (validate_PL(robot)) {
            if (state == S_TR_DOOR_RT) {
                traverse_door_rt(robot);
                return;
            } else if (state == S_TR_DOOR_LF) {
                traverse_door_lf(robot);
                return;
            } else if (state == S_WF_RT) {
                wall_follow_rt_side(robot);
                return;
            } else if (state == S_WF_LF) {
                wall_follow_lf_side(robot);
                return;
            } else if (state == S_TURN_LEFT) {
                turn_left(robot);
                return;
            } else if (state == S_TURN_RIGHT) {
                turn_right(robot);
                return;
            } else if (state == S_DO_180) {
                do_180(robot);
                return;
            } else { // S_READY
                return;
            }
        }
    } else {
        // program_mode == REVERSE
        reverse_do(robot);
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
