
#include <iostream>
#include <thread>
#include <math.h>
#include <map>
#include <time.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <deque>
#include <mutex>

#include "viz.hh"
#include "robot.hh"
#include "pathing.hh"

using namespace std;

#define PI  3.14159265

typedef pair<long,long> Posn;

#define LEFT    1
#define RIGHT   2
#define FORWARD 3
#define A_STAR  4
#define GOAL    5

#define NORTH   0
#define NE      1
#define EAST    2
#define SE      3
#define SOUTH   4
#define SW      5
#define WEST    6
#define NW      7

#define OCC_FREE -0.5
#define OCC_OCC 0.9

double speed_modifier = 0.9;

time_t last_on_wall = 0;
time_t timeout = 0;
time_t start_time = 0;

int last_wf = 1;

long x_minute = 0;
long y_minute = 0;

int drive_state = FORWARD;
int prev_dir = NORTH;
double prev_ang = 0;
int turns = 0;

double x_pos_vals[20];
double y_pos_vals[20];

double x_mean = 0;
double x_var = 0;
double y_mean = 0;
double y_var = 0;

struct Pose {
    long xx;
    long yy;
    double theta;
    int dir;
};

struct Pose2 {
    double xx;
    double yy;
    double theta;
    int dir;
};

struct Err_corr {
    double x_coef;
    double x_int;
    double y_coef;
    double y_int;
};

mutex mx_target;
mutex mx_grid;

map<Posn, Cell> o_grid; // global occupancy grid

Pose prev_pose = {0, 0, 0, 0};
Pose exp_pose = {0, 0, 0, 0};
Pose2 pose_change = {0, 0, 0, 0};
Pose2 acc_pose_change = {0, 0, 0, 0};

Err_corr err = {1, 0, 1, 0};

double last_velo_0 = 0;
double last_velo_1 = 0;
time_t last_cmd = 0;

// PATH {

// if can't get goal vals
const double goal_x = 20.0;
const double goal_y = 0.0;

const long g_x_og = 200; // (long) goal_x * 10
const long g_y_og = 0;

bool ready_a = false;
bool read_d = false;

double thet_aim = 0;
int dir_aim = 0;

deque<pathNode> targets;
pathNode current_target = {g_x_og, g_y_og, 0, 0, 0, NULL};
bool done_current_target = false;
time_t targets_age;

void set_current_target() {
    mx_target.lock();
    current_target = targets.front();
    targets.pop_front();
    mx_target.unlock();
}

// } PATH

// 700 by 700 approx. [-35, 35] x y (increments of 0.1)
// See the *10 in get_x_occ and get_y_occ

// raw values (not adjusted for resolution)
void update_pos_bufs(double xx, double yy) {
    int length = 20;
    for (int i = 1; i < length; i++) {
        x_pos_vals[i-1] = x_pos_vals[i];
        y_pos_vals[i-1] = y_pos_vals[i];
    }
    x_pos_vals[length - 1] = xx - acc_pose_change.xx;
    // cout << "EEEEX " <<  x_pos_vals[length - 1] << endl;
    y_pos_vals[length - 1] = yy - acc_pose_change.yy;
    // cout << "EEEEY " <<  y_pos_vals[length - 1] << endl;
}

void set_mean() {
    double acc_x = 0;
    double acc_y = 0;
    int i = 0;
    while (i < 20) {
        if (x_pos_vals[i] == 0 && x_pos_vals[i] == 0) {
            break;
        }
        acc_x = acc_x + x_pos_vals[i];
        acc_y = acc_y + y_pos_vals[i];
        i++;
    }
    // cout << "acc x " << acc_x << endl;
    // cout << "acc y " << acc_y << endl;
    x_mean = acc_x / double(i + 1);
    y_mean = acc_y / double(i + 1);
}

void set_var() {
    double acc_x = 0;
    double acc_y = 0;
    int i = 0;
    while (i < 20) {
        if (x_pos_vals[i] == 0 && x_pos_vals[i] == 0) {
            break;
        }
        acc_x = acc_x + pow((x_pos_vals[i] - x_mean), 2.0);
        acc_y = acc_y + pow((y_pos_vals[i] - y_mean), 2.0);
        i++;
    }
    x_var = acc_x / double(i);
    y_var = acc_y / double(i);
}

int get_dir(double thet) {
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

int get_x_occ(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * x_rob) + (10 * d_hit * (cos(t_rob + t_hit))));
}

int get_y_occ(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * y_rob) + (10 * d_hit * (sin(t_rob + t_hit))));
}

int get_x_occ_free(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * x_rob) + (9 * d_hit * (cos(t_rob + t_hit))));
}

int get_y_occ_free(double x_rob, double y_rob, double t_rob, double d_hit, double t_hit) {
    return ceil((10 * y_rob) + (9 * d_hit * (sin(t_rob + t_hit))));
}

double get_x_r(Robot* robot) {
    // cout << "ps x " << pose_change.xx << endl;
    // cout << "x-me " << x_mean << endl;
    // cout << "robx " << robot->pos_x << endl;
    acc_pose_change.xx = acc_pose_change.xx + pose_change.xx;
    return x_mean + acc_pose_change.xx;
    // return x_mean;
    // return robot->raw_x;
}

double get_y_r(Robot* robot) {
    // cout << "ps y " << pose_change.yy << endl;
    // cout << "y-me " << y_mean << endl;
    // cout << "roby " << robot->pos_y << endl;
    acc_pose_change.yy = acc_pose_change.yy + pose_change.yy;
    return y_mean + acc_pose_change.yy;
    // return y_mean;
    // return robot->raw_y;
}

void set_expected_raw(time_t new_time, Robot* robot) {
    int time_diff = new_time - last_cmd;
    double velo_avg = (last_velo_0 + last_velo_1) / 2;

    double dist = 0.9 * velo_avg * (time_diff / 10);

    pose_change.xx = ((dist * cos(robot->pos_t)));
    pose_change.yy = ((dist * sin(robot->pos_t)));
    exp_pose.xx = (ceil(prev_pose.xx + (10 * (dist * cos(robot->pos_t)))));
    exp_pose.yy = (ceil(prev_pose.yy + (10 * (dist * sin(robot->pos_t)))));
    exp_pose.theta = robot->pos_t;
    exp_pose.dir = get_dir(robot->pos_t);
}

// return if the expected xx/yy/theta match (within bounds)
bool here_odom(Robot *robot) {
    set_expected_raw(time(NULL), robot);
    if ((abs(ceil(10 * robot->pos_x) - exp_pose.xx) < 5) &&
        (abs(ceil(10 * robot->pos_y) - exp_pose.yy) < 5)) {
            prev_pose.xx = robot->pos_x;
            prev_pose.yy = robot->pos_y;
            prev_pose.theta = robot->pos_t;
            return true;
    } else {
        if ((abs(ceil(10 * get_x_r(robot)) - exp_pose.xx) < 5) &&
            (abs(ceil(10 * get_y_r(robot)) - exp_pose.yy) < 5)) {
            prev_pose.xx = ceil(10 * get_x_r(robot));
            prev_pose.yy = ceil(10 * get_y_r(robot));
        } else {
            // update_error_vals(robot);
            prev_pose.xx = ceil(10 * get_x_r(robot));
            prev_pose.yy = ceil(10 * get_y_r(robot));
        }
        prev_pose.theta = robot->pos_t;
        return false;
    }
}

void draw_current(long xx, long yy, double occ_add) {
    double occ_val = occ_add;
    std::map<Posn, Cell>::iterator iti = o_grid.find(make_pair(xx, yy));
    if (iti != o_grid.end()) {
        occ_val = clamp(-5, iti->second.occupied + occ_add, 5);
        o_grid.erase(iti);
    }
    Cell ci = {true, occ_val};
    o_grid.emplace(make_pair(xx, yy), ci); // emplace checks if key is unique
    viz_hit(xx, yy, occ_val);
}

void bresenham_free(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (y2 - y1); 
    int slope_error_new = m_new - (x2 - x1); 
    
    for (int x = x1, y = y1; x <= x2; x++) 
    { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) 
        { 
            y++; 
            slope_error_new -= 2 * (x2 - x1); 
        } 
    }
}

void bresenham_free_big(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (x2 - x1); 
    int slope_error_new = m_new - (y2 - y1); 
    
    for (int x = x1, y = y1; y <= y2; y++) 
    { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) 
        { 
            x++; 
            slope_error_new -= 2 * (y2 - y1); 
        } 
    }
}

void bresenham_free_rev_big(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (x2 - x1); 
    int slope_error_new = m_new - (y1 - y2); 
    
    for (int x = x2, y = y2; y <= y1; y++) 
    { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) 
        { 
            x--; 
            slope_error_new -= 2 * (y1 - y2); 
        } 
    }
}

void bresenham_free_rev(long x1, long y1, long x2, long y2) {
    int m_new = 2 * (y1 - y2); 
    int slope_error_new = m_new - (x2 - x1); 
    
    for (int x = x2, y = y2; x >= x1; x--) 
    { 
        draw_current(x, y, OCC_FREE);

        // Add slope to increment angle formed 
        slope_error_new += m_new; 

        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new >= 0) 
        { 
            y++; 
            slope_error_new -= 2 * (x2 - x1); 
        } 
    }
}

void bs_line_free(long x1, long y1, long x2, long y2) {
    if (x1 != x2 || y1 != y2) {
        float slo = (float(y2) - float(y1)) / (float(x2) - float(x1));
        float sloREV = (float(y1) - float(y2)) / (float(x1) - float(x2));
        if (x1 <= x2 && y1 <= y2 && slo <= 1.0) {
            bresenham_free(x1, y1, x2, y2);
        } else if (x1 <= x2 && y1 <= y2 && slo > 1.0) {
            bresenham_free_big(x1, y1, x2, y2);
        } else if (x1 >= x2 && y1 >= y2 && sloREV <= 1.0) {
            bresenham_free(x2, y2, x1, y1);
        } else if (x1 >= x2 && y1 >= y2 && sloREV > 1.0) {
            bresenham_free_big(x2, y2, x1, y1);
        } else if (x1 <= x2 && y1 >= y2 && (slo < 0 && slo >= -1)) {
            bresenham_free_rev(x1, y1, x2, y2);
        } else if (x1 <= x2 && y1 >= y2 && (slo < -1)) {
            bresenham_free_rev_big(x1, y1, x2, y2);
        } else if (x1 >= x2 && y1 <= y2 && (sloREV < 0 && sloREV >= -1)) {
            bresenham_free_rev(x2, y2, x1, y1);
        } else if (x1 >= x2 && y1 <= y2 && (sloREV < -1)) {
            bresenham_free_rev_big(x2, y2, x1, y1);
        }
    } else {
        draw_current(x1, y1, OCC_FREE);
    }
}

void map_make(Robot* robot) {
    // for (std::map<Posn, Cell>::iterator it=o_grid.begin(); it!=o_grid.end(); ++it) {
    //     cout << it->first.first << " , " << it->first.second << " => " << it->second.occupied << '\n';
    // }
    update_pos_bufs(robot->pos_x, robot->pos_y);
    // update_pos_bufs(robot->raw_x, robot->raw_y);

    set_mean();
    set_var();

    double xr = get_x_r(robot);
    double yr = get_y_r(robot);
    for (auto hit : robot->ranges) {
        // cout << hit.range << " @ " << hit.angle << endl;
        if (hit.range < 100) {
            long xx = get_x_occ(xr, yr, robot->pos_t, hit.range, hit.angle);
            long yy = get_y_occ(xr, yr, robot->pos_t, hit.range, hit.angle);

            long xx2 = get_x_occ_free(xr, yr, robot->pos_t, hit.range, hit.angle);
            long yy2 = get_y_occ_free(xr, yr, robot->pos_t, hit.range, hit.angle);
            // bresenham + not walls
            bs_line_free((ceil(10 * xr)), (ceil(10 * yr)), xx2, yy2);

            draw_current(xx, yy, OCC_OCC);

        } else { // infinite range --> range goes to 2 meters.
            long xx = get_x_occ_free(xr, yr, robot->pos_t, 2.0, hit.angle);
            long yy = get_y_occ_free(xr, yr, robot->pos_t, 2.0, hit.angle);
            // bresenham + not walls
            bs_line_free((ceil(10 * xr)), (ceil(10 * yr)), xx, yy);
        }
    }
    draw_robot_pose((ceil(10 * xr)), (ceil(10 * yr)), robot->pos_t);
    if (targets.size() < 2) {
        ready_a = true;
    }
}

void drive_forward(Robot* robot, double spd) {
    int current_dir = get_dir(robot->pos_t);
    if (prev_dir != current_dir) {
        double ang_diff = robot->pos_t - prev_ang;
        robot->set_vel(spd + ang_diff, spd - ang_diff);
        last_velo_0 = spd + ang_diff;
        last_velo_1 = spd - ang_diff;
    } else {
        robot->set_vel(spd, spd);
        last_velo_0 = spd;
        last_velo_1 = spd;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
    }
    return;
}

void wall_follow_right(Robot* robot) {
    // RIGHT
    double min_range = 10;
    for (auto hit : robot->ranges) {
        if (hit.range < 100 && (hit.angle <= (PI/4)) && hit.angle >= (-1 * PI/4)) {
            if (hit.range < min_range) {
                min_range = hit.range;
            }
        }
    }

// RIGHT
    if (min_range < 0.8) {
        robot->set_vel(speed_modifier* -3.0, speed_modifier* 2.0);
        last_velo_0 = speed_modifier* -3.0;
        last_velo_1 = speed_modifier* 2.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
        last_cmd = time(NULL);
        last_on_wall = time(NULL);
        return;
    }
// RIGHT
    if ((robot->ranges[5].range < 1.5 || robot->ranges[1].range < 1.5) &&
            robot->ranges[2].range > 100 && robot->ranges[3].range > 100 &&
            robot->ranges[4].range > 100) {
                drive_forward(robot, 4.0 * speed_modifier);
                last_cmd = time(NULL);
                last_on_wall = time(NULL);
                return;
            }

// RIGHT
    if (min_range > 1.9 && (time(NULL) - last_on_wall) < 5) {
        robot->set_vel(speed_modifier* 3.0, speed_modifier * -2.0);
        last_velo_0 = speed_modifier* 3.0;
        last_velo_1 = speed_modifier* -2.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
    }
    else {
        drive_forward(robot, 4.0 * speed_modifier);
    }
    // RIGHT
    last_cmd = time(NULL);
    return;
}

void wall_follow_left(Robot* robot) {
    // LEFT
    double min_range = 10;
    for (auto hit : robot->ranges) {
        if (hit.range < 100 && (hit.angle <= (PI/4)) && hit.angle > (-1 * PI/4)) {
            if (hit.range < min_range) {
                min_range = hit.range;
            }
        }
    }
// LEFT
    if (min_range < 0.8) {
        robot->set_vel(speed_modifier* 2.0, speed_modifier* -3.0);
        last_velo_0 = speed_modifier* 2.0;
        last_velo_1 = speed_modifier* -3.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
        last_cmd = time(NULL);
        last_on_wall = time(NULL);
        return;
    }
// LEFT
    if ((robot->ranges[5].range < 1.5 || robot->ranges[1].range < 1.5) &&
            robot->ranges[2].range > 100 && robot->ranges[3].range > 100 &&
            robot->ranges[4].range > 100) {
                drive_forward(robot, 4.0 * speed_modifier);
                last_cmd = time(NULL);
                last_on_wall = time(NULL);
                return;
            }
// LEFT
    if (min_range > 1.9 && (time(NULL) - last_on_wall) < 5) {
        robot->set_vel(speed_modifier* -2.0, speed_modifier *3.0);
        last_velo_0 = speed_modifier* -2.0;
        last_velo_1 = speed_modifier* 3.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
    }
    else {
        drive_forward(robot, 4.0 * speed_modifier);
    }
    // LEFT
    last_cmd = time(NULL);
    return;
}

void aim_drive(Robot* robot, long xr, long yr, int rob_dir, long targ_x, long targ_y) {
    thet_aim = atan2(targ_y - yr, targ_x - xr);
    cout << thet_aim << endl;
    dir_aim = get_dir(thet_aim);

    double ang_diff = thet_aim - robot->pos_t;

    if (dir_aim == rob_dir && abs(thet_aim - robot->pos_t) < 0.3) {
        drive_forward(robot, 4.0 * speed_modifier);
        last_cmd = time(NULL);
        return;
    } else if (abs(ang_diff) < (PI/3)) {
        if (thet_aim - robot->pos_t > 0) {
            // turn right
            robot->set_vel(speed_modifier* (-2.0 + ang_diff), speed_modifier * (2.0 + ang_diff));
            last_velo_0 = speed_modifier* (-2.0 + ang_diff);
            last_velo_1 = speed_modifier* (2.0 + ang_diff);
            prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
            prev_ang = robot->pos_t;
            last_cmd = time(NULL);
            return;
        } else {
            // turn left
            robot->set_vel(speed_modifier* (2.0 + ang_diff), speed_modifier * (-2.0 + ang_diff));
            last_velo_0 = speed_modifier* (2.0 + ang_diff);
            last_velo_1 = speed_modifier* (-2.0 + ang_diff);
            prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
            prev_ang = robot->pos_t;
            last_cmd = time(NULL);
            return;
        }
    } else if (thet_aim - robot->pos_t > 0) {
        // turn right
        robot->set_vel(speed_modifier* -2.0, speed_modifier * 2.0);
        last_velo_0 = speed_modifier* -2.0;
        last_velo_1 = speed_modifier* 2.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
        last_cmd = time(NULL);
        return;
    } else {
        // turn left
        robot->set_vel(speed_modifier* 2.0, speed_modifier * -2.0);
        last_velo_0 = speed_modifier* 2.0;
        last_velo_1 = speed_modifier* -2.0;
        prev_dir = floor(get_dir(robot->pos_t) / 2) * 2;
        prev_ang = robot->pos_t;
        last_cmd = time(NULL);
        return;
    }
}

void goal_drive(Robot* robot) {
    long xr = (10 * get_x_r(robot));
    long yr = (10 * get_y_r(robot));
    int rob_dir = get_dir(robot->pos_t);

    aim_drive(robot, xr, yr, rob_dir, g_x_og, g_y_og);
}

void drive_a_star(Robot* robot) {
    double min_range = 10;
    for (auto hit : robot->ranges) {
        if (hit.range < 100 && (hit.angle <= (PI/4)) && hit.angle > (-1 * PI/4)) {
            if (hit.range < min_range) {
                min_range = hit.range;
            }
        }
    }

    cout << "aim: " << current_target.x << " " << current_target.y << endl;
    long xr = (10 * get_x_r(robot));
    long yr = (10 * get_y_r(robot));
    int rob_dir = get_dir(robot->pos_t);

    if (abs(current_target.x - xr) < 2 && abs(current_target.y - yr) < 2 ) {
        done_current_target = true;
    }

    while(!targets.empty() && done_current_target) {
        set_current_target();
        done_current_target = (abs(current_target.x - xr) < 2 && abs(current_target.y - yr) < 2);
    }
    
    if((targets.empty() && done_current_target) || min_range < 0.6) {
        if (abs(xr - g_x_og) < 50 && abs(yr - g_y_og) < 50) {
            // aim_to_goal(Robot* robot);
            drive_state = GOAL;
        } else {
            if ((  robot->ranges[5].range < 1.9 ||
                    robot->ranges[4].range < 1.9 ||
                    robot->ranges[6].range < 1.9) &&
            robot->ranges[0].range > 100 &&
            robot->ranges[2].range > 100 &&
            robot->ranges[1].range > 100) {
                    last_wf = LEFT;
                    drive_state = LEFT;
            } else if ((  robot->ranges[0].range < 1.9 ||
                            robot->ranges[1].range < 1.9 ||
                            robot->ranges[2].range < 1.9) &&
            robot->ranges[4].range > 100 &&
            robot->ranges[5].range > 100 &&
            robot->ranges[6].range > 100) {
                    last_wf = RIGHT;
                    drive_state = RIGHT;
            } else if (robot->ranges[0].range > 100 &&
                        robot->ranges[1].range > 100 &&
                        robot->ranges[2].range > 100 &&
                        robot->ranges[4].range > 100 &&
                        robot->ranges[5].range > 100 &&
                        robot->ranges[6].range > 100) {
                if (last_wf == LEFT) {
                    last_wf = RIGHT;
                    drive_state = RIGHT;
                } else {
                    last_wf = LEFT;
                    drive_state = LEFT;
                }
            } else {
                int cnt = 0;
                cnt = (int)(robot->ranges[0].range < 1.9) +
                        (int)(robot->ranges[1].range < 1.9) + 
                        (int)(robot->ranges[2].range < 1.9) -
                        (int)(robot->ranges[4].range < 1.9) -
                        (int)(robot->ranges[5].range < 1.9) -
                        (int)(robot->ranges[6].range < 1.9);
                if (cnt > 0) {
                    last_wf = RIGHT;
                    drive_state = RIGHT;
                } else {
                    last_wf = LEFT;
                    drive_state = LEFT;
                }
            }
        }
    }
    aim_drive(robot, xr, yr, rob_dir, current_target.x, current_target.y);
}

void move_robot(Robot* robot) {
    // aim_drive(robot, 4.0 * speed_modifier);
    // float lft = clamp(0.0, 1.5 *robot->ranges[2].range, 5.0);
    // float fwd = clamp(0.0, 1.5 *robot->ranges[3].range, 5.0);
    // float rgt = clamp(0.0, 1.5 *robot->ranges[4].range, 5.0);
    // cout << "lft,fwd,rgt = "
    //      << lft << ","
    //      << fwd << ","
    //      << rgt << endl;

    // float spd = fwd - 1.0;
    // float trn = clamp(-3.0, lft - rgt, 3.0);

    // if (fwd < 1.2) {
    //   spd = 0;
    //   trn = 1;
    // }

    // cout << "spd,trn = " << spd << "," << trn << endl;
    // robot->set_vel(spd + trn, spd - trn);

    double min_range = 10;
    for (auto hit : robot->ranges) {
        if (hit.range < 100 && (hit.angle <= (PI/4)) && hit.angle > (-1 * PI/4)) {
            if (hit.range < min_range) {
                min_range = hit.range;
            }
        }
    }

    if (drive_state == A_STAR) {
        if ((robot->ranges[3].range < 0.8 && abs(thet_aim - robot->pos_t) < 0.3) ||
            (((time(NULL) - targets_age) > 60) && targets.size() < 6) ||
            min_range < 0.6) {
            targets.clear();
            // done_current_target = true;
        }
    }

    if (!targets.empty()) {
        drive_state = A_STAR;
    }

    if (!targets.empty() && drive_state != A_STAR && !done_current_target) {
        set_current_target();
        drive_state = A_STAR;
    }

    cout << drive_state << endl;

    if (drive_state == LEFT) {
        wall_follow_left(robot);
    } else if (drive_state == RIGHT) {
        wall_follow_right(robot);
    } else if (drive_state == A_STAR) {
        drive_a_star(robot);
    } else if (drive_state == GOAL) {
        goal_drive(robot);
    } else {
        // FORWARD
        drive_forward(robot, 4.0 * speed_modifier);
        last_cmd = time(NULL);
        drive_state = LEFT;
    }

}

void callback(Robot* robot) {
    // viz_draw_path(targets);
    cout << "------------------------------" << endl;
    // cout << "stamp " << robot->stamp << endl;
    cout << "pos X " << robot->pos_x << endl;
    cout << "pos Y " << robot->pos_y << endl;
    cout << "pos T " << robot->pos_t << endl;

    if (prev_pose.xx != 0 && prev_pose.yy != 0) {
        // here_odom(robot);
        prev_pose.theta = robot->pos_t;
        set_expected_raw(time(NULL), robot);
    } else {
        acc_pose_change.xx = robot->pos_x;
        acc_pose_change.yy = robot->pos_y;
        prev_pose.xx = (ceil(10 * robot->pos_x));
        prev_pose.yy = (ceil(10 * robot->pos_y));
        prev_pose.theta = robot->pos_t;
        prev_pose.dir = get_dir(robot->pos_t);
        set_expected_raw(time(NULL), robot);
    }

    map_make(robot);
    if (read_d) {
        viz_draw_path(targets);
        read_d = false;
    }

    if (robot->ranges.size() < 5) {
        return;
    }

    if ((time(NULL) - start_time < 10) || (time(NULL) - timeout) > 40) {
        x_minute = (10* get_x_r(robot));
        y_minute = (10* get_y_r(robot));
        timeout = time(NULL);
    }

    if ((abs((10* get_x_r(robot)) - x_minute) < 30) &&
        (abs((10* get_x_r(robot)) - x_minute) < 30) &&
        (time(NULL) - timeout) > 40) {
        start_time = time(NULL);
        timeout = time(NULL);
        targets.clear();
    }

    if (time(NULL) - start_time > 90) {
        move_robot(robot);
    } else {
        wall_follow_left(robot);
    }
}

void robot_thread(Robot* robot) {
    robot->do_stuff();
}

void path_find_thread(Robot *robot) {
    set_map(&o_grid);
    while (1) {
        if (ready_a) {
            double xr = get_x_r(robot);
            double yr = get_y_r(robot);
            mx_grid.lock();
            mx_grid.unlock();
            mx_target.lock();
            targets.clear();
            targets = a_star_do(long(10* xr), long(10* yr), 200, 0);
            targets_age = time(NULL);
            mx_target.unlock();
            set_current_target();
            ready_a = false;
            read_d = true;
        // } else {
        //     usleep(30);
        }
    }
}

int main(int argc, char* argv[]) {
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread pthr(path_find_thread, &robot);
    start_time = time(NULL);

    return viz_run(argc, argv);
}
