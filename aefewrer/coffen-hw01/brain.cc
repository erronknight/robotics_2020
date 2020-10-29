
#include <iostream>
#include <math.h>

#include "robot.hh"

#define PI 3.14159265

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

// clamp function from in class demo, Professor Tuck
double
clamp(double x, double min, double max)
{
    if (x < min)
        {x = min;}
    else if (x > max)
        {x = max;}
    return x;
}

void
callback(Robot* robot)
{
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;

    // Determine target heading
    double target_heading = atan2(dy, dx);

    // Determine angle differential between robot and target
    double turn_val = robot->pos_t - target_heading;
    
    // If turning more than Pi radians in a direction,
    //    modify to turn equivalently in other direction
    if (abs(turn_val) > PI) {
        if (turn_val < 0) {
            // negative value
            turn_val = 2 * PI + turn_val;
        }
        else {
            // positive value
            turn_val = turn_val - 2 * PI;
        }
    }

    // Determine if at goal
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    bool turn = false;
    int turn_dir = 1; // 1=to_left, -1=to_right

    // Weighting values for lidar hits
    int left_weight = 0;
    int right_weight = 0;

    for (LaserHit hit : robot->hits) {
        if (hit.range < 1.5) {
            // Tier 1: In front of robot ( -Pi/6, Pi/6 )
            if (hit.angle < (PI / 6) && hit.angle > (-1 *(PI / 6))) {
                turn = true;
            
                if (hit.angle < (PI / 6) && hit.angle > 0) {
                    // right side
                    right_weight = right_weight + 3;
                }
                else {
                    // left side
                    left_weight = left_weight + 3;
                }
            }
            
            // Tier 2: Indirectly in front of robot ( -Pi/3, Pi/3 )
            else if (hit.angle < (PI / 3) && hit.angle > (-1 *(PI / 3))) {
                turn = true;

                if (hit.angle < (PI / 3) && hit.angle > 0) {
                    // right side
                    right_weight = right_weight + 2;
                }
                else {
                    // left side
                    left_weight = left_weight + 2;
                }
            }

            // Tier 3: Sides of robot ( -Pi/2, Pi/2 )
            else if (hit.angle < (PI / 2) && hit.angle > (-1 *(PI / 2))) {
                if (hit.angle < (PI / 2) && hit.angle > 0) {
                    // right side
                    right_weight = right_weight + 1;
                }
                else {
                    // left side
                    left_weight = left_weight + 1;
                }
            }
        }
    }

    // Target Heading bias to weights
    if (turn_val < 0) {
        right_weight = right_weight + 2;
    }
    else {
        left_weight = left_weight + 2;
    }

    // Determine turn direction
    if (left_weight != right_weight) {
        if (left_weight > right_weight) {
            turn_dir = -1;
        }
        else {
            turn_dir = 1;
        }
    }

    // Motion Command Publishing
    if (turn) {
        robot->set_vel(4.0);
        robot->set_turn(turn_dir * 0.5);
    }
    else {
        robot->set_vel(5.0);
        robot->set_turn(clamp(turn_val, -0.5, 0.5));
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
