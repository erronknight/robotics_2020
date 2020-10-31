#ifndef VIZ_H
#define VIZ_H

#include <deque>
#include "pathing.hh"

int viz_run(int argc, char **argv);
int viz_hit(long xx, long yy, double color_val);
void draw_robot_pose(long xx, long yy, double theta);
void viz_draw_path(std::deque<pathNode> path);

#endif
