#ifndef PATHING_H
#define PATHING_H

// #include <iostream>
// #include <math.h>
#include <map>
// #include <time.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <deque>
// #include <vector>

// #include "viz.hh"
// #include "robot.hh"
// #include "pathing.hh"

typedef std::pair<long,long> Posn;

struct pathNode {
    long x; // x position (in .1 resolution value {* 10 and truncated})
    long y; // y position (in .1 resolution value {* 10 and truncated})
    double f; // total estimated cost of path to goal through node
    double g; // cost so far to reach node
    double h; // estimated cost from node to goal (use heuristic_H function)
    pathNode* parent; // parent node
};

struct Cell {
    // Posn loc; // cell location (redundant value)
    bool seen;  // has the cell been seen?
    double occupied; // how occupied it is. negative is free/empty, positive is a wall
    // long not_wall; // how much of empty it is
    // long wall; // how much of wall it is
};

void set_map(std::map<Posn, Cell> *ocg);
double heuristic_H(long x1, long y1, long x2, long y2);
std::deque<pathNode> a_star_do(long x_start, long y_start, long x_goal, long y_goal);

#endif