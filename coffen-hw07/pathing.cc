#include <iostream>
#include <math.h>
#include <map>
#include <time.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <deque>
#include <vector>
#include <thread>
#include <mutex>

#include "viz.hh"
#include "robot.hh"
#include "pathing.hh"

using namespace std;

typedef pair<long,long> Posn;

map<Posn, Cell> *occ_gr;

mutex mx_p;
mutex mx_trz;

// if can't get goal vals
const double goal_x = 20.0;
const double goal_y = 0.0;

const long g_x_og = 200; // (long) goal_x * 10
const long g_y_og = 0;

deque<pathNode> open_list;
deque<pathNode> closed_list;
deque<pathNode> edge_nodes;
deque<pathNode> target;

void set_map(std::map<Posn, Cell> *ocg) {
    occ_gr = ocg;
}

// true if space is free and seen on grid
pair<bool, bool> check_grid(long x, long y) {
    std::map<Posn, Cell>::iterator iti = occ_gr->find(make_pair(x, y));
    if (iti != occ_gr->end() && iti->second.seen == true) {
        if (iti->second.occupied < 0) {
            return make_pair(true, true);
        } else {
            return make_pair(true, false);
        }
    }
    return make_pair(false, false);
}

// true if space is free and seen on grid
void check_grid_v(long x, long y, pair<int, int>* vals_tr) {
    std::map<Posn, Cell>::iterator iti = occ_gr->find(make_pair(x, y));
    if (iti != occ_gr->end() && iti->second.seen == true) {
        mx_trz.lock();
        if (iti->second.occupied < 0) {
            vals_tr->first = vals_tr->first + 1;
        } else {
            vals_tr->first = vals_tr->first + 1;
            vals_tr->second = vals_tr->second + 1;
        }
        mx_trz.unlock();
    }
}

// true if space is free and seen on grid for zone of 2s * 2s
// https://thispointer.com/c11-how-to-create-vector-of-thread-objects/
pair<bool, bool> check_grid_zone(long x, long y, int s) {
    pair<int, int> seen_occ;
    seen_occ.first = 0;
    seen_occ.second = 0;
    // thread t1(check_grid_v, x, y, &seen_occ);

    vector<thread> thrs;

    for(int r = 0; r < (2*s); r++) {
        for (int c = 0; c < (2*s); c++) {
            // check_grid_v((x-s+r), (y-s+c), &seen_occ);
            thread t(check_grid_v, (x-s+r), (y-s+c), &seen_occ);
            // t.join();
            thrs.push_back(std::move(t));
        }
    }

    for (thread & th : thrs)
    {
        // If thread Object is Joinable then Join that thread.
        if (th.joinable())
            th.join();
    }

    if (seen_occ.first >= ((4 * s * s) / 2)) {
        if (seen_occ.second < (1.5 * s)) {
            return make_pair(true, true);
        } else {
            return make_pair(true, false);
        }
    }
    return make_pair(false, false);
}

// double free_heur_H(long x, long y) {
//     return 
// }

double heuristic_H(long x1, long y1, long x2, long y2) {
    // Euclidean
    double d = ((double)sqrt((double)(((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)))));

    // Manhattan
    // double d = (double)(abs(x1-x2) + abs(y1-y2));

    return d;
}

bool smol_f(pathNode p1, pathNode p2) {
    return (p1.f < p2.f);
}

void put_neighbor(long x_n, long y_n, long x_g, long y_g, pathNode* p, bool ortho, vector<pathNode>* brs) {
    // pathNode {x, y, f, g, h}
    mx_p.lock();
    pair<bool, bool> cn = check_grid_zone(x_n, y_n, 4);
    mx_p.unlock();
    pathNode n_n = {x_n, y_n, 0, p->g+1, 0, NULL};
    // n_n.g = heuristic_H(st_x, st_y, p.x, p.y+1);
    n_n.h = heuristic_H(n_n.x, n_n.y, x_g, y_g);
    n_n.f = n_n.g + n_n.h;
    mx_p.lock();
    if (cn.first && cn.second) {
        brs->push_back(n_n);
    }
    if (!cn.first && ortho) {
        // on the edge of occupancy grid
        n_n.parent = p;
        edge_nodes.push_back(n_n);
    }
    mx_p.unlock();
}

// get list of neighbors, check against grid restrictions
// allowable nodes are
//      in 4x4 (?) that does not include walls (robot is ~0.35 wide)
//      seen nodes >= 1/2(size)
vector<pathNode> get_neighbors(long st_x, long st_y, pathNode p, long x_g, long y_g, pathNode* padd) {
    vector<pathNode> nbrs;
    int val_zone = 5;

    thread n_n(put_neighbor, p.x, p.y+val_zone, x_g, y_g, padd, true, &nbrs);
    thread n_ne(put_neighbor, p.x+val_zone, p.y+val_zone, x_g, y_g, padd, false, &nbrs);
    thread n_e(put_neighbor, p.x+val_zone, p.y, x_g, y_g, padd, true, &nbrs);
    thread n_se(put_neighbor, p.x+val_zone, p.y-val_zone, x_g, y_g, padd, false, &nbrs);
    thread n_s(put_neighbor, p.x, p.y-val_zone, x_g, y_g, padd, true, &nbrs);
    thread n_sw(put_neighbor, p.x-val_zone, p.y-val_zone, x_g, y_g, padd, false, &nbrs);
    thread n_w(put_neighbor, p.x-val_zone, p.y, x_g, y_g, padd, true, &nbrs);
    thread n_nw(put_neighbor, p.x-val_zone, p.y+val_zone, x_g, y_g, padd, false, &nbrs);

    n_n.join();
    n_ne.join();
    n_e.join();
    n_se.join();
    n_s.join();
    n_sw.join();
    n_w.join();
    n_nw.join();

    // pathNode {x, y, f, g, h}
    // pair<bool, bool> cn = check_grid(p.x, p.y+1);
    // pathNode n_n = {p.x, p.y+1, 0, p.g+1, 0, NULL};
    // // n_n.g = heuristic_H(st_x, st_y, p.x, p.y+1);
    // n_n.h = heuristic_H(n_n.x, n_n.y, x_g, y_g);
    // n_n.f = n_n.g + n_n.h;
    // if (cn.first && cn.second) {
    //     nbrs.push_back(n_n);
    // }
    // if (!cn.first) {
    //     // on the edge of occupancy grid
    //     n_n.parent = padd;
    //     edge_nodes.push_back(n_n);
    // }
    
    // cn = check_grid(p.x+1, p.y+1);
    // if (cn.first && cn.second) {
    //     pathNode n_ne = {p.x+1, p.y+1, 0, p.g+1, 0, NULL};
    //     // n_ne.g = heuristic_H(st_x, st_y, p.x+1, p.y+1);
    //     n_ne.h = heuristic_H(n_ne.x, n_ne.y, x_g, y_g);
    //     n_ne.f = n_ne.g + n_ne.h;
    //     nbrs.push_back(n_ne);
    // }

    // cn = check_grid(p.x+1, p.y);
    // pathNode n_e = {p.x+1, p.y, 0, p.g+1, 0, NULL};
    // // n_e.g = heuristic_H(st_x, st_y, p.x+1, p.y);
    // n_e.h = heuristic_H(n_e.x, n_e.y, x_g, y_g);
    // n_e.f = n_e.g + n_e.h;
    // if (cn.first && cn.second) {
    //     nbrs.push_back(n_e);
    // }
    // if (!cn.first) {
    //     // on the edge of occupancy grid
    //     n_e.parent = padd;
    //     edge_nodes.push_back(n_n);
    // }
    
    // cn = check_grid(p.x+1, p.y-1);
    // if (cn.first && cn.second) {
    //     pathNode n_se = {p.x+1, p.y-1, 0, p.g+1, 0, NULL};
    //     // n_se.g = heuristic_H(st_x, st_y, p.x+1, p.y-1);
    //     n_se.h = heuristic_H(n_se.x, n_se.y, x_g, y_g);
    //     n_se.f = n_se.g + n_se.h;
    //     nbrs.push_back(n_se);
    // }
    
    // cn = check_grid(p.x, p.y-1);
    // pathNode n_s = {p.x, p.y-1, 0, p.g+1, 0, NULL};
    // // n_s.g = heuristic_H(st_x, st_y, p.x, p.y-1);
    // n_s.h = heuristic_H(n_s.x, n_s.y, x_g, y_g);
    // n_s.f = n_s.g + n_s.h;
    // if (cn.first && cn.second) {
    //     nbrs.push_back(n_s);
    // }
    // if (!cn.first) {
    //     // on the edge of occupancy grid
    //     n_s.parent = padd;
    //     edge_nodes.push_back(n_n);
    // }

    
    // cn = check_grid(p.x-1, p.y-1);
    // if (cn.first && cn.second) {
    //     pathNode n_sw = {p.x-1, p.y-1, 0, p.g+1, 0, NULL};
    //     // n_sw.g = heuristic_H(st_x, st_y, p.x-1, p.y-1);
    //     n_sw.h = heuristic_H(n_sw.x, n_sw.y, x_g, y_g);
    //     n_sw.f = n_sw.g + n_sw.h;
    //     nbrs.push_back(n_sw);
    // }
    
    // cn = check_grid(p.x-1, p.y);
    // pathNode n_w = {p.x-1, p.y, 0, p.g+1, 0, NULL};
    // // n_w.g = heuristic_H(st_x, st_y, p.x-1, p.y);
    // n_w.h = heuristic_H(n_w.x, n_w.y, x_g, y_g);
    // n_w.f = n_w.g + n_w.h;
    // if (cn.first && cn.second) {
    //     nbrs.push_back(n_w);
    // }
    // if (!cn.first) {
    //     // on the edge of occupancy grid
    //     n_w.parent = padd;
    //     edge_nodes.push_back(n_n);
    // }

    // cn = check_grid(p.x-1, p.y+1);
    // if (cn.first && cn.second) {
    //     pathNode n_nw = {p.x-1, p.y+1, 0, p.g+1, 0, NULL};
    //     // n_nw.g = heuristic_H(st_x, st_y, p.x-1, p.y+1);
    //     n_nw.h = heuristic_H(n_nw.x, n_nw.y, x_g, y_g);
    //     n_nw.f = n_nw.g + n_nw.h;
    //     nbrs.push_back(n_nw);
    // }

    return nbrs;
}

// set_map before do
deque<pathNode> a_star_do(long x_start, long y_start, long x_goal, long y_goal) {
    target.clear();
    open_list.clear();
    closed_list.clear();
    edge_nodes.clear();
    bool end_conditions_met = false;

    pathNode current = {x_start, y_start, 0, 0, 0};
    current.h = heuristic_H(x_start, y_start, x_goal, y_goal);
    current.f = current.g + current.h;

    open_list.push_front(current);

    while(end_conditions_met || !open_list.empty()) {
        std::sort(open_list.begin(), open_list.end(), smol_f);
        current = open_list.front();
        open_list.pop_front();
        if ((current.x == x_goal) && (current.y == y_goal)) {
            break;
            // end loop. we be #done.
        } else {
            closed_list.push_back(current);
            // Neighbors
            vector<pathNode> neighbors = get_neighbors(x_start, y_start, current, x_goal, y_goal, &closed_list.back());
            while(!neighbors.empty()) {
                pathNode c_nbr = neighbors.back();
                neighbors.pop_back();

                std::deque<pathNode>::iterator iti_cl;
                for (iti_cl = closed_list.begin(); iti_cl != closed_list.end(); ++iti_cl) {
                    if (iti_cl->x == c_nbr.x && iti_cl->y == c_nbr.y) {
                        break;
                    }
                }

                std::deque<pathNode>::iterator iti_ol;
                for (iti_ol = open_list.begin(); iti_ol != open_list.end(); ++iti_ol) {
                    if (iti_ol->x == c_nbr.x && iti_ol->y == c_nbr.y) {
                        break;
                    }
                }

                if (iti_cl != closed_list.end() && c_nbr.g < current.g) {
                    // in closed list
                    c_nbr.parent = &closed_list.back();
                    closed_list.erase(iti_cl);
                    closed_list.push_back(c_nbr);
                } else if (iti_ol != open_list.end() && iti_ol->g > current.g) { //c_nbr.g > current.g) {
                    // in open list
                    c_nbr.parent = &closed_list.back();
                    open_list.erase(iti_ol);
                    open_list.push_back(c_nbr);
                } else if(iti_cl == closed_list.end() && iti_ol == open_list.end()) {
                    c_nbr.parent = &closed_list.back();
                    open_list.push_back(c_nbr);
                }
            }
        }
        // check if end_conditions_met?
    }
    

    // cout << "A STAR ---V----\n";

    // iterate over closed list ?
    // for (auto it = closed_list.cbegin(); it != closed_list.cend(); ++it) {
    //     cout << it->x <<  ' ' << it->y << ' ' << endl;
    // }
    
    // trace current.parent back for path?
    pathNode* n;
    pathNode* min_en = NULL;
    if((current.x == x_goal) && (current.y == y_goal)) {
        n = &current;
    } else {
        // iterate over edge_nodes list
        if (!edge_nodes.empty()) {
            std::sort(edge_nodes.begin(), edge_nodes.end(), smol_f);
            min_en = &edge_nodes.front();
            // for (std::deque<pathNode>::iterator it = edge_nodes.begin(); it != edge_nodes.end(); ++it) {
            //     if (it->parent != NULL) {
            //         if (min_en == NULL) {
            //             min_en = it->parent;
            //         }
            //         if (it->parent->f < min_en->f) {
            //             min_en = it->parent;
            //         }
            //         // cout << it->parent->x << " " << it->parent->y << " " << endl;
            //     }
            // }
        }
        n = min_en;
    }

    while(n != NULL) {
        // cout << n->x << " " << n->y << " " << endl;
        target.push_front(*n);
        n = n->parent;
    }

    target.pop_back();
    // cout << "A STAR ---A----\n";
    return target;
}

/*
https://brilliant.org/wiki/a-star-search/
make an openlist containing only the starting node
make an empty closed list
while (the destination node has not been reached):
    consider the node with the lowest f score in the open list
    if (this node is our destination node) :
        we are finished 
    if not:
        put the current node in the closed list and look at all of its neighbors
        for (each neighbor of the current node):
            if (neighbor has lower g value than current and is in the closed list) :
                replace the neighbor with the new, lower, g value 
                current node is now the neighbor's parent            
            else if (current g value is lower and this neighbor is in the open list ) :
                replace the neighbor with the new, lower, g value 
                change the neighbor's parent to our current node

            else if this neighbor is not in both lists:
                add it to the open list and set its g

---

For each node n in graph
    n.f = inf, n.g = inf
Create empty list
start.g = 0, start.f = H(start), add start to list
while list not empty
    let current = node in list with smallest f value, remove current from list
    if (current == goal node) report success
    for each node, n that is adjacent to current
        if (n.g > (current.g + cost of edge from n to current))
            n.g = current.g + cost of edge from n to current
            n.f = n.g + H(n)
            n.parent = current
            add n to list if it isn't there already
*/

