#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {}

double Search::heuristic(const Node &node, const Map &map, const EnvironmentOptions &options, int algorithm)
{
    if (algorithm == CN_SP_ST_BFS || algorithm == CN_SP_ST_DIJK) {
        return 0;
    }

    if (options.metrictype == CN_SP_MT_MANH) {
        return map.getCellSize() * (abs(node.i - map.getGoalX()) + abs(node.j - map.getGoalY()));
    }

    if (options.metrictype == CN_SP_MT_CHEB) {
        return map.getCellSize() * std::max(abs(node.i - map.getGoalX()), abs(node.j - map.getGoalY()));
    }

    if (options.metrictype == CN_SP_MT_EUCL || options.metrictype == CN_SP_MT_DIAG) {
        return map.getCellSize() * sqrt(pow(node.i - map.getGoalX(), 2) + pow(node.j - map.getGoalY(), 2));
    }

    return 0;
}

int argmin(const std::vector<Node *> &OPEN) {
    if (OPEN.empty())
        return -1;
    int result = 0;
    double result_f = OPEN[0]->F;
    for (int i = 1; i < (int)OPEN.size(); ++i) {
        if (OPEN[i]->F < result_f) {
            result = i;
            result_f = OPEN[i]->F;
        }
    }
    return result;
}

std::vector<Node *> Search::generateAllSuccs(Node *cur, const Map &map, const EnvironmentOptions &options, int algorithm) {
    std::vector<Node *> SUCC = {};
    if (map.CellOnGrid(cur->i, cur->j - 1)) {
        if (map.CellIsTraversable(cur->i, cur->j - 1)) {
            Node *add = new Node;
            add->i = cur->i;
            add->j = cur->j - 1;
            add->g = cur->g + map.getCellSize();
            add->H = Search::heuristic(*add, map, options, algorithm);
            add->F = add->g + add->H;
            add->parent = cur;
            SUCC.push_back(add);
        }
    }
    if (map.CellOnGrid(cur->i, cur->j + 1)) {
        if (map.CellIsTraversable(cur->i, cur->j + 1)) {
            Node *add = new Node;
            add->i = cur->i;
            add->j = cur->j + 1;
            add->g = cur->g + map.getCellSize();
            add->H = Search::heuristic(*add, map, options, algorithm);
            add->F = add->g + add->H;
            add->parent = cur;
            SUCC.push_back(add);
        }
    }
    if (map.CellOnGrid(cur->i - 1, cur->j)) {
        if (map.CellIsTraversable(cur->i - 1, cur->j)) {
            Node *add = new Node;
            add->i = cur->i - 1;
            add->j = cur->j;
            add->g = cur->g + map.getCellSize();
            add->H = Search::heuristic(*add, map, options, algorithm);
            add->F = add->g + add->H;
            add->parent = cur;
            SUCC.push_back(add);
        }
    }
    if (map.CellOnGrid(cur->i + 1, cur->j)) {
        if (map.CellIsTraversable(cur->i + 1, cur->j)) {
            Node *add = new Node;
            add->i = cur->i + 1;
            add->j = cur->j;
            add->g = cur->g + map.getCellSize();
            add->H = Search::heuristic(*add, map, options, algorithm);
            add->F = add->g + add->H;
            add->parent = cur;
            SUCC.push_back(add);
        }
    }
    if (options.allowdiagonal) {
        if (map.CellOnGrid(cur->i + 1, cur->j - 1)) {
            if (map.CellIsTraversable(cur->i + 1, cur->j - 1)) {
                if ((map.CellIsTraversable(cur->i, cur->j - 1) && map.CellIsTraversable(cur->i + 1, cur->j)) ||
                    (map.CellIsTraversable(cur->i, cur->j - 1) && options.cutcorners) ||
                    (map.CellIsTraversable(cur->i + 1, cur->j) && options.cutcorners) ||
                    (map.CellIsObstacle(cur->i, cur->j - 1) && map.CellIsObstacle(cur->i + 1, cur->j) && options.allowsqueeze)) {
                    Node *add = new Node;
                    add->i = cur->i + 1;
                    add->j = cur->j - 1;
                    add->g = cur->g + CN_SQRT_TWO * map.getCellSize();
                    add->H = Search::heuristic(*add, map, options, algorithm);
                    add->F = add->g + add->H;
                    add->parent = cur;
                    SUCC.push_back(add);
                }
            }
        }
        if (map.CellOnGrid(cur->i - 1, cur->j - 1)) {
            if (map.CellIsTraversable(cur->i - 1, cur->j - 1)) {
                if ((map.CellIsTraversable(cur->i, cur->j - 1) && map.CellIsTraversable(cur->i - 1, cur->j)) ||
                    (map.CellIsTraversable(cur->i, cur->j - 1) && options.cutcorners) ||
                    (map.CellIsTraversable(cur->i - 1, cur->j) && options.cutcorners) ||
                    (map.CellIsObstacle(cur->i, cur->j - 1) && map.CellIsObstacle(cur->i - 1, cur->j) && options.allowsqueeze)) {
                    Node *add = new Node;
                    add->i = cur->i - 1;
                    add->j = cur->j - 1;
                    add->g = cur->g + CN_SQRT_TWO * map.getCellSize();
                    add->H = Search::heuristic(*add, map, options, algorithm);
                    add->F = add->g + add->H;
                    add->parent = cur;
                    SUCC.push_back(add);
                }
            }
        }
        if (map.CellOnGrid(cur->i + 1, cur->j + 1)) {
            if (map.CellIsTraversable(cur->i + 1, cur->j + 1)) {
                if ((map.CellIsTraversable(cur->i + 1, cur->j) && map.CellIsTraversable(cur->i, cur->j + 1)) ||
                    (map.CellIsTraversable(cur->i + 1, cur->j) && options.cutcorners) ||
                    (map.CellIsTraversable(cur->i, cur->j + 1) && options.cutcorners) ||
                    (map.CellIsObstacle(cur->i + 1, cur->j) && map.CellIsObstacle(cur->i, cur->j + 1) && options.allowsqueeze)) {
                    Node *add = new Node;
                    add->i = cur->i + 1;
                    add->j = cur->j + 1;
                    add->g = cur->g + CN_SQRT_TWO * map.getCellSize();
                    add->H = Search::heuristic(*add, map, options, algorithm);
                    add->F = add->g + add->H;
                    add->parent = cur;
                    SUCC.push_back(add);
                }
            }
        }
        if (map.CellOnGrid(cur->i - 1, cur->j + 1)) {
            if (map.CellIsTraversable(cur->i - 1, cur->j + 1)) {
                if ((map.CellIsTraversable(cur->i - 1, cur->j) && map.CellIsTraversable(cur->i, cur->j + 1)) ||
                    (map.CellIsTraversable(cur->i - 1, cur->j) && options.cutcorners) ||
                    (map.CellIsTraversable(cur->i, cur->j + 1) && options.cutcorners) ||
                    (map.CellIsObstacle(cur->i - 1, cur->j) && map.CellIsObstacle(cur->i, cur->j + 1) && options.allowsqueeze)) {
                    Node *add = new Node;
                    add->i = cur->i - 1;
                    add->j = cur->j + 1;
                    add->g = cur->g + CN_SQRT_TWO * map.getCellSize();
                    add->H = Search::heuristic(*add, map, options, algorithm);
                    add->F = add->g + add->H;
                    add->parent = cur;
                    SUCC.push_back(add);
                }
            }
        }
    }
    return SUCC;
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options, int algorithm)
{
    //need to implement
    Node* start = new Node;
    start->i = map.getStartX();
    start->j = map.getStartY();
    start->g = 0;
    start->H = heuristic(*start, map, options, algorithm);
    start->F = start->H;
    start->parent = nullptr;

    std::vector<Node *> OPEN = {start};
    std::vector<Node *> CLOSED = {};

    while (!OPEN.empty()) {
        int cur_pos = argmin(OPEN);
        Node* cur = OPEN[cur_pos];
        CLOSED.push_back(cur);
        OPEN.erase(OPEN.begin() + cur_pos);
        if (cur->i == map.getGoalX() && cur->j == map.getGoalY()) {
            sresult.pathfound = true;
            sresult.pathlength = cur->g;
            while (cur != nullptr) {
                lppath.push_front(*cur);
                cur = cur->parent;
            }
            sresult.nodescreated = OPEN.size() + CLOSED.size();
            sresult.lppath = &lppath;
            sresult.hppath = &hppath;
            return sresult;
        }
        std::vector<Node *> SUCC = generateAllSuccs(cur, map, options, algorithm);
        for (Node *node : SUCC) {
            bool found_in_OPEN = false;
            for (Node *op : OPEN) {
                if (op->i == node->i && op->j == node->j) {
                    if (node->g < op->g) {
                        op->g = node->g;
                        op->F = node->F;
                        op->parent = node->parent;
                    }
                    found_in_OPEN = true;
                    break;
                }
            }
            if (!found_in_OPEN) {
                bool found_in_CLOSED = false;
                for (Node *cl : CLOSED) {
                    if (cl->i == node->i && cl->j == node->j) {
                        found_in_CLOSED = true;
                    }
                }
                if (!found_in_CLOSED) {
                    OPEN.push_back(node);
                }
            }
        }
    }

    sresult.pathfound = false;
    /*sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;*/
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

/*void Search::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
