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
        return abs(node.i - map.getGoalX()) + abs(node.j - map.getGoalY());
    }

    if (options.metrictype == CN_SP_MT_CHEB) {
        return std::max(abs(node.i - map.getGoalX()), abs(node.j - map.getGoalY()));
    }

    if (options.metrictype == CN_SP_MT_EUCL) {
        return sqrt(pow(node.i - map.getGoalX(), 2) + pow(node.j - map.getGoalY(), 2));
    }

    if (options.metrictype == CN_SP_MT_DIAG) {
        return CN_SQRT_TWO * std::min(abs(node.i - map.getGoalX()), abs(node.j - map.getGoalY())) +
               abs(abs(node.i - map.getGoalX()) - abs(node.j - map.getGoalY()));
    }

    return 0;
}

int argmin(const std::vector<Node> &OPEN) {
    if (OPEN.empty())
        return -1;
    int result = 0;
    double result_f = OPEN[0].F;
    for (int i = 1; i < (int)OPEN.size(); ++i) {
        if (OPEN[i].F < result_f) {
            result = i;
            result_f = OPEN[i].F;
        }
    }
    return result;
}

std::vector<Node> Search::generateAllSuccs(Node &cur, const Map &map, const EnvironmentOptions &options, int algorithm) {
    std::vector<Node> SUCC = {};
    if (map.CellOnGrid(cur.i, cur.j - 1)) {
        if (map.CellIsTraversable(cur.i, cur.j - 1)) {
            Node add{};
            add.i = cur.i;
            add.j = cur.j - 1;
            add.g = cur.g + 1;
            add.H = Search::heuristic(add, map, options, algorithm);
            add.F = add.g + add.H;
            add.parent = {cur.i, cur.j};
            SUCC.push_back(add);
        }
    }
    if (map.CellOnGrid(cur.i, cur.j + 1)) {
        if (map.CellIsTraversable(cur.i, cur.j + 1)) {
            Node add{};
            add.i = cur.i;
            add.j = cur.j + 1;
            add.g = cur.g + 1;
            add.H = Search::heuristic(add, map, options, algorithm);
            add.F = add.g + add.H;
            add.parent = {cur.i, cur.j};
            SUCC.push_back(add);
        }
    }
    if (map.CellOnGrid(cur.i - 1, cur.j)) {
        if (map.CellIsTraversable(cur.i - 1, cur.j)) {
            Node add{};
            add.i = cur.i - 1;
            add.j = cur.j;
            add.g = cur.g + 1;
            add.H = Search::heuristic(add, map, options, algorithm);
            add.F = add.g + add.H;
            add.parent = {cur.i, cur.j};
            SUCC.push_back(add);
        }
    }
    if (map.CellOnGrid(cur.i + 1, cur.j)) {
        if (map.CellIsTraversable(cur.i + 1, cur.j)) {
            Node add{};
            add.i = cur.i + 1;
            add.j = cur.j;
            add.g = cur.g + 1;
            add.H = Search::heuristic(add, map, options, algorithm);
            add.F = add.g + add.H;
            add.parent = {cur.i, cur.j};
            SUCC.push_back(add);
        }
    }
    if (options.allowdiagonal) {
        if (map.CellOnGrid(cur.i + 1, cur.j - 1)) {
            if (map.CellIsTraversable(cur.i + 1, cur.j - 1)) {
                if ((map.CellIsTraversable(cur.i, cur.j - 1) && map.CellIsTraversable(cur.i + 1, cur.j)) ||
                    (map.CellIsTraversable(cur.i, cur.j - 1) && options.cutcorners) ||
                    (map.CellIsTraversable(cur.i + 1, cur.j) && options.cutcorners) ||
                    (map.CellIsObstacle(cur.i, cur.j - 1) && map.CellIsObstacle(cur.i + 1, cur.j) && options.allowsqueeze)) {
                    Node add{};
                    add.i = cur.i + 1;
                    add.j = cur.j - 1;
                    add.g = cur.g + CN_SQRT_TWO;
                    add.H = Search::heuristic(add, map, options, algorithm);
                    add.F = add.g + add.H;
                    add.parent = {cur.i, cur.j};
                    SUCC.push_back(add);
                }
            }
        }
        if (map.CellOnGrid(cur.i - 1, cur.j - 1)) {
            if (map.CellIsTraversable(cur.i - 1, cur.j - 1)) {
                if ((map.CellIsTraversable(cur.i, cur.j - 1) && map.CellIsTraversable(cur.i - 1, cur.j)) ||
                    (map.CellIsTraversable(cur.i, cur.j - 1) && options.cutcorners) ||
                    (map.CellIsTraversable(cur.i - 1, cur.j) && options.cutcorners) ||
                    (map.CellIsObstacle(cur.i, cur.j - 1) && map.CellIsObstacle(cur.i - 1, cur.j) && options.allowsqueeze)) {
                    Node add{};
                    add.i = cur.i - 1;
                    add.j = cur.j - 1;
                    add.g = cur.g + CN_SQRT_TWO;
                    add.H = Search::heuristic(add, map, options, algorithm);
                    add.F = add.g + add.H;
                    add.parent = {cur.i, cur.j};
                    SUCC.push_back(add);
                }
            }
        }
        if (map.CellOnGrid(cur.i + 1, cur.j + 1)) {
            if (map.CellIsTraversable(cur.i + 1, cur.j + 1)) {
                if ((map.CellIsTraversable(cur.i + 1, cur.j) && map.CellIsTraversable(cur.i, cur.j + 1)) ||
                    (map.CellIsTraversable(cur.i + 1, cur.j) && options.cutcorners) ||
                    (map.CellIsTraversable(cur.i, cur.j + 1) && options.cutcorners) ||
                    (map.CellIsObstacle(cur.i + 1, cur.j) && map.CellIsObstacle(cur.i, cur.j + 1) && options.allowsqueeze)) {
                    Node add{};
                    add.i = cur.i + 1;
                    add.j = cur.j + 1;
                    add.g = cur.g + CN_SQRT_TWO;
                    add.H = Search::heuristic(add, map, options, algorithm);
                    add.F = add.g + add.H;
                    add.parent = {cur.i, cur.j};
                    SUCC.push_back(add);
                }
            }
        }
        if (map.CellOnGrid(cur.i - 1, cur.j + 1)) {
            if (map.CellIsTraversable(cur.i - 1, cur.j + 1)) {
                if ((map.CellIsTraversable(cur.i - 1, cur.j) && map.CellIsTraversable(cur.i, cur.j + 1)) ||
                    (map.CellIsTraversable(cur.i - 1, cur.j) && options.cutcorners) ||
                    (map.CellIsTraversable(cur.i, cur.j + 1) && options.cutcorners) ||
                    (map.CellIsObstacle(cur.i - 1, cur.j) && map.CellIsObstacle(cur.i, cur.j + 1) && options.allowsqueeze)) {
                    Node add{};
                    add.i = cur.i - 1;
                    add.j = cur.j + 1;
                    add.g = cur.g + CN_SQRT_TWO;
                    add.H = Search::heuristic(add, map, options, algorithm);
                    add.F = add.g + add.H;
                    add.parent = {cur.i, cur.j};
                    SUCC.push_back(add);
                }
            }
        }
    }
    return SUCC;
}

int Search::xDiff(Node &first, Node &second) {
    return second.i - first.i;
}

int Search::yDiff(Node &first, Node &second) {
    return second.j - first.j;
}

struct PairHash {
    std::size_t operator() (const std::pair<int, int> &pair) const {
        return std::hash<std::size_t>() ((size_t) pair.first * 1000000000 + (size_t) pair.second);
    }
};

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options, int algorithm)
{
    unsigned int start_time = clock();
    Node start{};
    start.i = map.getStartX();
    start.j = map.getStartY();
    start.g = 0;
    start.H = heuristic(start, map, options, algorithm);
    start.F = start.H;
    start.parent = {-1, -1};

    std::vector<Node> OPEN = {start};
    std::unordered_map<std::pair<int, int>, Node, PairHash> CLOSED;

    sresult.numberofsteps = 0;
    while (!OPEN.empty()) {
        ++sresult.numberofsteps;
        int cur_pos = argmin(OPEN);
        Node cur = OPEN[cur_pos];
        CLOSED[{cur.i, cur.j}] = cur;
        OPEN.erase(OPEN.begin() + cur_pos);
        if (cur.i == map.getGoalX() && cur.j == map.getGoalY()) {
            sresult.pathfound = true;
            sresult.pathlength = cur.g;
            lppath.push_front(cur);
            hppath.push_front(cur);
            while (cur.parent != std::make_pair(-1, -1)) {
                cur = CLOSED.find(cur.parent)->second;
                if ((int)lppath.size() >= 2 &&
                    (xDiff(*lppath.begin(), *next(lppath.begin())) != xDiff(cur, *lppath.begin()) ||
                     yDiff(*lppath.begin(), *next(lppath.begin())) != yDiff(cur, *lppath.begin()))) {
                    hppath.push_front(*lppath.begin());
                }
                lppath.push_front(cur);
                if (cur == start) {
                    hppath.push_front(cur);
                }
            }
            sresult.nodescreated = OPEN.size() + CLOSED.size();
            sresult.lppath = &lppath;
            sresult.hppath = &hppath;
            unsigned int end_time = clock();
            sresult.time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
            return sresult;
        }
        std::vector<Node> SUCC = generateAllSuccs(cur, map, options, algorithm);
        for (Node &node : SUCC) {
            bool found_in_OPEN = false;
            for (Node &op : OPEN) {
                if (node == op) {
                    if (node.g < op.g) {
                        op.g = node.g;
                        op.F = node.F;
                        op.parent = node.parent;
                    }
                    found_in_OPEN = true;
                    break;
                }
            }
            if (!found_in_OPEN) {
                bool found_in_CLOSED = (CLOSED.find({node.i, node.j}) != CLOSED.end());
                if (!found_in_CLOSED) {
                    OPEN.push_back(node);
                }
            }
        }
    }

    sresult.pathfound = false;
    sresult.pathlength = 0;
    /*sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;*/
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    unsigned int end_time = clock();
    sresult.time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
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