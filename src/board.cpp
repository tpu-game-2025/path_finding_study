#include "board.h"
#include <queue>
#include <unordered_map>

bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const {
    const std::vector<Point> directions = {
        { 0, -1 }, { 0, 1 }, { -1, 0 }, { 1, 0 }
    };

    auto heuristic = [](const Point& a, const Point& b) {
        int dx = abs(a.x - b.x);
        int dy = abs(a.y - b.y);
        return dx + dy;
        };

    struct Node {
        Point pos;
        float cost;

        bool operator>(const Node& other) const {
            return cost > other.cost;
        }
    };

    int h = static_cast<int>(mass.size());
    int w = static_cast<int>(mass[0].size());

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<int, Point> came_from;
    std::unordered_map<int, float> cost_so_far;

    auto encode = [w](const Point& p) { return p.y * w + p.x; };

    open.push({ start, 0 });
    came_from[encode(start)] = start;
    cost_so_far[encode(start)] = 0;

    while (!open.empty()) {
        Point current = open.top().pos;
        open.pop();

        if (current == goal)
            break;

        for (const Point& dir : directions) {
            Point next = current + dir;

            if (next.y < 0 || next.y >= h || next.x < 0 || next.x >= w)
                continue;
            if (!mass[next.y][next.x].canMove())
                continue;

            float move_cost = mass[next.y][next.x].getCost();
            float new_cost = cost_so_far[encode(current)] + move_cost;

            if (!cost_so_far.count(encode(next)) || new_cost < cost_so_far[encode(next)]) {
                cost_so_far[encode(next)] = new_cost;
                float priority = new_cost + heuristic(next, goal);
                open.push({ next, priority });
                came_from[encode(next)] = current;
            }
        }
    }

    // 経路の復元
    Point current = goal;
    while (current != start) {
        Point prev = came_from[encode(current)];
        if (prev == current) break;
        mass[current.y][current.x].markPath();
        current = prev;
    }

    return true;
}