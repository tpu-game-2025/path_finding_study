#include "board.h"            
#include <queue>              
#include <limits>            
#include <cmath>  

struct Node {
	Point pos;
	float g; 
	float f; 
	bool operator<(const Node& rhs) const {
		return f > rhs.f; 
	}
};

std::map<Mass::status, MassInfo> Mass::statusData = {
	{ Mass::BLANK,    { 1.0f, ' ' } },
	{ Mass::WALL,     { -1.0f, '#' } },
	{ Mass::WATER,    { 3.0f, '~' } },
	{ Mass::ROAD,     { 1.0f / 3.0f, '$' } },
	{ Mass::START,    { 1.0f, 'S' } },
	{ Mass::GOAL,     { 1.0f, 'G' } },
	{ Mass::WAYPOINT, { 1.0f, '.' } },
};


bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const
{
	const int H = mass.size();
	const int W = mass[0].size();

	std::priority_queue<Node> open;
	std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));
	std::vector<std::vector<Point>> came_from(H, std::vector<Point>(W, { -1, -1 }));
	std::vector<std::vector<float>> cost_so_far(H, std::vector<float>(W, std::numeric_limits<float>::infinity()));

	auto heuristic = [](const Point& a, const Point& b) {
		return std::abs(a.x - b.x) + std::abs(a.y - b.y);
	};

	open.push({ start, 0.0f, static_cast<float>(heuristic(start, goal)) });
	cost_so_far[start.y][start.x] = 0.0f;

	std::vector<Point> directions = { {1,0}, {-1,0}, {0,1}, {0,-1} };

	while (!open.empty()) {
		Node current = open.top();
		open.pop();

		if (current.pos == goal)
			break;

		if (visited[current.pos.y][current.pos.x])
			continue;
		visited[current.pos.y][current.pos.x] = true;

		for (const Point& dir : directions) {
			Point next = current.pos + dir;

			if (next.x < 0 || next.x >= W || next.y < 0 || next.y >= H)
				continue;

			const Mass& m = mass[next.y][next.x];

			if (!m.canMove())
				continue;

			float cost = m.getCost();
			if (cost < 0) continue;

			float new_cost = cost_so_far[current.pos.y][current.pos.x] + cost;
			if (new_cost < cost_so_far[next.y][next.x]) {
				cost_so_far[next.y][next.x] = new_cost;
				float f = new_cost + heuristic(next, goal);
				open.push({ next, new_cost, f });
				came_from[next.y][next.x] = current.pos;
			}
		}
	}

	
	Point p = goal;
	while (p != start) {
		if (p == goal) {
			mass[p.y][p.x].set(Mass::GOAL);
		}
		else {
			mass[p.y][p.x].set(Mass::WAYPOINT);
		}
		p = came_from[p.y][p.x];
		if (p.x == -1 && p.y == -1) {

			return false;
		}
	}
	mass[start.y][start.x].set(Mass::START);

	return true;
}
