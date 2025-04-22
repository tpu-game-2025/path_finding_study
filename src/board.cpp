#include "board.h"

std::map<Mass::status, MassInfo> Mass::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// 動的な要素
	{ START,	{-1.0f, 'S'}},
	{ GOAL,		{-1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const
{
	mass[start.y][start.x].setStatus(Mass::START);
	mass[goal.y][goal.x].setStatus(Mass::GOAL);

	std::multimap<float, std::pair<Point, float>> q;	//multimapで優先度付きキューを実装
	mass[start.y][start.x].visit(start, 0.0f);
	q.insert({ Point::distance(start,goal), {start,0.0f} });

	// 経路探索
	while (!q.empty()) {
		auto current = q.begin()->second.first;
		auto qSteps = q.begin()->second.second;
		q.erase(q.begin());

		auto currentSteps = mass[current.y][current.x].getSteps();
		if (currentSteps < qSteps) continue;	// 以前の方が距離が短い

		const static Point moveValue[] = { {-1,0},{1,0},{0,-1},{0,1} };
		for (const auto& move : moveValue) {

			auto next = current + move;
			auto& nextmass = mass[next.y][next.x];
			if (!nextmass.canMove() && next != goal) continue;

			auto pastSteps = nextmass.getSteps();
			auto nextSteps = currentSteps + nextmass.getCost();
			if (pastSteps <= nextSteps) continue;	// 以前の方が距離が短い

			nextmass.visit(current, nextSteps);
			q.insert({ nextSteps + Point::distance(next,goal), {next,nextSteps} });

			if (next != goal) continue;
			auto goalSteps = nextmass.getSteps();
			if (goalSteps < q.begin()->first) break;	// 最も小さい評価値よりも小さいなら十分適切とみなす


			// 確実に最適解であることを保証する確認
			// queueにある全てのsteps値がgoalのものより大きいか調べる
			//bool isOptimal = true;
			//for (auto i : q) {
			//	if (goalSteps > i.second.second)
			//		isOptimal = false;
			//}
			//if (isOptimal) break;
		}
	}

	auto goalmass = mass[goal.y][goal.x];
	if (goalmass.getSteps() == INFINITY) return false;	//ゴール地点が未探索なら探索失敗

	// 辿った道のりに印をつける（スタートとゴールは書き換えない）
	auto current = goalmass.getParent();
	while (current != start) {
		auto& currentmass = mass[current.y][current.x];
		currentmass.setStatus(Mass::WAYPOINT);
		current = currentmass.getParent();
	}

	return true;
}
