﻿#include "board.h"

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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>>& mass) const
{

	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	std::multimap<float, Point> q;
	mass[始点.y][終点.x].visit(始点, mass[始点.y][始点.x]);
	q.insert({ Point::distance(始点,終点),始点 });
	while (!q.empty())
	{
		Point 現在 = q.begin()->second;
		int distance = mass[現在.y][現在.x].getSteps();
		q.erase(q.begin());
		mass[現在.y][現在.x].close();

		const static Point 移動量[] = { {-1,0},{+1,0},{0,-1},{0,+1} };
		for (const auto& 移動 : 移動量)
		{
			Point 次 = 現在 + 移動;
			Mass& 次のマス = mass[次.y][次.x];
			if (map_[次.y][次.x].canMove() && !次のマス.isClosed())
			{
				float 始点からの歩数 = static_cast<float>(distance) + 次のマス.getCost();
				int 以前の歩数 = 次のマス.getSteps();
				if (0 <= 以前の歩数)
				{
					if (以前の歩数 <= 始点からの歩数) continue;
					for (auto it = q.begin(); it != q.end(); ++it)
					{
						if (it->second == 次)
						{
							q.erase(it); break;
						}
					}
				}
				次のマス.visit(現在, 次のマス);
				q.insert({ 始点からの歩数 + Point::distance(次,終点),次 });

				if (次 == 終点) 
				{
					Point& 歩いた場所 = mass[終点.y][終点.x].getParent();
					while (歩いた場所 != 始点) 
					{
						Mass& Mass = mass[歩いた場所.y][歩いた場所.x];
						Mass.set(Mass::WAYPOINT);
						歩いた場所 = mass[歩いた場所.y][歩いた場所.x].getParent();
					}
					return true;
				}

			}
		}



		// 経路探索
		//Point 現在 = 始点;
		//while (現在 != 終点) {
		//	// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
		//	if (現在 != 始点) { mass[現在.y][現在.x].set(Mass::WAYPOINT); }

		//	// 終点に向かって歩く
		//	if (現在.x < 終点.x) { 現在.x++; continue; }
		//	if (終点.x < 現在.x) { 現在.x--; continue; }
		//	if (現在.y < 終点.y) { 現在.y++; continue; }
		//	if (終点.y < 現在.y) { 現在.y--; continue; }
		//}
	}
	return true;
}
