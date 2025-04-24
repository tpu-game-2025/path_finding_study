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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	struct ClOSE
	{
		Point point;//これが示す地点
		Point before;//直前の地点
	};

	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	std::multimap<float, Point> openList;//推定値, これの位置
	std::vector<ClOSE> closedList;
	openList.insert({ Point::distance(始点, 終点), 始点 });//まずは最初の地点を追加する
	closedList.push_back({ 始点, 始点 });

	//int count = 0;  //ダイクストラ法の以下のwhileが呼ばれたループ回数は79回でした。A*は36回でした。
	while (!openList.empty())
	{
		Point current = openList.begin()->second;
		//firstの値は終点までの推定値が足されているので引く
		float distance = openList.begin()->first - Point::distance(終点, current);

		openList.erase(openList.begin());

		//上下左右を確かめるための固定値
		const static Point moves[] = { {1,0}, {-1,0}, {0,1}, {0,-1} };
		for (const auto& move : moves) {
			Point next = current + move;

			if (mass[next.x][next.y].canMove()) {
				bool wasCheck = false;
				for (const auto& close : closedList)//すでに確認されたか調べる
				{
					if (close.point == next) {
						wasCheck = true;
						break;
					}
				}
				if (wasCheck) continue;

				float totalStep = distance + mass[next.x][next.y].getCost();

				closedList.push_back({ next, current });
				//推定値を与える
				openList.insert({ totalStep + Point::distance(終点, next) , next });
			}
			else if (next == 終点)//ゴールにたどり着いた
			{
				Point& before = current;//ゴールの直前のマス
				while (before != 始点)
				{
					Mass& m = mass[before.x][before.y];
					m.set(Mass::WAYPOINT);
					for (const auto& close : closedList)//前通ったマスを探索
					{
						if (close.point == before) {
							before = close.before;
							break;
						}
					}
				}
				return true;
			}
		}
	}
	return false;
}
