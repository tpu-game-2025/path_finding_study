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

struct QueueNode
{
	float 評価値;
	Point 座標;

	bool operator < (const QueueNode& other) const { return 評価値 > other.評価値; }
};

bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	const std::vector<Point> 方向 = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

	int 縦 = (int)mass.size();
	int 横 = (int)mass[0].size();

	std::vector<std::vector<float>> コスト(縦, std::vector<float>(横, 1e9));
	std::vector<std::vector<Point>> 来た場所(縦, std::vector<Point>(横, { -1, -1 }));
	std::priority_queue<QueueNode> 開くキュー;

	コスト[始点.y][始点.x] = 0.0f;
	開くキュー.push({ 0.0f, 始点 });

	while (!開くキュー.empty())
	{
		Point 現在 = 開くキュー.top().座標;
		開くキュー.pop();

		if (現在 == 終点) break;

		for (const Point& dir : 方向)
		{
			Point 次 = 現在 + dir;

			if (次.x < 0 || 次.y < 0 || 次.x >= 横 || 次.y >= 縦) continue;

			if (!mass[次.y][次.x].canMove()) continue;

			float 新コスト = コスト[現在.y][現在.x] + mass[次.y][次.x].getCost();

			if (新コスト < コスト[次.y][次.x])
			{
				コスト[次.y][次.x] = 新コスト;
				来た場所[次.y][次.x] = 現在;
				
				float 推定残り = Point::distance(次, 終点);
				float 評価値 = 新コスト + 推定残り;

				開くキュー.push({ 評価値, 次 });
			}
		}
	}

	if (来た場所[終点.y][終点.x].x == -1) return false;

	Point 現在 = 終点;
	while (現在 != 始点)
	{
		Point 前 = 来た場所[現在.y][現在.x];

		if (前 == 始点) break;

		mass[前.y][前.x].set(Mass::WAYPOINT);
		現在 = 前;
	}

	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	return true;
}
