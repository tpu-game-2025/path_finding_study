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


//ここにアルゴリズムなどを追加する部分
bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>>& mass) const
{
    struct Node {
        Point point;
        float f;
        bool operator<(const Node& rhs) const {
            return f > rhs.f; // 小さい順に並べるため、逆順に
        }
    };

    std::priority_queue<Node> openList;
    std::map<Point, Point> cameFrom;
    std::map<Point, float> gScore;

    openList.push({ 始点, Point::distance(始点, 終点) });
    gScore[始点] = 0.0f;

    const static Point directions[] = { {-1,0}, {+1,0}, {0,-1}, {0,+1} };

    while (!openList.empty())
    {
        Point current = openList.top().point;
        openList.pop();

        if (current == 終点)
        {
            Point trace = 終点;
            while (trace != 始点)
            {
                mass[trace.y][trace.x].set(Mass::WAYPOINT);
                trace = cameFrom[trace];
            }
            mass[始点.y][始点.x].set(Mass::START);
            mass[終点.y][終点.x].set(Mass::GOAL);
            return true;
        }

        for (const auto& dir : directions)
        {
            Point neighbor = current + dir;

            // 範囲チェック
            if (neighbor.y < 0 || neighbor.y >= (int)mass.size() ||
                neighbor.x < 0 || neighbor.x >= (int)mass[0].size())
                continue;

            if (!map_[neighbor.y][neighbor.x].canMove())
                continue;

            float tentative_gScore = gScore[current] + map_[neighbor.y][neighbor.x].getCost();

            if (!gScore.count(neighbor) || tentative_gScore < gScore[neighbor])
            {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                float f = tentative_gScore + Point::distance(neighbor, 終点);
                openList.push({ neighbor, f });
            }
        }
    }

    return false; // 経路が見つからなかった場合
}