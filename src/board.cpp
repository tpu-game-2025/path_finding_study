#include "board.h"
#include <queue>

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
	std::multimap<int,Point> q;//オープンリスト(始点からの距離(歩数)、マスの位置)
	mass[始点.y][始点.x].visit(始点, mass[始点.y][始点.x]);

	//始点をオープンリストに追加
	q.insert({ 0,始点 });

	while (!q.empty())//qが空になったら終点にたどり着けなかったということ
	{
		//最も始点から最短距離で行けるマスを調べる
		int distance = q.begin()->first;
		Point 現在 = q.begin()->second;
		Mass& 現在のマス = mass[現在.y][現在.x];//現在のマス
		//調べるマスをオープンリストからクローズドリストに
		q.erase(q.begin());
		現在のマス.close();

		const static Point 移動量[] = { {-1,0},{1,0},{0,-1},{0,1} };//左、右、上、下

		for (const auto& 移動 : 移動量)//現在地から全方向に調べる
		{
			//次のマスの設定
			Point 次 = 現在 + 移動;
			Mass& 次のマス = mass[次.y][次.x];

			//次の(調査)マスが行けるところかつまだクローズドしてないマスを調べる
			if (次のマス.canMove() && !次のマス.isClosed())
			{
				int 始点からの歩数 = distance + 1;
				int 以前の歩数 = 次のマス.getSteps();

				if (0 <= 以前の歩数)//既に訪れていたなら
				{
					//始点により近い方を登録
					
					//今回の方が遠かったら変更しない
					if (以前の歩数 <= 始点からの歩数) continue;
					
					//今回の方が近かったら...
					//キューから一旦削除(後で新しく追加)
					auto a = q.equal_range(以前の歩数);
					for (auto it = a.first; it != a.second; it++)
					{
						if (it->second == 次) { q.erase(it); break; }
					}
				}

				//調査マスを訪れ、キューに追加
				次のマス.visit(現在,現在のマス);
				q.insert({始点からの歩数, 次});

				//std::cout << 次のマス.getSteps() << std::endl;//確認用

				//次のマスが終点なら...
				//終点から始点までたどりながらWaypointをつけていく(始点と終点はつけない)
				if (次 == 終点)
				{
					Point& 歩いた場所 = mass[終点.y][終点.x].getParent();

					while (歩いた場所 != 始点)
					{
						Mass& 歩いたマス = mass[歩いた場所.y][歩いた場所.x];
						歩いたマス.set(Mass::WAYPOINT);
						歩いた場所 = 歩いたマス.getParent();
					}

					//始点と終点に印をつける
					mass[始点.y][始点.x].set(Mass::START);
					mass[終点.y][終点.x].set(Mass::GOAL);

					return true;
				}
			}
		}
	}

	return false;
}
