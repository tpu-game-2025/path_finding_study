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
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	std::multimap<float, Point>open;//コストと座標の情報を持つキュー
	
	 mass[始点.y][始点.x].Visit(始点, mass[始点.y][始点.x]);
	 open.insert({ Point::distance(始点, 終点), 始点 });
// 経路探索
while (!open.empty()) {

	Point 現在 = open.begin()->second;

	int distance = mass[現在.y][現在.x].getStep();//距離(地形コストを無視したコスト)を計算
	
	open.erase(open.begin());

	mass[現在.y][現在.x].close();//疑似的にクローズリストに追加する

	const static Point 移動量[] = { {-1,0},{+1,0},{0,-1},{0,+1} };

	for (const auto& 移動 : 移動量) {//この中で四方向をオープンする

		Point 次 = 現在 + 移動;
		Mass& 次のマス = mass[次.y][次.x];

		if (map_[次.y][次.x].canMove() && !次のマス.isClosed()) {//移動先が範囲内かつオープンの場合(元の場所に戻らないようにする)

			int 始点からのコスト = mass[次.y][次.x].getCost() + 次のマス.getStep();
			int 以前の歩数 = 次のマス.getStep();//(コストの計算)

			if (0 <= 以前の歩数) {

				if (以前の歩数 <= 始点からのコスト)continue;//以前のコストの方が少なければ削除しない


				for (auto it = open.begin(); it != open.end(); ++it) {//より良い経路が見つかった場合、以前の経路を削除する
					if (it->second == 次) {
						open.erase(it); break;
					}

				}
			}

			次のマス.Visit(現在, mass[現在.y][現在.x]);//最適な経路が見つかればオープンリストに登録する
			open.insert({ static_cast <float>(始点からのコスト) + Point::distance(次,終点),次 });//コストを引き継ぐ
		}

	}
}
//経路復元
Point 現在 = 終点;

while (現在 != 始点) {
	Point 親 = mass[現在.y][現在.x].getParent();//親を辿って印をつける
	if (親 == 始点) break;
	mass[親.y][親.x].set(Mass::WAYPOINT);
	現在 = 親;
}

	return true;
}
