#include "board.h"
#include <queue>

std::map<Mass::status, MassInfo> Mass::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// ���I�ȗv�f
	{ START,	{-1.0f, 'S'}},
	{ GOAL,		{-1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& �n�_, const Point& �I�_, std::vector<std::vector<Mass>>& mass) const
{
	mass[�n�_.y][�n�_.x].set(Mass::START);
	mass[�I�_.y][�I�_.x].set(Mass::GOAL);

	// �o�H�T��
	std::multimap<float,Point> q;
	mass[�n�_.y][�n�_.x].visit(�n�_, mass[�n�_.y][�n�_.x]);
	q.insert({ Point::distance(�n�_,�I�_), �n�_ });
	while (!q.empty())
	{
		Point ���� = q.begin()->second;
		int distance = mass[����.y][����.x].getSteps();
		q.erase(q.begin());
		mass[����.y][����.x].close();
		const static Point �ړ���[] = { {-1,0},{+1,0},{0,-1},{0,+1} };

		for (const auto& �ړ� : �ړ���)
		{
			Point �� = ���� + �ړ�;;
			Mass& ���̃}�X = mass[��.y][��.x];

			if (map_[��.y][��.x].canMove() && !���̃}�X.isClosed())
			{
				float �n�_����̕��� = static_cast<float>(distance) + ���̃}�X.getCost();
				int �ȑO�̕��� = ���̃}�X.getSteps();

				if (0 <= �ȑO�̕���) //���ɖK�ꂽ
				{
					if (�ȑO�̕��� <= �n�_����̕���) continue; //�ȑO�̕������������Ȃ�
					
					//�Â��L�[�̍폜(�s�������_���̃L�[�Ȃ̂Ō덷���l�����đS�T��)
					for (auto it = q.begin(); it != q.end(); ++it)
					{
						if (it->second == ��) { q.erase(it); break; }
					}
				}
				���̃}�X.visit(����, ���̃}�X);
				q.insert({ static_cast<float>(�n�_����̕���) + Point::distance(��,�I�_),��});
				if (�� == �I�_)
				{
					//�I�_���炳���̂ڂ���WAYPOINT��ݒ肷��
					Point& �������ꏊ = mass[�I�_.y][�I�_.x].getParent();

					while (�������ꏊ != �n�_) //�������ꏊ�Ɉ������(�n�_�͏��������Ȃ�)
					{
						Mass& m = mass[�������ꏊ.y][�������ꏊ.x];
						m.set(Mass::WAYPOINT);
						�������ꏊ = mass[�������ꏊ.y][�������ꏊ.x].getParent();
					}

					return true;
				}
			}
		}
	}
	return true;
}
