#pragma once
#include <cassert>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <tuple>

struct Point {
	int x = -1;
	int y = -1;

	bool operator==(const Point& rhs) const {
		return x == rhs.x && y == rhs.y;
	}
	bool operator!=(const Point& rhs) const { return !(*this == rhs); }
	bool operator<(const Point& rhs) const { return std::tie(y, x) < std::tie(rhs.y, rhs.x); }

	Point operator+(const Point& rhs) const { return { x + rhs.x, y + rhs.y }; }

	static float distance(const Point& p0, const Point& p1)
	{
		int dx = p1.x - p0.x;
		int dy = p1.y - p0.y;
		return sqrtf(float(dx * dx + dy * dy));
	}
};

struct MassInfo {
	bool isWall = false;
	bool isPath = false;
	char type;
	float cost;
	char chr;
};

class Mass {
public:
	enum status {
		BLANK, WALL, WATER, ROAD,
		START, GOAL, WAYPOINT,
		INVALID,
	};

private:
	static std::map<status, MassInfo> statusData;
	status s_ = BLANK;

public:
	void set(status s) { s_ = s; }

	void set(char c) {
		s_ = INVALID;
		for (auto& x : statusData) {
			if (x.second.chr == c) {
				s_ = x.first;
				return;
			}
		}
	}

	const std::string getText() const { return std::string{ statusData[s_].chr }; }

	bool canMove() const { return 0 <= statusData[s_].cost; }
	float getCost() const { return statusData[s_].cost; }
	bool isWall() const { return !canMove(); }

	void markPath() { s_ = WAYPOINT; }

	static void initStatusData() {
		statusData = {
			{ BLANK,    { false, false, ' ', 1.0f, ' ' } },
			{ WALL,     { true,  false, '#', -1.0f, '#' } },
			{ WATER,    { false, false, '~', 3.0f, '~' } },
			{ ROAD,     { false, false, '$', 1.0f / 3.0f, '$' } },
			{ START,    { false, false, 'S', 1.0f, 'S' } },
			{ GOAL,     { false, false, 'G', 1.0f, 'G' } },
			{ WAYPOINT, { false, true,  '*', 1.0f, '*' } },
		};
	}
};

inline std::map<Mass::status, MassInfo> Mass::statusData;

class Board {
private:
	std::vector<std::vector<Mass>> map_;

	void initialize(const std::vector<std::string>& map_data)
	{
		size_t h = map_data.size();
		size_t w = map_data[0].size();

		map_.resize(h);
		for (unsigned int y = 0; y < h; y++) {
			map_[y].resize(w);

			assert(map_data[y].size() == w);
			for (int x = 0; x < w; x++) {
				map_[y][x].set(map_data[y][x]);
			}
		}
	}

public:
	Board(const std::vector<std::string>& map_data) {
		Mass::initStatusData(); // ← 必ず初期化
		initialize(map_data);
	}
	~Board() {}

	std::vector<std::vector<Mass>> setup()
	{
		std::vector<std::vector<Mass>> mass;

		size_t h = map_.size();
		size_t w = map_[0].size();

		mass.resize(h);
		for (unsigned int y = 0; y < h; y++) {
			mass[y].resize(w);
			for (unsigned int x = 0; x < w; x++) {
				mass[y][x].set(map_[y][x].getText()[0]);
			}
		}

		return mass;
	}

	void show(const std::vector<std::vector<Mass>>& mass) const
	{
		size_t h = mass.size();
		size_t w = mass[0].size();

		std::cout << std::endl;
		for (unsigned int y = 0; y < h; y++) {
			std::cout << " ";
			for (unsigned int x = 0; x < w; x++) {
				std::cout << mass[y][x].getText();
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

	bool find(const Point& start, const Point& goal, std::vector<std::vector<Mass>>& mass) const;
};

