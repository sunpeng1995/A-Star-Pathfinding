#ifndef __MAP_NODE__
#define __MAP_NODE__

#include <iostream>
#include <algorithm>
#include "MinHeap.cpp"

class _pathfind_vec2
{
public:
	int x;
	int y;
	_pathfind_vec2(int x, int y) {
		this->x = x;
		this->y = y;
	}
	_pathfind_vec2() {

	}
	friend bool operator==(const _pathfind_vec2 v1, _pathfind_vec2 v2) {
		return (v1.x == v2.x && v1.y == v2.y);
	}
};

class _pathfind_node {
public:
	_pathfind_vec2 vec2;

	//the cost of path from start to this end.
	float G;

	//the cost of path from this node to goal.
	float H;

	//the best cost from start to goal through this node.
	float TotalCost;
	_pathfind_node* Parent;

	friend bool operator<(const _pathfind_node p1, const _pathfind_node p2) {
		return p1.TotalCost > p2.TotalCost;
	}

	friend bool operator>(const _pathfind_node p1, const _pathfind_node p2) {
		return p1.TotalCost < p2.TotalCost;
	}

	friend bool operator==(const _pathfind_node p1, const _pathfind_node p2) {
		return p1.vec2 == p2.vec2;
	}

	_pathfind_node(_pathfind_vec2 pos, _pathfind_node* parent) {
		this->vec2 = pos;
		this->Parent = parent;
	}
	_pathfind_node() {

	}
};

class AStarSearch {
public:
	MinHeap<_pathfind_node> open_list;
	std::vector<_pathfind_node> close_list;
	std::vector<_pathfind_node> 
		search(_pathfind_vec2 start_position, _pathfind_vec2 goal_position);
	std::vector<_pathfind_node>
		back_tracking(_pathfind_node start);
	bool can_reach(_pathfind_node node);


	//*need an int type two-demension array,
	//*the element of array is the cost of that grid.
	AStarSearch(int **map);
	AStarSearch();
	~AStarSearch();

	void input_map(int **map, int width, int height);
	void input_start(int x, int y);
	void input_goal(int x, int y);

private:
	int **m_map;
	int _width, _height;
	_pathfind_vec2 start_pos;
	_pathfind_vec2 goal_pos;

	int cost_estimate(_pathfind_vec2 s, _pathfind_vec2 g) {
		return abs(s.x - g.x) + abs(s.y - g.y);
	}
	_pathfind_node visit(_pathfind_vec2 node, int direction);
	float traverse_cost(_pathfind_node node1, _pathfind_node node2);
	_pathfind_node* find_open_list(_pathfind_node t);
	_pathfind_node* find_close_list(_pathfind_node t);

};

#endif