#ifndef __ASTAR_NODE__
#define __ASTAR_NODE__

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
		x = -1;
		y = -1;
	}
	friend bool operator==(const _pathfind_vec2 v1, const _pathfind_vec2 v2) {
		return (v1.x == v2.x && v1.y == v2.y);
	}
};

class _pathfind_node {
public:
	_pathfind_vec2 vec2;

	//the cost of path from start to this node.
	float G;

	//the cost of path from this node to goal.
	float H;

	//the best cost from start to goal through this node.
	float TotalCost;
	_pathfind_node* Parent;

	friend bool operator>(const _pathfind_node p1, const _pathfind_node p2) {
		return p1.TotalCost > p2.TotalCost;
	}

	friend bool operator<(const _pathfind_node p1, const _pathfind_node p2) {
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

#endif