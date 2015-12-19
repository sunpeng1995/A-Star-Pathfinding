#include "AStarSearch.h"
#include "MinHeap.cpp"



int main() {
// 	std::vector<_pathfind_vec2> vec;
// 	vec.push_back(_pathfind_vec2(1,2));
// 	vec.push_back(_pathfind_vec2(2,2));
// 	vec.push_back(_pathfind_vec2(3,1));
// 	_pathfind_vec2 a = _pathfind_vec2(1, 2);
// 	for (int i = 0; i < vec.size(); i++) {
// 		bool b = (a == vec[i]);
// 		b = false;
// 	}
// 
// 	std::vector<_pathfind_node> _vec;
// 	_vec.push_back(_pathfind_node(_pathfind_vec2(1, 2), nullptr));
// 	_vec.push_back(_pathfind_node(_pathfind_vec2(2, 3), nullptr));
// 	_pathfind_node d = _pathfind_node(_pathfind_vec2(1, 2), nullptr);
// 	_pathfind_node e = _pathfind_node(_pathfind_vec2(2, 3), &d);
// 	bool f = (d == _vec[0]);
// 	bool g = (e == _vec[1]);
// 
// 	_pathfind_node* p = &_vec[0];
// 
// 	std::remove(_vec.begin(), _vec.end(), *p);
// 	_vec.erase(std::remove(_vec.begin(), _vec.end(), *p));

	int map[6][10] = { 0 };
	map[1][4] = 1000;
	map[1][5] = 1000;
	map[2][5] = 1000;
	map[2][5] = 1000;
	map[3][5] = 1000;
	map[4][4] = 1000;
	map[4][5] = 1000;

	//AStarSearch* astar = AStarSearch(map);

	system("pause");
}

AStarSearch::AStarSearch() 
{
	
}

AStarSearch::AStarSearch(int **map) 
{
	this->m_map = map;
}

AStarSearch::~AStarSearch() 
{

}

std::vector<_pathfind_node> 
AStarSearch::search(_pathfind_vec2 start_vec2, _pathfind_vec2 goal_vec2) 
{
	open_list.clear();
	close_list.clear();
	_pathfind_node start_node = _pathfind_node(start_vec2, nullptr);
	start_node.G = 0;
	start_node.H = cost_estimate(start_node.vec2, goal_vec2);
	start_node.TotalCost = start_node.G + start_node.H;
	open_list.insert(start_node);
	while (!open_list.isEmpty()) {
		_pathfind_node current_node = open_list.top();
		if (current_node.vec2.x == goal_vec2.x &&
			current_node.vec2.y == goal_vec2.y) {
			return back_tracking(current_node);
		}
		else
		{
			for (int i = 0; i < 8; i++) {
				_pathfind_node near_node = visit(current_node.vec2, i);
				if(!can_reach(near_node))
					continue;
				float new_cost = current_node.G
					+ traverse_cost(current_node, near_node)
					+ m_map[near_node.vec2.y][near_node.vec2.x];

				//if in close_list or open_list and doesn't have a lower cost,
				//then skip it
				_pathfind_node* visit_cl = find_close_list(near_node);
				_pathfind_node* visit_ol = find_open_list(near_node);
				if(visit_cl != nullptr && visit_cl->G <= new_cost)
					continue;
				if(visit_ol != nullptr && visit_ol->G <= new_cost)
					continue;

				near_node.Parent = &current_node;
				near_node.G = new_cost;
				near_node.H = cost_estimate(near_node.vec2, goal_vec2);
				near_node.TotalCost = near_node.G + near_node.H;

				if (visit_cl != nullptr) {
					//have lower cost, need check again.
					close_list.erase(std::remove(
						close_list.begin(), close_list.end(), *visit_cl));
				}
				else if (visit_ol != nullptr) {
					open_list.remove(*visit_ol);
					open_list.insert(near_node);
				}
				else {
					open_list.insert(near_node);
				}
			}
		}
		close_list.push_back(current_node);
	}
	std::vector<_pathfind_node> nil;
	return nil;
}

_pathfind_node 
AStarSearch::visit(_pathfind_vec2 node, int direction) 
{
	_pathfind_vec2 newVec2;
	newVec2.x = node.x;
	newVec2.y = node.y;
	switch (direction)
	{
	case 0:
		newVec2.x--;
		break;
	case 1:
		newVec2.x--;
		newVec2.y++;
		break;
	case 2:
		newVec2.y++;
		break;
	case 3:
		newVec2.x++;
		newVec2.y++;
		break;
	case 4:
		newVec2.x++;
		break;
	case 5:
		newVec2.x++;
		newVec2.y--;
		break;
	case 6:
		newVec2.y--;
		break;
	case 7:
		newVec2.x--;
		newVec2.y--;
		break;
	default:
		break;
	}
	_pathfind_node newNode;
	newNode.vec2 = newVec2;
	return newNode;
}

float AStarSearch::traverse_cost(_pathfind_node node1, _pathfind_node node2) {
	float delta_x = node1.vec2.x - node2.vec2.x;
	float delta_y = node1.vec2.y - node2.vec2.y;
	return sqrt(delta_x*delta_x + delta_y*delta_y);
}

std::vector<_pathfind_node>
AStarSearch::back_tracking(_pathfind_node start) 
{
	std::vector<_pathfind_node> path;
	_pathfind_node* p = &start;
	while (p != nullptr) {
		path.push_back(*p);
		p = p->Parent;
	}
	return path;
}

bool AStarSearch::can_reach(_pathfind_node node) 
{
	int x = node.vec2.x;
	int y = node.vec2.y;
	if (x >= 0 && x < _width && y >= 0 && y < _height)
		return true;
	return false;
}

_pathfind_node* AStarSearch::find_open_list(_pathfind_node t) {
	int index = open_list.find(t);
	if (index != -1) {
		return &open_list.get(index);
	}
	return nullptr;
}

_pathfind_node* AStarSearch::find_close_list(_pathfind_node t) {
	int cl_size = close_list.size();
	for (int i = 0; i < cl_size; i++) {
		if (t == close_list[i])
			return &close_list[i];
	}
	return nullptr;
}

void AStarSearch::input_map(int **map, int width, int height) {
	m_map = map;
	_width = width;
	_height = height;
}

void AStarSearch::input_start(int x, int y) {
	start_pos = _pathfind_vec2(x, y);
}

void AStarSearch::input_goal(int x, int y) {
	goal_pos = _pathfind_vec2(x, y);
}
