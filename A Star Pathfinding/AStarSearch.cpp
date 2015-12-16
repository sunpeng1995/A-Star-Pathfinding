#include "AStarSearch.h"
#include "MinHeap.cpp"

class test {
public:
	int time;
	int distance;
	friend bool operator<(const test t1, const test t2) {
		return t1.distance > t2.distance;
	}
	test(int _time, int _distance) :
		time(_time), distance(_distance) {

	}
};

int main() {
	MinHeap<int> heap;
	int a[] = { 80,40,30,60,90,70,10,50,20 };
	for (int i = 0; i < 9; i++)
	{
		heap.insert(a[i]);
	}
	heap.print();
	printf("\n");
	heap.insert(15);
	heap.print();
	printf("\n");
	heap.remove(10);
	heap.print();
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
	_pathfind_node start_node = _pathfind_node(start_vec2, NULL);
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
				//TODO: how to get the adjacency node.
			}
		}
	}
}

std::vector<_pathfind_node>
AStarSearch::back_tracking(_pathfind_node start) 
{

}

bool AStarSearch::can_reach(_pathfind_node node) 
{

}