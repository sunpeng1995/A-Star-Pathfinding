#ifndef __MAP_NODE__
#define __MAP_NODE__

#include <iostream>
#include <algorithm>
#include "MinHeap.cpp"
#include "Node.h"

/**
 * A star search.
 * 
 *
 * @author Sun Peng
 * @date 2015/12/22
 */

class AStarSearch {
public:

	/**
	 * @method:    search
	 * @fullname:  AStarSearch::search
	 * @access:    public 
	 * @returns:   std::vector<_pathfind_node*>
	 * @qualifier:
	 * @parameter: _pathfind_vec2 start_position
	 * @parameter: _pathfind_vec2 goal_position
	 * 			   
	 * use the A-Star-Pathfinding searching the path,
	 * befor use this method, you should have input 
	 * the map.
	 */
	std::vector<_pathfind_node*> 
		search(_pathfind_vec2 start_position, _pathfind_vec2 goal_position);

	/**
	 * @method:    search
	 * @fullname:  AStarSearch::search
	 * @access:    public 
	 * @returns:   std::vector<_pathfind_node*>
	 * @qualifier:
	 * 			  
	 * use the A-Star-Pathfinding searching the path,
	 * before use this method, you should have input 
	 * the map, start node and goal node.
	 */
	std::vector<_pathfind_node*> search();

	/**
	 * @method:    AStarSearch
	 * @fullname:  AStarSearch::AStarSearch
	 * @access:    public 
	 * @returns:   
	 * @qualifier:
	 * @parameter: int * map
	 * @parameter: int width
	 * @parameter: int height
	 * 			   
	 * first parameter need an int type two-demension
	 * array's first element's address, whose element 
	 * value is passsing cost of that grid and -1 means 
	 * can't reach. 
	 */
	AStarSearch(int *map, int width, int height);
	AStarSearch();
	~AStarSearch();

	/**
	 * @method:    input_map
	 * @fullname:  AStarSearch::input_map
	 * @access:    public 
	 * @returns:   void
	 * @qualifier:
	 * @parameter: int * map
	 * @parameter: int width
	 * @parameter: int height
	 * 			   
	 * input the map which is an int type two-demension 
	 * array, first parameter need the first elements's
	 * address. the element value is passing cost of 
	 * this grid and -1 means can't reach.
	 */
	void input_map(int *map, int width, int height);

	void input_start(int x, int y);

	void input_goal(int x, int y);

private:
	int *m_map;
	int _width, _height;
	_pathfind_vec2 start_pos;
	_pathfind_vec2 goal_pos;
	MinHeap<_pathfind_node> open_list;
	std::vector<_pathfind_node> close_list;

	/**
	 * @method:    can_reach
	 * @fullname:  AStarSearch::can_reach
	 * @access:    private 
	 * @returns:   bool
	 * @qualifier:
	 * @parameter: _pathfind_node * node
	 * 			   
	 * check the node if out of range or is an obstacle.
	 */
	bool can_reach(_pathfind_node* node);

	/**
	 * @method:    back_tracking
	 * @fullname:  AStarSearch::back_tracking
	 * @access:    private 
	 * @returns:   std::vector<_pathfind_node*>
	 * @qualifier:
	 * @parameter: _pathfind_node * start
	 */
	std::vector<_pathfind_node*>
		back_tracking(_pathfind_node* start);

	/**
	 * @method:    cost_estimate
	 * @fullname:  AStarSearch::cost_estimate
	 * @access:    private 
	 * @returns:   int
	 * @qualifier:
	 * @parameter: _pathfind_vec2 s
	 * @parameter: _pathfind_vec2 g
	 * 			   
	 * calculate the estimate cost between two node
	 * with Manhattan method.
	 */
	int cost_estimate(_pathfind_vec2 s, _pathfind_vec2 g) {
		return abs(s.x - g.x) + abs(s.y - g.y);
	}
	bool is_obstacle(_pathfind_node* node, _pathfind_node* parent);

	/**
	 * @method:    visit
	 * @fullname:  AStarSearch::visit
	 * @access:    private 
	 * @returns:   _pathfind_node*
	 * @qualifier:
	 * @parameter: _pathfind_vec2 node
	 * @parameter: int direction
	 * 			   
	 * get the 8 near of the node.
	 */
	_pathfind_node* visit(_pathfind_vec2 node, int direction);

	/**
	 * @method:    traverse_cost
	 * @fullname:  AStarSearch::traverse_cost
	 * @access:    private 
	 * @returns:   float
	 * @qualifier:
	 * @parameter: _pathfind_node * node1
	 * @parameter: _pathfind_node * node2
	 */
	float traverse_cost(_pathfind_node* node1, _pathfind_node* node2);
	_pathfind_node* find_open_list(_pathfind_node* t);
	_pathfind_node* find_close_list(_pathfind_node* t);

};

#endif