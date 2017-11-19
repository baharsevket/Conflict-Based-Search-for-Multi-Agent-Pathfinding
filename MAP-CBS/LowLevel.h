#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "CBSDataStructures.h"

using namespace std;

class LowLevelCBS
{


public:

	//Vertex map[10][10];

	//TODO_BAHAR kesin daha guzeli vardýr bu ne sacmalik
	vector<vector<Vertex*>> map;	

	LowLevelCBS()
	{
		
	}

	void InitializeMap(int height, int width)
	{
		_gridHeight = height;
		_gridWidth = width;

		for (int i = 0; i < _gridHeight; i++)
		{
			vector<Vertex*> temp;
			map.push_back(temp);
			for (int j = 0; j < _gridWidth; j++)
			{
				map[i].push_back(new Vertex(i, j));
			}
		}
	}

	static void SplitStringByWhiteSpace(string str, vector <string> &cds)
	{
		string temp;
		stringstream s(str);
		while (s >> temp)
			cds.push_back(temp);
	}	

	bool AStar(Vertex* start, Vertex* goal, Path &path, const vector<Constraint*> &constraints)
	{
		clear_map_AStar_values();
		start->g = 0;
		start->f = heuristic_cost_estimate(*start, *goal);
		start->depth = 0;
		init_open_and_closed(start);

		//int timeStep = 0;

		//TODO_BAHAR need better limit case to terminate a*
		while (!_open.empty() && _closed.size() < _gridHeight * _gridWidth * 20)
		{
			int index = get_node_with_least_f(_open);
			Vertex* current = _open[index];
			std::vector<Vertex*>::iterator it = (_open.begin() + index);
			_open.erase(it);
			//_closed.push_back(current);	


			// 			for each neighbor of current
			// 				if neighbor in closedSet
			// 					continue		// Ignore the neighbor which is already evaluated.
			// 					// The distance from start to a neighbor
			//				tentative_gScore := gScore[current] + dist_between(current, neighbor)
			// 				  if neighbor not in openSet	// Discover a new node
			// 					  openSet.Add(neighbor)
			// 				  else if tentative_gScore >= gScore[neighbor]
			// 				  continue		// This is not a better path.
			// 
			// 					  // This path is the best until now. Record it!
			// 					  cameFrom[neighbor] := current
			// 					  gScore[neighbor] := tentative_gScore
			// 					  fScore[neighbor] := gScore[neighbor] + heuristic_cost_estimate(neighbor, goal)

			if ((*current) == (*goal))
			{
				path = reconstruct_path(current);
				return true;
			}

			vector<Vertex*> successors;
			fill_neighboors(*current, successors);

			for (int i = 0; i < successors.size(); i++)
			{
				// 				if (*successors[i] == *current && new_cost != successors[i]->g)
				// 				{
				// 					int a = 3;
				// 				}
				// 				else 

				if (successors[i]->x == current->x && successors[i]->y == current->y)
				{
					successors[i] = new Vertex(current->x, current->y, false);
					successors[i]->g = current->g;
					successors[i]->f = current->f;
					successors[i]->h = current->h;
				}


				if (std::find(_closed.begin(), _closed.end(), successors[i]) != _closed.end())
				{
					continue;
				}

				int new_cost = current->g + heuristic_cost_estimate(*successors[i], *current);


				if (std::find(_open.begin(), _open.end(), successors[i]) == _open.end()
					&& !has_conflict(successors[i], current->depth + 1, constraints))
				{					
					_open.push_back(successors[i]);
				}
				else if (new_cost >= successors[i]->g)
				{
					continue;
				}

				if (!has_conflict(successors[i], current->depth + 1, constraints))
				{
					successors[i]->Parent = current;
					successors[i]->depth = current->depth + 1;
					successors[i]->g = new_cost;
					successors[i]->f = successors[i]->g + heuristic_cost_estimate(*successors[i], *goal);
				}
			}

			_closed.push_back(current);

			//timeStep++;
		}

		return false;
	}

	int GetWidth()
	{
		return _gridWidth;
	}

	int GetHeight()
	{
		return _gridHeight;
	}

private:

	int _gridWidth;
	int _gridHeight;

	void init_open_and_closed(Vertex* start)
	{
		_open.clear();
		_closed.clear();
		_open.push_back(start);
	}

	void clear_map_AStar_values()
	{
		for (int i = 0; i < _gridWidth; i++)
		{
			for (int j = 0; j < _gridHeight; j++)
			{
				map[i][j]->g = 0;
				map[i][j]->f = 0;
				map[i][j]->h = 0;
				map[i][j]->Parent = NULL;
				map[i][j]->depth = 0;
			}
		}
	}

	void fill_neighboors(const Vertex& node, vector<Vertex*> &successors) const
	{
		//generate q's 8 successors and set their parents to q
		// 			vector<Vertex*> successors;
		// 			for (int i = clip(current->x - 1, 0, xMax); i <= clip(current->x + 1, 0, xMax); i++)
		// 			{
		// 				for (int j = clip(current->y - 1, 0, yMax); j <= clip(current->y + 1, 0, xMax); j++)
		// 				{
		// 					if(i != current->x && j != current->y)
		// 					{
		// 						Vertex* child = new Vertex(i, j);
		// 						successors.push_back(child);
		// 					}
		// 				}
		// 			}


		//get 4 in my case, can change it later
		pair<int, int> neighboorIndices[5];
		neighboorIndices[0] = make_pair(node.x - 1, node.y);
		neighboorIndices[1] = make_pair(node.x, node.y - 1);
		neighboorIndices[2] = make_pair(node.x, node.y + 1);
		neighboorIndices[3] = make_pair(node.x + 1, node.y);
		neighboorIndices[4] = make_pair(node.x, node.y);

		for (int i = 0; i < 5; i++)
		{
			if (neighboorIndices[i].first >= 0 && neighboorIndices[i].first < _gridWidth
				&& neighboorIndices[i].second >= 0 && neighboorIndices[i].second < _gridHeight)
			{
				//Vertex* child = new Vertex(neighboorIndices[i].first, neighboorIndices[i].second);

				Vertex *child = map[neighboorIndices[i].first][neighboorIndices[i].second];
				if (!child->Obstacle)
				{
					successors.push_back(child);
				}
			}
		}
	}


	int heuristic_cost_estimate(const Vertex& a, const Vertex& b) const
	{
		int manhattan_distance = get_manhattan_distance(a, b);
		return manhattan_distance == 0 ? 1 : manhattan_distance;
	}


	int get_manhattan_distance(const Vertex& a, const Vertex& b) const
	{
		return abs(a.x - b.x) + abs(a.y - b.y);
	}

	double get_square_distance_between_nodes(const Vertex& a, const Vertex& b) const
	{
		return pow((a.x - b.x), 2) + pow((a.y - b.y), 2);
	}

	int get_node_with_least_f(const vector<Vertex*> &list) const
	{
		//TODO
		int min = INT_MAX;
		int min_index = 0;
		for (int i = 0; i < list.size(); i++)
		{
			if (list[i]->f < min)
			{
				min = list[i]->f;
				min_index = i;
			}
		}

		return min_index;
	}

	Path reconstruct_path(Vertex* node)
	{
		vector<Vertex*> path_reverse;
		while (node != NULL)
		{
			path_reverse.push_back(node);
			node = node->Parent;
		}

		Path new_path;
		for (int i = path_reverse.size() - 1; i >= 0; i--)
		{
			new_path.Nodes.push_back(path_reverse[i]);
		}

		return new_path;
	}

	bool has_conflict(Vertex* vertex, int time, const vector<Constraint*> &constraints)
	{
		for (int i = 0; i < constraints.size(); i++)
		{
			if (constraints[i]->TimeStep == time && constraints[i]->Vertex->x == vertex->x && constraints[i]->Vertex->y == vertex->y)
			{
				return true;
			}
		}

		return false;
	}


	vector<Vertex*> _open;
	vector<Vertex*> _closed;
};


// 	// A*
// 	initialize the open list
//	initialize the closed list
//	put the starting node on the open list(you can leave its f at zero)

//	while the open list is not empty
//		find the node with the least f on the open list, call it "q"
//		pop q off the open list
//		generate q's 8 successors and set their parents to q
//		for each successor
//			if successor is the goal, stop the search
//				successor.g = q.g + distance between successor and q
//				successor.h = distance from goal to successor
//				successor.f = successor.g + successor.h

//				if a node with the same position as successor is in the OPEN list \
//					which has a lower f than successor, skip this successor
//					if a node with the same position as successor is in the CLOSED list \
						which has a lower f than successor, skip this successor
//						otherwise, add the node to the open list
//						end
//						push q on the closed list
//						end