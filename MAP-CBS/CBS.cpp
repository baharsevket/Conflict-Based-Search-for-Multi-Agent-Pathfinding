#include "CBS.h"
#include <iostream>
#include <algorithm>

HighLevelCBS::HighLevelCBS()
{
	ReadInput();
}


bool HighLevelCBS::validate_paths_in_node(CTNode& node)
{
	bool valid_solution = true;
	node.clear_conflicts();
	
	int lastTimeStep = 0;
	vector < Path*> solution = node.get_solution();

	if (solution.size() == 0)
	{
		return false;
	}

	for (int i = 0; i < solution.size(); i++)
	{
		int currentNodeSize = solution[i]->Nodes.size();
		if (lastTimeStep < currentNodeSize)
		{
			lastTimeStep = currentNodeSize;
		}
	}

	// 	//TODO what if all agents are at their goal state initially
	// 	if (lastTimeStep == 0)
	// 	{
	// 		valid_solution = false;
	// 	}


	//TODO same conflict is added twice for both agents, if all conflicts are added
	for (int i = 0; i < lastTimeStep; i++)
	{
		//TODO compare each agent with each agent, is there any other way?
		for (int j = 0; j < solution.size(); j++)
		{
			int a = std::min((int)solution[j]->Nodes.size() - 1, i);

			for (int k = 0; k < solution.size(); k++)
			{
				if (j == k)
					continue;
			
				int b = std::min((int)solution[k]->Nodes.size() - 1, i);


				if (*(solution[j]->Nodes[a]) == *(solution[k]->Nodes[b]))
				{
					node.add_conflict(new Conflict(_agents[solution[j]->agentIndex], _agents[solution[k]->agentIndex], solution[j]->Nodes[a], i));
					valid_solution = false;
					return valid_solution;
				}
			}
		}
	}

	return valid_solution;
}


//TODO instead of return maybe reference
vector < Path*> HighLevelCBS::find_paths_for_all_agents(CTNode &node)
{
	vector<Path*> paths;
	for (int i = 0; i < _agents.size(); i++)
	{
		Vertex *start = _lowLevelSolver.map[_agents[i]->StartStateX][_agents[i]->StartStateY];//new Vertex(_agents[i]->StartStateX, _agents[i]->StartStateY);
		Vertex *goal = _lowLevelSolver.map[_agents[i]->GoalStateX][_agents[i]->GoalStateY];//new Vertex(_agents[i]->GoalStateX, _agents[i]->GoalStateY);
		_lowLevelSolver.AStar(start, goal, *(_agents[i]->path), node.get_constraints());
		_agents[i]->path->agentIndex = i;
		paths.push_back(_agents[i]->path);
	}

	return paths;
}


bool HighLevelCBS::update_solution_by_invoking_low_level(CTNode &node, int agentIndex)
{
	//TODO assert	
	//TODO i should probably not use agent.path
	Vertex *start = _lowLevelSolver.map[_agents[agentIndex]->StartStateX][_agents[agentIndex]->StartStateY];//new Vertex(_agents[i]->StartStateX, _agents[i]->StartStateY);
	Vertex *goal = _lowLevelSolver.map[_agents[agentIndex]->GoalStateX][_agents[agentIndex]->GoalStateY];//new Vertex(_agents[i]->GoalStateX, _agents[i]->GoalStateY);
	if (_lowLevelSolver.AStar(start, goal, *(_agents[agentIndex]->path), node.get_constraints()))
	{
		_agents[agentIndex]->path->agentIndex = agentIndex;
		node.set_solution_for_agent(*_agents[agentIndex]);
		return true;
	}
	else
	{
		return false;
	}
	//paths.push_back(_agents[i]->path);
}

vector < Path*> HighLevelCBS::high_level_CBS()
{
	int debugIndex = 0;

	CTNode* root = new CTNode();
	root->debugIndex = debugIndex++;
	//root.solution = find individual paths by the low level()
	root->set_solution(find_paths_for_all_agents(*root));
	root->cost = get_SIC(root->get_solution());
	//root.cost = SIC(Root.solution)

	_open.push_back(root);

	while (!_open.empty())
	{
		CTNode *P;
		if (retrieve_and_pop_node_with_lowest_cost(&P))
		{
			print_solution(*P);

			bool valid = validate_paths_in_node(*P);

			if (valid)
			{
 				return P->get_solution();
			}

			Conflict conflict = P->get_first_conflict();

			for (int i = 0; i < 2; i++)
			{
				CTNode* new_ct_node = new CTNode();
				new_ct_node->add_constraints(P->get_constraints(), new Constraint(conflict.Agents[i], conflict.Vertex, conflict.TimeStep));
				new_ct_node->set_solution(P->get_solution());
				bool path_found = update_solution_by_invoking_low_level(*new_ct_node, conflict.Agents[i]->Index);

				if (path_found)
				{
					new_ct_node->cost = get_SIC(new_ct_node->get_solution());// SIC(A.solution)
																			 // a solution was found how??
					if (new_ct_node->cost < INT_MAX)
					{
						new_ct_node->debugIndex = debugIndex++;
						_open.push_back(new_ct_node);
					}
				}				
			}

			delete P;
		}
	}
}

void HighLevelCBS::print_solution(CTNode& node)
{
	vector<Path*> solution = node.get_solution();

	for (int k = 0; k < solution.size(); k++)
	{
		std::cout << "Agent " << k << ":" << std::endl;
		for (int i = 0; i < _lowLevelSolver.map.size(); i++)
		{
			for (int j = 0; j < _lowLevelSolver.map[i].size(); j++)
			{
				bool agentFoundInCell = false;
				//for (int k = 0; k < solution.size(); k++)
				//{
				for (int m = 0; m < solution[k]->Nodes.size(); m++)
				{
					if (solution[k]->Nodes[m] == _lowLevelSolver.map[i][j])
					{
						agentFoundInCell = true;
						std::cout << "|" << solution[k]->agentIndex << "," << m << "| ";
					}
				}
				//}

				if (!agentFoundInCell)
				{
					if (_lowLevelSolver.map[i][j]->Obstacle)
					{
						std::cout << "|_X_| ";
					}
					else
					{
						std::cout << "|___| ";
					}
				}
			}

			std::cout << std::endl;
		}
	}
}


float clip(float n, float lower, float upper)
{
	return std::max(lower, std::min(n, upper));
}

int HighLevelCBS::get_SIC(const vector < Path*> &solution)
{
	int cost = 0;
	for (int i = 0; i < solution.size(); i++)
	{
		cost += solution[i]->get_cost();
	}

	return cost;
}
