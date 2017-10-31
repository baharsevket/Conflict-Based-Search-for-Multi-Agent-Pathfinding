#pragma once
//#include "CBSDataStructures.h"
#include "LowLevel.h"
class CTNode
{
public:
	int cost;


	//TODO_BAHAR_1 destructiooooon

	const vector < Path*>& get_solution()
	{
		return _solution;
	}

	const Conflict& get_first_conflict() const
	{
		return *_conflicts[0];
	}

	void add_conflict(Conflict* new_conflict)
	{
		_conflicts.push_back(new_conflict);
	}

	void add_constraints(const vector<Constraint*> old_constraint_list, Constraint* new_constraint)
	{
		_constraints.clear();
		for (int i = 0; i< old_constraint_list.size(); i++)
		{
			_constraints.push_back(new Constraint(old_constraint_list[i]->Agent, old_constraint_list[i]->Vertex, old_constraint_list[i]->TimeStep));
		}

		_constraints.push_back(new_constraint);
	}

	const vector<Constraint*> get_constraints() const
	{
		return _constraints;
	}


	void set_solution(const vector <Path*> &new_solution)
	{
		_solution.clear();

		for (int i = 0; i < new_solution.size(); i++)
		{
			Path* new_path = new Path(new_solution[i]->agentIndex);
			new_path->Nodes = std::vector<Vertex*>(new_solution[i]->Nodes);
			
			for (int j = 0; j < new_solution[i]->Constraints.size(); j++)
			{
				new_path->Constraints.push_back(new Constraint(new_solution[i]->Constraints[j]->Agent, new_solution[i]->Constraints[j]->Vertex, new_solution[i]->Constraints[j]->TimeStep));
			}
			
			_solution.push_back(new_path);
		}
	}


	void set_solution_for_agent(Agent& agent)
	{
		_solution[agent.Index] = agent.path;
	}


private:

	vector<Constraint*> _constraints;

	//probably dont need this list
	vector < Path*> _solution;

	//TODO one or more conflicts
	vector <Conflict*> _conflicts;
};


class HighLevelCBS
{
public:
	HighLevelCBS();
	vector < Path*> high_level_CBS();
	void init_agents();
	int get_SIC(const vector < Path*> &solution);

private:

	LowLevelCBS _lowLevelSolver;
	vector <Agent*> _agents;
	vector<CTNode*> _open;

	bool validate_paths_in_node(CTNode& node);
	vector < Path*> find_paths_for_all_agents(CTNode &node);
	void update_solution_by_invoking_low_level(CTNode &node, int agentIndex);

	bool get_node_with_lowest_cost(CTNode** node) 
	{
		int minCostIndex = -1;
		for (int i = 0; i < _open.size(); i++)
		{
			int tempCost = _open[i]->cost;
			if (minCostIndex == -1 || _open[i]->cost < _open[minCostIndex]->cost)
			{
				minCostIndex = i;
			}
		}

		if (minCostIndex != -1)
		{
			*node = _open[minCostIndex];
			_open[minCostIndex] = _open.back();
			_open.pop_back();
			return true;
		}

		return false;
	}

	void print_solution(CTNode& node);
};