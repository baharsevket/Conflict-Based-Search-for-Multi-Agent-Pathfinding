#pragma once
//#include "CBSDataStructures.h"
#include "LowLevel.h"
class CTNode
{
public:

	int get_total_cost() const
	{
		return 10;

	}

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
			_constraints.push_back(old_constraint_list[i]);
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
			_solution.push_back(new_solution[i]);
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

private:
	
	LowLevelCBS _lowLevelSolver;
	vector <Agent*> _agents;
	vector<CTNode*> open;

	bool validate_paths_in_node(CTNode& node);
	vector < Path*> find_paths_for_all_agents(CTNode &node);
	void update_solution_by_invoking_low_level(CTNode &node, int agentIndex);

	CTNode& get_node_with_lowest_cost() const
	{
		//TODO
		return *open[0];
	}
	
	void print_solution(CTNode& node);
};