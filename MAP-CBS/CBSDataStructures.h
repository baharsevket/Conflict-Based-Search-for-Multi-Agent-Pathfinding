#pragma once
#include <vector>
#include <climits>
#include <algorithm>





using namespace std;

struct Constraint;

struct Vertex
{
	Vertex() :
		g(0), h(0), f(0)
	{
		Parent = NULL;
	}


	Vertex(int x, int y, bool obstacle = false) :
		x(x), y(y), g(0), h(0), f(0)
	{
		Parent = NULL;
		Obstacle = obstacle;
		depth = 0;
	}


	inline bool operator == (const Vertex &v) const
	{
		return v.x == this->x && v.y == this->y;
	}

	int x;
	int y;

	int g;
	int h;
	int f;

	Vertex* Parent;

	int depth;

	bool Obstacle;
};


struct Path
{
	Path()
	{

	}

	Path(int index) : agentIndex(index)
	{

	}


	int agentIndex;
	vector<Vertex*> Nodes;
	//vector<Vertex*> constraints;
	vector<Constraint*> Constraints;

	//TODO get real cost with nodes cost
	int get_cost()
	{
		return Nodes.size();
	}
};

struct Agent
{
	Agent(int index, int startStateX1, int startStateY1, int goalStateX1, int goalStateY1) : Index(index), StartStateX(startStateX1), StartStateY(startStateY1), GoalStateX(goalStateX1), GoalStateY(goalStateY1)
	{
		path = new Path(index);
	}

	int Index;

	//Better implementation
	int StartStateX;
	int StartStateY;
	int GoalStateX;
	int GoalStateY;


	//TODO this probably not needed here
	Path *path;
	//vector<Constraint*> constraints;
};

struct Constraint
{
	Constraint(Agent* agent, Vertex* vertex, int timeStep) :
		Agent(agent), Vertex(vertex), TimeStep(timeStep)
	{
	}

	Agent* Agent;
	Vertex* Vertex;
	int TimeStep;
};


struct Conflict
{
	Conflict(Agent* agent1, Agent* agent2, Vertex* vertex, int timeStep) :
		Vertex(vertex), TimeStep(timeStep)
	{
		Agents[0] = agent1;
		Agents[1] = agent2;
	}

	//private:
	Agent* Agents[2];
	Vertex* Vertex;
	int TimeStep;
};




float Clip(float n, float lower, float upper);

