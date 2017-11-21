//#include "CBSDataStructures.h"
#include <iostream>
#include "CBS.h"


//TODO general list
//--throw exception when input cannot be read   

//++ cells with obstacle
//++ solution for the conflict(needs to be tested)
//++ cost, get_best_node
//++ read from input file
//++ input with different size grid

//-Note that for a given CT node N, one does not have to save all its cumulative constraints.
//Instead, it can save only its latest constraint and extract the other constraints by traversing the path from N to the root via its ancestors.
//Similarly, with the exception of the root node, the low - level search should only be performed for agent ai which is associated with the 
//newly added constraint.The paths of other agents remain the same as no new constraints are added for them.


int main()
{

	HighLevelCBS CBS;
	CBS.high_level_CBS();

	return 0;
}