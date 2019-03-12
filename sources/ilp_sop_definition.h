/*
 * fischetti_op_milp.h
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#ifndef LP_GOP_FISCHETTI_OP_MILP_H_
#define LP_GOP_FISCHETTI_OP_MILP_H_

#include <limits>

#include <vector>
#include <list>
#include "crl/logging.h"
#include <cfloat>
#include <limits>

#include "coords.h"

#include "crl/gui/shapes.h"
#include "crl/gui/colormap.h"

#include "crl/gui/gui.h"
#include "canvasview_coords.h"

#include <unordered_set>

#include <ilcplex/ilocplex.h>

#include "my_defines.h"


#include <ilcplex/ilocplex.h>

#include "dataset_loader_op.h"
#include "heuristic_types.h"
#include "math_common.h"
typedef IloNumVarArray IloNumVarMatrix1D;
typedef IloArray<IloNumVarArray> IloNumVarMatrix2D;
typedef IloArray<IloArray<IloNumVarArray>> IloNumVarMatrix3D;
typedef IloArray<IloArray<IloArray<IloNumVarArray>>> IloNumVarMatrix4D;

#define VARIABLE_FROM_TO(fromCluster,fromClusterNode,toCluster,toClusterNode)  "x_"<<fromCluster<<"_"<<fromClusterNode<<"__"<<toCluster<<"_"<<toClusterNode
#define VARIABLE_FROM_TO_U(fromCluster,fromClusterNode,toCluster,toClusterNode)  "u_"<<fromCluster<<"_"<<fromClusterNode<<"__"<<toCluster<<"_"<<toClusterNode
#define VARIABLE_AUX_FROM_TO(fromCluster,toCluster)  "y_"<<fromCluster<<"__"<<toCluster
#define INDEX_1D( from , to , width ) ( width * from + to )
#define INDEX_2D_TO( id , width ) ( id % width )
#define INDEX_2D_FROM( id , width ) ( (id - ( id  % width )) / width )


namespace op {
class IlpSopSolver;

class IlpSopDefinition {
public:
	IlpSopDefinition(IloEnv & env_, IloModel & model_, std::vector<std::vector<GraphNode>> & nodesAllClusters_,
			std::vector<std::vector<std::vector<std::vector<double>>> > distances_,
			int startIndex_, int goalIndex_, double budget_, IlpSopSolver* solver_base_object_);
	virtual ~IlpSopDefinition();
	void adjustPriorities(IloCplex & cplex);
	void registerCallback(IloCplex & cplex);
	double getSolutionLength(IloCplex & cplex);
	std::vector<IndexSOP> parseSolutionVector(IloCplex & cplex);
	void setInitialSolution(IloCplex & cplex, std::vector<IndexSOP> initialSolution);
	int getLastImprovementIter();
	void setLastImprovementIter(int lastImprovementIter_);
	long getLastImprovementTime();
	void setLastImprovementTime(long lastImprovementTime_) ;
	std::vector<ImprovementLogRecord>  getImprovementLog();
	void setImprovementLog(std::vector<ImprovementLogRecord> improvementLog_);

	IloInt numAddedConstraints;
	IlpSopSolver* solver_base_object;
	IloExpr constraint_t_max_expr;

	private:
	void printAllSubsets(std::list<std::list<int>> & list_to_fill);
	void getAllSubsets(int n, std::list<std::list<int>> & list_to_fill);

	IloNumVarMatrix3D x;
	//IloIntVarMatrix4D x;
	IloNumVarArray y;



	IloNum lastObjVal;
	IloNum lastIncumbentLC;
	IloNum lastTime;

	IloObjective workerObj;
	int lastImprovementIter;
	long lastImprovementTime;
	std::vector<ImprovementLogRecord> improvementLog;

	IloEnv env;
	IloModel model;
	std::vector<std::vector<GraphNode>> nodesAllClusters;
	bool initial_solution_set;
	//std::vector<GraphNode> nodesAll;
	std::vector<std::vector<std::vector<std::vector<double>>> > distances;
	int startIndex;
	int goalIndex;
	double budget;
};

} /* namespace op */

#endif /* LP_GOP_FISCHETTI_OP_MILP_H_ */
