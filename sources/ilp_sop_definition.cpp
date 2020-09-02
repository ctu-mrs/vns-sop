/*
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#include "ilp_sop_definition.h"

#include "ilp_sop_solver.h"
using crl::logger;

using namespace op;
using namespace crl;
//using namespace crl::gui;


bool debug_ilp_sop_formulas = false;

const double M = std::numeric_limits<double>::max();

#define FLAT_X(fromClusterNode,toClusterNode,num_to_cluster_nodes)  (num_to_cluster_nodes * fromClusterNode + toClusterNode )

IlpSopDefinition::IlpSopDefinition(IloEnv & env_, IloModel & model_,
		std::vector<std::vector<GraphNode>> & nodesAllClusters_,
		std::vector<std::vector<std::vector<std::vector<double>>> > distances_, int startIndex_, int goalIndex_,
		double budget_, IlpSopSolver* solver_base_object_) {

	INFO("ilp creation");
	this->initial_solution_set = false;
	this->env = env_;
	this->model = model_;
	this->nodesAllClusters = nodesAllClusters_;
	//this->nodesAll = nodesAll_;
	this->startIndex = startIndex_;
	this->goalIndex = goalIndex_;
	this->budget = budget_;
	this->distances = distances_;
	this->solver_base_object = solver_base_object_;
	this->x = IloNumVarMatrix3D(env); //binary edges between nodes
	this->y = IloNumVarArray(env); //whether the set was visited
	numAddedConstraints = 0;
	lastObjVal = -IloInfinity;
	lastIncumbentLC = -1;
	lastTime = 0;
	lastImprovementIter = 0;
	lastImprovementTime = 0;

	/***         variables_y (10) and sop_objective_ilp (2)      ***/
	IloExpr objectiveExpression(env);
	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() ; ++fromCluster ) {
		std::stringstream varname;
		varname << "y_" << fromCluster;

		IloBoolVar newvar(env, varname.str().c_str());
		y.add(newvar);

		double p_i = nodesAllClusters[fromCluster][0].reward;
		objectiveExpression += p_i * newvar;
	}

	model.add(IloMaximize(env, objectiveExpression));
	INFO_COND(debug_ilp_sop_formulas, "objective created:");
	INFO_COND(debug_ilp_sop_formulas, objectiveExpression);

	/*******************************************************/

	/***         variables_x            (11)           ***/

	INFO("create objective function and variables x");
	//objective function and add variables x_from_to
	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {

		x.add(IloNumVarMatrix2D(env));

		for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {

			x[fromCluster].add(IloNumVarMatrix1D(env));

			for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ; ++fromClusterNode ) {
				for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ; ++toClusterNode ) {
					std::stringstream varname;
					varname << VARIABLE_FROM_TO(fromCluster, fromClusterNode, toCluster, toClusterNode);
					IloBoolVar newvar(env, varname.str().c_str());
					x[fromCluster][toCluster - 1].add(newvar);
				}
			}

		}
	}
	INFO_COND(debug_ilp_sop_formulas, "variables x:");
	INFO_COND(debug_ilp_sop_formulas, x);
	/*******************************************************/

	/***         constraint_budget            (3)           ***/

	INFO_COND(debug_ilp_sop_formulas, "constraint_t_max_expr:");
	constraint_t_max_expr = IloExpr(env);
	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
		for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {

			int num_to_cluster_nodes = nodesAllClusters[toCluster].size();

			for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ; ++fromClusterNode ) {
				for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ; ++toClusterNode ) {

					double distance_ij = distances_[fromCluster][toCluster][fromClusterNode][toClusterNode];
					constraint_t_max_expr +=
							distance_ij
									* x[fromCluster][toCluster - 1][FLAT_X(fromClusterNode, toClusterNode,
											num_to_cluster_nodes)];
				}
			}
		}
	}

	IloConstraint constraint_t_max(constraint_t_max_expr <= budget);
	constraint_t_max.setName("constraint_t_max");
	model.add(constraint_t_max);
	INFO_COND(debug_ilp_sop_formulas, "constraint_t_max:");
	INFO_COND(debug_ilp_sop_formulas, constraint_t_max);
	/*******************************************************/

	/***        constraint_start_cluster   (4)           ***/

	IloExpr constraint_start_y_expr(env);
	constraint_start_y_expr += y[0];
	IloConstraint constraint_start_y(constraint_start_y_expr == 1);
	constraint_start_y.setName("constraint_start_y");
	//model.add(constraint_start_y);
	INFO_COND(debug_ilp_sop_formulas, "constraint_start_y:");
	INFO_COND(debug_ilp_sop_formulas, constraint_start_y);

	/*******************************************************/

	/***        constraint_end_cluster      (5)           ***/
	INFO_COND(debug_ilp_sop_formulas, "constraint goal to be used");

	IloExpr constraint_goal_y_expr(env);
	constraint_goal_y_expr += y[y.getSize() - 1];
	IloConstraint constraint_goal_y(constraint_goal_y_expr == 1);
	constraint_goal_y.setName("constraint_goal_y");
	model.add(constraint_goal_y);
	INFO_COND(debug_ilp_sop_formulas, "constraint_goal_y:");
	INFO_COND(debug_ilp_sop_formulas, constraint_goal_y);

	/*******************************************************/

	/***		      vertex_in_out_equal (6)            ***/

	INFO_COND(debug_ilp_sop_formulas, "in_out_equal:");
	for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() - 1 ; ++toCluster ) {
		int num_to_cluster_nodes = nodesAllClusters[toCluster].size();
		for ( int toClusterNode = 0 ; toClusterNode < num_to_cluster_nodes ; ++toClusterNode ) {

			IloExpr constraint_in_out_equal_int_expr(env);
			IloExpr constraint_in_out_equal_out_expr(env);
			for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
				for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ;
						++fromClusterNode ) {
					constraint_in_out_equal_int_expr += x[fromCluster][toCluster - 1][FLAT_X(fromClusterNode,
							toClusterNode, num_to_cluster_nodes)];
				}
			}

			for ( int fromCluster = 1 ; fromCluster < nodesAllClusters.size() ; ++fromCluster ) {
				if ( toCluster != fromCluster ) {
					int num_from_cluster_nodes = nodesAllClusters[fromCluster].size();
					for ( int fromClusterNode = 0 ; fromClusterNode < num_from_cluster_nodes ; ++fromClusterNode ) {
						constraint_in_out_equal_out_expr += x[toCluster][fromCluster - 1][FLAT_X(toClusterNode,
								fromClusterNode, num_from_cluster_nodes)];
					}
				}
			}
			IloConstraint constraint_in_out_equal(constraint_in_out_equal_int_expr == constraint_in_out_equal_out_expr);
			std::stringstream constraint_name;
			constraint_name << "in_out_equal_" << toCluster << "_" << toClusterNode;
			constraint_in_out_equal.setName(constraint_name.str().c_str());
			model.add(constraint_in_out_equal);
			INFO_COND(debug_ilp_sop_formulas, constraint_in_out_equal);

		}
	}
	/*******************************************************/

	/***            max_to_cluster_visited (7)              ***/
	for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {
		IloExpr max_to_expr(env);

		for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
			if ( fromCluster != toCluster ) {
				for ( int clusterNodes = 0 ; clusterNodes < x[fromCluster][toCluster - 1].getSize() ; ++clusterNodes ) {
					max_to_expr += x[fromCluster][toCluster - 1][clusterNodes];
				}
			}
		}

		IloConstraint max_to(max_to_expr == y[toCluster]);
		std::stringstream constraint_in_name;
		constraint_in_name << "max_to_" << toCluster;
		max_to.setName(constraint_in_name.str().c_str());
		model.add(max_to);
	}
	/*******************************************************/

	/*     max_from_cluster_visited (8)                */
	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
		IloExpr max_from_expr(env);

		for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {
			if ( fromCluster != toCluster ) {
				for ( int clusterNodes = 0 ; clusterNodes < x[fromCluster][toCluster - 1].getSize() ; ++clusterNodes ) {
					max_from_expr += x[fromCluster][toCluster - 1][clusterNodes];
				}
			}
		}

		IloConstraint max_from(max_from_expr == y[fromCluster]);
		std::stringstream constraint_out_name;
		constraint_out_name << "max_from_" << fromCluster;
		max_from.setName(constraint_out_name.str().c_str());
		model.add(max_from);

	}
	/*******************************************************/

	//further only strengthening
	/*
	 for ( int cluster = 1 ; cluster < nodesAllClusters.size() - 1 ; ++cluster ) {
	 //if (nodesAllClusters[cluster].size() > 1) {
	 for ( int clusterNodes = 0 ; clusterNodes < x[cluster][cluster - 1].getSize() ; ++clusterNodes ) {
	 IloExpr constraint_nothing_inside_cluster_expr(env);
	 constraint_nothing_inside_cluster_expr = x[cluster][cluster - 1][clusterNodes];
	 IloConstraint constraint_nothing_inside_cluster(constraint_nothing_inside_cluster_expr == 0);
	 std::stringstream constraint_name;
	 constraint_name << "nothing_inside_cluster_" << cluster << "_bet_" << clusterNodes;
	 constraint_nothing_inside_cluster.setName(constraint_name.str().c_str());
	 model.add(constraint_nothing_inside_cluster);
	 }
	 }
	 */

}

IlpSopDefinition::~IlpSopDefinition() {

}

void IlpSopDefinition::printAllSubsets(std::list<std::list<int>> & list_to_fill) {
	for ( std::list<std::list<int>>::iterator iterator = list_to_fill.begin(), end = list_to_fill.end() ; iterator != end ;
			++iterator ) {
		std::stringstream ss;
		for ( std::list<int>::iterator iterator2 = iterator->begin(), end = iterator->end() ; iterator2 != end ;
				++iterator2 ) {
			ss << *iterator2 << " ";
		}
		INFO(ss.str());
	}
}

/*
 For n=1, the set of subsets is {{}, {1}}
 For n>1, find the set of subsets of 1,...,n-1 and make two copies of it. For one of them, add n to each subset. Then take the union of the two copies.
 */
void IlpSopDefinition::getAllSubsets(int n, std::list<std::list<int>> & list_to_fill) {
	INFO("getAllSubsets "<<n);
	if ( n > 1 ) {
		getAllSubsets(n - 1, list_to_fill);
		std::list<std::list<int>> list_to_fill_copy = list_to_fill;

		for ( std::list<std::list<int>>::iterator iterator = list_to_fill_copy.begin(), end = list_to_fill_copy.end() ;
				iterator != end ; ++iterator ) {
			(*iterator).push_back(n);
//std::list<int> * val = iterator;	//->pu push_back(n);
		}

		list_to_fill.insert(list_to_fill.end(), list_to_fill_copy.begin(), list_to_fill_copy.end());
		//list_to_fill.merge(list_to_fill_copy);
		if ( n < 6 ) {
			INFO("-------------")
			printAllSubsets(list_to_fill);
		}
	} else {
		if ( list_to_fill.size() > 0 ) {
			list_to_fill.clear();
		}
		std::list<int> without;
		std::list<int> with;
		with.push_back(1);

		list_to_fill.push_back(without);
		list_to_fill.push_back(with);
		//printAllSubsets(list_to_fill);

	}
}

ILOHEURISTICCALLBACK6(heuristic_callback,
		IloNumVarArray, y_vars,
		IloNumVarMatrix3D, x_vars,
		std::vector<std::vector<GraphNode>>, nodesAllClusters_,
		std::vector<IndexSOP>, initialSolution_,
		double , initial_solution_reward_,
		bool*, initial_solution_set) {

	if ( !(*initial_solution_set) && getIncumbentObjValue() < initial_solution_reward_ ) {
		INFO_GREEN("heuristic_callback to inject feasible solution");
		INFO_GREEN("setting initial solution to "<<initial_solution_reward_<<" because cplex bug and actual incumbent below "<<getIncumbentObjValue());
		IloEnv envUsed = getEnv();
		IloNumVarArray PstartVar(envUsed);
		IloNumArray PstartVal(envUsed);
		*initial_solution_set = true;
		for ( int fromCluster = 0 ; fromCluster < nodesAllClusters_.size() - 1 ; ++fromCluster ) {
			for ( int toCluster = 1 ; toCluster < nodesAllClusters_.size() ; ++toCluster ) {
				int num_to_cluster_nodes = nodesAllClusters_[toCluster].size();
				bool fromtoUsed = false;
				int fromClusterNodeInPath = -1;
				int toClusterNodeInPath = -1;
				for ( int initSolVar = 1 ; initSolVar < initialSolution_.size() ; ++initSolVar ) {
					if ( initialSolution_[initSolVar - 1].clusterIndex == fromCluster
							&& initialSolution_[initSolVar].clusterIndex == toCluster ) {
						//path between this vertices
						fromClusterNodeInPath = initialSolution_[initSolVar - 1].nodeIndex;
						toClusterNodeInPath = initialSolution_[initSolVar].nodeIndex;
						//INFO("fill initial solution from "<<fromCluster<<":"<<fromClusterNodeInPath<<" to "<<toCluster<<":"<<toClusterNodeInPath);
						fromtoUsed = true;
						break;
					}
				}

				for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters_[fromCluster].size() ; ++fromClusterNode ) {
					for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters_[toCluster].size() ; ++toClusterNode ) {
						//vals_lok_x[FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)] = 0;
						PstartVar.add(
								x_vars[fromCluster][toCluster - 1][FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)]);

						if ( fromtoUsed && fromClusterNodeInPath == fromClusterNode
								&& toClusterNodeInPath == toClusterNode ) {
							PstartVal.add(1);
							//INFO("set "<<PstartVar[PstartVar.getSize()-1]<<" to one");
						} else {
							PstartVal.add(0);
						}
					}
				}

			}
		}

		double initial_solution_reward = 0;
		//IloNumArray vals_lok_y(envUsed, this->y.getSize());
		for ( int fromCluster = 0 ; fromCluster < nodesAllClusters_.size() ; ++fromCluster ) {
			bool isUsed = false;
			for ( int initSolVar = 0 ; initSolVar < initialSolution_.size() ; ++initSolVar ) {
				if ( initialSolution_[initSolVar].clusterIndex == fromCluster ) {
					isUsed = true;
					break;
				}
			}
			PstartVar.add(y_vars[fromCluster]);
			if ( isUsed ) {
				PstartVal.add(1);
			} else {
				PstartVal.add(0);
			}
		}
		setSolution(PstartVar, PstartVal);
		PstartVar.end();
		PstartVal.end();
		INFO_GREEN("heuristic_callback end");
	}
}

ILOMIPINFOCALLBACK6(op_milp_info_callback,
		IloNumVarArray, y_vars,
		IloNumVarMatrix3D, x_vars,
		std::vector<std::vector<GraphNode>>,nodesAllClusters,
		IlpSopDefinition*, solver_object,
		IloNum, lastIncumbent,
		IloNum, lastTime) {
	//INFO("op_milp_callback beg");

	bool printIncumbentSolution = false;

	int newIncumbent = 0;
	int nodes = getNnodes();
	IloNum times = getCplexTime() - getStartTime();
	//INFO("hasIncumbent() "<<hasIncumbent());
	//INFO("getIncumbentObjValue() "<<getIncumbentObjValue());
	if ( hasIncumbent() && fabs(lastIncumbent - getIncumbentObjValue()) > 1e-5 * (1.0 + fabs(getIncumbentObjValue())) ) {
		lastIncumbent = getIncumbentObjValue();
		newIncumbent = 1;

	}

	if ( newIncumbent || times - lastTime > 30.0 ) {
		lastTime = times;
		int totals, seconds, hours, minutes;

		totals = ((int) times);
		minutes = totals / 60;
		seconds = totals % 60;
		hours = minutes / 60;
		minutes = minutes % 60;
		std::stringstream ss;
		ss << std::fixed << std::setprecision(2);
		std::stringstream timehms;
		timehms << std::setfill('0') << std::setw(2) << hours << ":" << std::setfill('0') << std::setw(2) << minutes
				<< ":" << std::setfill('0') << std::setw(2) << seconds;

		//lastLog = nodes;
		ss << "Time = " << totals << "s (" << timehms.str() << ") Nodes = " << nodes << '(' << getNremainingNodes()
				<< ')'

				<< "  Best objective = " << getBestObjValue() << " Gap = " << (-getMIPRelativeGap() * 100.0) << "% "
				<< " added constraints " << solver_object->numAddedConstraints;

		if ( hasIncumbent() ) {
			ss << "  Incumbent objective = " << getIncumbentObjValue();
		}
		//INFO(ss.str());

	}
	if ( newIncumbent ) {

		std::list<int> used_nodes;
		IloNumArray vals_lok_y(y_vars.getEnv());
		getIncumbentValues(vals_lok_y, y_vars);
		for ( int var = 0 ; var < nodesAllClusters.size() ; ++var ) {
			int vals_lok_int_value_y = round(vals_lok_y[var]);
			if ( vals_lok_int_value_y > 0 ) {
				used_nodes.push_back(var);
				INFO_COND(printIncumbentSolution, "y_"<<var<<"="<<vals_lok_y[var]);
			}

		}
		vals_lok_y.end();

		std::list<IndexPairSOP> solutionListUnordered;

		INFO("get x solution slow");
		for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
			for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {
				IloNumArray vals_lok_x(x_vars.getEnv());
				getIncumbentValues(vals_lok_x, x_vars[fromCluster][toCluster - 1]);

				int num_to_cluster_nodes = nodesAllClusters[toCluster].size();

				for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ;
						++fromClusterNode ) {

					for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ; ++toClusterNode ) {

						int vals_lok_int_value_x = round(
								vals_lok_x[FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)]);
						if ( vals_lok_int_value_x > 0 ) {
							INFO_COND(printIncumbentSolution,
									VARIABLE_FROM_TO(fromCluster,fromClusterNode,toCluster,toClusterNode)<<"="<<vals_lok_int_value_x);
							IndexPairSOP indexPair;
							indexPair.cluster_from = fromCluster;
							indexPair.cluster_from_node = fromClusterNode;
							indexPair.cluster_to = toCluster;
							indexPair.cluster_to_node = toClusterNode;

							solutionListUnordered.push_back(indexPair);
						}
					}

					//}
				}
				vals_lok_x.end();
			}
		}

		INFO("solution vector contains "<<solutionListUnordered.size()<<" edges");
		if ( solutionListUnordered.size() != used_nodes.size() - 1 ) {
			ERROR("bad size of edges");
		}

		int startIndex = 0;
		int goalIndex = nodesAllClusters.size() - 1;
		//std::vector<GraphNode> solution;
		std::vector<IndexSOP> returnedPath;
		int last_to_cluster_index = startIndex;
		int last_to_index = 0;
		for ( std::list<IndexPairSOP>::iterator iterator = solutionListUnordered.begin(), end =
				solutionListUnordered.end() ; iterator != end ; ++iterator ) {
			if ( (*iterator).cluster_from == startIndex ) {
				//solution.push_back(nodesAllClusters[(*iterator).cluster_from][(*iterator).cluster_from_node]);
				IndexSOP newInPathId;
				newInPathId.clusterIndex = (*iterator).cluster_from;
				newInPathId.nodeIndex = (*iterator).cluster_from_node;
				returnedPath.push_back(newInPathId);
				break;
			}
		}

		while ( last_to_cluster_index != goalIndex && solutionListUnordered.size() > 0 ) {
			for ( std::list<IndexPairSOP>::iterator iterator = solutionListUnordered.begin(), end =
					solutionListUnordered.end() ; iterator != end ; ++iterator ) {
				if ( (*iterator).cluster_from == last_to_cluster_index ) {
					last_to_index = (*iterator).cluster_to_node;
					last_to_cluster_index = (*iterator).cluster_to;
					//solution.push_back(nodesAllClusters[(*iterator).cluster_to][(*iterator).cluster_to_node]);
					IndexSOP newInPathId;
					newInPathId.clusterIndex = (*iterator).cluster_to;
					newInPathId.nodeIndex = (*iterator).cluster_to_node;
					returnedPath.push_back(newInPathId);
					solutionListUnordered.erase(iterator);
					break;
				}
			}
		}
		solver_object->solver_base_object->drawPath(0, &returnedPath);
		int actual_num_iters = getNiterations();
		long actual_impr_mstime = (lastTime * 1000.0);
		solver_object->setLastImprovementIter(actual_num_iters);

		INFO_VAR(lastTime);
		INFO_VAR(actual_impr_mstime);
		solver_object->setLastImprovementTime(actual_impr_mstime);
		std::vector<ImprovementLogRecord> improvementLog = solver_object->getImprovementLog();
		ImprovementLogRecord impr_log;
		impr_log.length = getIncumbentValue(solver_object->constraint_t_max_expr);
		impr_log.timeMS = actual_impr_mstime;
		impr_log.reward = lastIncumbent;
		improvementLog.push_back(impr_log);
		solver_object->setImprovementLog(improvementLog);


		INFO_GREEN("has new incumbent with "<< getIncumbentObjValue()<<" value !!!!!");
		//drawPath();
	}
	//INFO("op_milp_callback end");
}

void printCycles(std::list<std::list<int>> cycles) {
	INFO("cycles are:");
	for ( std::list<std::list<int>>::iterator iterator = cycles.begin(), end = cycles.end() ; iterator != end ;
			++iterator ) {
		std::stringstream cycless;
		for ( std::list<int>::iterator iterator2 = iterator->begin(), end2 = iterator->end() ; iterator2 != end2 ;
				++iterator2 ) {

			cycless << *iterator2 << " ";
		}
		INFO(cycless.str());
	}
}

std::list<IndexPairSOP> orderSolution(std::list<IndexPairSOP> solutionListUnordered) {
	std::list<IndexPairSOP> orderedSolution;
	std::list<IndexPairSOP> solutionListUnorderedCopy = solutionListUnordered;
	//add first edge to solution
	orderedSolution.push_back(solutionListUnorderedCopy.front());
	solutionListUnorderedCopy.pop_front();

	while ( solutionListUnorderedCopy.size() > 0 ) {
		for ( std::list<IndexPairSOP>::iterator it = solutionListUnorderedCopy.begin() ; it != solutionListUnorderedCopy.end() ; ++it ) {
			IndexPairSOP pair = *it;
			if ( pair.cluster_to == orderedSolution.front().cluster_from ) {
				//fond edge before actual one
				orderedSolution.push_front(pair);
				solutionListUnorderedCopy.erase(it);
				break;
			}
			if ( pair.cluster_from == orderedSolution.back().cluster_to ) {
				orderedSolution.push_back(pair);
				solutionListUnorderedCopy.erase(it);
				break;
			}

		}
	}
	return orderedSolution;
}

std::list<std::list<int>> findCycles(std::list<IndexPairSOP> solutionListUnordered, int num_sets) {
	bool debugcycles = false;

	std::list<std::list<int>> cycles;

	INFO_COND(debugcycles, "findCycles");

	//remove edges connected from start
	INFO_COND(debugcycles, "remove edges connected from start");
	std::list<IndexPairSOP> edges = solutionListUnordered;
	int search_cluster_from = 0;
	bool something_found = true;
	while ( edges.size() > 0 && something_found ) {
		something_found = false;
		for ( std::list<IndexPairSOP>::iterator iterator = edges.begin(), end = edges.end() ; iterator != end ;
				++iterator ) {
			if ( (*iterator).cluster_from == search_cluster_from ) {
				search_cluster_from = (*iterator).cluster_to;
				edges.erase(iterator);
				something_found = true;
				break;
			}
		}
	}

	//remove edges connected from end
	INFO_COND(debugcycles, "remove edges connected from end");
	something_found = true;
	int search_clurster_to = num_sets - 1;
	while ( edges.size() > 0 && something_found ) {
		something_found = false;
		for ( std::list<IndexPairSOP>::iterator iterator = edges.begin(), end = edges.end() ; iterator != end ;
				++iterator ) {
			if ( (*iterator).cluster_to == search_clurster_to ) {
				search_clurster_to = (*iterator).cluster_to;
				edges.erase(iterator);
				something_found = true;
				break;
			}
		}
	}

	INFO_COND(debugcycles, "find subtours in rest");
	//rest must be in subtours
	while ( edges.size() > 0 ) {
		std::list<int> newcycle;
		newcycle.push_back(edges.begin()->cluster_to);
		newcycle.push_back(edges.begin()->cluster_from);
		INFO_COND(debugcycles, "init cluster "<<edges.begin()->cluster_to<<" "<<edges.begin()->cluster_from);
		edges.erase(edges.begin());
		something_found = true;
		bool cycle_closed = false;
		while ( edges.size() > 0 && something_found && !cycle_closed ) {
			something_found = false;
			for ( std::list<int>::iterator newcycleit = newcycle.begin(), newcycleend = newcycle.end() ;
					newcycleit != newcycleend ; ++newcycleit ) {
				INFO_COND(debugcycles, "test connection to "<<(*newcycleit));

				for ( std::list<IndexPairSOP>::iterator iterator = edges.begin(), end = edges.end() ; iterator != end ;
						++iterator ) {
					INFO_COND(debugcycles, "edge "<<(*iterator).cluster_from<<" "<<(*iterator).cluster_to);

					if ( (*iterator).cluster_from == *newcycleit ) {
						if ( std::find(newcycle.begin(), newcycle.end(), (*iterator).cluster_to) == newcycle.end() ) {
							//not in cycle
							newcycle.push_back((*iterator).cluster_to);
							INFO_COND(debugcycles, "add "<<(*iterator).cluster_to);
						} else {
							cycle_closed = true;
						}
						something_found = true;
						edges.erase(iterator);
						INFO_COND(debugcycles, "break");
						break;
					}
					if ( (*iterator).cluster_to == *newcycleit ) {

						if ( std::find(newcycle.begin(), newcycle.end(), (*iterator).cluster_from) == newcycle.end() ) {
							//not in cycle
							newcycle.push_back((*iterator).cluster_from);

							INFO_COND(debugcycles, "add "<<(*iterator).cluster_from);
						} else {
							cycle_closed = true;
						}
						something_found = true;
						edges.erase(iterator);
						INFO_COND(debugcycles, "break");
						break;
					}

				}
				if ( something_found ) {
					break;
				}

			}
		}
		cycles.push_back(newcycle);
	}

	return cycles;
}

/*
 add subtour elimination constraints
 */
ILOLAZYCONSTRAINTCALLBACK6(op_milp_lazy_constraints_callback, IloNumVarArray, y_vars, IloNumVarMatrix3D,
		x_vars, std::vector<std::vector<GraphNode>>,nodesAllClusters,
		IloObjective, workerObj,IloInt , numAddedConstraints,
		std::vector<std::vector<std::vector<std::vector<double>>>>, distances) {
	INFO_CYAN("op_milp_lazy_constraints_callback beg");
	bool debuglazycontraints = false;

	int nodes = getNnodes();
	int ramaining_nodes = getNremainingNodes();
	double best_obj = getBestObjValue();
	double gap = getMIPRelativeGap();
	double actual_obj = getObjValue();

	INFO_COND(debuglazycontraints,
			"op_milp_lazy_constraints_callback, nodes="<<nodes<<", remaining nodes="<<ramaining_nodes<<", best objective="<<best_obj<<", gap"<<gap);

	IloInt i;
	IloEnv masterEnv = getEnv();

	//get solution of y
	IloNumArray sol_y(masterEnv);
	getValues(sol_y, y_vars);						//y is the tested subset

	IloInt numsets = sol_y.getSize();
	std::list<int> used_nodes;

	for ( int setID = 0 ; setID < numsets ; ++setID ) {
		int vals_lok_int_value_y = round(sol_y[setID]);
		if ( vals_lok_int_value_y > 0 ) {
			used_nodes.push_back(setID);
			INFO_COND(debuglazycontraints, "y_"<<setID<<"="<<sol_y[setID]);
		}
	}

	sol_y.end();

	/*get x solution start*/
	std::list<IndexPairSOP> solutionListUnordered;
	std::list<int> getting_solution_nodes = used_nodes;

	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
		for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {

			IloNumArray vals_lok_x(x_vars.getEnv());
			getValues(vals_lok_x, x_vars[fromCluster][toCluster - 1]);

			int num_to_cluster_nodes = nodesAllClusters[toCluster].size();
			bool fromThisClusteFound = false;

			for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ; ++fromClusterNode ) {

				for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ; ++toClusterNode ) {

					int vals_lok_int_value_x = round(
							vals_lok_x[FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)]);
					if ( vals_lok_int_value_x > 0 ) {
						IndexPairSOP indexPair;
						indexPair.cluster_from = fromCluster;
						indexPair.cluster_from_node = fromClusterNode;
						indexPair.cluster_to = toCluster;
						indexPair.cluster_to_node = toClusterNode;
						solutionListUnordered.push_back(indexPair);
						fromThisClusteFound = true;
						break;
					}
				}
				if ( fromThisClusteFound ) {
					break;
				}

			}
			vals_lok_x.end();

		}
	}
	/*get x solution end*/

	//vals_all_lok_x.end();
	INFO_COND(debuglazycontraints, "find cycles");

	std::list<std::list<int>> cycles = findCycles(solutionListUnordered, nodesAllClusters.size());
	if ( cycles.size() > 0 ) {
		INFO_COND(debuglazycontraints, "num cycles "<<cycles.size());
	}
	bool hassubtours = false;
	for ( std::list<std::list<int>>::iterator cycle_it = cycles.begin(), iterator1end = cycles.end() ;
			cycle_it != iterator1end ; ++cycle_it ) {
		//one cycle is here
		//do not add the first y as it has to have less edges than vertices
		std::stringstream ss;
		for ( std::list<int>::iterator it1 = cycle_it->begin(), it1end = cycle_it->end() ; it1 != it1end ; ++it1 ) {
			ss << *it1 << " ";
		}
		//INFO("considered cycle "<<ss.str());

		for ( std::list<int>::iterator it1 = cycle_it->begin(), it1end = cycle_it->end() ; it1 != it1end ; ++it1 ) {
			//one elimination is here with omiting it1 cluster

			IloExpr subtour_elimination_expr_y(masterEnv);
			IloExpr subtour_elimination_expr_x(masterEnv);

			int consideredCluster = *it1;

			for ( std::list<int>::iterator it2 = cycle_it->begin(), it1end = cycle_it->end() ; it2 != it1end ; ++it2 ) {
				if ( *it1 != *it2 ) {
					subtour_elimination_expr_y += y_vars[*it2];
				}
			}

			for ( std::list<int>::iterator fromit = cycle_it->begin(), fromitend = cycle_it->end() ; fromit != fromitend ;
					++fromit ) {
				int fromCluster = *fromit;
				//INFO("fromCluster "<<fromCluster);

				for ( std::list<int>::iterator toit = cycle_it->begin(), toitend = cycle_it->end() ; toit != toitend ;
						++toit ) {
					int toCluster = *toit;

					if ( fromCluster != toCluster ) {
						//INFO("toCluster "<<toCluster);
						int num_to_cluster_nodes = nodesAllClusters[toCluster].size();
						for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ;
								++fromClusterNode ) {
							for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ;
									++toClusterNode ) {
								subtour_elimination_expr_x += x_vars[fromCluster][toCluster - 1][FLAT_X(fromClusterNode,
										toClusterNode, num_to_cluster_nodes)];
							}
						}

					}
				}
			}
			//INFO("subtour "<<subtour_elimination_expr_y);
			IloConstraint subtour_elimination(subtour_elimination_expr_x <= subtour_elimination_expr_y);
			//INFO("add subtour elimination constraint:"<<subtour_elimination);
			//INFO_COND(debuglazycontraints, "add constraint:"<<subtour_elimination);
			hassubtours = true;
			add(subtour_elimination);
			subtour_elimination.end();
			subtour_elimination_expr_x.end();
			subtour_elimination_expr_y.end();
			numAddedConstraints += 1;
		}
	}

	if ( hassubtours ) {
		INFO_CYAN("op_milp_lazy_constraints_callback end - added constraints "<<numAddedConstraints<<" exit after having subtour ");
		return;
	} else {
		INFO_GREEN("has no subtours!!!!!");
		INFO_GREEN("actual obj value "<<actual_obj);
		INFO_GREEN("best obj value "<<best_obj);
		double reward_calculated = 0;
		double distance_calculated = 0;
		for ( std::list<IndexPairSOP>::iterator iterator = solutionListUnordered.begin(), end = solutionListUnordered.end() ; iterator != end ;
				++iterator ) {
			IndexPairSOP soppair = (*iterator);
			reward_calculated += nodesAllClusters[soppair.cluster_from][0].reward;
			//INFO("from "<<soppair.cluster_from<<":"<<soppair.cluster_from_node<<" to "<<soppair.cluster_to<<":"<<soppair.cluster_to_node);
			distance_calculated += distances[soppair.cluster_from][soppair.cluster_to][soppair.cluster_from_node][soppair.cluster_to_node];
		}
		INFO_GREEN("reward_calculated "<<reward_calculated);
		INFO_GREEN("distance_calculatedcm "<<distance_calculated);

		/*
		 IloExpr not_used_clusters(masterEnv);
		 IloExpr all_clusters(masterEnv);
		 std::list<int> used_clusters_list = used_nodes;

		 for ( int yid = 0 ; yid < y_vars.getSize() ; ++yid ) {
		 bool is_in_solution = false;
		 for ( std::list<int>::iterator iterator = used_clusters_list.begin(), end = used_clusters_list.end() ; iterator != end ;
		 ++iterator ) {
		 if ( (*iterator) == yid ) {
		 used_clusters_list.erase(iterator);
		 is_in_solution = true;
		 break;
		 }
		 }
		 if ( !is_in_solution ) {
		 not_used_clusters += y_vars[yid];
		 }
		 all_clusters += y_vars[yid] * nodesAllClusters[yid][0].reward;
		 }
		 IloConstraint different_cluster_also_used(not_used_clusters >= 1);
		 add(different_cluster_also_used);
		 INFO("added different_cluster_also_used "<<different_cluster_also_used);
		 IloConstraint all_clusters_bigger_reward(all_clusters > reward_calculated);
		 add(all_clusters_bigger_reward);
		 INFO("added all_clusters_bigger_reward "<<all_clusters_bigger_reward);
		 */
	}

	int added_cluster_node_selects = 0;

	INFO_CYAN(
			"op_milp_lazy_constraints_callback end - added constraints "<<numAddedConstraints<<" added specific cluster "<<added_cluster_node_selects);

}

void IlpSopDefinition::registerCallback(IloCplex & cplex) {
	cplex.use(op_milp_info_callback(env, y, x, nodesAllClusters, this, lastObjVal, lastTime));
	cplex.use(op_milp_lazy_constraints_callback(env, y, x, nodesAllClusters, workerObj, numAddedConstraints, distances));
}

void IlpSopDefinition::adjustPriorities(IloCplex & cplex) {
	int x_ids = 1;
	int y_ids = 2;
	for ( int id1 = 0 ; id1 < x.getSize() ; id1++ ) {
		for ( int id2 = 0 ; id2 < x[id1].getSize() ; id2++ ) {
			for ( int id3 = 0 ; id3 < x[id1][id2].getSize() ; id3++ ) {

				cplex.setPriority(x[id1][id2][id3], x_ids);

			}
		}
	}

	for ( int id1 = 0 ; id1 < y.getSize() ; id1++ ) {
		cplex.setPriority(y[id1], y_ids);
	}
}

void IlpSopDefinition::setInitialSolution(IloCplex & cplex, std::vector<IndexSOP> initialSolution) {
	INFO("setInitialSolution");
	IloEnv envUsed = cplex.getEnv();
	IloNumVarArray PstartVar(envUsed);
	IloNumArray PstartVal(envUsed);

	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
		for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {
			//IloNumArray vals_lok_x(envUsed, this->x[fromCluster][toCluster - 1].getSize());
			int num_to_cluster_nodes = nodesAllClusters[toCluster].size();
			//INFO("nodesAllClusters[fromCluster].size() "<<nodesAllClusters[fromCluster].size());
			//INFO("nodesAllClusters[toCluster].size() "<<nodesAllClusters[toCluster].size());

			bool fromtoUsed = false;
			int fromClusterNodeInPath = -1;
			int toClusterNodeInPath = -1;
			for ( int initSolVar = 1 ; initSolVar < initialSolution.size() ; ++initSolVar ) {
				if ( initialSolution[initSolVar - 1].clusterIndex == fromCluster
						&& initialSolution[initSolVar].clusterIndex == toCluster ) {
					//path between this vertices
					fromClusterNodeInPath = initialSolution[initSolVar - 1].nodeIndex;
					toClusterNodeInPath = initialSolution[initSolVar].nodeIndex;
					//INFO("fill initial solution from "<<fromCluster<<":"<<fromClusterNodeInPath<<" to "<<toCluster<<":"<<toClusterNodeInPath);
					fromtoUsed = true;
					break;
				}
			}

			for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ; ++fromClusterNode ) {
				for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ; ++toClusterNode ) {
					//vals_lok_x[FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)] = 0;
					PstartVar.add(
							x[fromCluster][toCluster - 1][FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)]);

					if ( fromtoUsed && fromClusterNodeInPath == fromClusterNode
							&& toClusterNodeInPath == toClusterNode ) {
						PstartVal.add(1);
						//INFO("set "<<PstartVar[PstartVar.getSize()-1]<<" to one");
					} else {
						PstartVal.add(0);
					}
				}
			}

		}
	}

	double initial_solution_reward = 0;
//IloNumArray vals_lok_y(envUsed, this->y.getSize());
	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() ; ++fromCluster ) {
		bool isUsed = false;
		for ( int initSolVar = 0 ; initSolVar < initialSolution.size() ; ++initSolVar ) {
			if ( initialSolution[initSolVar].clusterIndex == fromCluster ) {
				isUsed = true;
				break;
			}
		}
		PstartVar.add(y[fromCluster]);
		if ( isUsed ) {
			PstartVal.add(1);
			initial_solution_reward += nodesAllClusters[fromCluster][0].reward;
			//INFO("set "<<PstartVar[PstartVar.getSize()-1]<<" to one");
		} else {
			PstartVal.add(0);
		}
	}

	IloInt mip_start_index = cplex.addMIPStart(PstartVar, PstartVal, IloCplex::MIPStartSolveMIP);
	INFO("initial_solution_reward "<<initial_solution_reward);
	INFO("setting heuristic callback to be sure to set initial solution when addMIPStart ignored");
	this->initial_solution_set = false;
	cplex.use(heuristic_callback(envUsed, y, x, nodesAllClusters, initialSolution, initial_solution_reward, &initial_solution_set));
	//cplex.refineMIPStartConflict(mip_start_index,cplex.getc);
	INFO("setInitialSolution done");
}

int IlpSopDefinition::getLastImprovementIter() {
	return this->lastImprovementIter;
}

void IlpSopDefinition::setLastImprovementIter(int lastImprovementIter_) {
	this->lastImprovementIter = lastImprovementIter_;
}

long IlpSopDefinition::getLastImprovementTime() {
	return this->lastImprovementTime;
}

void IlpSopDefinition::setLastImprovementTime(long lastImprovementTime_) {
	this->lastImprovementTime = lastImprovementTime_;
}

double IlpSopDefinition::getSolutionLength(IloCplex & cplex) {
	return cplex.getValue(constraint_t_max_expr);
}

std::vector<ImprovementLogRecord> IlpSopDefinition::getImprovementLog() {
	return this->improvementLog;
}
void IlpSopDefinition::setImprovementLog(std::vector<ImprovementLogRecord> improvementLog_) {
	this->improvementLog = improvementLog_;
}

std::vector<IndexSOP> IlpSopDefinition::parseSolutionVector(IloCplex & cplex) {
	INFO("parsing solution vector");

	std::vector<int> usedVertices;
	IloNumArray vals_lok_y(env);
	cplex.getValues(vals_lok_y, y);
	for ( int var = 0 ; var < nodesAllClusters.size() ; ++var ) {
		int vals_lok_int_value_y = round(vals_lok_y[var]);
		if ( vals_lok_int_value_y > 0 ) {
			usedVertices.push_back(var);
			INFO("y_"<<var<<"="<<vals_lok_y[var]);
		}

	}

	std::list<IndexPairSOP> solutionListUnordered;
//std::vector
	for ( int fromCluster = 0 ; fromCluster < nodesAllClusters.size() - 1 ; ++fromCluster ) {
		for ( int toCluster = 1 ; toCluster < nodesAllClusters.size() ; ++toCluster ) {
			IloNumArray vals_lok_x(env);
			cplex.getValues(vals_lok_x, x[fromCluster][toCluster - 1]);
			int num_to_cluster_nodes = nodesAllClusters[toCluster].size();
			for ( int fromClusterNode = 0 ; fromClusterNode < nodesAllClusters[fromCluster].size() ; ++fromClusterNode ) {
//	IloNumArray vals_lok_x(env);
//	cplex.getValues(vals_lok_x, x[fromCluster][toCluster - 1][fromClusterNode]);
				for ( int toClusterNode = 0 ; toClusterNode < nodesAllClusters[toCluster].size() ; ++toClusterNode ) {
					int vals_lok_int_value_x = round(
							vals_lok_x[FLAT_X(fromClusterNode, toClusterNode, num_to_cluster_nodes)]);
					if ( vals_lok_int_value_x > 0 ) {
						//INFO(VARIABLE_AUX_FROM_TO(fromCluster,toCluster)<<"="<<vals_lok_int_value_y);
						INFO(
								VARIABLE_FROM_TO(fromCluster,fromClusterNode,toCluster,toClusterNode)<<"="<<vals_lok_int_value_x);
						IndexPairSOP indexPair;
						indexPair.cluster_from = fromCluster;
						indexPair.cluster_from_node = fromClusterNode;
						indexPair.cluster_to = toCluster;
						indexPair.cluster_to_node = toClusterNode;
						solutionListUnordered.push_back(indexPair);
					}
				}
			}
		}
	}
	INFO("solution vector contains "<<solutionListUnordered.size()<<" edges");
	if ( solutionListUnordered.size() != usedVertices.size() - 1 ) {
		ERROR("bad size of edges");
	}

	std::vector<IndexSOP> solution;
	int last_to_cluster_index = startIndex;
	int last_to_index = 0;
	for ( std::list<IndexPairSOP>::iterator iterator = solutionListUnordered.begin(), end =
			solutionListUnordered.end() ; iterator != end ; ++iterator ) {
		if ( (*iterator).cluster_from == startIndex ) {
			//solution.push_back(nodesAllClusters[(*iterator).cluster_from][(*iterator).cluster_from_node]);
			IndexSOP newInPathId;
			newInPathId.clusterIndex = (*iterator).cluster_from;
			newInPathId.nodeIndex = (*iterator).cluster_from_node;
			solution.push_back(newInPathId);
			break;
		}
	}

	while ( last_to_cluster_index != goalIndex && solutionListUnordered.size() > 0 ) {
		for ( std::list<IndexPairSOP>::iterator iterator = solutionListUnordered.begin(), end =
				solutionListUnordered.end() ; iterator != end ; ++iterator ) {
			if ( (*iterator).cluster_from == last_to_cluster_index ) {
				last_to_index = (*iterator).cluster_to_node;
				last_to_cluster_index = (*iterator).cluster_to;
				//solution.push_back(nodesAllClusters[(*iterator).cluster_to][(*iterator).cluster_to_node]);
				IndexSOP newInPathId;
				newInPathId.clusterIndex = (*iterator).cluster_to;
				newInPathId.nodeIndex = (*iterator).cluster_to_node;
				solution.push_back(newInPathId);
				solutionListUnordered.erase(iterator);
				break;
			}
		}
	}

	INFO("solution contains "<<solution.size()<<" vertices");
	return solution;
}

