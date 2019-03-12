/*
 * VNSDOP.cpp
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#include "ilp_sop_solver.h"
#include <iomanip>
#include <ilcplex/ilocplex.h>


#define MAX_FLOAT (std::numeric_limits<double>::max())

using crl::logger;

using namespace op;
using namespace crl;
using namespace crl::gui;

#define foreach BOOST_FOREACH
#define MIN_CHANGE_EPS 0.00001
#define DD " , "
#define DEBUG_DOP_TRY_OPERATIONS false

#define NUMBER_PRECISION 9
#define VARIABLE_FROM_TO(fromCluster,fromClusterNode,toCluster,toClusterNode)  "x_"<<fromCluster<<"_"<<fromClusterNode<<"__"<<toCluster<<"_"<<toClusterNode
#define VARIABLE_AUX_FROM_TO(fromCluster,toCluster)  "y_"<<fromCluster<<"__"<<toCluster
#define INDEX_1D( from , to , width ) ( width * from + to )
#define INDEX_2D_TO( id , width ) ( id % width )
#define INDEX_2D_FROM( id , width ) ( (id - ( id  % width )) / width )

bool debug_ilp_formulas = false;

IlpSopSolver::IlpSopSolver(crl::CConfig& config, const std::string& problemFile) :
		Base(config), SAVE_RESULTS(config.get<bool>("save-results")), SAVE_SETTINGS(config.get<bool>("save-settings")), BORDER(
				config.get<double>("canvas-border")), SAVE_INFO(config.get<bool>("save-info")), SAVE_TARGETS(
				config.get<bool>("save-targets")), SAVE_SAMPLED_PATH(config.get<bool>("save-sampled-path")) {

	SOP_Prolem problem = SOPLoader::getSOPDefinition(config, problemFile, true);
	this->name = config.get<std::string>("name");
	if (this->name.compare("no_name") == 0) {
		INFO("setting name to name from dataset "<<problem.name);
		this->name = problem.name;
	}

	this->gopType = problem.gop_type;
	this->budget = problem.budget;
	this->startIndex = problem.startIndex;
	this->goalIndex = problem.goalIndex;
	this->nodesAllClusters = problem.samples;
	this->allDistances = problem.distances;
	this->oldgoalIndex = problem.oldGoalIndex;
	this->oldstartIndex = problem.oldStartIndex;
	this->deleted_cluster_id = problem.deleted_cluster_id;
	this->ids_originally_from_one = problem.ids_originally_from_one;

	maximalRewardAll = 0;
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		maximalRewardAll += nodesAllClusters[var][0].reward;
	}

	INFO_VAR(startIndex)
	INFO_VAR(oldstartIndex)
	INFO_VAR(goalIndex)
	INFO_VAR(oldgoalIndex)

	this->draw_cluster_points = config.get<bool>("draw-cluster-points");
	this->num_threads = config.get<int>("num-threads");
	this->maximal_memory_MB = config.get<int>("maximal-memory-MB");
	this->maximal_calculation_time_sec = config.get<int>("maximal-calculation-time-sec");
	INFO("using budget:"<<budget);
	INFO("using num_threads:"<<num_threads);
	INFO("using maximal_memory_MB:"<<maximal_memory_MB);
	INFO("using maximal_calculation_time_sec:"<<maximal_calculation_time_sec);

	this->stop = false;
	this->online_subtour_elimination = config.get<bool>("lp-online-subtour-elimination");
	this->adjust_milp_priorities = config.get<bool>("lp-adjust-milp-priorities");
	this->set_greedy_initial_solution = config.get<bool>("initial-greedy-solution");

}

IlpSopSolver::~IlpSopSolver() {

}

void IlpSopSolver::solve() {
	Base::solve();
}

void IlpSopSolver::my_problem_definition(IloEnv & env, IloModel & model) {

}

void IlpSopSolver::iterate(int iter) {
	int numNodesTotally = 0;
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		numNodesTotally += nodesAllClusters[var].size();
	}

	Timer testTouring;
	testTouring.resetRealClock();

	double objective_value = 0;
	double lp_solution_length = 0;
	int act_iter = 0;
	int numItersLastImprovement = 0;
	long timeLastImprovement = 0;
	int optimal_solution = 0;
	double gap_perc = -1;
	long comp_time[3] = { 0, 0, 0 };
	std::vector<ImprovementLogRecord> improvementLog;

	IloEnv env;
	try {
		IloModel model(env);

		INFO("dataset with " << nodesAllClusters.size() << " clusters");
		INFO("dataset with total number of nodes "<<numNodesTotally);
		INFO("startID " << startIndex);
		INFO("goalID " <<goalIndex);
		//int num_nodes = 10;

		op::IlpSopDefinition * my_problem_definition;
		//in preparation for alternative definitions
		my_problem_definition = new op::IlpSopDefinition(env, model, nodesAllClusters, allDistances, startIndex,
				goalIndex, budget, this);

		IloCplex cplex(model);
		//cplex.getPr
		INFO("cplex created");

		if (SAVE_SAMPLED_PATH) {
			std::string dir;
			std::string file = getOutputIterPath(config.get<std::string>("lp-model-file"), dir);
			INFO("saving file to "<<file.c_str());
			crl::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
			cplex.exportModel(file.c_str());
		}

		INFO("set num threads to "<<num_threads);
		cplex.setParam(IloCplex::Param::Threads, num_threads);

		if (set_greedy_initial_solution) {
			INFO("try to find greedy solution");
			std::vector<IndexSOP> greedySolution = getGreedySolution();
			my_problem_definition->setInitialSolution(cplex, greedySolution);
			drawPath(0, &greedySolution);
			INFO("gready solution created");
			timeLastImprovement = testTouring.getRTimeMS();
		}
		if (adjust_milp_priorities) {
			INFO("changing milp priorities");
			my_problem_definition->adjustPriorities(cplex);
		}
		my_problem_definition->registerCallback(cplex);

		//exit(1);

		// Turn off CPLEX logging
		//cplex.setParam(IloCplex::Param::MIP::Display, 0);

		int automatic = IloCplex::MIPsearch::AutoSearch;
		int branch_and_cut = IloCplex::MIPsearch::Traditional;
		cplex.setParam(IloCplex::MIPSearch, automatic);

		int node_file_on_disk = 3;		//Node file on disk and compressed
		cplex.setParam(IloCplex::MemoryEmphasis, true);	//Directs CPLEX that it should conserve memory where possible.
		cplex.setParam(IloCplex::WorkMem, maximal_memory_MB); //maximal working memory
		cplex.setParam(IloCplex::NodeFileInd, node_file_on_disk);
		cplex.setParam(IloCplex::TiLim, maximal_calculation_time_sec);

		if (!cplex.solve()) {
			ERROR("Failed to optimize LP");
			throw(-1);
		}

		lp_solution_length = my_problem_definition->getSolutionLength(cplex);
		numItersLastImprovement = my_problem_definition->getLastImprovementIter();
		timeLastImprovement = my_problem_definition->getLastImprovementTime();
		improvementLog = my_problem_definition->getImprovementLog();
		objective_value = cplex.getObjValue();
		act_iter = cplex.getNiterations();
		if (cplex.getStatus() == IloAlgorithm::Status::Optimal) {
			optimal_solution = 1;
			gap_perc = 0;
		} else {
			gap_perc = cplex.getMIPRelativeGap() * 100.0;
		}
		INFO("Solution status = " << cplex.getStatus());
		INFO_VAR(optimal_solution);
		INFO("Gap value  = " << cplex.getMIPRelativeGap());
		INFO("Solution value  = " << cplex.getObjValue());
		INFO("Maximum bound violation = " << cplex.getQuality(IloCplex::Quality::DualObj));

		INFO("solution is:");

		//change
		std::stringstream solutionIDs;
		finalTourOP = my_problem_definition->parseSolutionVector(cplex);

		for (int var = 0; var < finalTourOP.size(); ++var) {
			solutionIDs << finalTourOP[var].clusterIndex << " ";
		}
		INFO("ids: "<<solutionIDs.str());

	} catch (IloAlgorithm::CannotExtractException& e) {

		ERROR("CannoExtractException: " << e);
		IloExtractableArray failed = e.getExtractables();

		for (IloInt i = 0; i < failed.getSize(); ++i) {
			ERROR("\t" << failed[i]);
			// Handle exception ...
		}
	} catch (IloException& e) {
		ERROR("Concert exception caught: " << e);
	} catch (const std::exception &e) {
		ERROR("Error " << e.what());
	} catch (const std::string &e) {
		ERROR("Error " << e);
	} catch (...) {
		ERROR("Unknown exception caught");
	}

	tSolve.stop();
	tSolve.addTime(comp_time);
	INFO("computational time ms = " << comp_time[0]);

	env.end();

	drawPath();
	double final_length = 0;
	double final_reward = 0;

	for (int var = 0; var < finalTourOP.size(); ++var) {
		if (var != 0) {
			final_length +=
					allDistances[finalTourOP[var - 1].clusterIndex][finalTourOP[var].clusterIndex][finalTourOP[var - 1].nodeIndex][finalTourOP[var].nodeIndex];
		}
		final_reward += nodesAllClusters[finalTourOP[var].clusterIndex][0].reward;
	}

	if (fabs(objective_value - final_reward) > 0.001) {
		ERROR(
				"the lp objective value ( "<<objective_value<<" ) and the calculated reward "<<final_reward<<" are different");
		optimal_solution = 0;
		exit(1);
	}

	if (fabs(lp_solution_length - final_length) > 0.001) {
		ERROR(
				"the lp solution length ( "<<lp_solution_length<<" ) and the calculated length "<<final_length<<" are different");
		optimal_solution = 0;
		exit(1);
	}

	INFO_CYAN("TODO uncomment change ids");

	fillResultRecord(act_iter, final_length, final_reward, optimal_solution, gap_perc, numItersLastImprovement,
			timeLastImprovement, improvementLog);
	INFO("write result log");
	resultLog << crl::result::endrec;
	INFO("set finalTourDOP");
	INFO_GREEN(
			"found tour with reward "<<final_reward<<" and length "<<final_length<<" out of "<<budget<<" budget within " <<comp_time[0] <<" ms");
}

std::vector<IndexSOP> IlpSopSolver::getGreedySolution() {

	INFO("getGreedySolution begin");
	int waitTimeus = 10000;
	bool somethingAdded = true;
	std::vector<IndexSOP> initial_solution;

	INFO("startIndex "<<startIndex);
	INFO("goalIndex "<<goalIndex);

	double total_reward = 0;

	std::list<int> testing_cluster_ids;
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		if (var != startIndex && var != goalIndex) {
			testing_cluster_ids.push_back(var);
		}
	}

	double minStartGoal = MAX_FLOAT;

	int shortestStartNodeID = -1;
	int shortestGoalNodeID = -1;
	for (int startNodeID = 0; startNodeID < nodesAllClusters[startIndex].size(); ++startNodeID) {
		for (int goalNodeID = 0; goalNodeID < nodesAllClusters[goalIndex].size(); ++goalNodeID) {
			double testing_dist = allDistances[startIndex][goalIndex][startNodeID][goalNodeID];
			//INFO("start goal distance "<<testing_dist<<" ids "<<startNodeID<<" "<<goalNodeID);

			if (testing_dist < minStartGoal) {
				minStartGoal = testing_dist;
				shortestStartNodeID = startNodeID;
				shortestGoalNodeID = goalNodeID;
			}
		}
	}

	initial_solution.push_back(IndexSOP(startIndex, shortestStartNodeID));
	initial_solution.push_back(IndexSOP(goalIndex, shortestGoalNodeID));

	total_reward += nodesAllClusters[startIndex][shortestStartNodeID].reward;
	total_reward += nodesAllClusters[goalIndex][shortestGoalNodeID].reward;

	double actualLength = minStartGoal;
	while (somethingAdded) {
		//INFO("inserting");

		somethingAdded = false;
		double minimalAddDistPerReward = MAX_FLOAT;
		double newLengthAfterAdd = actualLength;
		IndexSOP nodeToAdd(-1, -1);
		int idTourNodeBeforMinimal = -1;

		for (std::list<int>::iterator iterator = testing_cluster_ids.begin(), end = testing_cluster_ids.end();
				iterator != end; ++iterator) {
			//try to add between start ang goal

			int testing_cluster_to_add = *iterator;
			//INFO("testing_cluster_to_add "<<testing_cluster_to_add);
			for (int idTourNode = 1; idTourNode < initial_solution.size(); ++idTourNode) {
				//try to add between (idTourNode-1) and idTourNode
				IndexSOP nodeBef = initial_solution[idTourNode - 1];
				IndexSOP nodeAfter = initial_solution[idTourNode];

				for (int testing_cluster_node_to_add = 0;
						testing_cluster_node_to_add < nodesAllClusters[testing_cluster_to_add].size();
						++testing_cluster_node_to_add) {
					double additionalDistance =
							allDistances[nodeBef.clusterIndex][testing_cluster_to_add][nodeBef.nodeIndex][testing_cluster_node_to_add]
									+ allDistances[testing_cluster_to_add][nodeAfter.clusterIndex][testing_cluster_node_to_add][nodeAfter.nodeIndex]
									- allDistances[nodeBef.clusterIndex][nodeAfter.clusterIndex][nodeBef.nodeIndex][nodeAfter.nodeIndex];

					double newDistance = actualLength + additionalDistance;
					double additionalDistPerReward = additionalDistance
							/ nodesAllClusters[testing_cluster_to_add][testing_cluster_node_to_add].reward;
					//INFO("distance "<<newDistance);
					if (additionalDistPerReward < minimalAddDistPerReward && newDistance <= budget) {
						//INFO("additionalDistPerReward bettew "<<additionalDistPerReward);
						minimalAddDistPerReward = additionalDistPerReward;
						newLengthAfterAdd = newDistance;
						nodeToAdd.clusterIndex = testing_cluster_to_add;
						nodeToAdd.nodeIndex = testing_cluster_node_to_add;
						idTourNodeBeforMinimal = idTourNode;
					}
				}
			}

		}

		if (nodeToAdd.clusterIndex >= 0 && nodeToAdd.nodeIndex >= 0 && idTourNodeBeforMinimal >= 0
				&& newLengthAfterAdd <= budget) {

			total_reward += nodesAllClusters[nodeToAdd.clusterIndex][nodeToAdd.nodeIndex].reward;
			somethingAdded = true;
			initial_solution.insert(initial_solution.begin() + idTourNodeBeforMinimal, nodeToAdd);
			actualLength = newLengthAfterAdd;
			testing_cluster_ids.remove(nodeToAdd.clusterIndex);

			drawPath(waitTimeus, &initial_solution);
		}

	}

	double solution_before = getTourLength(initial_solution);
	INFO_VAR(solution_before);

	bool something_changed = true;
	while (something_changed) {
		something_changed = false;
		for (int var = 1; var < initial_solution.size() - 1; ++var) {
			int ci1 = initial_solution[var - 1].clusterIndex;
			int ni1 = initial_solution[var - 1].nodeIndex;
			int ci2 = initial_solution[var].clusterIndex;
			int ci3 = initial_solution[var + 1].clusterIndex;
			int ni3 = initial_solution[var + 1].nodeIndex;

			double minDist = MAX_FLOAT;
			int minNodeIdx = 0;
			for (int nodeInClId = 0; nodeInClId < nodesAllClusters[ci2].size(); ++nodeInClId) {
				double dist = allDistances[ci1][ci2][ni1][nodeInClId] + allDistances[ci2][ci3][nodeInClId][ni3];
				if (dist < minDist) {
					minDist = dist;
					minNodeIdx = nodeInClId;
				}
			}

			if (initial_solution[var].nodeIndex != minNodeIdx) {
				INFO("change minimal index");
				initial_solution[var].nodeIndex = minNodeIdx;
			}
		}
		INFO("try again");
	}
	actualLength = getTourLength(initial_solution);
	INFO_VAR(actualLength);
	INFO("greedy solution with "<<total_reward<<" reward and "<<actualLength<<" length");
	INFO("getGreedySolution end");
	return initial_solution;

}

double IlpSopSolver::getTourLength(std::vector<IndexSOP> indexedSolution) {
	double distance = 0;
	for (int var = 1; var < indexedSolution.size(); ++var) {
		int ci1 = indexedSolution[var - 1].clusterIndex;
		int ni1 = indexedSolution[var - 1].nodeIndex;
		int ci2 = indexedSolution[var].clusterIndex;
		int ni2 = indexedSolution[var].nodeIndex;
		distance += allDistances[ci1][ci2][ni1][ni2];
	}
	return distance;
}

double IlpSopSolver::getTourReward(std::vector<GraphNode> tour) {
	double price = 0;
	for (int var = 0; var < tour.size(); ++var) {
		price += tour[var].reward;
	}
	return price;
}

std::string IlpSopSolver::getVersion(void) {
	return "LP_OP v. 1.0";
}

void IlpSopSolver::load(void) {
	// nothing to load, structures are passed to the constructor
	INFO("load");
	int n = nodesAllClusters.size();
	if (canvas) {
		CoordsVector points;
		//INFO("BORDER " << BORDER);
		for (int clusterid = 0; clusterid < nodesAllClusters.size(); ++clusterid) {
			for (int clusternodeid = 0; clusternodeid < nodesAllClusters[clusterid].size(); ++clusternodeid) {
				GraphNode station = nodesAllClusters[clusterid][clusternodeid];
				Coords coord_up(station.x + BORDER, station.y + BORDER);
				Coords coord_down(station.x - BORDER, station.y - BORDER);
				points.push_back(coord_up);
				points.push_back(coord_down);
			}
		}

		*canvas << canvas::AREA;
		//INFO("draw points");
		foreach(Coords coords, points){
		//INFO(coords.x<<" "<<coords.y);
		*canvas << coords;
	}
	//INFO("set to canvas");
		*canvas << canvas::END;

		if (config.get<bool>("draw-stations")) {
			std::string pallete = config.get<std::string>("draw-targets-reward-palette");
			if (config.get<bool>("draw-targets-reward") and crl::isFile(pallete)) {
				crl::gui::CColorMap map;
				map.load(pallete);
				double minReward = DBL_MAX;
				double maxReward = -DBL_MAX;
				for (int var = 0; var < nodesAllClusters.size(); ++var) {
					if (nodesAllClusters[var][0].reward < minReward) {
						minReward = nodesAllClusters[var][0].reward;
					}
					if (nodesAllClusters[var][0].reward > maxReward) {
						maxReward = nodesAllClusters[var][0].reward;
					}
				}
				if (minReward == maxReward) {
					minReward = 0.99 * maxReward;
				}
				map.setRange(minReward, maxReward);
				*canvas << "targets" << CShape(config.get<std::string>("draw-shape-targets")) << canvas::POINT;
				for (int clusterid = 0; clusterid < nodesAllClusters.size(); ++clusterid) {
					for (int clusternodeid = 0; clusternodeid < nodesAllClusters[clusterid].size(); ++clusternodeid) {
						GraphNode station = nodesAllClusters[clusterid][clusternodeid];
						//DEBUG("reward: " << target->reward << " color: " << map.getColor((double)target->reward));
						Coords cord(station.x, station.y);
						*canvas << crl::gui::canvas::FILL_COLOR << map.getColor((double) station.reward) << cord;
					}
				}

				//draw convex hull around cluster poinst
				for (int clusterid = 0; clusterid < nodesAllClusters.size(); ++clusterid) {
					PointConvexHull points_in_cluster[nodesAllClusters[clusterid].size()];
					for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterid].size(); ++clusterNodeID) {
						points_in_cluster[clusterNodeID] = PointConvexHull(nodesAllClusters[clusterid][clusterNodeID].x,
								nodesAllClusters[clusterid][clusterNodeID].y);
					}
					vector < PointConvexHull > hull = convex_hull::get_convex_hull(points_in_cluster,
							nodesAllClusters[clusterid].size());

					std::stringstream ss;
					ss << "convexhulls" << clusterid;
					CShape colorhull;
					colorhull.setPenColor(map.getColor((double) nodesAllClusters[clusterid][0].reward));
					*canvas << ss.str().c_str() << colorhull << canvas::LINESTRING;

					if (hull.size() > 0) {
						for (int hullidx = 0; hullidx < hull.size(); ++hullidx) {
							Coords cord(hull[hullidx].x, hull[hullidx].y);
							*canvas << cord;
						}
						Coords cord(hull[0].x, hull[0].y);
						*canvas << cord;
					} else if (nodesAllClusters[clusterid].size() > 0) {
						for (int clusternodeid = 0; clusternodeid < nodesAllClusters[clusterid].size();
								++clusternodeid) {
							Coords cord(nodesAllClusters[clusterid][clusternodeid].x,
									nodesAllClusters[clusterid][clusternodeid].y);
							*canvas << cord;
						}
						Coords cord(nodesAllClusters[clusterid][0].x, nodesAllClusters[clusterid][0].y);
						*canvas << cord;
					}
				}

			} else {
				//INFO("draw stations");
				*canvas << "stations" << CShape(config.get<std::string>("draw-shape-stations")) << canvas::POINT;
				for (int clusterid = 0; clusterid < nodesAllClusters.size(); ++clusterid) {
					for (int clusternodeid = 0; clusternodeid < nodesAllClusters[clusterid].size(); ++clusternodeid) {
						GraphNode station = nodesAllClusters[clusterid][clusternodeid];
						Coords cord(station.x, station.y);
						//INFO(cord.x<<" ; "<<cord.y);

						*canvas << cord << canvas::END;
					}
				}
			}
		}

		canvas->redraw();
		//usleep(500000);
	} //end canvas
}

void IlpSopSolver::drawNeighborhoodPoints() {
	if (canvas && draw_cluster_points) {
		//INFO("drawNeighborhoodPoints");
		usleep(2000);
		*canvas << canvas::CLEAR << "neighborhoodp" << "neighborhoodp" << canvas::POINT;
		CShape neighborhoodPoint("blue", "blue", 1, 1);
		for (int nodeID = 0; nodeID < nodesAllClusters.size(); ++nodeID) {
			//INFO_VAR(nodeID);

			const int neighborhoodSize = nodesAllClusters[nodeID].size();
			//INFO_VAR(neighborhoodSize);
			for (int neighID = 0; neighID < neighborhoodSize; ++neighID) {
				//INFO(neighID<<" draw neighborhood pos "<<neighborhood[neighID][0].node.x<<" "<<neighborhood[neighID][0].node.y);
				Coords cord(nodesAllClusters[nodeID][neighID].x, nodesAllClusters[nodeID][neighID].y);
				*canvas << neighborhoodPoint << cord;
			}
		}
		*canvas << canvas::END;
		canvas->redraw();
		usleep(2000);
		//INFO("drawNeighborhoodPoints end");
	}
}

/// - private method -----------------------------------------------------------
void IlpSopSolver::drawPath(int usleepTime, std::vector<IndexSOP> * toShow) {
	//INFO("drawPath begin");

	if (canvas) {
		*canvas << canvas::CLEAR << "path" << "path" << canvas::LINESTRING;


		std::vector<IndexSOP> pathToDraw = finalTourOP;

		if (toShow != NULL) {
			pathToDraw = *toShow;
		}

		//INFO("pathToDraw size "<<pathToDraw.size());
		CoordsVector coords = SOPLoader::getCoordPath(pathToDraw, nodesAllClusters, gopType, config);
		CShape blackPoint("black", "black", 1, 4);
		CShape blackLine("black", "black", 2, 0);
		for (int var = 1; var < coords.size(); ++var) {
			*canvas << canvas::LINE << blackLine;
			*canvas << coords[var - 1].x << coords[var - 1].y;
			*canvas << coords[var].x << coords[var].y;
			*canvas << canvas::END;
		}

		*canvas << canvas::CLEAR << "pathpoint" << "pathpoint" << canvas::POINT;
		for (int var = 0; var < pathToDraw.size(); ++var) {
			GraphNode gn = nodesAllClusters[pathToDraw[var].clusterIndex][pathToDraw[var].nodeIndex];
			Coords coord(gn.x, gn.y);
			*canvas << blackPoint << coord;
		}

		*canvas << canvas::END;
		canvas->redraw();
	}
	//INFO("drawPath end");
}

void IlpSopSolver::initialize(void) {

}

std::string IlpSopSolver::getRevision(void) {
	return "$Id$";
}

void IlpSopSolver::after_init(void) {

}

/// - protected method ---------------------------------------------------------
void IlpSopSolver::save(void) {
	std::string dir;
	//updateResultRecordTimes(); //update timers as load and initilization is outside class
	//DEBUG("LOAD_TIME_CPU: " << tLoad.cpuTime());
	//DEBUG("INIT_TIME_CPU: " << tInit.cpuTime());
	//DEBUG("SAVE_TIME_CPU: " << tSave.cpuTime());
	//DEBUG("SOLVE_TIME_CPU: " << tSolve.cpuTime());
	if (SAVE_SETTINGS) {
		saveSettings(getOutputIterPath(config.get<std::string>("settings"), dir));
	}
	if (SAVE_INFO) {
		saveInfo(getOutputIterPath(config.get<std::string>("info"), dir));
	}
	if (SAVE_RESULTS) {
		std::string file = getOutputIterPath(config.get<std::string>("result-path"), dir);
		crl::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream ofs(file.c_str());
		crl::assert_io(ofs.good(), "Cannot create path '" + file + "'");
		ofs << std::setprecision(14);

		std::vector<IndexSOP> finalPath = finalTourOP;
		for (int var = 0; var < finalPath.size(); ++var) {
			ofs << finalPath[var].clusterIndex << " " << finalPath[var].nodeIndex << std::endl;

		}

		crl::assert_io(ofs.good(), "Error occur during path saving");
		ofs.close();
	}


	if (SAVE_SAMPLED_PATH) {
		std::string file = getOutputIterPath(config.get<std::string>("sampled-path"), dir);
		crl::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream ofs(file.c_str());
		crl::assert_io(ofs.good(), "Cannot create path '" + file + "'");
		ofs << std::setprecision(14);

		std::string file2 = getOutputIterPath("dop.txt", dir);
		crl::assert_io(createDirectory(dir), "Can not create file in path'" + file2 + "'");
		std::ofstream ofs2(file2.c_str());
		crl::assert_io(ofs2.good(), "Cannot create path '" + file2 + "'");
		ofs2 << std::setprecision(14);

		crl::assert_io(ofs.good(), "Error occur during path saving");
		ofs.close();
	}
	if (SAVE_TARGETS) {
		std::string file = getOutputIterPath(config.get<std::string>("targets"), dir);
		crl::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream ofs(file.c_str());
		crl::assert_io(ofs.good(), "Cannot create path '" + file + "'");
		ofs << std::setprecision(14);
		for (int clusterid = 0; clusterid < nodesAllClusters.size(); ++clusterid) {
			for (int clusternodeid = 0; clusternodeid < nodesAllClusters[clusterid].size(); ++clusternodeid) {
				GraphNode station = nodesAllClusters[clusterid][clusternodeid];
				ofs << station.id << DD << station.x << DD << station.y << DD << station.reward << DD
						<< station.cluster_id << std::endl;
			}
		}
		crl::assert_io(ofs.good(), "Error occur during path saving");
		ofs.close();
	}
	if (canvas) { // map must be set
		*canvas << canvas::CLEAR << "ring";
		if (config.get<bool>("draw-path")) {
			drawPath();
		}
		saveCanvas();
	}
}

void IlpSopSolver::release(void) {

}

void IlpSopSolver::visualize(void) {

}

void IlpSopSolver::defineResultLog(void) {
	static bool resultLogInitialized = false;
	if (!resultLogInitialized) {
		resultLog << result::newcol << "NAME";
		resultLog << result::newcol << "METHOD";
		resultLog << result::newcol << "CTIME";
		resultLog << result::newcol << "NUM_ITERS";
		resultLog << result::newcol << "BUDGET";
		resultLog << result::newcol << "REWARDS";
		resultLog << result::newcol << "IS_OPTIMAL";
		resultLog << result::newcol << "GAP_PERCENT";
		resultLog << result::newcol << "MAX_ACHIEVABLE_REWARDS";
		resultLog << result::newcol << "LENGTH";
		resultLog << result::newcol << "NUM_ITERS_LAST_IMPR";
		resultLog << result::newcol << "CTIME_LAST_IMPR";
		resultLog << result::newcol << "MAX_ALLOWED_CALC_TIME_MS";
		resultLog << result::newcol << "GREEDY_INITIAL_SOLUTION";
		resultLog << result::newcol << "RESULT_TARGET_IDS";
		resultLog << result::newcol << "RESULT_CLUSTER_IDS";
		resultLog << result::newcol << "SOLUTION_TIME_REWARD_LENGTH_IMPROVEMENTS";
		resultLog << result::newcol << "LOWER_BOUND_ABOVE_BUDGET_THEN_SKIP";
		resultLogInitialized = true;
	}
}

void IlpSopSolver::fillResultRecord(int numIters, double length, double reward, int optimal_solution, double gap_prec,
		int numItersLastImprovement, long timeLastImprovement, std::vector<ImprovementLogRecord> improvementLog) {
	long t[3] = { 0, 0, 0 };

	tSolve.addTime(t);

	double final_reward = reward;

	std::vector<IndexSOP> finalTourOP_copy = finalTourOP;
	std::vector<GraphNode> solution;
	for (int var = 0; var < finalTourOP.size(); ++var) {
		int cid = finalTourOP[var].clusterIndex;
		int nid = finalTourOP[var].nodeIndex;
		solution.push_back(nodesAllClusters[cid][nid]);
	}

	SOPLoader::fixClusterNodeRenumbering(solution, startIndex, goalIndex, oldstartIndex, oldgoalIndex,
			deleted_cluster_id, ids_originally_from_one);

	std::stringstream tourNodes;
	std::stringstream tourClusters;
	std::stringstream tourRewards;
	for (int var = 0; var < solution.size(); ++var) {
		if (var != 0) {
			tourNodes << ",";
			tourClusters << ",";
			tourRewards << ",";
		}

		tourNodes << solution[var].id;
		tourClusters << solution[var].cluster_id;
		tourRewards << solution[var].reward;
	}
	INFO("fillResultRecord:")
	INFO("tour cluster IDs:   "<<tourClusters.str());
	INFO("tour node IDs:      "<<tourNodes.str());
	INFO("tour cluster rews:  "<<tourRewards.str());

	std::stringstream tourImprovements;
	for (int var = 0; var < improvementLog.size(); ++var) {
		if (var != 0) {
			tourImprovements << ",";
		}
		tourImprovements << improvementLog[var].timeMS << "|" << improvementLog[var].reward << "|"
				<< improvementLog[var].length;
	}

	double final_length = length;

	resultLog << result::newrec << name << getMethod() << t[0] << numIters << this->budget << final_reward
			<< optimal_solution << gap_prec << maximalRewardAll << final_length << numItersLastImprovement
			<< timeLastImprovement << (maximal_calculation_time_sec * 1000) << set_greedy_initial_solution
			<< tourNodes.str() << tourClusters.str() << tourImprovements.str() << false;
}

crl::CConfig & IlpSopSolver::getConfig(crl::CConfig & config) {
	return config;
}

