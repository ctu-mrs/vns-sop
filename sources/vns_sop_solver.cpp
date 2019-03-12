/*
 * VNSVNSGOPPath.cpp
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#include "vns_sop_solver.h"

using crl::logger;

using namespace op;
using namespace crl;
using namespace crl::gui;

#define foreach BOOST_FOREACH
#define MIN_CHANGE_EPS 0.00001
#define DD " , "
#define DEBUG_DOP_TRY_OPERATIONS false
#define DEBUG_CHECK_CONSISTENCY false
#define DEBUG_DOP_IMRPOVE_OPERATIONS false
#define DEBUG_LOWER_BOUND true
#define SAVE_ITER_PATHS false

#define M (std::numeric_limits<double>::max())

VnsSopSolver::VnsSopSolver(crl::CConfig& config, const std::string& problemFile) :
		Base(config), SAVE_RESULTS(config.get<bool>("save-results")), SAVE_SETTINGS(config.get<bool>("save-settings")), BORDER(
				config.get<double>("canvas-border")), SAVE_INFO(config.get<bool>("save-info")), SAVE_TARGETS(
				config.get<bool>("save-targets")), SAVE_SAMPLED_PATH(config.get<bool>("save-sampled-path")) {

	SOP_Prolem problem = SOPLoader::getSOPDefinition(config, problemFile, true);
	INFO("has SOP definition");
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
	this->nodesAllDistances = problem.distances;
	this->oldgoalIndex = problem.oldGoalIndex;
	this->oldstartIndex = problem.oldStartIndex;
	this->deleted_cluster_id = problem.deleted_cluster_id;
	this->ids_originally_from_one = problem.ids_originally_from_one;

	maximalRewardAll = 0;
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		//INFO_VAR(nodesAllClusters[var].size());
		maximalRewardAll += nodesAllClusters[var][0].reward;
	}
	INFO_GREEN("maximalRewardAll "<<this->maximalRewardAll);

	this->maxOverGuessedTry = 0;
	this->avgsaved = NAN;
	this->numTestedAvg = 0;
	this->stop = false;
	this->path_var = 0;
	this->numAvailableNodes = 0;
	this->initial_reward = 0;
	this->useRVNS = config.get<bool>("use-rvns");
	this->numIterations = config.get<int>("num-iterations");
	this->numIterationsUnimproved = config.get<int>("num-iterations-unimproved");
	this->maximal_calculation_time_sec = config.get<int>("maximal-calculation-time-sec");
	this->maximal_calculation_time_MS = maximal_calculation_time_sec * 1000.0;
	this->lower_bound_above_budget_then_skip = config.get<bool>("lower-bound-above-budget-then-skip");
	INFO_VAR(lower_bound_above_budget_then_skip);
	this->set_greedy_initial_solution = config.get<bool>("initial-greedy-solution");
	this->generate_examples_of_operations = config.get<bool>("generate-examples-of-operations");
}

VnsSopSolver::~VnsSopSolver() {

}

void VnsSopSolver::solve() {
	Base::solve();
}

void VnsSopSolver::iterate(int iter) {

	INFO("num-iterations " << numIterations);
	INFO("num-iterations-unimproved " << numIterationsUnimproved);
	this->calcClusterDistancesBounds();
	VnsSopPath::setSamplesDistances(nodesAllClusters, nodesAllDistances, clustersMinDistances, clustersMaxDistances);

	INFO_VAR(startIndex);
	INFO_VAR(goalIndex);
	this->nodesForInsertion = this->nodesAllClusters;
	INFO("print clusters before iterate");
	//SOPLoader::printAllClustersNodes(nodesAllClusters);
	if (startIndex != goalIndex) {
		this->nodesForInsertion.erase(nodesForInsertion.begin() + MAX(startIndex, goalIndex));
		this->nodesForInsertion.erase(nodesForInsertion.begin() + MIN(startIndex, goalIndex));
	} else {
		this->nodesForInsertion.erase(nodesForInsertion.begin() + startIndex);
	}

//firstly select points in eclipse
	INFO("start timer")
	Timer testTouring;
	INFO("resetRealClock")
	testTouring.resetRealClock();

//if (radius == 0) {
//	VNSGOPPath::checkEucllideanDistances();
//}
	INFO("drawNeighborhoodPoints")
	drawNeighborhoodPoints();
	this->availableNodes = getReachableNodes(nodesForInsertion);

	INFO("we have " << availableNodes.size() << " nodes in ellipse out of " << nodesForInsertion.size() << " add nodes");
	this->numAvailableNodes = availableNodes.size();
	INFO("initialize SOP");

	tourVNSGOPPath = VnsSopPath(startIndex, goalIndex, gopType);
	for (int var = 0; var < availableNodes.size(); ++var) {
		vnsVector.push_back(availableNodes[var][0].cluster_id);
		//INFO("add cluster "<<availableNodes[var][0].cluster_id<<" to vns vector");
	}
	INFO("tourVNSGOPPath created");
	drawPath(2000);
	path_var = 1;

	if (set_greedy_initial_solution) {
		generateInitialSolution(tourVNSGOPPath, vnsVector);
	}

	if (generate_examples_of_operations) {
		drawPath(2000000, &tourVNSGOPPath);
		INFO("generating examples of operations");
		exampleOneCluserMoveRandom(tourVNSGOPPath);
		exampleOneCluserExchangeRandom(tourVNSGOPPath);

		examplePathMove(tourVNSGOPPath);
		examplePathExchange(tourVNSGOPPath);

		INFO("exit after generating examples of operations");
		exit(0);
	}

//for example generation of neighborhood operations
	int numItersLastImprovement = 1;
	long timeLastImprovement = testTouring.getRTimeMS();
	this->initial_reward = tourVNSGOPPath.getReward();
	std::vector<ImprovementLogRecord> improvementLog;

	ImprovementLogRecord initialImprovementRecord;
	initialImprovementRecord.length = tourVNSGOPPath.getPathLength();
	initialImprovementRecord.reward = tourVNSGOPPath.getReward();
	initialImprovementRecord.timeMS = timeLastImprovement;
	improvementLog.push_back(initialImprovementRecord);

	int act_iter = 0;

//NeighImrpovementValue imprValue = improveNeighLocations(tourVNSGOPPath, this->neighborhood_improvement_minimal_distance);
//insertNeighborhoods(tourVNSGOPPath, imprValue.originalNeighAngIds, imprValue.improvedNode, imprValue.actualNeighAngles, imprValue.actualGraphNodes);
	std::vector<IndexSOP> returnedPath = tourVNSGOPPath.getPath();

//INFO_GREEN("initial reward "<<tourVNSGOPPath.getReward());
//INFO_GREEN("initial length "<<tourVNSGOPPath.getPathLength());
	INFO_GREEN(
			"initial solution with reward "<<tourVNSGOPPath.getReward()<<", length "<<tourVNSGOPPath.getPathLength()<<" and budget "<<budget<<" at time "<<testTouring.getRTimeMS()<<" ms");

	stop = false;
	int initialNDepth = 1;
	int maximalNDepth = 2;
	while (!stop) {
		//INFO("act_iter "<<act_iter);
		act_iter++;
		//INFO("itteration "<<numIttertation);
		if (act_iter % 200 == 0) {
			INFO(
					"itteration "<<act_iter <<" with best reward "<<tourVNSGOPPath.getReward()<<", length "<<tourVNSGOPPath.getPathLength()<<" and budget "<<budget<<" at time "<<testTouring.getRTimeMS()<<" ms");
		}
		if (act_iter >= numIterations) {
			INFO("stop after maximal number of iterattion "<<numIterations);
			stop = true;
		}
		if (act_iter - numItersLastImprovement >= numIterationsUnimproved) {
			INFO("stop after maximal number of iterattion without improvement "<<numIterationsUnimproved);
			stop = true;
		}
		int k = initialNDepth;
		//INFO("k "<<k);
		while (k <= maximalNDepth) {
			if (testTouring.getRTimeMS() >= maximal_calculation_time_MS) {
				INFO(
						"stop at "<<testTouring.getRTimeMS()<<" after maximal number of misiliseconds "<<maximal_calculation_time_MS<< " obtained from "<<maximal_calculation_time_sec<<" maximal seconds");
				stop = true;
			}
			//INFO("k "<<k);
			VnsSopPath actualVNSGOPPath = tourVNSGOPPath;

			std::vector<int> actualVNS = vnsVector;
			double rewardBefore = actualVNSGOPPath.getReward();
			double lengthBefore = actualVNSGOPPath.getPathLength();

			GraphNode localNeighChain = GraphNode();
			shake(actualVNSGOPPath, actualVNS, k, localNeighChain);

			//checkLengths(actualVNSGOPPath," before local search");
			//INFO("before local search");

			randomLocalSearch(actualVNSGOPPath, actualVNS, k, localNeighChain);

			double newReward = actualVNSGOPPath.getReward();
			double newLength = actualVNSGOPPath.getPathLength(localNeighChain);
			//savePaths(&actualVNSGOPPath);
			//INFO_GREEN("actual best  "<<tourVNSGOPPath.getReward());
			if (newReward > rewardBefore || (newReward == rewardBefore && newLength < lengthBefore)) {
				//checkLengths(actualVNSGOPPath, actualVNS, " when improved");
				INFO_GREEN(
						"improved to reward "<<newReward<<" with length "<<newLength<<" and budget "<<budget<<" at time "<<testTouring.getRTimeMS()<<" ms");
				timeLastImprovement = testTouring.getRTimeMS();
				numItersLastImprovement = act_iter;

				//insert the improvement record to the log
				ImprovementLogRecord newImprovementRecord;
				newImprovementRecord.length = newLength;
				newImprovementRecord.reward = newReward;
				newImprovementRecord.timeMS = timeLastImprovement;
				improvementLog.push_back(newImprovementRecord);

				tourVNSGOPPath = actualVNSGOPPath;
				vnsVector = actualVNS;
				double calculated_length = tourVNSGOPPath.getPathLengthCalculate();
				if (calculated_length > budget || newLength > budget || calculated_length - newLength > 0.01) {
					ERROR("badly calculated length in new solution");
					INFO_VAR(budget);
					INFO_VAR(newLength);
					INFO_VAR(calculated_length);
					checkLengths(tourVNSGOPPath, vnsVector, "in badly cal new solution");
					exit(1);
				}

				drawPath(2000, &actualVNSGOPPath);
				//saveCanvas();
				k = initialNDepth;
			} else {
				k++;
			}

		}
		//stop = true;
	}

	finalTourVNSGOPPath = tourVNSGOPPath;
	tSolve.stop();
	INFO("fillResultRecord");
	fillResultRecord(act_iter, tourVNSGOPPath.getPathLength(), numItersLastImprovement, timeLastImprovement,
			improvementLog);
	INFO("write result log");

	INFO_GREEN(
			"found tour with reward "<<finalTourVNSGOPPath.getReward()<<" and length "<<finalTourVNSGOPPath.getPathLength()<<" out of "<<budget<<" budget");
	resultLog << crl::result::endrec;
}

void VnsSopSolver::calcClusterDistancesBounds() {
	INFO("calc lower and upper bound on cluster distances - begin")
	std::vector<std::vector<double>> minDistances;
	std::vector<std::vector<double>> maxDistances;
	minDistances.resize(nodesAllDistances.size());
	maxDistances.resize(nodesAllDistances.size());
	for (int cl1 = 0; cl1 < nodesAllDistances.size(); ++cl1) {
		minDistances[cl1].resize(nodesAllDistances[cl1].size());
		maxDistances[cl1].resize(nodesAllDistances[cl1].size());

		for (int cl2 = 0; cl2 < nodesAllDistances[cl1].size(); ++cl2) {
			double minDistance = M;
			double maxDistance = -M;
			int cl1NodesNum = nodesAllDistances[cl1][cl2].size();

			for (int cl1node = 0; cl1node < cl1NodesNum; ++cl1node) {
				int cl2NodesNum = nodesAllDistances[cl1][cl2][cl1node].size();

				for (int cl2node = 0; cl2node < cl2NodesNum; ++cl2node) {
					if (nodesAllDistances[cl1][cl2][cl1node][cl2node] < minDistance) {
						minDistance = nodesAllDistances[cl1][cl2][cl1node][cl2node];
					}

					if (nodesAllDistances[cl1][cl2][cl1node][cl2node] > maxDistance) {
						maxDistance = nodesAllDistances[cl1][cl2][cl1node][cl2node];
					}
				}
			}
			minDistances[cl1][cl2] = minDistance;
			maxDistances[cl1][cl2] = maxDistance;
		}
	}
	this->clustersMinDistances = minDistances;
	this->clustersMaxDistances = maxDistances;
	INFO("calc lower and upper bound on cluster distances - end")
}

void VnsSopSolver::checkLengths(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, std::string text) {
	double pl = actualVNSGOPPath.getPathLength();
	double plr = actualVNSGOPPath.getPathLengthRear();
	double plc = actualVNSGOPPath.getPathLengthCalculate();
	double plcr = actualVNSGOPPath.getPathLengthCalculateRear();
	actualVNSGOPPath.checkSameSamples(text);
	checkConsistency(actualVNSGOPPath, actualVNS);
	if (fabs(pl - plr) > 0.05 || fabs(pl - plc) > 0.05 || fabs(plc - plr) > 0.05) {
		INFO("bad sizes comparison "<<text);
		INFO_VAR_RED(pl);
		INFO_VAR_RED(plr);
		INFO_VAR_RED(plc);
		INFO_VAR_RED(plcr);
		std::vector<IndexSOP> returnedPath = actualVNSGOPPath.getPath();

		shortest_matrix shortest = actualVNSGOPPath.getShortestRef();
		shortest_matrix shortest_back = actualVNSGOPPath.getShortestBackRef();
		actualVNSGOPPath.checkShortestConsistency();
		actualVNSGOPPath.checkSameSamples();

		actualVNSGOPPath.update();
		INFO("after update pl "<< actualVNSGOPPath.getPathLength());
		INFO("after update plr "<< actualVNSGOPPath.getPathLengthRear());
		INFO("after update plc "<< actualVNSGOPPath.getPathLengthCalculate());
		INFO("after update plcr "<< actualVNSGOPPath.getPathLengthCalculateRear());
		exit(1);
	}
}

void VnsSopSolver::shake(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
		GraphNode & chainGeneratingNeighAngEnd) {
//INFO("shake "<<k);
	if (k == 1) {
		pathInsert(actualVNSGOPPath, actualVNS, 1);
	} else if (k == 2) {
		pathExchange(actualVNSGOPPath, actualVNS, 1);
	}
	if ( DEBUG_DOP_TRY_OPERATIONS) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		checkConsistency(actualVNSGOPPath, actualVNS);
	}
//INFO("shake done "<<k);
}

bool VnsSopSolver::randomLocalSearch(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
		GraphNode & chainGeneratingNeighAngEnd) {
//INFO("randomLocalSearch "<<k);
	int numIters = numAvailableNodes * numAvailableNodes;
	if (k == 1) {
		//checkLengths(actualVNSGOPPath,actualVNS, "before insertRandom");
		insertRandom(actualVNSGOPPath, actualVNS, numIters);
	} else if (k == 2) {
		//checkLengths(actualVNSGOPPath,actualVNS, "before exchangeRandom");
		exchangeRandom(actualVNSGOPPath, actualVNS, numIters);
	}
//INFO("length "<<actualVNSGOPPath.getPathLength());
//INFO("localSearch done with improvements "<<numImprovements);
	if ( DEBUG_DOP_TRY_OPERATIONS) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		checkConsistency(actualVNSGOPPath, actualVNS);
	}
//INFO("randomLocalSearch done "<<k);
	return true;
}

void VnsSopSolver::fitDOPtoBudget(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS) {
//remove nodes over budget
	while (actualVNSGOPPath.getPathLength() > budget) {
		actualVNSGOPPath.removePoint(actualVNSGOPPath.getNumTargets() - 1);
	}
	bool canAdd = true;

//add nodes to fill budget
	while (canAdd) {
		canAdd = false;
		int idToAdd = actualVNSGOPPath.getNumTargets();
		if (idToAdd < actualVNS.size()) {
			int testingNode = actualVNS[idToAdd];
			double testAdd = actualVNSGOPPath.tryToAdd(testingNode, idToAdd);
			if (testAdd <= budget) {
				actualVNSGOPPath.addPoint(testingNode, idToAdd);
				canAdd = true;
			}
		}
	}

}

void VnsSopSolver::onePointMove(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS) {
	INFO("onePointMove() begin");

	int bestTargetIDFrom = -1;
	int bestTargetIDTo = -1;
	double actualPathLength = actualVNSGOPPath.getPathLength();
	double actualPathReward = actualVNSGOPPath.getReward();
	double minimalLength = actualPathLength;
	double maximalReward = actualPathReward;

	for (int targetIDFrom = actualVNSGOPPath.getNumTargets(); targetIDFrom < actualVNS.size(); ++targetIDFrom) {
		int testingRelocateID = actualVNS[targetIDFrom];
		double testingRelocateReward = nodesAllClusters[testingRelocateID][0].reward;

		for (int targetIDTo = 0; targetIDTo <= actualVNSGOPPath.getNumTargets(); ++targetIDTo) {
			if (targetIDFrom != targetIDTo) {
				double addedLength = 0;
				double addedReward = 0;
				if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
					addedLength = actualVNSGOPPath.tryToAdd(testingRelocateID, targetIDTo);
					addedReward = testingRelocateReward;
				}

				double rewardAfter = actualPathReward + addedReward;
				double lengthAfter = actualPathLength + addedLength;
				/*
				 if (lengthAfter <= budget) {
				 INFO("rewardChange "<<(addedReward - removedReward));
				 INFO("lengthChange "<<(addedLength - removedLength));
				 }
				 */
				if (lengthAfter <= budget
						&& (rewardAfter > maximalReward || (rewardAfter == maximalReward && lengthAfter < minimalLength))) {
					bestTargetIDFrom = targetIDFrom;
					bestTargetIDTo = targetIDTo;
					maximalReward = rewardAfter;
					minimalLength = lengthAfter;
				}
			}
		}
	}

	if (bestTargetIDFrom != -1) {
		INFO("bestTargetIDFrom "<<bestTargetIDFrom);
		INFO("bestTargetIDTo "<<bestTargetIDTo);
		INFO("maximalReward "<<maximalReward);
		INFO("minimalLength "<<minimalLength);
		int testingRelocateID = actualVNS[bestTargetIDFrom];
		if (bestTargetIDFrom > bestTargetIDTo) {
			if (bestTargetIDFrom < actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.removePoint(bestTargetIDFrom);
			}
			actualVNS.erase(actualVNS.begin() + bestTargetIDFrom);

			if (bestTargetIDTo <= actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.addPoint(testingRelocateID, bestTargetIDTo);
			}
			actualVNS.insert(actualVNS.begin() + bestTargetIDTo, testingRelocateID);
		} else {
			if (bestTargetIDTo <= actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.addPoint(testingRelocateID, bestTargetIDTo);
			}
			actualVNS.insert(actualVNS.begin() + bestTargetIDTo, testingRelocateID);

			if (bestTargetIDFrom < actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.removePoint(bestTargetIDFrom);
			}
			actualVNS.erase(actualVNS.begin() + bestTargetIDFrom);
		}
	}
	INFO("onePointMove() end");
}

void VnsSopSolver::generateInitialSolution(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS) {
	INFO("generateInitialSolution() begin");
	INFO("availableNodes size "<<availableNodes.size());
	greedy_insertion(actualVNSGOPPath, actualVNS);
	INFO("reward after insertion "<<actualVNSGOPPath.getReward());
	INFO("num targets " <<actualVNSGOPPath.getNumTargets());
	INFO("generateInitialSolution() end");
}

bool VnsSopSolver::insertRandom(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes) {
//INFO("insertRandom beg");
	const int sizeV = actualVNS.size() - 1;
//double max_neigh_distance_between = neighborhood_radius * 2 + M_2PI * radius;
//INFO_VAR(max_neigh_distance_between);
//INFO("insert "<<k);
//checkLengths(actualVNSGOPPath,actualVNS, "insertRandom begin");

	double actualLength = actualVNSGOPPath.getPathLength();
	double actualReward = actualVNSGOPPath.getReward();
	double bestTourReward = tourVNSGOPPath.getReward();
	double maximally_achieved_reward = MAX(actualReward, bestTourReward);
	double maxOverbudget = 0;
	int num_improvable = 0;
	int num_test_improve = 0;
	int lastImprovementIndex = 0;

	int numLoweboundTryToAdd = 0;
	int numTestLowerboundTryToAdd = 0;
	int numLoweboundTryToMove = 0;
	int numTestLowerboundTryToMove = 0;
	int numTestImprovableReward = 0;

	int num_unchanged_iters = 0;
	int stop_after_iters = MAX(100, num_changes / 5);

	VnsSopPath bestOverRewardVNSGOPPath = actualVNSGOPPath;
	std::vector<int> bestOverRewardVNS = actualVNS;

	if (actualVNS.size() >= 3) {
		for (int var = 0; var < num_changes; ++var) {
			/*
			 if ( num_unchanged_iters > stop_after_iters ) {
			 INFO_CYAN("stopping insertRandom after unsuccessful "<<num_unchanged_iters<<" iterations");
			 break;
			 }
			 */

			int targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
			int targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets());
			//int targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
			while (abs(targetIDFrom - targetIDTo) <= 1) {
				targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
				//if (randDoubleMinMax(0.0, 1.0) < 0.5 && actualReward > initial_reward) {
				//	targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
				//} else {
				targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets());
				//}
				//INFO("targetIDFrom "<<targetIDFrom);
				//INFO("targetIDTo "<<targetIDTo);
			}

			//checkLengths(actualVNSGOPPath,actualVNS, "insertRandom before change");

			int testingRelocateID = actualVNS[targetIDFrom];
			double testingRelocateReward = nodesAllClusters[testingRelocateID][0].reward;

			double lengthAfterRemove = actualLength;
			if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
				lengthAfterRemove = actualVNSGOPPath.tryToRemove(targetIDFrom);
			}

			double lengthRemoved = actualLength - lengthAfterRemove;
			double lengthAfterMove = 0;
			double addedReward = 0;
			if (targetIDFrom < actualVNSGOPPath.getNumTargets() && targetIDTo <= actualVNSGOPPath.getNumTargets()) {
				//both targetIDTo and targetIDFrom inside existing path - exchange points inside path
				double lengthAfterMoveLowerBound = actualVNSGOPPath.tryToMoveLowerBound(targetIDFrom, targetIDTo);

				numTestLowerboundTryToMove++;
				if (lower_bound_above_budget_then_skip && lengthAfterMoveLowerBound > budget) {
					continue;
				}

				lengthAfterMove = actualVNSGOPPath.tryToMove(targetIDFrom, targetIDTo);

				if ( DEBUG_LOWER_BOUND && lengthAfterMoveLowerBound - lengthAfterMove > 0.01) {
					INFO_VAR(actualLength);
					INFO_VAR(lengthAfterMove);
					INFO_VAR(lengthAfterMoveLowerBound);
					ERROR("bad lower bound count tryToMoveLowerBound - lowerbound larger than reality");
					exit(0);
				}

				if ( DEBUG_LOWER_BOUND && lengthAfterMoveLowerBound > budget) {
					numLoweboundTryToMove++;
					if (lengthAfterMove <= budget) {
						INFO_VAR(actualLength);
						INFO_VAR(lengthAfterMove);
						INFO_VAR(lengthAfterMoveLowerBound);
						ERROR("bad lower bound count tryToMoveLowerBound");
						exit(0);
					}
				}

				//check outcome
				check_try_operation(actualVNSGOPPath, actualVNS, targetIDFrom, targetIDTo, lengthAfterMove, false);

			} else if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
				//targetIDTo inside exisitng path - adding new point to path

				double tryToAddLowerBound = actualVNSGOPPath.tryToAddLowerBound(testingRelocateID, targetIDTo);
				double lengthAfterMoveLowerBound = tryToAddLowerBound - lengthRemoved;

				numTestLowerboundTryToAdd++;
				if (lower_bound_above_budget_then_skip && lengthAfterMoveLowerBound > budget) {
					continue;
				}

				double lengthAfterAdd = actualVNSGOPPath.tryToAdd(testingRelocateID, targetIDTo);

				double lengthAdded = lengthAfterAdd - actualLength;
				addedReward = ((targetIDFrom >= actualVNSGOPPath.getNumTargets()) ? (testingRelocateReward) : (0));

				lengthAfterMove = actualLength - lengthRemoved + lengthAdded;

				if ( DEBUG_LOWER_BOUND && lengthAfterMoveLowerBound - lengthAfterMove > 0.01) {
					INFO_VAR(lengthAfterMove);
					INFO_VAR(lengthAfterMoveLowerBound);
					ERROR("bad lower bound count tryToAddLowerBound, lowerbound bigger than reality");
					exit(0);
				}

				if ( DEBUG_LOWER_BOUND && lengthAfterMoveLowerBound > budget) {
					numLoweboundTryToAdd++;
					if (lengthAfterMove <= budget) {
						INFO_VAR(lengthAfterMove);
						INFO_VAR(lengthAfterMoveLowerBound);
						ERROR("bad lower bound count tryToAddLowerBound");
						exit(0);
					}
				}

			} else {

				//targetIDTo out of path
				if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
					//targetIDFrom in path - removing int from path
					//INFO("boooooo below")
					lengthAfterMove = lengthAfterRemove;
					addedReward = -testingRelocateReward;
				} else {
					//points outside of path moved
					lengthAfterMove = actualLength;
				}

				//INFO("boooooo")
			}

			//INFO("test budget");
			if (lengthAfterMove <= budget && (addedReward > 0 || (actualLength - lengthAfterMove) > MIN_CHANGE_EPS)) {
				lastImprovementIndex = var;
				int testingRelocateID = actualVNS[targetIDFrom];

				//checkLengths(actualVNSGOPPath,actualVNS, "insertRandom before change");

				if (targetIDTo > targetIDFrom) {
					//move up
					if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
						actualVNSGOPPath.addPoint(testingRelocateID, targetIDTo);
					}
					if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
						//INFO("remove point "<<targetIDFrom);
						actualVNSGOPPath.removePoint(targetIDFrom);
					}
					actualVNS.insert(actualVNS.begin() + targetIDTo, testingRelocateID);
					actualVNS.erase(actualVNS.begin() + targetIDFrom);
				} else {
					if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
						//INFO("remove point "<<targetIDFrom);
						actualVNSGOPPath.removePoint(targetIDFrom);
					}
					if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
						actualVNSGOPPath.addPoint(testingRelocateID, targetIDTo);
					}
					actualVNS.erase(actualVNS.begin() + targetIDFrom);
					actualVNS.insert(actualVNS.begin() + targetIDTo, testingRelocateID);
				}
				num_unchanged_iters = 0;
				//DEBUG TESTING
				actualLength = actualVNSGOPPath.getPathLength();
				double newReward = actualVNSGOPPath.getReward();
				if ( DEBUG_LOWER_BOUND && fabs(newReward - (actualReward + addedReward)) > 0.1) {
					INFO("wrongly calculated addedReward");
					INFO_VAR(newReward);
					INFO_VAR((actualReward + addedReward));
					exit(1);
				}

				actualReward = newReward;
				if (actualReward > maximally_achieved_reward) {
					maximally_achieved_reward = actualReward;
				}

				if (actualLength - lengthAfterMove > 0.25) {
					INFO("wrongly calculated insertRandom");
					INFO_VAR(actualLength);
					INFO_VAR(lengthAfterMove);
					INFO_VAR(targetIDFrom);
					INFO_VAR(targetIDTo);
					INFO_VAR(lengthRemoved);
					INFO_VAR(actualVNSGOPPath.getNumTargets());
					INFO_VAR(actualVNSGOPPath.getPathLengthCalculate());
					INFO_VAR(actualVNSGOPPath.getPathLengthRear());
					actualVNSGOPPath.listIds();
					exit(1);
				}

			} else {
				num_unchanged_iters++;
			}
		}
	}

	return true;
}

bool VnsSopSolver::exchangeRandom(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes) {
//INFO("exchangeRandom beg");
	const int sizeV = actualVNS.size() - 1;
	double actualLength = actualVNSGOPPath.getPathLength();
	double actualReward = actualVNSGOPPath.getReward();
	double bestTourReward = tourVNSGOPPath.getReward();
	double maximally_achieved_reward = MAX(actualReward, bestTourReward);

	int num_unchanged_iters = 0;
	int stop_after_iters = MAX(100, num_changes / 5);

//checkLengths(actualVNSGOPPath,actualVNS, "exchangeRandom beign");

	int lastImprovementIndex = 0;
	int numTestImprovableReward = 0;

	int numLoweboundTryToReplace = 0;
	int numTestLowerboundTryToReplace = 0;

	int numLowerboundTryToExchange = 0;
	int numTestLowerboundTryToExchange = 0;

	VnsSopPath bestOverRewardVNSGOPPath = actualVNSGOPPath;
	std::vector<int> bestOverRewardVNS = actualVNS;

	if (actualVNS.size() >= 4 && actualVNSGOPPath.getNumTargets() > 0) {
		for (int var = 0; var < num_changes; ++var) {
			/*
			 if ( num_unchanged_iters > stop_after_iters ) {
			 INFO_CYAN("stopping exchangeRandom after unsuccessful "<<num_unchanged_iters<<" iterations");
			 break;
			 }
			 */

			double minLength = actualLength;
			double maxAddReward = 0;

			int targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
			int targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets() - 1);
			//int targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
			while (abs(targetIDFrom - targetIDTo) <= 2) {
				targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
				targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets() - 1);
				//targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
			}

			//checkLengths(actualVNSGOPPath,actualVNS, "exchangeRandom before change");

			int testingRelocateFromID = actualVNS[targetIDFrom];
			double testingRelocateFromReward = nodesAllClusters[testingRelocateFromID][0].reward;

			if (abs(targetIDFrom - targetIDTo) > 2) {
				int testingRelocateToID = actualVNS[targetIDTo];
				double testingRelocateToReward = nodesAllClusters[testingRelocateToID][0].reward;
				double addedReward = 0;
				double lengthAfterMove = 0;
				if (targetIDFrom < actualVNSGOPPath.getNumTargets() && targetIDTo < actualVNSGOPPath.getNumTargets()) {
					//exchange two nodes that are in the feasible path
					//INFO("if before tryToExchangeLowerBound");
					double lengthAfterExchangeLowerBound = actualVNSGOPPath.tryToExchangeLowerBound(targetIDFrom,
							targetIDTo);

					numTestLowerboundTryToExchange++;
					if (lower_bound_above_budget_then_skip && lengthAfterExchangeLowerBound > budget) { //LOWER_BOUND_ABOVE_BUDGET_SKIP
						continue;
					}

					lengthAfterMove = actualVNSGOPPath.tryToExchange(targetIDFrom, targetIDTo);

					if ( DEBUG_LOWER_BOUND && lengthAfterExchangeLowerBound - lengthAfterMove > 0.01) {
						INFO_VAR(lengthAfterMove);
						INFO_VAR(lengthAfterExchangeLowerBound);
						ERROR("bad lower bound count tryToExchangeLowerBound  - lowerbound larget than reality");
						exit(1);
					}

					if ( DEBUG_LOWER_BOUND && lengthAfterExchangeLowerBound > budget) {
						numLowerboundTryToExchange++;
						if (lengthAfterMove <= budget) {
							INFO_VAR(lengthAfterMove);
							INFO_VAR(lengthAfterExchangeLowerBound);
							ERROR("bad lower bound count tryToExchangeLowerBound");
							exit(1);
						}
					}

					check_try_operation(actualVNSGOPPath, actualVNS, targetIDFrom, targetIDTo, lengthAfterMove, true);

				} else {
					//testingRelocateFrom is out of feasible
					double lengthAfterReplaceToLowerBound = actualVNSGOPPath.tryToReplaceLowerBound(
							testingRelocateFromID, targetIDTo);
					numTestLowerboundTryToReplace++;
					if (lower_bound_above_budget_then_skip && lengthAfterReplaceToLowerBound > budget) {
						continue;
					}

					double lengthAfterReplaceTo = actualVNSGOPPath.tryToReplace(testingRelocateFromID, targetIDTo);
					addedReward += testingRelocateFromReward - testingRelocateToReward;

					lengthAfterMove = lengthAfterReplaceTo;
					if ( DEBUG_LOWER_BOUND && lengthAfterReplaceToLowerBound - lengthAfterMove > 0.01) {
						INFO_VAR(lengthAfterMove);
						INFO_VAR(lengthAfterReplaceToLowerBound);
						ERROR("bad lower bound count tryToAddLowerBound, lowerbound bigger than reality");
						exit(0);
					}

					if ( DEBUG_LOWER_BOUND && lengthAfterReplaceToLowerBound > budget) {
						numLoweboundTryToReplace++;
						if (lengthAfterMove <= budget) {
							INFO_VAR(lengthAfterMove);
							INFO_VAR(lengthAfterReplaceToLowerBound);
							ERROR("bad lower bound count tryToReplaceLowerBound");
							exit(1);
						}
					}
				}

				if (lengthAfterMove <= budget
						&& (addedReward > maxAddReward || (minLength - lengthAfterMove) > MIN_CHANGE_EPS)) {
					lastImprovementIndex = var;
					int testingRelocateFromID = actualVNS[targetIDFrom];
					int testingRelocateToID = actualVNS[targetIDTo];
					//INFO("before change");
					if (targetIDTo > targetIDFrom) {
						actualVNSGOPPath.removePoint(targetIDTo);
						actualVNSGOPPath.addPoint(testingRelocateFromID, targetIDTo);
						if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
							actualVNSGOPPath.removePoint(targetIDFrom);
							actualVNSGOPPath.addPoint(testingRelocateToID, targetIDFrom);
						}
						int temp = actualVNS[targetIDTo];
						actualVNS[targetIDTo] = actualVNS[targetIDFrom];
						actualVNS[targetIDFrom] = temp;
					} else {
						if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
							actualVNSGOPPath.removePoint(targetIDFrom);
							actualVNSGOPPath.addPoint(testingRelocateToID, targetIDFrom);
						}
						actualVNSGOPPath.removePoint(targetIDTo);
						actualVNSGOPPath.addPoint(testingRelocateFromID, targetIDTo);

						int temp = actualVNS[targetIDTo];
						actualVNS[targetIDTo] = actualVNS[targetIDFrom];
						actualVNS[targetIDFrom] = temp;
					}
					num_unchanged_iters = 0;
					//actualVNSGOPPath.evaluateUsage();
					double newReward = actualVNSGOPPath.getReward();

					actualLength = actualVNSGOPPath.getPathLength();
					actualReward = newReward;
					if (actualReward > maximally_achieved_reward) {
						maximally_achieved_reward = actualReward;
					}

					if (actualLength - lengthAfterMove > 0.02) {
						INFO("wrongly calculated exchangeRandom");
						INFO_VAR(actualLength);
						INFO_VAR(lengthAfterMove);
						INFO_VAR(targetIDFrom);
						INFO_VAR(targetIDTo);
						INFO_VAR(actualVNSGOPPath.getNumTargets());
						INFO_VAR(actualVNSGOPPath.getPathLengthCalculate());
						INFO_VAR(actualVNSGOPPath.getPathLengthRear());
						actualVNSGOPPath.listIds();
						exit(1);
					}
					//INFO("IMPROVED NORMAL");
				} else {
					num_unchanged_iters++;
				}
			}
		}
	}

	return true;
}

bool VnsSopSolver::check_try_operation(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int targetIDFrom,
		int targetIDTo, double lengthAfterMove, bool relocate) {

	if ( DEBUG_DOP_TRY_OPERATIONS && !relocate) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		int testingRelocateID = actualVNS[targetIDFrom];

		VnsSopPath copyAdd = actualVNSGOPPath;
		if (targetIDTo > targetIDFrom) {
			copyAdd.addPoint(testingRelocateID, targetIDTo);
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
			}
		} else {
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
			}
			copyAdd.addPoint(testingRelocateID, targetIDTo);
		}
		copyAdd.update();
		double lengthAfterAddCopy = copyAdd.getPathLength();
		if (fabs(lengthAfterAddCopy - lengthAfterMove) > 0.1) {
			ERROR("lengthAfterMove does not match lengthAfterAdd");
			ERROR("insertRandom from "<<targetIDFrom<<" to "<<targetIDTo);
			ERROR("id "<<testingRelocateID);
			ERROR(lengthAfterAddCopy<< " and "<< lengthAfterMove);
			actualVNSGOPPath.listIds();
			exit(1);
		}
	}

	if ( DEBUG_DOP_TRY_OPERATIONS && relocate) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		VnsSopPath copyAdd = actualVNSGOPPath;
		int testingRelocateFromCopyID = actualVNS[targetIDFrom];
		int testingRelocateToCopyID = actualVNS[targetIDTo];

		if (targetIDTo > targetIDFrom) {
			copyAdd.removePoint(targetIDTo);
			copyAdd.addPoint(testingRelocateFromCopyID, targetIDTo);
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
				copyAdd.addPoint(testingRelocateToCopyID, targetIDFrom);
			}
		} else {
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
				copyAdd.addPoint(testingRelocateToCopyID, targetIDFrom);
			}
			copyAdd.removePoint(targetIDTo);
			copyAdd.addPoint(testingRelocateFromCopyID, targetIDTo);
		}
		copyAdd.update();
		double lengthAfterAddCopy = copyAdd.getPathLength();
		if (fabs(lengthAfterAddCopy - lengthAfterMove) > 0.1) {
			ERROR("lengthAfterMove does not match lengthAfterAdd");
			ERROR("exchangeRandom from "<<targetIDFrom<<" to "<<targetIDTo);
			ERROR(lengthAfterAddCopy<< " and "<< lengthAfterMove);
			//actualVNSGOPPath.listIds();
			exit(1);
		}
	}
	return true;
}

void VnsSopSolver::pathInsert(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int numExchanges) {
//INFO("pathInsert begin");
	const int sizeV = actualVNS.size() - 1;
	if (sizeV >= 2) {
		for (int var = 0; var < numExchanges; ++var) {
			std::unordered_set<int> randNumbersSet;
			while (randNumbersSet.size() < 3) {
				randNumbersSet.insert(randIntMinMax(0, sizeV));
			}
			std::vector<int> randNumbers;
			int i = 0;
			for (auto it = randNumbersSet.begin(); it != randNumbersSet.end(); ++it) {
				randNumbers.push_back(*it);
			}
			CSort<int>::quicksort(&randNumbers, 0, randNumbers.size() - 1);

			int exchangeFromStart = randNumbers[0];
			int exchangeFromEnd = randNumbers[1];
			int insertTo = randNumbers[2];
			bool insertUp = true;
			if (randIntMinMax(0, 1) == 1) {
				insertTo = randNumbers[0];
				exchangeFromStart = randNumbers[1];
				exchangeFromEnd = randNumbers[2];
				insertUp = false;
			}

			if (insertUp) {
				//insert up

				std::vector<int> newVec(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);

				actualVNS.insert(actualVNS.begin() + insertTo, newVec.begin(), newVec.end());
				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);

				if (insertTo < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.addPoint(newVec, insertTo);
				}
				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
				}

			} else {
				//insert bellow

				std::vector<int> newVec(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);

				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);
				actualVNS.insert(actualVNS.begin() + insertTo, newVec.begin(), newVec.end());

				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
				}
				if (insertTo < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.addPoint(newVec, insertTo);
				}

			}

			fitDOPtoBudget(actualVNSGOPPath, actualVNS);

			//actualVNSGOPPath.evaluateUsage();
		}
	}
//INFO("pathInsert end");
//INFO("pathInsert "<<numExchanges<<" end");
}

void VnsSopSolver::pathExchange(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int numExchanges) {
//INFO("pathExchange begin");
//INFO("pathExchange "<<numExchanges<<" begin");
	const int sizeV = actualVNS.size() - 1;
	if (sizeV >= 3) {
		for (int var = 0; var < numExchanges; ++var) {
			std::unordered_set<int> randNumbersSet;
			while (randNumbersSet.size() < 4) {
				randNumbersSet.insert(randIntMinMax(0, sizeV));
			}
			std::vector<int> randNumbers;
			for (auto it = randNumbersSet.begin(); it != randNumbersSet.end(); ++it) {
				randNumbers.push_back(*it);
			}
			CSort<int>::quicksort(&randNumbers, 0, randNumbers.size() - 1);

			int exchangeFromStart = randNumbers[0];
			int exchangeFromEnd = randNumbers[1];
			int exchangeToStart = randNumbers[2];
			int exchangeToEnd = randNumbers[3];
			bool exchangeUp = true;
			if (randIntMinMax(0, 1) == 1) {
				exchangeFromStart = randNumbers[2];
				exchangeFromEnd = randNumbers[3];
				exchangeToStart = randNumbers[0];
				exchangeToEnd = randNumbers[1];
				exchangeUp = false;
			}

			if (exchangeUp) {
				std::vector<int> newVecFrom(actualVNS.begin() + exchangeFromStart,
						actualVNS.begin() + exchangeFromEnd + 1);		//is below
				std::vector<int> newVecTo(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);//is above

				actualVNS.erase(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeToStart, newVecFrom.begin(), newVecFrom.end());
				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeFromStart, newVecTo.begin(), newVecTo.end());

				//INFO("add newVecFrom");
				if (exchangeToStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeToStart,
							MIN(exchangeToEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecFrom, exchangeToStart);
				}
				//INFO("add newVecTo");
				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecTo, exchangeFromStart);
				}

			} else {
				std::vector<int> newVecFrom(actualVNS.begin() + exchangeFromStart,
						actualVNS.begin() + exchangeFromEnd + 1);
				std::vector<int> newVecTo(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);

				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeFromStart, newVecTo.begin(), newVecTo.end());

				actualVNS.erase(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeToStart, newVecFrom.begin(), newVecFrom.end());

				//INFO("size after "<<actualVNS.size());

				//INFO("add newVecTo");
				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecTo, exchangeFromStart);
				}

				//INFO("add newVecFrom");
				if (exchangeToStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeToStart,
							MIN(exchangeToEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecFrom, exchangeToStart);
				}

			}

			fitDOPtoBudget(actualVNSGOPPath, actualVNS);

			//actualVNSGOPPath.evaluateUsage();
		}
	}
//INFO("pathExchange end");
//INFO("pathExchange "<<numExchanges<<" end");
}

void VnsSopSolver::greedy_insertion(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS) {
	INFO("greedy_insertion begin");
//actualVNSGOPPath.printShortestDistances();
	int waitTimeus = 1000;
	bool somethingAdded = true;
//savePaths(&actualVNSGOPPath);
//INFO_VAR(actualVNS.size())
	while (somethingAdded) {
		//INFO("inserting");
		somethingAdded = false;
		double minimalAddDistPerReward = DBL_MAX;
		double actualLength = actualVNSGOPPath.getPathLength();
		double newLengthAfterAdd = actualLength;
		int idAvNodeMinimal = -1;
		int idTourNodeMinimal = -1;

		int dopSize = actualVNSGOPPath.getNumTargets();
		//INFO_VAR(dopSize);
		for (int idAvNode = dopSize; idAvNode < actualVNS.size(); ++idAvNode) {
			//try to add between start ang goal
			int testingNodeID = actualVNS[idAvNode];
			double testingNodeReward = nodesAllClusters[testingNodeID][0].reward;
			for (int idTourNode = 0; idTourNode < actualVNSGOPPath.getSize(); ++idTourNode) {
				//INFO("try to add idAvNode "<<idAvNode<<" with reward "<<testingNodeReward<<" to position "<<idTourNode);
				double newDistance = actualVNSGOPPath.tryToAdd(testingNodeID, idTourNode);
				double additionalDistance = newDistance - actualLength;
				double additionalDistPerReward = additionalDistance / testingNodeReward;
				//INFO("distance "<<newDistance);
				if (additionalDistPerReward < minimalAddDistPerReward && newDistance <= budget) {
					//INFO("additionalDistPerReward bettew "<<additionalDistPerReward);
					minimalAddDistPerReward = additionalDistPerReward;
					newLengthAfterAdd = newDistance;
					idAvNodeMinimal = idAvNode;
					idTourNodeMinimal = idTourNode;
				}
			}
		}

		if (idAvNodeMinimal >= 0 && idTourNodeMinimal >= 0 && newLengthAfterAdd <= budget) {
			somethingAdded = true;
			//INFO("add idAvNode "<<idAvNodeMinimal<<" to position "<<idTourNodeMinimal);

			int insertingID = actualVNS[idAvNodeMinimal];
			actualVNSGOPPath.addPoint(insertingID, idTourNodeMinimal);
			//savePaths(&actualVNSGOPPath);
			//INFO("improveNeighLocations after insert");
			//improveNeighLocations(actualVNSGOPPath, actualVNS, 1);
			//INFO("improveNeighLocations after insert end");

			if (idAvNodeMinimal > idTourNodeMinimal) {
				actualVNS.erase(actualVNS.begin() + idAvNodeMinimal);
				actualVNS.insert(actualVNS.begin() + idTourNodeMinimal, insertingID);
			} else {
				actualVNS.insert(actualVNS.begin() + idTourNodeMinimal, insertingID);
				actualVNS.erase(actualVNS.begin() + idAvNodeMinimal);
			}
			checkConsistency(actualVNSGOPPath, actualVNS);
			drawPath(waitTimeus);
		}

	}

	checkConsistency(actualVNSGOPPath, actualVNS);
	INFO("insertion end");
}

samples_type VnsSopSolver::getReachableNodes(samples_type nodesForInsertion_) {
	INFO("getReachableNodes num nodes for insetion "<<nodesForInsertion_.size());
	return nodesForInsertion_;
	samples_type reachableNodes_;
	reachableNodes_.resize(nodesForInsertion_.size());

	VnsSopPath sgVNSGOPPath = VnsSopPath(startIndex, goalIndex, gopType);

	for (int varCluster = 0; varCluster < nodesForInsertion_.size(); ++varCluster) {
		//INFO("sgVNSGOPPath.tryToAdd");
		int cluster_id = nodesForInsertion_[varCluster][0].cluster_id;
		//INFO_VAR(cluster_id);

		for (int inClusterNodeId = 0; inClusterNodeId < nodesAllClusters[cluster_id].size(); ++inClusterNodeId) {
			//INFO("inClusterNodeId "<<inClusterNodeId)
			double minFromStart = M;
			for (int startNodeIndex = 0; startNodeIndex < nodesAllClusters[startIndex].size(); ++startNodeIndex) {
				//INFO_VAR(cluster_id);
				double dist = nodesAllDistances[startIndex][cluster_id][startNodeIndex][inClusterNodeId];
				if (dist < minFromStart) {
					minFromStart = dist;
				}
			}
			//INFO_VAR(minFromStart)
			double minToGoal = M;
			for (int goalNodeIndex = 0; goalNodeIndex < nodesAllClusters[goalIndex].size(); ++goalNodeIndex) {
				double dist = nodesAllDistances[cluster_id][goalIndex][inClusterNodeId][goalNodeIndex];
				if (dist < minToGoal) {
					minToGoal = dist;
				}
			}
			//INFO_VAR(minToGoal)
			double distanceNode = minFromStart + minToGoal;

			if (distanceNode <= this->budget) {
				reachableNodes_[varCluster].push_back(nodesForInsertion_[varCluster][inClusterNodeId]);
			}
		}
	}
	INFO("we have "<<reachableNodes_.size()<<" reachable nodes");
	INFO("getReachableNodes end");
	return reachableNodes_;
}

void VnsSopSolver::checkConsistency(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS) {
	if ( DEBUG_CHECK_CONSISTENCY) {
		INFO("checkConsistency begin");
		if (actualVNSGOPPath.getPathLength() > budget) {
			ERROR("inconsistent actualVNSGOPPath is over budget "<<actualVNSGOPPath.getPathLength()<<" > "<<budget);
			exit(1);
		}

		for (int var1 = 0; var1 < actualVNS.size(); ++var1) {
			if (var1 < actualVNSGOPPath.getNumTargets()) {
				if (actualVNS[var1] != actualVNSGOPPath.getTarget(var1)) {
					ERROR(
							"inconsistent actualVNS not same as actualVNSGOPPath at position"<<var1<<" ids are "<<actualVNS[var1]<<" and "<<actualVNSGOPPath.getTarget(var1));
					exit(1);
				}
			}
			for (int var2 = var1 + 1; var2 < actualVNS.size(); ++var2) {
				if (actualVNS[var1] == actualVNS[var2]) {
					ERROR("inconsistent actualVNS repeat "<<var1<<" and "<<var2);
					for (int nodeID = 0; nodeID < actualVNS.size(); ++nodeID) {
						INFO(nodeID<<" id "<<actualVNS[nodeID]);
					}
					exit(1);
				}
			}
		}
		INFO("checkConsistency end");
	}
}

std::string VnsSopSolver::getVersion(void) {
	return "OPFourPhaseHeuristic 0.1";
}

void VnsSopSolver::load(void) {
// nothing to load, structures are passed to the constructor
	INFO("load");
	int n = nodesAllClusters.size();
	if (canvas) {
		CoordsVector points;
		//INFO("BORDER " << BORDER);
		for (int clusterID = 0; clusterID < nodesAllClusters.size(); ++clusterID) {
			for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterID].size(); ++clusterNodeID) {
				GraphNode station = nodesAllClusters[clusterID][clusterNodeID];
				Coords coord_up(station.x + BORDER, station.y + BORDER);
				Coords coord_down(station.x - BORDER, station.y - BORDER);
				points.push_back(coord_up);
				points.push_back(coord_down);
			}
		}

		*canvas << canvas::AREA;
		//INFO("draw points");
		for (int var = 0; var < points.size(); ++var) {
			*canvas << points[var];
		}
		INFO("set to canvas");
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
				for (int clusterID = 0; clusterID < nodesAllClusters.size(); ++clusterID) {

					for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterID].size(); ++clusterNodeID) {
						Coords cord(nodesAllClusters[clusterID][clusterNodeID].x,
								nodesAllClusters[clusterID][clusterNodeID].y);
						*canvas << crl::gui::canvas::FILL_COLOR
								<< map.getColor((double) nodesAllClusters[clusterID][clusterNodeID].reward) << cord;

					}
				}

				//draw convex hull around cluster poinst
				for (int clusterid = 0; clusterid < nodesAllClusters.size(); ++clusterid) {
					PointConvexHull points_in_cluster[nodesAllClusters[clusterid].size()];
					for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterid].size(); ++clusterNodeID) {
						points_in_cluster[clusterNodeID] = PointConvexHull(nodesAllClusters[clusterid][clusterNodeID].x, nodesAllClusters[clusterid][clusterNodeID].y);
					}
					vector<PointConvexHull> hull = convex_hull::get_convex_hull(points_in_cluster,
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
				for (int clusterID = 0; clusterID < nodesAllClusters.size(); ++clusterID) {
					for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterID].size(); ++clusterNodeID) {

						//INFO(cord.x<<" ; "<<cord.y);
						Coords cord(nodesAllClusters[clusterID][clusterNodeID].x,
								nodesAllClusters[clusterID][clusterNodeID].y);
						*canvas << cord << canvas::END;
					}
				}
			}
		}

		canvas->redraw();
		//usleep(500000);
	} //end canvas
}

void VnsSopSolver::drawNeighborhoodPoints() {
	if (canvas) {
		//INFO("drawNeighborhoodPoints");
		//usleep(2000);
		*canvas << canvas::CLEAR << "neighborhoodp" << "neighborhoodp" << canvas::POINT;
		CShape neighborhoodPoint("blue", "blue", 1, 1);
		for (int nodeID = 0; nodeID < VnsSopPath::getAllSamples()->size(); ++nodeID) {
			//INFO_VAR(nodeID);
			single_cluster_samples &neighborhood = VnsSopPath::getAllSamples()->at(nodeID);
			const int neighborhoodSize = neighborhood.size();
			//INFO_VAR(neighborhoodSize);
			for (int neighID = 0; neighID < neighborhoodSize; ++neighID) {
				//INFO(neighID<<" draw neighborhood pos "<<neighborhood[neighID][0].node.x<<" "<<neighborhood[neighID][0].node.y);
				Coords cord(neighborhood[neighID].x, neighborhood[neighID].y);
				*canvas << neighborhoodPoint << cord;
			}
		}
		*canvas << canvas::END;
		canvas->redraw();
		//usleep(2000);
		//INFO("drawNeighborhoodPoints end");
	}
}

/// - private method -----------------------------------------------------------
void VnsSopSolver::drawPath(int usleepTime, VnsSopPath * toShow, const GraphNode & chainGeneratingNeighAngFrom) {
//INFO("drawPath begin");
//CoordsVector path;
//get_ring_path(step, path);
	if (canvas) {
		//INFO("clear path");
		*canvas << canvas::CLEAR << "path" << "path";
		//INFO("path cleared");

		std::vector<IndexSOP> returnedPath;
		if (toShow != NULL) {
			returnedPath = toShow->getPath(chainGeneratingNeighAngFrom);
		} else {
			returnedPath = this->tourVNSGOPPath.getPath(chainGeneratingNeighAngFrom);
		}

		//INFO("pathDubins size "<<pathDubins.size());

		//if(gopType == OP){
		CoordsVector coords = SOPLoader::getCoordPath(returnedPath, nodesAllClusters, gopType, config);
		CShape blackPoint("black", "black", 1, 4);
		CShape blackLine("black", "black", 2, 0);
		for (int var = 1; var < coords.size(); ++var) {
			*canvas << canvas::LINE << blackLine;
			*canvas << coords[var - 1].x << coords[var - 1].y;
			*canvas << coords[var].x << coords[var].y;
			*canvas << canvas::END;
		}

		*canvas << canvas::CLEAR << "pathpoint" << "pathpoint" << canvas::POINT;
		for (int var = 0; var < returnedPath.size(); ++var) {
			GraphNode gn = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex];
			Coords coord(gn.x, gn.y);
			*canvas << blackPoint << coord;
		}
		*canvas << canvas::END;

		canvas->redraw();
		//INFO("draw wait "<<usleepTime);
		//usleep(usleepTime);
		//usleep(200000);
		//drawNeighborhoodPoints();

	}
//INFO("drawPath end");
}

void VnsSopSolver::initialize(void) {

}

std::string VnsSopSolver::getRevision(void) {
	return "$Id$";
}

void VnsSopSolver::after_init(void) {

}

void VnsSopSolver::saveRewToFile(double reward, double length, std::string filename) {
//	std::cout << "saveToFile " << filename << std::endl << std::flush;
	std::ofstream out(filename);
	if (out.is_open()) {
		out << reward << " " << length << std::endl;
		out.close();
	} else {
		std::cerr << "Cannot open " << filename << std::endl;
	}
}

/// - protected method ---------------------------------------------------------
void VnsSopSolver::save(void) {
	std::string dir;
//updateResultRecordTimes(); //update timers as load and initilization is outside class
//DEBUG("LOAD_TIME_CPU: " << tLoad.cpuTime());
//DEBUG("INIT_TIME_CPU: " << tInit.cpuTime());
//DEBUG("SAVE_TIME_CPU: " << tSave.cpuTime());
	DEBUG("SOLVE_TIME_CPU: " << tSolve.cpuTime());
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

		std::vector<IndexSOP> finalPath = finalTourVNSGOPPath.getPath();
		for (int var = 0; var < finalPath.size(); ++var) {
			ofs << finalPath[var].clusterIndex << " " << finalPath[var].nodeIndex << std::endl;
			//ofs << pt.getStart().point.x << " " << pt.getStart().point.y << std::endl;
			//ofs << pt.getEnd().point.x << " " << pt.getEnd().point.y << std::endl;
		}

		crl::assert_io(ofs.good(), "Error occur during path saving");
		ofs.close();
	}

	std::string file2 = getOutputIterPath("gop.txt", dir);
	crl::assert_io(createDirectory(dir), "Can not create file in path'" + file2 + "'");
	std::ofstream ofs2(file2.c_str());
	crl::assert_io(ofs2.good(), "Cannot create path '" + file2 + "'");
	ofs2 << std::setprecision(14);

	std::vector<IndexSOP> finalPath = finalTourVNSGOPPath.getPath();
	int fps = finalPath.size();
	for (int var = 0; var < fps; ++var) {
		ofs2 << finalPath[var].clusterIndex << " " << finalPath[var].nodeIndex << std::endl;
	}

	crl::assert_io(ofs2.good(), "Error occur during path saving");
	ofs2.close();

	if (SAVE_TARGETS) {
		std::string file = getOutputIterPath(config.get<std::string>("targets"), dir);
		crl::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream ofs(file.c_str());
		crl::assert_io(ofs.good(), "Cannot create path '" + file + "'");
		ofs << std::setprecision(14);
		int S = nodesAllClusters.size();
		for (int var = 0; var < S; ++var) {
			int S2 = nodesAllClusters[var].size();
			for (int var2 = 0; var2 < S2; ++var2) {
				ofs << nodesAllClusters[var][var2].id << DD << nodesAllClusters[var][var2].x << DD
						<< nodesAllClusters[var][var2].y << DD << nodesAllClusters[var][var2].reward << DD
						<< nodesAllClusters[var][var2].cluster_id << std::endl;
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

void VnsSopSolver::release(void) {

}

void VnsSopSolver::visualize(void) {

}

void VnsSopSolver::defineResultLog(void) {
	static bool resultLogInitialized = false;
	if (!resultLogInitialized) {
		resultLog << result::newcol << "NAME";
		resultLog << result::newcol << "METHOD";
		resultLog << result::newcol << "CTIME";
		resultLog << result::newcol << "NUM_ITERS";
		resultLog << result::newcol << "BUDGET";
		resultLog << result::newcol << "REWARDS";
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

void VnsSopSolver::fillResultRecord(int numIters, double length, int numItersLastImprovement, long timeLastImprovement,
		std::vector<ImprovementLogRecord> improvementLog) {
	long t[3] = { 0, 0, 0 };

	tSolve.addTime(t);

	double final_reward = this->finalTourVNSGOPPath.getReward();

	samples_type samples = this->finalTourVNSGOPPath.getSamples();
	GraphNode dummy = GraphNode();
	std::vector<GraphNode> solution = this->finalTourVNSGOPPath.getPathNeighAngIds(dummy);

	SOPLoader::fixClusterNodeRenumbering(solution, startIndex, goalIndex, oldstartIndex, oldgoalIndex,
			deleted_cluster_id, ids_originally_from_one);

	std::stringstream tourNodes;
	std::stringstream tourClusters;
	std::stringstream tourRewards;
	//int full_precision = std::numeric_limits<double>::max_digits10;
	int Ss = solution.size();
	for (int var = 0; var < Ss; ++var) {
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
	int S = improvementLog.size();
	for (int var = 0; var < S; ++var) {
		if (var != 0) {
			tourImprovements << ",";
		}
		tourImprovements << improvementLog[var].timeMS << "|" << improvementLog[var].reward << "|"
				<< improvementLog[var].length;
	}

	double final_length = this->finalTourVNSGOPPath.getPathLength();

	resultLog << result::newrec << name << getMethod() << t[0] << numIters << this->budget << final_reward
			<< maximalRewardAll << final_length << numItersLastImprovement << timeLastImprovement
			<< maximal_calculation_time_MS << set_greedy_initial_solution << tourNodes.str() << tourClusters.str()
			<< tourImprovements.str() << lower_bound_above_budget_then_skip;

}

crl::CConfig & VnsSopSolver::getConfig(crl::CConfig & config) {
// basic config

	return config;
}

int VnsSopSolver::greaterThanGraphNodeReward(int gn1, int gn2, struct SortCompareVar<int> data) {
	samples_type * nodesClusters = (samples_type *) data.data;
//nodesClusters->at(gn1).at(__n))
//INFO("greaterThanGraphNodeReward");
	if (nodesClusters->at(gn1)[0].reward < nodesClusters->at(gn2)[0].reward) {
		return 1;
	} else if (nodesClusters->at(gn1)[0].reward > nodesClusters->at(gn2)[0].reward) {
		return -1;
	} else {
		return 0;
	}
}

void VnsSopSolver::exampleOneCluserMoveRandom(VnsSopPath &actualVNSGOPPath) {
	INFO("exampleInsertRandom");

	VnsSopPath copyAdd = VnsSopPath(startIndex, goalIndex, gopType);
	copyAdd.addPoint(5, 0);
	copyAdd.addPoint(4, 1);
	copyAdd.addPoint(6, 2);

	copyAdd.update();
	//double lengthBeforeAddCopy = copyAdd.getPathLength();
	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);
	saveSampled("python/orienteering/operator_example_paths/beforeOneCluserMoveRandom.txt", copyAdd);
//do it back
	copyAdd.addPoint(3, 1);
	copyAdd.update();
	//double lengthAfterAddCopy = copyAdd.getPathLength();
	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);
	saveSampled("python/orienteering/operator_example_paths/afterOneCluserMoveRandom.txt", copyAdd);

}

void VnsSopSolver::exampleOneCluserExchangeRandom(VnsSopPath &actualVNSGOPPath) {
	INFO("exampleExchangeRandom");

	VnsSopPath copyAdd = VnsSopPath(startIndex, goalIndex, gopType);
	copyAdd.addPoint(4, 0);
	copyAdd.addPoint(5, 1);
	copyAdd.addPoint(1, 2);
	copyAdd.addPoint(6, 3);

	copyAdd.update();
	INFO("ids before OneCluserExchange");
	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);
	saveSampled("python/orienteering/operator_example_paths/beforeOneCluserExchangeRandom.txt", copyAdd);

	copyAdd.removePoint(0);
	INFO("after remove")
	copyAdd.listIds();
	copyAdd.addPoint(1, 0);

	copyAdd.removePoint(2);
	copyAdd.addPoint(4, 2);

	copyAdd.update();
	INFO("ids after OneCluserExchange");
	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);
	saveSampled("python/orienteering/operator_example_paths/afterOneCluserExchangeRandom.txt", copyAdd);

}

void VnsSopSolver::examplePathMove(VnsSopPath &actualVNSGOPPath) {
	INFO("examplePathInsert");
	int exchangeFromStart = 0;
	int exchangeFromEnd = 1;
	int insertTo = 2;

	INFO_VAR(exchangeFromStart)
	INFO_VAR(exchangeFromEnd)
	INFO_VAR(insertTo)

	VnsSopPath copyAdd = actualVNSGOPPath;
	copyAdd.listIds();
	/*
	 drawPath(1000000, &copyAdd);
	 usleep(1000000);
	 bool insertUp = true;
	 std::vector<int> targets = copyAdd.getAllTargets();
	 if ( insertUp ) {
	 //insert up
	 std::vector<int> newVec(targets.begin() + exchangeFromStart, targets.begin() + exchangeFromEnd + 1);
	 if ( insertTo < copyAdd.getNumTargets() ) {
	 INFO("exchange add")
	 copyAdd.addPoint(newVec, insertTo);
	 copyAdd.listIds();
	 }
	 if ( exchangeFromStart < copyAdd.getNumTargets() ) {
	 INFO("exchange remove")
	 copyAdd.removePoint(exchangeFromStart, MIN(exchangeFromEnd, copyAdd.getNumTargets() - 1));
	 copyAdd.listIds();
	 }

	 } else {
	 //insert bellow
	 std::vector<int> newVec(targets.begin() + exchangeFromStart, targets.begin() + exchangeFromEnd + 1);

	 if ( exchangeFromStart < copyAdd.getNumTargets() ) {
	 copyAdd.removePoint(exchangeFromStart, MIN(exchangeFromEnd, copyAdd.getNumTargets() - 1));
	 }
	 if ( insertTo < copyAdd.getNumTargets() ) {
	 copyAdd.addPoint(newVec, insertTo);
	 }
	 }

	 copyAdd.update();
	 */

	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);

	saveSampled("python/orienteering/operator_example_paths/afterPathMoveRandom.txt", copyAdd);

	int temp = exchangeFromStart;
	exchangeFromStart = insertTo;
	exchangeFromEnd = exchangeFromStart + exchangeFromEnd - temp;
	insertTo = temp;

	INFO_VAR(exchangeFromStart)
	INFO_VAR(exchangeFromEnd)
	INFO_VAR(insertTo)

	bool insertUp = false;
	std::vector<int> targets = copyAdd.getAllTargets();
	if (insertUp) {
		//insert up
		std::vector<int> newVec(targets.begin() + exchangeFromStart, targets.begin() + exchangeFromEnd + 1);
		if (insertTo < copyAdd.getNumTargets()) {
			copyAdd.addPoint(newVec, insertTo);
			copyAdd.listIds();
		}
		if (exchangeFromStart < copyAdd.getNumTargets()) {
			copyAdd.removePoint(exchangeFromStart, MIN(exchangeFromEnd, copyAdd.getNumTargets() - 1));
			copyAdd.listIds();
		}

	} else {
		//insert bellow
		std::vector<int> newVec(targets.begin() + exchangeFromStart, targets.begin() + exchangeFromEnd + 1);

		if (exchangeFromStart < copyAdd.getNumTargets()) {
			copyAdd.removePoint(exchangeFromStart, MIN(exchangeFromEnd, copyAdd.getNumTargets() - 1));
		}
		if (insertTo < copyAdd.getNumTargets()) {
			copyAdd.addPoint(newVec, insertTo);
		}
	}

	copyAdd.update();

	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);

	saveSampled("python/orienteering/operator_example_paths/beforePathMoveRandom.txt", copyAdd);
}

void VnsSopSolver::examplePathExchange(VnsSopPath &actualVNSGOPPath) {
	INFO("examplePathExchange");

	VnsSopPath copyAdd = actualVNSGOPPath;
	copyAdd.listIds();

	drawPath(1000000, &copyAdd);
	usleep(1000000);

	saveSampled("python/orienteering/operator_example_paths/afterPathExchangeRandom.txt", copyAdd);

	int exchangeFromStart = 0;
	int exchangeFromEnd = 1;
	int exchangeToStart = 3;
	int exchangeToEnd = 3;

	bool exchangeUp = true;
	std::vector<int> targets = copyAdd.getAllTargets();
	if (exchangeUp) {
		std::vector<int> newVecFrom(targets.begin() + exchangeFromStart, targets.begin() + exchangeFromEnd + 1); //is below
		std::vector<int> newVecTo(targets.begin() + exchangeToStart, targets.begin() + exchangeToEnd + 1); //is above

		if (exchangeToStart < copyAdd.getNumTargets()) {
			copyAdd.removePoint(exchangeToStart, MIN(exchangeToEnd, copyAdd.getNumTargets() - 1));
			INFO("after remove")
			copyAdd.listIds();
			copyAdd.addPoint(newVecFrom, exchangeToStart);
			INFO("after add")
			copyAdd.listIds();
		}
		//INFO("add newVecTo");
		if (exchangeFromStart < copyAdd.getNumTargets()) {
			copyAdd.removePoint(exchangeFromStart, MIN(exchangeFromEnd, copyAdd.getNumTargets() - 1));
			INFO("after remove 2")
			copyAdd.listIds();
			copyAdd.addPoint(newVecTo, exchangeFromStart);
			INFO("after add 2")
			copyAdd.listIds();
		}

	} else {
		std::vector<int> newVecFrom(targets.begin() + exchangeFromStart, targets.begin() + exchangeFromEnd + 1);
		std::vector<int> newVecTo(targets.begin() + exchangeToStart, targets.begin() + exchangeToEnd + 1);

		if (exchangeFromStart < copyAdd.getNumTargets()) {
			copyAdd.removePoint(exchangeFromStart, MIN(exchangeFromEnd, copyAdd.getNumTargets() - 1));
			copyAdd.addPoint(newVecTo, exchangeFromStart);
			INFO("after add part ")
			copyAdd.listIds();
		}

		//INFO("add newVecFrom");
		if (exchangeToStart < copyAdd.getNumTargets()) {
			copyAdd.removePoint(exchangeToStart + 1, MIN(exchangeToEnd, copyAdd.getNumTargets() - 1) + 1);
			INFO("after remove part ")
			copyAdd.listIds();
			copyAdd.addPoint(newVecFrom, exchangeToStart + 1);
			INFO("after add  rest ")
			copyAdd.listIds();
		}

	}

	copyAdd.update();

	copyAdd.listIds();
	drawPath(1000000, &copyAdd);
	usleep(1000000);
	saveSampled("python/orienteering/operator_example_paths/beforePathExchangeRandom.txt", copyAdd);

}

void VnsSopSolver::saveSampled(std::string filename, VnsSopPath &actualVNSGOPPath) {
	std::vector<IndexSOP> idPath = actualVNSGOPPath.getPath();
	CoordsVector coords = SOPLoader::getCoordPath(idPath, nodesAllClusters, gopType, config);
	std::string file = filename;
	std::ofstream ofs(filename.c_str());
	crl::assert_io(ofs.good(), "Cannot create path '" + filename + "'");
	ofs << std::setprecision(14);
	int S = coords.size();
	for (int var = 0; var < S; ++var) {
		ofs << coords[var].x << DD << coords[var].y << DD
				<< nodesAllClusters[idPath[var].clusterIndex][idPath[var].nodeIndex].id << DD
				<< nodesAllClusters[idPath[var].clusterIndex][idPath[var].nodeIndex].cluster_id << std::endl;
	}
	crl::assert_io(ofs.good(), "Error occur during path saving");
	ofs.close();
}

void VnsSopSolver::saveTextToFile(std::string filename, std::string text) {
	std::string file = filename;
	std::ofstream ofs(filename.c_str());
	crl::assert_io(ofs.good(), "Cannot create path '" + filename + "'");
	ofs << text;
	crl::assert_io(ofs.good(), "Error occur during path saving");
	ofs.close();
}

