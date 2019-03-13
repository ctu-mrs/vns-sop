/*
 * VNSDOP.h
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_LP_OP_H_
#define SRC_LP_OP_H_

#include <vector>
#include <list>
#include "crl/logging.h"
#include <cfloat>
#include <limits>

#include "coords.h"

#include <boost/foreach.hpp>

#include <crl/config.h>
#include <crl/alg/algorithm.h>
#include <crl/loader/text_format_loader.h>
#include "crl/random.h"
#include "crl/timerN.h"
#include "crl/perf_timer.h"
#include "crl/logging.h"
#include "crl/stringconversions.h"
#include "crl/loader/nodeloader.h"
#include "crl/alg/text_result_log.h"
#include "crl/file_utils.h"
#include "crl/assert.h"

#include "crl/gui/shapes.h"
#include "crl/gui/colormap.h"

#include "crl/gui/gui.h"
#include "canvasview_coords.h"

#include <unordered_set>

#include <ilcplex/ilocplex.h>

#include "my_defines.h"
#include "sop_loader.h"
#include "convexhull.h"
#include "dataset_loader_op.h"
#include "ilp_sop_definition.h"
#include "math_common.h"

namespace op {

class IlpSopSolver: public crl::CAlgorithm {
	typedef crl::CAlgorithm Base;
	typedef std::vector<int> IntVector;

public:
	IlpSopSolver(crl::CConfig& config, const std::string& problemFile);
	virtual ~IlpSopSolver();
	//Tour find(DatasetOP dataset);
	//Tour find(std::vector<GraphNode> nodes, double budget, int startIndex_, int goalIndex_);

	//std::vector<State> getPathSampled(std::vector<GraphNode> finalPath, double sampled_path_distance);
	//CAlgorithm functions
	void solve(void);
	std::string getRevision(void);
	static crl::CConfig& getConfig(crl::CConfig& config);
	static std::string getName(void) {
		return "ilp_sop";
	}
	std::string getMethod(void) {
		return getName();
	}
	void drawPath(int usleepTime = 0, std::vector<IndexSOP> * toShow = NULL);

	void drawNeighborhoodPoints();

	std::vector<IndexSOP> getGreedySolution();
	protected:
	//CAlgorithm functions
	std::string getVersion(void);
	void load(void);
	void initialize(void);
	void after_init(void);
	void iterate(int step);

	void my_problem_definition(IloEnv & env, IloModel & model);

	void save(void);
	void release(void);
	void visualize(void);
	void defineResultLog(void);
	//void fillResultRecord(int numIters, double length, double reward, int optimal_solution);
	void fillResultRecord(int numIters, double length, double reward, int optimal_solution, double gap_prec, int numItersLastImprovement, long timeLastImprovement,
			std::vector<ImprovementLogRecord> improvementLog);
	static std::vector<GraphNode> parseSolutionVectorIncumbent(IloEnv& env, IloNumVarMatrix2D& y, IloNumVarMatrix4D& x);

	const double BORDER;
	const bool SAVE_RESULTS;
	const bool SAVE_INFO;
	const bool SAVE_SETTINGS;
	const bool SAVE_SAMPLED_PATH;
	const bool SAVE_TARGETS;

private:
	GOPTYPE gopType;

	bool draw_cluster_points;

	SOP_Prolem getOP(std::vector<GraphNode> opNodes);
	SOP_Prolem getOPN(std::vector<GraphNode> opNodes, int startIndex_, int goalIndex_, double neigh_radius, int neighborhood_resolution);

	double getTourLength(std::vector<IndexSOP> indexedSolution);
	//double getTourLength(std::vector<GraphNode> tour);
	double getTourReward(std::vector<GraphNode> tour);

	std::vector<std::vector<GraphNode>> initialPaths;
	std::vector<std::vector<GraphNode>> nodesForInsertionInitialPaths;

	static int compareByDistanceFromStart();

	//default params
	//std::vector<GraphNode> nodesAll;
	std::vector<std::vector<std::vector<std::vector<double>>>> allDistances;
	std::vector<std::vector<GraphNode>> nodesAllClusters;

	std::string problem_file;

	double budget;
	int startIndex;
	int goalIndex;

	//original goal index in problem definition
	int oldgoalIndex;
	int oldstartIndex;
	int deleted_cluster_id;
	bool ids_originally_from_one;

	double maximalRewardAll;

	std::vector<IndexSOP> finalTourOP;

	double p = 10;

	int num_threads;
	int maximal_memory_MB;
	int maximal_calculation_time_sec;

	bool online_subtour_elimination;
	bool adjust_milp_priorities;
	bool set_greedy_initial_solution;


	std::vector<GraphNode> reachableNodes;
	std::vector<GraphNode> nodesInEllipseWithStartGoal;

	std::vector<int> deletedIds;

	bool stop;

};

}
#endif /* SRC_LP_OP_H_ */
