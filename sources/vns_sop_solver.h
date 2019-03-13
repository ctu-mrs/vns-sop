/*
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_VNSGOP_VNSGOP_H_
#define SRC_VNSGOP_VNSGOP_H_

#include <vector>
#include <list>
#include "crl/logging.h"
#include <cfloat>
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
#include <vector>

#include <limits>

#include <algorithm>    // std::random_shuffle

#include "sop_loader.h"
#include "vns_sop_path.h"
#include "convexhull.h"
#include "csort.h"
#include "dataset_loader_op.h"
#include "heuristic_types.h"
#include "math_common.h"

namespace op {

class VnsSopSolver: public crl::CAlgorithm {
	typedef crl::CAlgorithm Base;
	typedef std::vector<int> IntVector;

public:
	VnsSopSolver(crl::CConfig& config, const std::string& problemFile);
	virtual ~VnsSopSolver();
	//Tour find(DatasetOP dataset);
	//Tour find(std::vector<GraphNode> nodes, double budget, int startIndex_, int goalIndex_);

	static int greaterThanDistanceStartGoal(GraphNode gn1, GraphNode gn2, struct SortCompareVar<GraphNode> data);
	static int greaterThanGraphNodeReward(int gn1, int gn2, struct SortCompareVar<int> data);
	//CAlgorithm functions
	void solve(void);
	std::string getRevision(void);
	static crl::CConfig& getConfig(crl::CConfig& config);
	static std::string getName(void) {
		return "vns_sop";
	}
	std::string getMethod(void) {
		return getName();
	}

protected:
	//CAlgorithm functions
	std::string getVersion(void);
	void load(void);
	void initialize(void);
	void after_init(void);
	void iterate(int step);
	void save(void);
	void release(void);
	void visualize(void);
	void defineResultLog(void);
	void fillResultRecord(int numIters, double length, int numItersLastImprovement, long timeLastImprovement,
			std::vector<ImprovementLogRecord> improvementLog);
	CoordsVector getCoordPath(std::vector<IndexSOP> idPaths);
	void drawPath(int usleepTime = 0, VnsSopPath * toShow = NULL, const GraphNode & chainGeneratingNeighAngFrom =
			GraphNode());
	void drawNeighborhoodPoints();
	const double BORDER;
	const bool SAVE_RESULTS;
	const bool SAVE_INFO;
	const bool SAVE_SETTINGS;
	const bool SAVE_SAMPLED_PATH;
	const bool SAVE_TARGETS;

private:
	void calcClusterDistancesBounds();
	samples_type getReachableNodes(samples_type nodesForInsertion_);
	void checkConsistency(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS);
	void checkLengths(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, std::string text = std::string(""));

	void shake(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
			GraphNode & chainGeneratingNeighAngEnd);
	bool randomLocalSearch(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
			GraphNode & chainGeneratingNeighAngEnd);

	//methods for local search
	void onePointMove(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS);

	void fitDOPtoBudget(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS);

	void generateInitialSolution(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS);
	void greedy_insertion(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS);


	bool insertRandom(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes);
	bool exchangeRandom(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes);
	bool check_try_operation(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int targetIDFrom, int targetIDTo, double lengthAfterMove,
			bool relocate);

	void pathInsert(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int k);
	void pathExchange(VnsSopPath &actualVNSGOPPath, std::vector<int> &actualVNS, int k);


	//bool twoopt(VNSGOPPath &actualVNSGOPPath, std::vector<GraphNode> &actualVNS);

	double getTourLength(std::vector<GraphNode> tour);
	double getTourReward(std::vector<GraphNode> tour);

	//just for generating examples
	void exampleOneCluserMoveRandom(VnsSopPath &actualVNSGOPPath);
	void exampleOneCluserExchangeRandom(VnsSopPath &actualVNSGOPPath);
	void examplePathMove(VnsSopPath &actualVNSGOPPath);
	void examplePathExchange(VnsSopPath &actualVNSGOPPath);

	//void savePaths(VNSGOPPath * toShow);
	//void saveToFile(std::vector<State> &toSave, std::string filename);
	void saveRewToFile(double reward, double length, std::string filename);

	void saveSampled(std::string filename, VnsSopPath &actualVNSGOPPath);
	void saveTextToFile(std::string filename, std::string text);

	static int compareByDistanceFromStart();

	//just as saving id
	int path_var;
	GOPTYPE gopType;

	//default params
	samples_type nodesAllClusters;
	samples_distances_type nodesAllDistances;

	std::vector<std::vector<double>> clustersMinDistances;
	std::vector<std::vector<double>> clustersMaxDistances;

	std::string problem_file;

	samples_type nodesForInsertion;

	samples_type availableNodes;
	int numAvailableNodes;
	double budget;
	int startIndex;
	int goalIndex;
	//int K;
	//int I;

	int oldgoalIndex;
	int oldstartIndex;
	int deleted_cluster_id;
	bool ids_originally_from_one;

	double maximalRewardAll;
	double initial_reward;

	VnsSopPath tourVNSGOPPath, finalTourVNSGOPPath;
	std::vector<int> vnsVector;

	double maximal_calculation_time_sec;
	long maximal_calculation_time_MS;

	bool lower_bound_above_budget_then_skip;

	bool generate_examples_of_operations;
	bool set_greedy_initial_solution;
	bool useRVNS;
	int numIterations;
	int numIterationsUnimproved;

	//Tour path_op;
	//std::vector<Tour> path_nop;
	//double record;
	//double deviation;
	double p = 10;
	//Tour bestTour;


	std::vector<int> deletedIds;

	bool stop;

	double maxOverGuessedTry;
	double avgsaved;
	double numTestedAvg;
};

}
#endif /* SRC_VNSGOP_VNSGOP_H_ */
