/*
 * HeuristicTypes.h
 *
 *  Created on: Feb 22, 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_FOURPHASEHEURISTIC_HEURISTICTYPES_H_
#define SRC_FOURPHASEHEURISTIC_HEURISTICTYPES_H_

#include <vector>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>
#include <limits>
#include <iomanip>
#include "crl/logging.h"
#include "coords.h"
#include <crl/config.h>

#include "my_defines.h"

enum TSP_EDGE_WEIGHT_TYPE {
	EUC_2D, CEIL_2D, EXPLICIT
};
#define TSP_EDGE_EUC_2D_STR "EUC_2D"
#define TSP_EDGE_CEIL_2D_STR "CEIL_2D"
#define TSP_EDGE_CEIL_EXPLICIT_STR "EXPLICIT"

enum GOPTYPE {
	OP, OPN, DOP, DOPN, SOP
};

typedef struct GraphNode {

	GraphNode() {
		x = NAN;
		y = NAN;
		reward = 0;
		id = 0;
		cluster_id = -1;
	}

	GraphNode(double x_, double y_, double price_, unsigned int id_, int cluster_id_) {
		x = x_;
		y = y_;
		reward = price_;
		id = id_;
		cluster_id = cluster_id_;
	}

	std::vector<double> toVector() {
		std::vector<double> vector;
		vector.push_back(x);
		vector.push_back(y);
		vector.push_back(reward);
		return vector;
	}

	/*
	Point toPoint() {
		return Point(this->x, this->y);
	}
	*/

	double distanceTo(GraphNode gn1) {
		if ( !std::isnan(x) && !std::isnan(y) ) {
			double diffx = gn1.x - this->x;
			double diffy = gn1.y - this->y;
			return sqrt(diffx * diffx + diffy * diffy);
		} else {
			return NAN;
		}
	}

	double x;
	double y;
	double reward;
	unsigned int id;
	int cluster_id;
} GraphNode;

typedef struct Tour {
	Tour() {
		this->reward = 0;
		this->length = 0;
	}
	Tour(double reward_, double length_, std::vector<GraphNode> tour_) {
		this->reward = reward_;
		this->length = length_;
		this->tour = tour_;
	}
	double reward;
	double length;
	std::vector<GraphNode> tour;
	/*
	std::vector<Point> getPointTour() {
		std::vector<Point> pointTour;
		int S = tour.size();
		for ( int var = 0 ; var < S ; ++var ) {
			pointTour.push_back(tour[var].toPoint());
		}
		return pointTour;
	}
	*/
} Tour;

typedef struct DatasetOP {
	std::vector<GraphNode> graph;
	double Tmax; //= available time budget per path
	unsigned int P; //= number of paths (=1)
	unsigned int startID;
	unsigned int goalID;
} datasetOP;

typedef struct ClusterSOP {
	int cluster_id;
	double reward;
	std::vector<int> nodeIDs;
} ClusterSOP;

typedef struct DatasetSOP {
	std::vector<ClusterSOP> clusters;
	std::vector<GraphNode> nodesAll;
	std::vector<std::vector<double>> distance_matrix;
	bool is_with_nodes;
	std::string name;
	std::string type;
	std::string comment;
	TSP_EDGE_WEIGHT_TYPE edge_weight_type;
	int dimension;
	int sets;
	unsigned int startID;
	unsigned int goalID;
	double Tmax; //= available time budget per path
} DatasetSOP;

typedef struct SolutionGTSP {
	std::vector<int> node_ids;
	double length;
	int num_nodes;
} SolutionGTSP;

typedef struct InsertionHistoryRecord {
	InsertionHistoryRecord(std::vector<GraphNode> graph_, GraphNode insertionNode_) {
		this->graph = graph_;
		this->insertionNode = insertionNode_;
	}
	std::vector<GraphNode> graph;
	GraphNode insertionNode;
} InsertionHistoryRecord;

typedef struct TimedImprovement {
	TimedImprovement(long timeMS_, double length_, double reward_, int iteration_) {
		this->timeMS = timeMS_;
		this->length = length_;
		this->reward = reward_;
		this->iteration = iteration_;
	}
	long timeMS;
	double length;
	double reward;
	int iteration;
} TimedImprovement;

typedef struct StartGoalNodes {
	GraphNode start;
	GraphNode goal;
} StartGoalNodes;

typedef struct ImprovementLogRecord {
	double length;
	double reward;
	long timeMS;
} ImprovementLogRecord;

bool operator==(const GraphNode& lhs, const GraphNode& rhs);
std::ostream& operator <<(std::ostream &o, GraphNode &p);
bool operator>(GraphNode &dt1, GraphNode &dt2);
bool operator<(GraphNode &dt1, GraphNode &dt2);

typedef struct ClusterNodeDist {
	ClusterNodeDist() {
		idClusterNode = -1;
		distance = 0;
	}
	ClusterNodeDist(int idClusterNode_, double distance_) {
		idClusterNode = idClusterNode_;
		distance = distance_;
	}
	int idClusterNode;
	double distance;
} ClusterNodeDist;

typedef std::vector<std::vector<ClusterNodeDist>> shortest_matrix;
typedef std::vector<ClusterNodeDist> single_cluster_shortest_matrix;

typedef std::vector<std::vector<GraphNode>> samples_type;
typedef std::vector<GraphNode> single_cluster_samples;

typedef std::vector<std::vector<std::vector<std::vector<double>>>> samples_distances_type;
typedef std::vector<std::vector<double>> cluster_bound_distances;

typedef struct SOP_Prolem {
	GOPTYPE gop_type;
	std::string name;
	double budget;
	int startIndex;
	int goalIndex;
	int oldGoalIndex = -1;
	int oldStartIndex = -1;
	int deleted_cluster_id = -1;
	bool ids_originally_from_one;
	samples_type samples;
	samples_distances_type distances;
} GOP_Prolem;

/*
 inline std::string trim(std::string& str) {
 str.erase(0, str.find_first_not_of(' '));       //prefixing spaces
 str.erase(str.find_last_not_of(' ') + 1);         //surfixing spaces
 return str;
 }

 inline std::string trim(std::string& str) {
 if (str.size() == 0) {
 return str;
 }
 size_t first = str.find_first_not_of(' ');
 size_t last = str.find_last_not_of(' ');
 return str.substr(first, (last - first + 1));
 }
 */

inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
	s.erase(0, s.find_first_not_of(t));
	s.erase(s.find_last_not_of(t) + 1);
	return s;
}

inline std::vector<std::string> tokenize(std::string s, std::string delimiter) {
	//INFO("tokenize:\""<<s<<"\" using delimiter :\""<<delimiter<<"\"")
	std::vector<std::string> tokens;
	size_t pos = 0;
	std::string token;
	while ( (pos = s.find(delimiter)) != std::string::npos ) {
		token = s.substr(0, pos);
		//INFO("add token \""<<token<<"\"")
		tokens.push_back(token);
		s.erase(0, pos + delimiter.length());
		//INFO("s after erase\""<<s<<"\"")
	}
	if ( s.length() > 0 ) {
		//INFO("add token \""<<s<<"\"")
		tokens.push_back(s);
	}
	return tokens;
}

inline bool replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if ( start_pos == std::string::npos )
	return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

TSP_EDGE_WEIGHT_TYPE parse_tsp_edge_type(std::string edge_type_string);

class HeuristicTypes {
public:
	HeuristicTypes();
	virtual ~HeuristicTypes();
};

#endif /* SRC_FOURPHASEHEURISTIC_HEURISTICTYPES_H_ */
