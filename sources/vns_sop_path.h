/*
 *  Created on: Apr 30, 2016
 *      Author: Robert Penicka, Petr Vana
 */

#pragma once

#include "comrob_lite/algorithm.h"
#include "comrob_lite/logging.h"
#include <tuple>
#include <algorithm>

#include "heuristic_types.h"
#include "math_common.h"
#include "my_defines.h"

namespace op {

// Class for the Dubins Orienteering Problem
// There is are two fixed positions - start and end
// Departure and termination headings are not given
// unlimited number of points can be inserted between start and end
class VnsSopPath {
public:
	VnsSopPath() {

	}

	VnsSopPath(int startClusterIdx_, int endClusterIdx_, GOPTYPE gop_type);

	static void setSamplesDistances(samples_type nodesAllClusters_, samples_distances_type distances_, cluster_bound_distances clustersMinDistances_,
			cluster_bound_distances clustersMaxDistances_);
	static samples_type *getAllSamples();

	std::vector<int> getAllTargets();

	virtual ~VnsSopPath();

	// returns number of Dubins maneuvers (targets.size() + 1)
	int getSize() const;

	// returns all Dubins maneuver in the shortest found path
	std::vector<IndexSOP> getPath(const GraphNode & chainGeneratingNeighAngEnd = GraphNode()) const;

	single_cluster_samples getPathNeighAngIds(GraphNode & chainGeneratingNeighAngEnd, bool fillchainGenNA = false) const;

	// returns only the distance of the shortest found path
	// it schould be slightly faster than using getPath() and sum the distance
	double getPathLength(const GraphNode & chainGeneratingNeighAngFrom = GraphNode()) const;
	double getPathLengthRear(const GraphNode & chainGeneratingNeighAngFrom = GraphNode()) const;
	double getPathLengthCalculate(const GraphNode & chainGeneratingNeighAngFrom = GraphNode()) const;
	double getPathLengthCalculateRear(const GraphNode & chainGeneratingNeighAngStart = GraphNode()) const;

	// add new target to specific position
	// example: addPoint(newPoint, 0)
	// before:  start > t1 > end
	// after:   start > newPoint > t1 > end
	void addPoint(int clusterId, int idx);

	// add new targets to specific position
	// example: addPoint(points, 0)
	// before:  start > t1 > end
	// after:   start > points[0] >  points[last] > t1 > end
	void addPoint(std::vector<int> clusterIds, int idx);

	// add point to the best position
	void addPoint(int clusterId);

	// remove one target with specific index
	// idx is in intervals [0, getSize()-2] = [0, getNumTargets()-1] = [0, targets.size()]
	void removePoint(int idx);

	// remove targets starting with idxStart and ending with idxStop
	void removePoint(int idxStart, int idxStop);

	//lower bound of tryToAdd
	double tryToAddLowerBound(int cluster_id, int idx) const;
	// same as addPoint but it does not insert the point
	// idx is in intervals [0, getSize()] = [0, targets.size()+1]
	// return length of new path
	double tryToAdd(int cluster_id, int idx) const;

	double tryToAdd(std::vector<int> p, int idx) const;

	//lowerbound estimate of the tryToReplace operation
	double tryToReplaceLowerBound(int cluster_id, int idx, int numToReplace = 1) const;
	//replaces single target at idx with GraphNode p
	double tryToReplace(int clusterId, int idx, int numToReplace = 1) const;

	//replaces single target at idx with vector of targets p
	double tryToReplace(std::vector<int> clusterIds, int idx) const;

	//lower bound of tryToExchange operation
	double tryToExchangeLowerBound(int idx1, int idx2);
	//returns length of path with exchanged idx1 and idx2
	double tryToExchange(int idx1, int idx2);

	//lower bond on try to move
	double tryToMoveLowerBound(int idxWhatToMove, int idxWhereToMove);

	//returns length of path with moved idxWhatToMove to idxWhereToMove
	double tryToMove(int idxWhatToMove, int idxWhereToMove);

	// same as removePoint but it does not remove the point
	// idx is in intervals [0, getSize()-2] = [0, getNumTargets()-1] = [0, targets.size()]
	// return length of new path
	double tryToRemove(int idx) const;

	double tryToRemove(int idxStart, int idxStop) const;

	// remove the worst point
	void removeOne();

	// test 2 opt operation between idx1 and idx2
	// idxFrom and idxTo are in intervals [0, getNumTargets()-1] = [0, targets.size()-1]
	// the path idxFrom - idxTo is inserted in reverse mode
	// return length of new path
	// reverse path between idx1+1 and idx2
	double tryTwoOpt(int idxFrom, int idxTo);

	//performs the two opt
	// idxFrom and idxTo are in intervals [0, getNumTargets()-1] = [0, targets.size()-1]
	// the path idxFrom - idxTo is inserted in reverse mode
	void twoOpt(int idxFrom, int idxTo);

	//return number of targets - number of nodes between start a and end
	int getNumTargets();

	//returns target at position idx - idx in intervals [0, targets.size-1]
	int getTarget(int idx);

	//return target subvector
	std::vector<int> getTargets(int idxFrom, int idxTo);

	//return samples
	samples_type getSamples();
	samples_type& getSamplesRef();
	shortest_matrix & getShortestRef();
	shortest_matrix & getShortestBackRef();
	//return number of samples - number of nodes in path totally = with start and end
	int getNumSamples();

	void checkShortestConsistency();
	void checkSameSamples(std::string text = std::string(""));

	//prints out ids of nodes
	void listIds();
	void listNodeIds();
	void listRewads();
	void printShortestDistances();

	std::vector<int>::iterator targetsBegin();
	std::vector<int>::iterator targetsEnd();

	void update();

	double getReward();
	static samples_distances_type allDistances;
	static samples_type allSamples;
	static cluster_bound_distances clustersMinDistances;
	static cluster_bound_distances clustersMaxDistances;

protected:

	//static void generateSamples(single_cluster_samples & samples, std::vector<GraphNode> p, int clusterID, int startClusterIdx_, int endClusterIdx_);

	void updateAfterInsert(int idxStart, int idxEnd);
	void updateAfterRemove(int idxStart, int idxEnd);

	int startClusterIdx;
	int endClusterIdx;

	std::vector<int> targets;

	// all samples
	// [1st index] - choose point/target
	// [2nd index] - choose neighborhood point in target
	// [3nd index] - choose state (sample) for this neighborhood
	samples_type samples;

	// find all shortest paths
	// matrix:
	//    1) to cluster (layer)
	//    2) to cluster node (layer)
	shortest_matrix shortest;

	// find all shortest paths - backward path
	// matrix:
	//    1) to cluster (layer)
	//    2) to cluster node (layer)
	shortest_matrix shortest_back;

	GOPTYPE gop_type;

};

}
