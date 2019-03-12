/*
 * GOPLoader.h
 *
 *  Created on: Dec 11, 2017
 *      Author: penicrob
 */

#ifndef LP_GOP_GOPLOADER_H_
#define LP_GOP_GOPLOADER_H_

#include "dataset_loader_op.h"
#include "dataset_loader_sop.h"
#include "heuristic_types.h"
#include "math_common.h"

class SOPLoader {
public:
	SOPLoader();
	virtual ~SOPLoader();

	static CoordsVector getCoordPath(std::vector<IndexSOP>& idPaths, samples_type & nodesAllClusters, GOPTYPE gopType, crl::CConfig& config);

	static SOP_Prolem getSOPDefinition(crl::CConfig& config, const std::string& problemFile, bool change_end_index_to_last = false);
	static void fixClusterNodeRenumbering(std::vector<GraphNode>& finalPath, int startIndex, int goalIndex, int oldstartIndex, int oldgoalIndex,
			int deleted_cluster_id, bool numbered_from_one);
	//for classical OP
	static SOP_Prolem getOP(std::vector<GraphNode> opNodes);
	static CoordsVector getOPPath(std::vector<IndexSOP> &gopPath, samples_type & nodesAllClusters);

	//for OP with neighborhood (OPN)
	static SOP_Prolem getOPN(std::vector<GraphNode> opNodes, int startIndex_, int goalIndex_, double neigh_radius,
			int neighborhood_resolution);
	static CoordsVector getOPNPath(std::vector<IndexSOP> &gopPath, samples_type & nodesAllClusters);

	static SOP_Prolem getSOPFromWeightMatrix(std::vector<ClusterSOP> clusters, std::vector<std::vector<double>> distance_matrix, int startIndex_,
			int goalIndex_);
	static SOP_Prolem getSOPFromNodes(std::vector<ClusterSOP> clusters, std::vector<GraphNode> nodes, int startIndex_, int goalIndex_,
			TSP_EDGE_WEIGHT_TYPE edge_weight_type_);

	static void printAllClustersNodes(samples_type samples);
};

#endif /* LP_GOP_GOPLOADER_H_ */
