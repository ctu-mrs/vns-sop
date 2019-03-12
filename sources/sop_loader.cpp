/*
 * GOPLoader.cpp
 *
 *  Created on: Dec 11, 2017
 *      Author: penicrob
 */

#include "sop_loader.h"
using crl::logger;

SOPLoader::SOPLoader() {
}

SOPLoader::~SOPLoader() {
}

CoordsVector SOPLoader::getCoordPath(std::vector<IndexSOP>& idPaths, samples_type & nodesAllClusters, GOPTYPE gopType,
		crl::CConfig& config) {
	CoordsVector coords;
	//INFO("getCoordPath begin");
	switch (gopType) {
	case OP:
	case SOP: {
		coords = getOPPath(idPaths, nodesAllClusters);
		break;
	}
	case OPN: {
		coords = getOPNPath(idPaths, nodesAllClusters);
		break;
	}
	default: {
		INFO("draw path not implemented for gop type "<<gopType);
		break;
	}
	}
	//INFO("getCoordPath end");
	return coords;
}

SOP_Prolem SOPLoader::getSOPDefinition(crl::CConfig& config, const std::string& problemFile,
		bool change_end_index_to_last) {
	SOP_Prolem problem;
	std::string sop_type = config.get<std::string>("sop-type");
	sop_type = trim(sop_type);
	INFO("compare sop-type:"<<sop_type);
	if (sop_type.compare("opn") == 0 || sop_type.compare("dop") == 0 || sop_type.compare("dopn") == 0
			|| sop_type.compare("op") == 0) {
		INFO("sop type is op|opn|dop|dopn");
		DatasetOP loadedDataset = DatasetLoaderOP::loadDataset(problemFile);

		INFO(
				"testing OP dataset with " << loadedDataset.graph.size() << " nodes, " << loadedDataset.Tmax << " max time budget and " << loadedDataset.P << " paths to find");
		INFO("use budget " << loadedDataset.Tmax);
		INFO("use startIndex " << loadedDataset.startID);
		INFO("use goalIndex " << loadedDataset.goalID);
		problem.name = config.get<std::string>("name");
		std::vector<GraphNode> nodesAll = loadedDataset.graph;

		problem.budget = loadedDataset.Tmax;
		problem.startIndex = loadedDataset.startID;
		problem.goalIndex = loadedDataset.goalID;

		if (change_end_index_to_last) {
			INFO("changing the index of the goal to be the last ...... "<<problem.goalIndex);
			problem.oldGoalIndex = problem.goalIndex;
			GraphNode endNode = nodesAll[problem.goalIndex];
			GraphNode lastNode = nodesAll[nodesAll.size() - 1];
			nodesAll[problem.oldGoalIndex] = lastNode;
			nodesAll[problem.oldGoalIndex].id = problem.oldGoalIndex;
			problem.goalIndex = nodesAll.size() - 1;
			nodesAll[problem.goalIndex] = endNode;
			nodesAll[problem.goalIndex].id = problem.goalIndex;

			INFO("changing the index of the start to be the first ...... ");
			problem.oldStartIndex = problem.startIndex;
			GraphNode startNode = nodesAll[problem.startIndex];
			GraphNode firstNode = nodesAll[0];
			nodesAll[problem.oldStartIndex] = firstNode;
			nodesAll[problem.oldStartIndex].id = problem.oldStartIndex;
			problem.startIndex = 0;
			nodesAll[problem.startIndex] = startNode;
			nodesAll[problem.startIndex].id = problem.startIndex;
		}

		if (sop_type.compare("opn") == 0) {
			INFO("defined GOP as OPN");
			problem.gop_type = OPN;
			double radius = config.get<double>("neighborhood-radius");
			int resolution = config.get<int>("neighborhood-resolution");
			INFO("using neighborhood-radius:"<<radius);
			INFO("using neighborhood-resolution:"<<resolution);
			SOP_Prolem problemOPN = getOPN(nodesAll, problem.startIndex, problem.goalIndex, radius, resolution);
			problem.samples = problemOPN.samples;
			problem.distances = problemOPN.distances;
		} else {
			INFO("using classical euclidean OP:");
			problem.gop_type = OP;
			SOP_Prolem problemOP = getOP(nodesAll);
			problem.samples = problemOP.samples;
			problem.distances = problemOP.distances;
		}
	} else if (sop_type.compare("sop") == 0) {
		INFO("gop type is sop");
		problem.gop_type = SOP;
		DatasetSOP loadedDataset = DatasetLoaderSOP::loadDataset(problemFile);
		INFO("has loadedDataset")
		INFO(
				"testing SOP dataset with " << loadedDataset.clusters.size() << " clusters, " << loadedDataset.Tmax << " max time budget");
		INFO("use budget " << loadedDataset.Tmax);
		INFO("use startIndex " << loadedDataset.startID);
		INFO("use goalIndex " << loadedDataset.goalID);
		problem.name = config.get<std::string>("name");
		problem.startIndex = loadedDataset.startID;
		problem.goalIndex = loadedDataset.goalID;
		problem.name = loadedDataset.name;
		problem.budget = loadedDataset.Tmax;

		if (change_end_index_to_last) {
			INFO("changing the index of the goal to be the last ...... "<<problem.goalIndex);
			problem.oldGoalIndex = problem.goalIndex;
			std::string sop_solver = config.get<std::string>("sop-solver");

			ClusterSOP endCluster = loadedDataset.clusters[problem.goalIndex];
			ClusterSOP lastCluster = loadedDataset.clusters[loadedDataset.clusters.size() - 1];
			if (problem.goalIndex == problem.startIndex) {
				loadedDataset.clusters.push_back(endCluster);
			} else {
				loadedDataset.clusters[problem.oldGoalIndex] = lastCluster;
				loadedDataset.clusters[problem.oldGoalIndex].cluster_id = problem.oldGoalIndex;
			}
			problem.goalIndex = loadedDataset.clusters.size() - 1;
			loadedDataset.clusters[problem.goalIndex] = endCluster;
			loadedDataset.clusters[problem.goalIndex].cluster_id = problem.goalIndex;

			if (problem.startIndex != 0) {
				INFO("changing the index of the start to be the first ...... ");
				problem.oldStartIndex = problem.startIndex;
				ClusterSOP startNode = loadedDataset.clusters[problem.startIndex];
				ClusterSOP firstNode = loadedDataset.clusters[0];
				loadedDataset.clusters[problem.oldStartIndex] = firstNode;
				loadedDataset.clusters[problem.oldStartIndex].cluster_id = problem.oldStartIndex;
				problem.startIndex = 0;

				loadedDataset.clusters[problem.startIndex] = startNode;
				loadedDataset.clusters[problem.startIndex].cluster_id = problem.startIndex;
			} else {
				problem.startIndex = 0;
				problem.oldStartIndex = 0;

			}

			INFO_VAR(problem.startIndex);
			INFO_VAR(problem.oldStartIndex);
			INFO_VAR(problem.goalIndex);
			INFO_VAR(problem.oldGoalIndex);
		}

		//DatasetLoaderSOP::printSets(loadedDataset);
		std::vector<std::vector<GraphNode>> clusterNodes;
		INFO("load dataset problem.startIndex "<<problem.startIndex)
		INFO("load dataset problem.goalIndex "<<problem.goalIndex)
		SOP_Prolem problemSOP;
		if (loadedDataset.is_with_nodes) {
			problemSOP = getSOPFromNodes(loadedDataset.clusters, loadedDataset.nodesAll, problem.startIndex,
					problem.goalIndex, loadedDataset.edge_weight_type);
			problem.distances = problemSOP.distances;
			problem.samples = problemSOP.samples;
		} else {
			problemSOP = getSOPFromWeightMatrix(loadedDataset.clusters, loadedDataset.distance_matrix,
					problem.startIndex, problem.goalIndex);
			problem.distances = problemSOP.distances;
			problem.samples = problemSOP.samples;
		}
		//printAllClustersNodes(problem.samples);

	} else {
		ERROR("unknown gop type "<<sop_type);
		exit(1);
	}

	INFO("return problem definition");
	return problem;
}

void SOPLoader::fixClusterNodeRenumbering(std::vector<GraphNode>& solution, int startIndex, int goalIndex,
		int oldstartIndex, int oldgoalIndex, int deleted_cluster_id, bool numbered_from_one) {

	//for debuging
	INFO("fixClusterNodeRenumbering begin");
	INFO_VAR(startIndex);
	INFO_VAR(goalIndex);
	INFO_VAR(oldstartIndex);
	INFO_VAR(oldgoalIndex);
	INFO_VAR(deleted_cluster_id);
	std::stringstream tourClustersBef;
	for (int var = 0; var < solution.size(); ++var) {
		if (var != 0) {
			tourClustersBef << ",";
		}
		tourClustersBef << solution[var].cluster_id;
	}
	INFO("cluster IDs before: "<<tourClustersBef.str());

	for (int var = 0; var < solution.size(); ++var) {
		if (oldgoalIndex != -1) {
			if (solution[var].cluster_id == goalIndex) {
				solution[var].cluster_id = oldgoalIndex;
				INFO("change "<<goalIndex<< " id to "<<oldgoalIndex<<" at pos "<<var);
				continue;
			}
			if (solution[var].cluster_id == oldgoalIndex && oldstartIndex != oldgoalIndex) {
				solution[var].cluster_id = goalIndex;
				INFO("change "<<oldgoalIndex<< " id to "<<goalIndex<<" at pos "<<var);
				continue;
			}
		}
		if (oldstartIndex != startIndex) {
			if (solution[var].cluster_id == oldstartIndex) {
				solution[var].cluster_id = startIndex;
				INFO("change "<<oldstartIndex<< " id to "<<startIndex<<" at pos "<<var);
				continue;
			}
			if (solution[var].cluster_id == startIndex) {
				solution[var].cluster_id = oldstartIndex;
				INFO("change "<<startIndex<< " id to "<<oldstartIndex<<" at pos "<<var);
				continue;
			}
		}
	}

	if (deleted_cluster_id >= 0) {
		for (int var = 0; var < solution.size(); ++var) {
			if (solution[var].cluster_id >= deleted_cluster_id) {
				solution[var].cluster_id = solution[var].cluster_id + 1;
			}
		}
	}

	//for debuging
	std::stringstream tourClustersAft;
	for (int var = 0; var < solution.size(); ++var) {
		if (var != 0) {
			tourClustersAft << ",";
		}
		tourClustersAft << solution[var].cluster_id;
	}
	INFO("cluster IDs after: "<<tourClustersAft.str());
	INFO("fixClusterNodeRenumbering end");

}

SOP_Prolem SOPLoader::getOP(std::vector<GraphNode> opNodes) {
	INFO("getOP");

	int S = opNodes.size();
	samples_type samples;
	samples.resize(S);
	for (int nodeID = 0; nodeID < S; ++nodeID) {
		std::vector<GraphNode> onlyNode(1);
		onlyNode[0] = opNodes[nodeID];
		onlyNode[0].cluster_id = nodeID;
		samples[nodeID] = onlyNode;
	}

	samples_distances_type distances;
	distances.resize(S);
	for (int setid1 = 0; setid1 < S; ++setid1) {
		distances[setid1].resize(S);

		for (int setid2 = 0; setid2 < S; ++setid2) {
			int maxneighID1 = 1;
			distances[setid1][setid2].resize(maxneighID1);
			for (int neighID1 = 0; neighID1 < maxneighID1; ++neighID1) {
				int maxneighID2 = 1;
				distances[setid1][setid2][neighID1].resize(maxneighID2);

				for (int neighID2 = 0; neighID2 < maxneighID2; ++neighID2) {
					distances[setid1][setid2][neighID1][neighID2] = samples[setid1][neighID1].distanceTo(
							samples[setid2][neighID2]);
				}
			}
		}
	}

	SOP_Prolem toReturn;
	toReturn.samples = samples;
	toReturn.distances = distances;
	return toReturn;
}

CoordsVector SOPLoader::getOPPath(std::vector<IndexSOP> &gopPath, samples_type & nodesAllClusters) {
	//INFO("getOPPath begin");
	int S = gopPath.size();
	//INFO("size "<<S);
	CoordsVector ret(S);
	for (int var = 0; var < S; ++var) {
		//INFO("ci "<<gopPath[var].clusterIndex<<" ni "<<gopPath[var].nodeIndex);
		GraphNode gn = nodesAllClusters[gopPath[var].clusterIndex][gopPath[var].nodeIndex];
		ret[var] = Coords(gn.x, gn.y);
	}
	//INFO("getOPPath end");
	return ret;
}

SOP_Prolem SOPLoader::getOPN(std::vector<GraphNode> opNodes, int startIndex_, int goalIndex_, double neigh_radius,
		int neighborhood_resolution) {
	INFO("getOPN");

	int S = opNodes.size();
	samples_type samples;
	samples.resize(S);
	int id_node = 0;
	for (int cluster_id = 0; cluster_id < S; ++cluster_id) {
		if (cluster_id == goalIndex_ || cluster_id == startIndex_) {
			std::vector<GraphNode> onlyStartGoal(1);
			onlyStartGoal[0] = opNodes[cluster_id];
			samples[cluster_id] = onlyStartGoal;
			samples[cluster_id][0].id = id_node;
			id_node++;
			samples[cluster_id][0].cluster_id = cluster_id;

		} else {
			std::vector<GraphNode> neighborhood(neighborhood_resolution);
			for (int neighID = 0; neighID < neighborhood_resolution; neighID++) {
				double neigh_ang = (2 * M_PI * neighID) / neighborhood_resolution;
				GraphNode p_neigh = opNodes[cluster_id];
				p_neigh.x += neigh_radius * cos(neigh_ang);
				p_neigh.y += neigh_radius * sin(neigh_ang);
				//p_neigh.x += neigh_radius * randDoubleMinMax(0.99, 1.0) * cos(neigh_ang);
				//p_neigh.y += neigh_radius * randDoubleMinMax(0.99, 1.0) * sin(neigh_ang);
				p_neigh.id = id_node;
				id_node++;
				p_neigh.cluster_id = cluster_id;
				neighborhood[neighID] = p_neigh;
			}
			samples[cluster_id] = neighborhood;
		}
	}

	samples_distances_type distances;
	distances.resize(S);
	for (int setid1 = 0; setid1 < S; ++setid1) {
		distances[setid1].resize(S);
		//INFO_VAR(distances[setid1].size());

		for (int setid2 = 0; setid2 < S; ++setid2) {

			int maxneighID1 = neighborhood_resolution;
			if (setid1 == startIndex_ || setid1 == goalIndex_) {
				maxneighID1 = 1;
			}
			distances[setid1][setid2].resize(maxneighID1);
			for (int neighID1 = 0; neighID1 < maxneighID1; ++neighID1) {
				int maxneighID2 = neighborhood_resolution;
				if (setid2 == startIndex_ || setid2 == goalIndex_) {
					maxneighID2 = 1;
				}
				distances[setid1][setid2][neighID1].resize(maxneighID2);
				for (int neighID2 = 0; neighID2 < maxneighID2; ++neighID2) {
					distances[setid1][setid2][neighID1][neighID2] = samples[setid1][neighID1].distanceTo(
							samples[setid2][neighID2]);
				}
			}
		}
	}
	SOP_Prolem toReturn;
	toReturn.samples = samples;
	toReturn.distances = distances;
	return toReturn;
}

CoordsVector SOPLoader::getOPNPath(std::vector<IndexSOP> &gopPath, samples_type & nodesAllClusters) {
	int S = gopPath.size();
	CoordsVector ret(S);
	for (int var = 0; var < S; ++var) {
		GraphNode gn = nodesAllClusters[gopPath[var].clusterIndex][gopPath[var].nodeIndex];
		ret[var] = Coords(gn.x, gn.y);
	}
	return ret;
}

SOP_Prolem SOPLoader::getSOPFromWeightMatrix(std::vector<ClusterSOP> clusters,
		std::vector<std::vector<double>> distance_matrix, int startIndex_, int goalIndex_) {
	INFO("getSOPFromWeightMatrix end start "<<startIndex_<<" goal "<<goalIndex_);
	samples_type samples;
	samples_distances_type distances;
	int S = clusters.size();
	distances.resize(S);
	samples.resize(S);
	int nodesS = distance_matrix.size();
	float nextPart = 0.1;
	int numNodes = 0;
	INFO_VAR(S)
	for (int setid1 = 0; setid1 < S; ++setid1) {
		INFO_VAR(setid1)
		samples[setid1].resize(clusters[setid1].nodeIDs.size());
		for (int setNodeID = 0; setNodeID < clusters[setid1].nodeIDs.size(); ++setNodeID) {
			samples[setid1][setNodeID] = GraphNode();
			samples[setid1][setNodeID].cluster_id = clusters[setid1].cluster_id;
			samples[setid1][setNodeID].reward = clusters[setid1].reward;
			samples[setid1][setNodeID].id = clusters[setid1].nodeIDs[setNodeID];
		}
	}
	INFO("created samples")
	for (int setid1 = 0; setid1 < S; ++setid1) {
		distances[setid1].resize(S);

		if (setid1 / ((float) S) > nextPart) {
			INFO("calc distances "<<(100*nextPart)<<"%");
			nextPart += 0.1;
		}

		for (int setid2 = 0; setid2 < S; ++setid2) {
			int numNodes1 = clusters[setid1].nodeIDs.size();
			distances[setid1][setid2].resize(numNodes1);
			for (int neighID1 = 0; neighID1 < numNodes1; ++neighID1) {
				int numNodes2 = clusters[setid2].nodeIDs.size();
				distances[setid1][setid2][neighID1].resize(numNodes2);
				for (int neighID2 = 0; neighID2 < numNodes2; ++neighID2) {
					int from_node_id = clusters[setid1].nodeIDs[neighID1];
					int to_node_id = clusters[setid2].nodeIDs[neighID2];

					distances[setid1][setid2][neighID1][neighID2] = distance_matrix[from_node_id][to_node_id];

				}
			}
		}
	}

	INFO("calc distances "<<(100)<<"%");
	SOP_Prolem toReturn;
	toReturn.distances = distances;
	toReturn.samples = samples;
	INFO("getSOPFromWeightMatrix end start "<<startIndex_<<" goal "<<goalIndex_);
	return toReturn;
}

SOP_Prolem SOPLoader::getSOPFromNodes(std::vector<ClusterSOP> clusters, std::vector<GraphNode> nodes, int startIndex_,
		int goalIndex_, TSP_EDGE_WEIGHT_TYPE edge_weight_type_) {
	INFO("getSOPFromNodes begin start "<<startIndex_<<" goal "<<goalIndex_);
	int S = clusters.size();
	samples_distances_type distances;
	samples_type samples;
	distances.resize(S);
	samples.resize(S);
	float nextPart = 0.1;
	int numNodes = 0;
	for (int setid1 = 0; setid1 < S; ++setid1) {
		samples[setid1].resize(clusters[setid1].nodeIDs.size());
		for (int setNodeID = 0; setNodeID < clusters[setid1].nodeIDs.size(); ++setNodeID) {
			samples[setid1][setNodeID] = nodes[clusters[setid1].nodeIDs[setNodeID]];
			samples[setid1][setNodeID].cluster_id = clusters[setid1].cluster_id;
			samples[setid1][setNodeID].reward = clusters[setid1].reward;
		}
	}

	for (int setid1 = 0; setid1 < S; ++setid1) {
		distances[setid1].resize(S);

		if (setid1 / ((float) S) > nextPart) {
			INFO("calc distances "<<(100*nextPart)<<"%");
			nextPart += 0.1;
		}

		for (int setid2 = 0; setid2 < S; ++setid2) {
			int numNodes1 = clusters[setid1].nodeIDs.size();
			distances[setid1][setid2].resize(numNodes1);
			for (int neighID1 = 0; neighID1 < numNodes1; ++neighID1) {
				int numNodes2 = clusters[setid2].nodeIDs.size();
				distances[setid1][setid2][neighID1].resize(numNodes2);
				for (int neighID2 = 0; neighID2 < numNodes2; ++neighID2) {
					int from_node_id = clusters[setid1].nodeIDs[neighID1];
					int to_node_id = clusters[setid2].nodeIDs[neighID2];
					switch (edge_weight_type_) {
					case EUC_2D:
						distances[setid1][setid2][neighID1][neighID2] = round(
								nodes[from_node_id].distanceTo(nodes[to_node_id]));
						break;
					case CEIL_2D:
						distances[setid1][setid2][neighID1][neighID2] = ceil(
								nodes[from_node_id].distanceTo(nodes[to_node_id]));
						break;
					default:
						ERROR("unknown edge weight type "<<edge_weight_type_)
						exit(1);
					}

				}
			}
		}
	}

	INFO("calc distances "<<(100)<<"%");
	SOP_Prolem toReturn;
	toReturn.distances = distances;
	toReturn.samples = samples;
	INFO("getSOPFromNodes end start "<<startIndex_<<" goal "<<goalIndex_);
	return toReturn;
}

void SOPLoader::printAllClustersNodes(samples_type samples) {
	for (int var = 0; var < samples.size(); ++var) {
		INFO("cluster "<<var<<":")
		for (int var2 = 0; var2 < samples[var].size(); ++var2) {
			INFO(
					"\t node "<<var2<<" id "<<samples[var][var2].id<<" reward "<<samples[var][var2].reward<<" pos "<<samples[var][var2].x<<":"<<samples[var][var2].y<<" clusted_id "<<samples[var][var2].cluster_id);
		}
	}
}
