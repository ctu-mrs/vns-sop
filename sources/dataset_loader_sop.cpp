/*
 * DatasetLoader.cpp
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#include "dataset_loader_sop.h"

#define NAME "NAME"
#define TYPE "TYPE"
#define DIMENSION "DIMENSION"
#define NODE_COORD_SECTION "NODE_COORD_SECTION"
#define CONSTRAINING_SET_SECTION "GTSP_SET_SECTION"
#define GTSP_SET_CENTER_COORD_SECTION "GTSP_SET_CENTER_COORD_SECTION"

#define EDGE_WEIGHT_SECTION "EDGE_WEIGHT_SECTION"

#define COMMENT "COMMENT"
#define SETS "SETS"
#define TMAX "TMAX"

#define START_SET "START_SET"
#define END_SET "END_SET"

#define NODE_COORD_SECTION_END "EOF"
#define EDGE_WEIGHT_TYPE "EDGE_WEIGHT_TYPE"

using crl::logger;

DatasetLoaderSOP::DatasetLoaderSOP() {

}

DatasetLoaderSOP::~DatasetLoaderSOP() {

}

DatasetSOP DatasetLoaderSOP::loadDataset(std::string filename) {
	INFO("DatasetLoaderGOP::loadDataset");
	DatasetSOP loadedDataset;

	int depot_id = 0;
	loadedDataset.startID = depot_id;
	loadedDataset.goalID = depot_id;
	std::vector<GraphNode> nodes;
	std::vector<std::vector<double>> weight_matrix;
	bool use_nodes = true;
	//exit(1);
	std::ifstream in(filename.c_str(), std::ifstream::in);
	INFO("reading problem file from "<<filename);
	if ( !in ) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;
		bool dataSection, setSection, weightMatrixSection = false;
		std::string name;
		std::string type;
		std::string dimension_str;
		std::string comment;
		std::string tmax_str;
		std::string edge_type_str;
		std::string sets_str;
		std::string start_set_str;
		std::string end_set_str;
		std::string delimiter(" ");
		TSP_EDGE_WEIGHT_TYPE edge_type;
		int dimension;
		int loaded = 0;

		while ( getline(in, line) ) {
			lineNumber++;
			int delimiterPos = line.find(":");
			//INFO("line:"<<line);
			if ( delimiterPos != std::string::npos ) {
				std::string bef = line.substr(0, delimiterPos);
				std::string aftr("");
				//INFO_VAR(line.size());
				//INFO(delimiterPos + 1);
				if ( line.size() > delimiterPos + 1 ) {
					//INFO("subs");
					aftr = line.substr(delimiterPos + 1);
				}
				aftr = trim(aftr);
				bef = trim(bef);

				//INFO("NAME "<<NAME);
				//INFO("TYPE "<<TYPE);
				//INFO("DIMENSION"<<DIMENSION);

				//INFO(bef<<" "<<std::strcmp(bef.c_str(), CONSTRAINING_SET_SECTION));

				if ( !std::strcmp(bef.c_str(), NAME) ) {
					name = aftr;
					loadedDataset.name = name;
				} else if ( !std::strcmp(bef.c_str(), TYPE) ) {
					type = aftr;
					loadedDataset.type = name;
				} else if ( !std::strcmp(bef.c_str(), DIMENSION) ) {
					dimension_str = aftr;
					dimension = std::stoi(dimension_str);
					loadedDataset.dimension = dimension;
				} else if ( !std::strcmp(bef.c_str(), COMMENT) ) {
					comment = aftr;
					loadedDataset.comment = comment;
				} else if ( !std::strcmp(bef.c_str(), TMAX) ) {
					tmax_str = aftr;
					loadedDataset.Tmax = std::stoi(tmax_str);
				} else if ( !std::strcmp(bef.c_str(), START_SET) ) {
					start_set_str = aftr;
					loadedDataset.startID = std::stoi(start_set_str);
				} else if ( !std::strcmp(bef.c_str(), END_SET) ) {
					end_set_str = aftr;
					loadedDataset.goalID = std::stoi(end_set_str);
				} else if ( !std::strcmp(bef.c_str(), EDGE_WEIGHT_TYPE) ) {
					edge_type_str = aftr;
					loadedDataset.edge_weight_type = parse_tsp_edge_type(edge_type_str);
				} else if ( !std::strcmp(bef.c_str(), SETS) ) {
					sets_str = aftr;
					loadedDataset.sets = std::stoi(sets_str);
				} else if ( !std::strcmp(bef.c_str(), CONSTRAINING_SET_SECTION) ) {
					//INFO("switch to "<<CONSTRAINING_SET_SECTION);
					dataSection = false;
					weightMatrixSection = false;
					setSection = true;
				} else if ( !std::strcmp(bef.c_str(), GTSP_SET_CENTER_COORD_SECTION) ) {
					dataSection = false;
					weightMatrixSection = false;
					setSection = false;
				} else if ( !std::strcmp(bef.c_str(), EDGE_WEIGHT_SECTION) ) {
					dataSection = false;
					setSection = false;
					weightMatrixSection = true;
				}
			} else {
				//INFO("else");
				line = trim(line);

				//INFO(line<<" "<<std::strcmp(line.c_str(), NODE_COORD_SECTION));
				if ( !std::strcmp(line.c_str(), NODE_COORD_SECTION) ) {
					//INFO("switch to NODE_COORD_SECTION start");
					dataSection = true;
					setSection = false;
					weightMatrixSection = false;
				} else if ( !std::strcmp(line.c_str(), NODE_COORD_SECTION_END) ) {
					//INFO("switch to "<<NODE_COORD_SECTION_END);
					dataSection = false;
					setSection = false;
					weightMatrixSection = false;
				} else if ( !std::strcmp(line.c_str(), CONSTRAINING_SET_SECTION) ) {
					//INFO("switch to "<<CONSTRAINING_SET_SECTION);
					dataSection = false;
					setSection = true;
					weightMatrixSection = false;
				} else if ( !std::strcmp(line.c_str(), EDGE_WEIGHT_SECTION) ) {
					dataSection = false;
					setSection = false;
					weightMatrixSection = true;
				} else if ( dataSection ) {
					//INFO("data section");
					std::istringstream s(line);
					GraphNode newGN;
					s >> newGN.id;
					newGN.id--; //in cop lib the nodes are numbered from 1
					s >> newGN.x;
					s >> newGN.y;
					newGN.reward = 0;
					if ( newGN.id != loaded ) {
						ERROR("wrong id assignment to dataset nodes");
						ERROR(newGN.id<<" "<<newGN.x<<" "<<newGN.y)
					}
					newGN.id = loaded; //owerwrite id of point
					newGN.cluster_id = 0;
					nodes.push_back(newGN);
					//INFO(newGN.id<<" "<<newGN.x<<" "<<newGN.y)
					loaded++;
					if ( nodes.size() != loaded ) {
						ERROR("wrong id assignment to dataset nodes");
						ERROR(newGN.id<<" "<<newGN.x<<" "<<newGN.y)
						exit(1);
					}
				} else if ( setSection ) {
					//INFO("set section");

					std::vector<std::string> tokens = tokenize(line, delimiter);
					if ( tokens.size() >= 3 ) {
						//INFO("token size ok")
						ClusterSOP cluster;
						double cluster_reward = 0;
						//INFO("large enough");
						int cluster_id = 0;
						for ( int var = 0 ; var < tokens.size() ; ++var ) {
							//INFO("token:"<<tokens[var])
							std::istringstream tikeniss(tokens[var]);
							if ( var == 0 ) {
								tikeniss >> cluster_id; // keep the cluster id to be indexed from 1 as the first cluster with 0 is start
								cluster.cluster_id = cluster_id;
								//INFO("cluster id "<<cluster_id);
							} else if ( var == 1 ) {
								tikeniss >> cluster_reward;
								cluster.reward = cluster_reward;
								//INFO_GREEN("cluster "<<cluster_id<<" reward "<<cluster_reward);
							} else {
								int id_node = 0;
								tikeniss >> id_node;
								id_node--; //in cop lib the nodes are numbered from 1
								//INFO("id_node "<<id_node)
								if ( (use_nodes && id_node >= nodes.size()) || (!use_nodes && id_node >= weight_matrix.size()) ) {
									ERROR("want to assign non-existing node to cluster");
									ERROR("nodes.size() "<<nodes.size()<< " id_node "<<id_node);
									exit(1);
								}
								if ( id_node >= 0 ) {
									//ommiting starting node and end of one cluster section
									cluster.nodeIDs.push_back(id_node);
									if ( cluster_id != loadedDataset.clusters.size() ) {
										ERROR("wrong cluster_id assignment to dataset nodes");
										ERROR("loadedDataset.clusterNodes.size() "<<loadedDataset.clusters.size()<<" cluster_id "<<cluster_id);
										exit(1);
									}
								}

							}
						}
						if ( cluster.nodeIDs.size() > 0 ) {
							loadedDataset.clusters.push_back(cluster);
						} else {
							ERROR("empty cluster");
							exit(1);
						}
					} else {
						ERROR(
								"not enought cluster numbers per line - min >=3 received "<<tokens.size()<<" line is:"<<line);
						exit(1);
					}
				} else if ( weightMatrixSection ) {
					//INFO("loading weight matrix "<<line);
					use_nodes = false;
					std::vector<std::string> tokens = tokenize(line, delimiter);
					//INFO("tokens.size "<<tokens.size())

					for ( int var = 0 ; var < tokens.size() ; ++var ) {
						if ( tokens[var].empty() ) {
							continue;								//ommit empty char tokens
						}
						if ( weight_matrix.size() == 0 || weight_matrix.back().size() >= dimension ) {
							if ( weight_matrix.size() != 0 ) {
								//INFO("add new row, previous has "<<weight_matrix.back().size()<<" cols");
							}
							weight_matrix.push_back(std::vector<double>());
							//INFO("added weight_matrix row "<<weight_matrix.size());
						}
						std::istringstream tikeniss(tokens[var]);
						double distance = 0;
						tikeniss >> distance;
						weight_matrix[weight_matrix.size() - 1].push_back(distance);
					}
				}
			}
		}
		INFO("loaded "<<loaded <<" out of dimension "<<dimension);
	}

	if ( !use_nodes ) {
		//check distance matrix dimensions
		if ( weight_matrix.size() != loadedDataset.dimension ) {
			ERROR("edge weight matrix rows not equal to dimension "<<weight_matrix.size()<<"!="<<loadedDataset.dimension);
			exit(1);
		}
		for ( int var = 0 ; var < weight_matrix.size() ; ++var ) {
			if ( weight_matrix[var].size() != loadedDataset.dimension ) {
				ERROR("edge weight matrix row "<<var<<" has not equal to dimension "<<weight_matrix[var].size()<<"!="<<loadedDataset.dimension);
				exit(1);
			}
		}
	}

	loadedDataset.is_with_nodes = use_nodes;
	loadedDataset.nodesAll = nodes;
	loadedDataset.distance_matrix = weight_matrix;

	//printSets(loadedDataset);
	INFO("return loaded dataset")
	return loadedDataset;
}

void DatasetLoaderSOP::printSets(DatasetSOP &gopDataset) {
	for ( int var = 0 ; var < gopDataset.clusters.size() ; ++var ) {
		INFO(
				"cluster "<<var<<" with id"<<gopDataset.clusters[var].cluster_id<<", reward "<<gopDataset.clusters[var].reward<<" and size "<<gopDataset.clusters[var].nodeIDs.size());
		std::stringstream ss;
		for ( int var2 = 0 ; var2 < gopDataset.clusters[var].nodeIDs.size() ; ++var2 ) {
			ss << gopDataset.clusters[var].nodeIDs[var2] << " ";
		}
		INFO("\t node ids:"<<ss.str());
	}
}

std::vector<GraphNode> DatasetLoaderSOP::parseInitialPositions(std::string string) {
	std::string delimiterPositions = "|";
	std::string delimiterXY = ";";
	std::vector<GraphNode> positions;
	std::vector<std::string> positionsArr;
	size_t pos = 0;
	std::string token;
	while ( (pos = string.find(delimiterPositions)) != std::string::npos ) {
		token = string.substr(0, pos);
		string.erase(0, pos + delimiterPositions.length());
		if ( token.length() > 0 ) {
			positionsArr.push_back(token);
		}
	}
	if ( string.length() > 0 ) {
		positionsArr.push_back(string);
	}
	for ( int var = 0 ; var < positionsArr.size() ; ++var ) {
		std::string singlePosition = positionsArr[var];
		std::vector<std::string> xyPosition;
		INFO(singlePosition);
		size_t posIN = 0;
		std::string token;
		while ( (posIN = singlePosition.find(delimiterXY)) != std::string::npos ) {
			token = singlePosition.substr(0, posIN);
			singlePosition.erase(0, posIN + delimiterXY.length());

			if ( token.length() > 0 ) {
				xyPosition.push_back(token);
			}
		}
		if ( singlePosition.length() > 0 ) {
			xyPosition.push_back(singlePosition);
		}

		if ( xyPosition.size() == 2 ) {
			float x;
			float y;
			std::sscanf(xyPosition[0].c_str(), "x=%f", &x);
			std::sscanf(xyPosition[1].c_str(), "y=%f", &y);
			INFO(x);
			INFO(y);
			positions.push_back(GraphNode(x, y, 0, 0, 0));
		}
		for ( int var2 = 0 ; var2 < xyPosition.size() ; ++var2 ) {
			INFO(xyPosition[var2]);
		}
	}
	return positions;
}

SolutionGTSP DatasetLoaderSOP::readSolution(std::string filename) {
	SolutionGTSP solution;
	std::vector<GraphNode> nodes;
	std::ifstream in(filename.c_str(), std::ifstream::in);
	INFO("reading solution file from filename");
	if ( !in ) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		int num_clusters = 0;
		int lineNumber = 0;
		while ( getline(in, line) ) {
			std::istringstream s(line);
			if ( lineNumber == 0 ) {
				s >> solution.num_nodes;
			} else if ( lineNumber == 1 ) {
				s >> solution.length;
			} else {
				int node_id = 0;
				s >> node_id;
				//substract to have id numbering from 0
				node_id--;
				solution.node_ids.push_back(node_id);
			}
			lineNumber++;
		}
	}
	INFO("readed solution with "<<solution.num_nodes<<" nodes and "<<solution.length<<" length");
	std::stringstream ss;
	for ( int var = 0 ; var < solution.node_ids.size() ; ++var ) {
		ss << solution.node_ids[var] << " ";
	}
	INFO("with node ids "<<ss.str());
	return solution;
}
