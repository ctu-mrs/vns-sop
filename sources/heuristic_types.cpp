/*
 * HeuristicTypes.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: Robert Penicka
 */

#include "heuristic_types.h"
using crl::logger;

HeuristicTypes::HeuristicTypes() {

}

HeuristicTypes::~HeuristicTypes() {

}

bool operator==(const GraphNode& lhs, const GraphNode& rhs) {
	//WINFO("operator== " << lhs.x  <<" == "<< rhs.x);
	return lhs.x == rhs.x && lhs.y == rhs.y && lhs.reward == rhs.reward;
}

std::ostream& operator <<(std::ostream &o, GraphNode& p) {
	std::cout.precision(6);
	o << std::fixed << " " << std::setprecision(6) << p.x << " " << std::setprecision(6) << p.y << " "
			<< std::setprecision(6) << p.reward;
	return o;
}

bool operator>(GraphNode &dt1, GraphNode &dt2) {
	return dt1.reward > dt2.reward;
}

bool operator<(GraphNode &dt1, GraphNode &dt2) {
	return dt1.reward < dt2.reward;
}

TSP_EDGE_WEIGHT_TYPE parse_tsp_edge_type(std::string edge_type_string) {
	if ( !std::strcmp(edge_type_string.c_str(), TSP_EDGE_EUC_2D_STR) ) {
		return EUC_2D;
	} else if ( !std::strcmp(edge_type_string.c_str(), TSP_EDGE_CEIL_2D_STR) ) {
		return CEIL_2D;
	} else if ( !std::strcmp(edge_type_string.c_str(), TSP_EDGE_CEIL_EXPLICIT_STR) ) {
		return EXPLICIT;
	} else {
		ERROR("unknown tsp edge weight type "<<edge_type_string);
		exit(1);
	}
}

