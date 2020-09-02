/*
 * my_defines.h
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#ifndef LP_GOP_MY_DEFINES_H_
#define LP_GOP_MY_DEFINES_H_

#include "comrob_lite/logging.h"
#include <string>
#include <iostream>

#define OUTPUT_DEFAULT "\033[0m"
#define OUTPUT_BLACK "\033[30m"
#define OUTPUT_RED "\033[31m"
#define OUTPUT_GREEN "\033[32m"
#define OUTPUT_YELLOW "\033[33m"
#define OUTPUT_BLUE "\033[34m"
#define OUTPUT_MAGENTA "\033[35m"
#define OUTPUT_CYAN "\033[36m"
#define OUTPUT_WHITE "\033[37m"

#define INFO_RED(x) INFO( OUTPUT_RED << x << OUTPUT_DEFAULT )
#define INFO_YELLOW(x) INFO( OUTPUT_YELLOW << x << OUTPUT_DEFAULT )
#define INFO_MAGENTA(x) INFO( OUTPUT_MAGENTA <<  x << OUTPUT_DEFAULT )
#define INFO_CYAN(x) INFO( OUTPUT_CYAN <<  x << OUTPUT_DEFAULT )
#define INFO_GREEN(x) INFO( OUTPUT_GREEN <<  x << OUTPUT_DEFAULT )
#define INFO_WHITE(x) INFO( OUTPUT_WHITE <<  x << OUTPUT_DEFAULT )
#define INFO_BLUE(x) INFO( OUTPUT_BLUE <<  x << OUTPUT_DEFAULT )
#define INFO_BLACK(x) INFO( OUTPUT_BLACK <<  x << OUTPUT_DEFAULT )
#define INFO_COND( cond , x ) if(cond){ INFO( x ); }

#define VARIABLE_STR(s) #s
#define STR(s) VARIABLE_STR(s)
#define ROVNASE1(X) X =
#define ROVNASE(X)  ROVNASE1(X)
#define INFO_VAR(x) INFO( STR(x) << " = " <<  x )

#define INFO_VAR_RED(x) INFO_RED( STR(x) << " = " <<  x )


typedef struct VarValue {
	std::string varName;
	int varValue;
} VarValue;

typedef struct IndexPairSOP {
	int cluster_from_node;
	int cluster_to_node;
	int cluster_from;
	int cluster_to;
} IndexPairSOP;

typedef struct IndexSOP {
	IndexSOP() {
		this->clusterIndex = -1;
		this->nodeIndex = -1;
	}

	IndexSOP(int clusterIndex_, int nodeIndex_) {
		this->clusterIndex = clusterIndex_;
		this->nodeIndex = nodeIndex_;
	}
	int clusterIndex;
	int nodeIndex;
} IndexSOP;

#endif /* LP_GOP_MY_DEFINES_H_ */
