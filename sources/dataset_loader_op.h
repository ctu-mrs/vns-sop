/*
 * DatasetLoader.h
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_DATASETLOADER_OP_H_
#define SRC_DATASETLOADER_OP_H_
#include "crl/logging.h"
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include "heuristic_types.h"

class DatasetLoaderOP {
public:
	DatasetLoaderOP();
	virtual ~DatasetLoaderOP();
	static DatasetOP loadDataset(std::string filename);
};

#endif /* SRC_DATASETLOADER_OP_H_ */
