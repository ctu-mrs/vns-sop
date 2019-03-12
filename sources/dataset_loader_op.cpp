/*
 * DatasetLoader.cpp
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#include "dataset_loader_op.h"

DatasetLoaderOP::DatasetLoaderOP() {


}

DatasetLoaderOP::~DatasetLoaderOP() {

}

/**
 * first line loaded as description of dataset
 * Description:
 *
 File contents

 *********************
 * OP test instances *
 *********************

 The first line contains the following data:

 Tmax P

 Where
 Tmax 	= available time budget per path
 P	= number of paths (=1)


 The remaining lines contain the data of each point.
 For each point, the line contains the following data:

 x y S

 Where
 x	= x coordinate
 y	= y coordinate
 S	= score

 * REMARKS *
 - The first point is the starting point.
 - The second point is the ending point.
 - The Euclidian distance is used.

 *
 *
 */

DatasetOP DatasetLoaderOP::loadDataset(std::string filename) {

	DatasetOP loadedDataset;
	loadedDataset.startID = 0;
	loadedDataset.goalID = 1;
	std::ifstream in(filename.c_str(), std::ifstream::in);

	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;
		while (getline(in, line)) {
			lineNumber++;
			if (lineNumber <= 1) {
				//load description of datafile
				std::istringstream s(line);
				s >> loadedDataset.Tmax;
				s >> loadedDataset.P;
			} else {
				//load node 10.5	14.4	0  - x	y	reward
				std::istringstream s(line);
				GraphNode newGN;
				s >> newGN.x;
				s >> newGN.y;
				s >> newGN.reward;
				newGN.id = actualGNID;
				actualGNID++;
				loadedDataset.graph.push_back(newGN);
			}
		}
	}

	return loadedDataset;
}
