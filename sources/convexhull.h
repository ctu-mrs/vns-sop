/*
 * convexhull.h
 *
 *  Created on: Mar 8, 2019
 *      Author: robert
 */

#ifndef CONVEXHULL_H_
#define CONVEXHULL_H_

// A C++ program to find convex hull of a set of points. Refer
// https://www.geeksforgeeks.org/orientation-3-ordered-points/
// for explanation of orientation()
#include <iostream>
#include <stack>
#include <vector>
#include <stdlib.h>
#include <cmath>
using namespace std;

struct PointConvexHull {
	PointConvexHull() {
		this->x = NAN;
		this->y = NAN;
	}
	;
	PointConvexHull(double x_, double y_) {
		this->x = x_;
		this->y = y_;
	}
	;
	double x, y;
};

class convex_hull {
public:
	static vector<PointConvexHull> get_convex_hull(PointConvexHull points[], int n);

private:
	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are colinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise
	static int orientation(PointConvexHull p, PointConvexHull q, PointConvexHull r);

	// A utility function to return square of distance
	// between p1 and p2
	static int distSq(PointConvexHull p1, PointConvexHull p2);

	// A utility function to swap two points
	static void swap(PointConvexHull &p1, PointConvexHull &p2);

	// A utility function to find next to top in a stack
	static PointConvexHull nextToTop(stack<PointConvexHull> &S);

	// A globle point needed for sorting points with reference
	// to the first point Used in compare function of qsort()

	// A function used by library function qsort() to sort an array of
	// points with respect to the first point
	static int compare(const void *vp1, const void *vp2);
	static PointConvexHull p0;
};

#endif /* CONVEXHULL_H_ */
