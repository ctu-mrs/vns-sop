/*
 * convexhull.cpp
 *
 *  based on https://www.geeksforgeeks.org/convex-hull-set-2-graham-scan/
 */

#include "convexhull.h"

PointConvexHull convex_hull::p0;

vector<PointConvexHull> convex_hull::get_convex_hull(PointConvexHull points[], int n) {

	/*
	cout << "points beg" << endl;
	for (int i = 0; i < n; i++) {
		cout << "(" << points[i].x << ", " << points[i].y << ")" << endl;
	}
	cout << "points end" << endl;
	*/

	int ymin = points[0].y, min = 0;
	for (int i = 1; i < n; i++) {
		int y = points[i].y;

		if ((y < ymin) || (ymin == y && points[i].x < points[min].x))
			ymin = points[i].y, min = i;
	}

	swap(points[0], points[min]);

	p0 = points[0];
	qsort(&points[1], n - 1, sizeof(PointConvexHull), compare);

	int m = 1;
	for (int i = 1; i < n; i++) {
		while (i < n - 1 && orientation(p0, points[i], points[i + 1]) == 0)
			i++;

		points[m] = points[i];
		m++;
	}

	stack<PointConvexHull> S;
	std::vector<PointConvexHull> hull_points;

	if (m < 3)
		return hull_points;

	S.push(points[0]);
	S.push(points[1]);
	S.push(points[2]);


	for (int i = 3; i < m; i++) {
		while (orientation(nextToTop(S), S.top(), points[i]) != 2)
			S.pop();
		S.push(points[i]);
	}

	//cout << "hull beg" << endl;
	while (!S.empty()) {
		PointConvexHull p = S.top();
		hull_points.push_back(p);
		//cout << "(" << p.x << ", " << p.y << ")" << endl;
		S.pop();
	}
	//cout << "hull end" << endl;
	return hull_points;
}


PointConvexHull convex_hull::nextToTop(stack<PointConvexHull> &S) {
	PointConvexHull p = S.top();
	S.pop();
	PointConvexHull res = S.top();
	S.push(p);
	return res;
}

void convex_hull::swap(PointConvexHull &p1, PointConvexHull &p2) {
	PointConvexHull temp = p1;
	p1 = p2;
	p2 = temp;
}

int convex_hull::distSq(PointConvexHull p1, PointConvexHull p2) {
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}


int convex_hull::orientation(PointConvexHull p, PointConvexHull q, PointConvexHull r) {
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (val == 0)
		return 0;
	return (val > 0) ? 1 : 2;
}

int convex_hull::compare(const void *vp1, const void *vp2) {
	PointConvexHull *p1 = (PointConvexHull *) vp1;
	PointConvexHull *p2 = (PointConvexHull *) vp2;

	int o = orientation(p0, *p1, *p2);
	if (o == 0)
		return (distSq(p0, *p2) >= distSq(p0, *p1)) ? -1 : 1;

	return (o == 2) ? -1 : 1;
}

