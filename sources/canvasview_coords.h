/*
 * File name: canvasview_coords.h
 * Date:      2013/10/13 09:26
 * Author:    Jan Faigl
 */

#ifndef __CANVASVIEW_COORDS_H__
#define __CANVASVIEW_COORDS_H__

#include <boost/foreach.hpp>

#include "crl/gui/gui.h"
#include "crl/gui/colors.h"
#include "crl/gui/renderer.h"
#include "crl/gui/canvas.h"

#include "coords.h"

#include "crl/logging.h"
using crl::logger;

#include "heuristic_types.h"

/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase& canvas, const Coords& coords) {
	//INFO(coords.x <<" , "<< coords.y);
	canvas << coords.x << coords.y;
	return canvas;
}

/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase& canvas, const std::vector<Coords>& points) {
	BOOST_FOREACH(const Coords& pt, points) {
		canvas << pt;
	}
	return canvas;
}



/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase& canvas, const GraphNode& coords) {
	//INFO(coords.x <<" , "<< coords.y);
	canvas << coords.x << coords.y;
	return canvas;
}



/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase& canvas, const Tour tour) {
	int S = tour.tour.size();
	for (int var = 0; var < S; ++var) {
		canvas << tour.tour[var];
	}
	return canvas;
}

#endif

/* end of canvasview_coords.h */
