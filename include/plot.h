#ifndef PLOT_H
#define PLOT_H

#include "matplotlibcpp.h"
#include "ekf.h"
#include "obj.h"
#include <deque>

void testMatplotlib();
void plotTrajectory(std::deque<tracking::obj_m> trajectory);
void plotTruthvsPred(std::deque<tracking::state> groudtruth, std::deque<tracking::state> prediction);

#endif /*PLOT_H*/