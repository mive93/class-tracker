#ifndef PLOT_H
#define PLOT_H

#include "matplotlibcpp.h"
#include "ekf.h"
#include "obj.h"

void testMatplotlib();
void plotTrajectory(std::vector<tracking::obj_m> trajectory);
void plotTruthvsPred(std::vector<tracking::state> groudtruth, std::vector<tracking::state> prediction);

#endif /*PLOT_H*/