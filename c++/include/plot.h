#ifndef PLOT_H
#define PLOT_H

#include "matplotlibcpp.h"
#include "ekf.h"
#include "Data.h"

void testMatplotlib();
void plotTrajectory(std::vector<Data> trajectory);
void plotTruthvsPred(std::vector<State> groudtruth, std::vector<State> prediction);

#endif /*PLOT_H*/