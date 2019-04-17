#include "plot.h"


void testMatplotlib()
{
    const std::string name = "ciao";
    std::vector<int> x, y;
    x.push_back(1);
    x.push_back(2);
    x.push_back(3);
    x.push_back(4);

    y.push_back(1);
    y.push_back(2);
    y.push_back(3);
    y.push_back(4);

    matplotlibcpp::named_plot(name,x, y);
    matplotlibcpp::annotate("1", 1,1);
    matplotlibcpp::annotate("2", 2,2);
    matplotlibcpp::annotate("3", 3, 3);
    matplotlibcpp::annotate("4", 4, 4);
    matplotlibcpp::legend();
    matplotlibcpp::show();
}

void plotTrajectory(std::vector<Data> trajectory)
{
    std::vector<float> x;
    std::vector<float> y;
    for(auto d: trajectory)
    {
        x.push_back(d.x_);
        y.push_back(d.y_);
    }
    matplotlibcpp::plot(x,y);
    for(auto d: trajectory)
        matplotlibcpp::annotate(std::to_string(d.frame_), d.x_, d.y_);
    matplotlibcpp::title("Trajectory");

    matplotlibcpp::show();
}

void plotTruthvsPred(std::vector<State> groudtruth, std::vector<State> prediction)
{

    std::vector<float> x_gt;
    std::vector<float> y_gt;
    for(auto gt: groudtruth)
    {
        x_gt.push_back(gt.x_);
        y_gt.push_back(gt.y_);
    }

    std::vector<float> x_pr;
    std::vector<float> y_pr;
    for(auto pr: prediction)
    {
        x_pr.push_back(pr.x_);
        y_pr.push_back(pr.y_);
    }

    matplotlibcpp::named_plot("Groundtruth", x_gt,y_gt, "r--");
    matplotlibcpp::named_plot("Prediction",x_pr,y_pr, "g");
    matplotlibcpp::legend();
    matplotlibcpp::show();
}


