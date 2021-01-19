#include "filter.h"

namespace tracking{

void Filter::printInternalState(){
    std::cout << "***************Internal state*********************" << std::endl;
    std::cout << "n_states: " << nStates << std::endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "P: \n"
            << P << std::endl;
    std::cout << "Q: \n"
            << Q << std::endl;
    std::cout << "R: \n"
            << R << std::endl;
    std::cout << "H: \n"
            << H << std::endl;
    xEst.print();
    std::cout << "**************************************************" << std::endl;
}

state Filter::getEstimatedState(){ return xEst; }
}
