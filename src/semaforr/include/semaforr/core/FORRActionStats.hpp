/*!
 * FORRActionStats.h
 *
 * Class that collects data about a decision
 *
 * \author Raj Korpan <rkorpan@gradcenter.cuny.edu>
 */
#ifndef SEMAFORR_CORE_FORR_ACTION_STATS_H
#define SEMAFORR_CORE_FORR_ACTION_STATS_H

#include <string>
class FORRActionStats {

  public:
    double planningComputationTime{0.0};
    double learningComputationTime{0.0};
    double graphingComputationTime{0.0};
    std::string chosenPlanner;
};


#endif
