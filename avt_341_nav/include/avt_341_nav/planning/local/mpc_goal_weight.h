#ifndef AVT_341_MPC_GOAL_WEIGHT_H
#define AVT_341_MPC_GOAL_WEIGHT_H

#include <cmath>

// The active task's override when it has one, else the w_distance_to_goal parameter.
inline double EffectiveGoalWeight(double override_weight, double parameter_weight)
{
    return override_weight > 0.0 && std::isfinite(override_weight) ? override_weight : parameter_weight;
}

#endif
