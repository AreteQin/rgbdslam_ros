#include "util/tukeys_biweight.h"

double TukeysBiweight(double error, double threshold_c)
{
    if (abs(error) > threshold_c)
    {
        return 0;
    }
    else
    {
        return abs(error * (1 - error * error / (threshold_c*threshold_c)) * (1 - error * error / (threshold_c*threshold_c)));
    }
}