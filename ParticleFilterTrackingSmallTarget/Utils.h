//
// Created by ynzhang on 18-3-30.
//

#ifndef SMALL_TARGET_TRACKING_BASE_PARTICLE_FILTER_UTILS_H
#define SMALL_TARGET_TRACKING_BASE_PARTICLE_FILTER_UTILS_H

class Utils
{
public:
    static void RandomGaussian(float *randomArray, const int n, const float mean = 0.0, const float sigma = 0.6);

private:
    static float randGaussian(float mean, float sigma);

    static float rand01();
};

#endif //SMALL_TARGET_TRACKING_BASE_PARTICLE_FILTER_UTILS_H
