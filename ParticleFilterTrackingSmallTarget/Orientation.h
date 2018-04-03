//
// Created by ynzhang on 18-3-30.
//

#ifndef SMALL_TARGET_TRACKING_BASE_PARTICLE_FILTER_ORIENTATION_H
#define SMALL_TARGET_TRACKING_BASE_PARTICLE_FILTER_ORIENTATION_H

class Orientation
{
public:
    explicit Orientation(int centerX = 0,
                         int centerY = 0,
                         int halfWidthOfTarget = 0,
                         int halfHeightOfTarget = 0)
            : _centerX(centerX),
              _centerY(centerY),
              _halfWidthOfTarget(halfWidthOfTarget),
              _halfHeightOfTarget(halfHeightOfTarget) {}

    int _centerX;
    int _centerY;
    int _halfWidthOfTarget;
    int _halfHeightOfTarget;
};

#endif //SMALL_TARGET_TRACKING_BASE_PARTICLE_FILTER_ORIENTATION_H
