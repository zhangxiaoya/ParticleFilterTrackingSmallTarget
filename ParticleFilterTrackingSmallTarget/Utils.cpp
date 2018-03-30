//
// Created by ynzhang on 18-3-30.
//

#include <cmath>
#include <cstdlib>
#include "Utils.h"

float Utils::randGaussian(float mean, float sigma)
{
    float v1;
    float s = 100.0;
    /*
    使用筛选法产生正态分布N(0,1)的随机数(Box-Mulles方法)
    1. 产生[0,1]上均匀随机变量X1,X2
    2. 计算V1=2*X1-1,V2=2*X2-1,s=V1^2+V2^2
    3. 若s<=1,转向步骤4，否则转1
    4. 计算A=(-2ln(s)/s)^(1/2),y1=V1*A, y2=V2*A
    y1,y2为N(0,1)随机变量
    */
    while (s > 1.0)
    {
        auto x1 = rand01();
        auto x2 = rand01();
        v1 = 2 * x1 - 1;
        auto v2 = 2 * x2 - 1;
        s = v1*v1 + v2*v2;
    }
    auto y = static_cast<float>(sqrt(-2.0 * std::log(s) / s) * v1);
    /*
    根据公式
    z = sigma * y + u
    将y变量转换成N(u,sigma)分布
    */
    return(sigma * y + mean);
}

float Utils::rand01()
{
    return (rand() / float(RAND_MAX));
}

void Utils::RandomGaussian(float *randomArray, const int n, const float mean, const float sigma)
{
    if(nullptr == randomArray || n <= 0)
        return;
    for(auto i = 0; i < n;++ i)
    {
        randomArray[i] = randGaussian(mean, sigma);
    }
}
