#pragma once
#include <opencv2/core/core.hpp>
#include "State.h"
#include "Orientation.h"

#define BaseBin (16 * 256)
#define BIN (BaseBin + 3)  // 直方图条数
#define SHIFT 4 //log2( 256/8 )为移动位数

#define SIGMA2 0.02
#define ALPHA_COEFFICIENT 0.2 // 目标模型更新权重

class Tracker
{
public:
	explicit Tracker(unsigned short width, unsigned short height)
			: _width(width),
			  _height(height),
			  _curFrame(nullptr),
			  _particles(nullptr),
			  _DELTA_T(0.05),
			  _VELOCITY_DISTURB(40.0),
			  _SCALE_DISTURB(0.001),
			  _SCALE_CHANGE_D(0.001),
			  _nParticle(100),
			  _modelHist(nullptr),
			  _particleWeights(nullptr),
			  _nBin(BIN),
			  _piThreshold(0.9)
	{
	}

    ~Tracker();

	bool ParticleTracking(unsigned short *imageData, Orientation &trackingOrientation, float &maxWeight, cv::Mat& colorFrame);

	bool Initialize(const Orientation &initialOrientation, unsigned short *imageData);

    void SetParticleCount(unsigned int particleCount);

private:
	void ReSelect();

	void ImportanceSampling(int *ResampleIndex);

	void NormalizeCumulatedWeight(float *weight, float *cumulateWeight);

    static int BinearySearch(float value, float* NCumuWeight, int N);

	void CalcuModelHistogram(unsigned short *imageData, float *hist, const Orientation &orientation);

	void Propagate(cv::Mat& colorFrame);

	void Observe(unsigned short *imageData);

	float CalcuBhattacharyya(float* histA, float* histB) const;

	float CalcuWeightedPi(float rho) const;

	void Estimation(SpaceState &EstState);

	void ModelUpdate(SpaceState EstState, unsigned short *imageData);

	void GenerateParticles(const Orientation &initialOrientation) const;

	bool InitSpace();

private:
	unsigned short _width;  // Frame size : width
	unsigned short _height; // Frame size : height

	unsigned short* _curFrame;

	SpaceState* _particles; // 状态数组

    cv::Mat _trackingImg;

	float _DELTA_T;
	float _VELOCITY_DISTURB; // 速度扰动幅值
	float _SCALE_DISTURB;    // 窗宽高扰动幅度
	float _SCALE_CHANGE_D;   // 尺度变换速度扰动幅度

    unsigned int _nParticle;     // 粒子个数
    unsigned int _nBin;          // 直方图条数
	float* _modelHist;  // 模型直方图
	float* _particleWeights; // 每个粒子的权重
	float _piThreshold;      // 权重阈值
	Orientation previousOrientation; // 前一时刻的方位
	bool CheckDist(Orientation orientation, Orientation previousOrientation);
};
