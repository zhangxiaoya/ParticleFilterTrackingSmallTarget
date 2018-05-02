#include <iostream>
#include "Tracker.h"
#include "Utils.h"

/****************************************
 * 粒子滤波跟踪算法
 ***************************************/
bool Tracker::ParticleTracking(unsigned short *imageData, Orientation &trackingOrientation, float &maxWeight)
{
	SpaceState estimateState;

	// 重采样
    ReSelect();

	// 传播：采样状态方程，对状态变量进行预测
    Propagate();

	// 观测：对状态量进行更新
    Observe(imageData);

	// 估计：对状态量进行估计，提取位置量
    Estimation(estimateState);

    // 检查是否发生了很大的偏移（跟踪失败）。如果没有，跟踪成功
	if(CheckDist(estimateState._orientation, previousOrientation))
	{
        // 更新目标的方位
		trackingOrientation = estimateState._orientation;

        // 更新前一时刻的状态
		previousOrientation = estimateState._orientation;

		// 模型更新
        ModelUpdate(estimateState, imageData);
	}
	else // 若失败，重新初始化粒子
	{
		GenerateParticles(previousOrientation);

		std::cout << "Lost target!" << std::endl;
	}

	// 计算最大权重值
	maxWeight = _particleWeights[0];
	for (auto i = 1; i < _nParticle; i++)
		maxWeight = maxWeight < _particleWeights[i] ? _particleWeights[i] : maxWeight;

	// 进行合法性检验，不合法返回-1
	if (trackingOrientation._centerX < 0
        || trackingOrientation._centerY < 0
        || trackingOrientation._centerX >= this->_width
        || trackingOrientation._centerY >= this->_height
        || trackingOrientation._halfWidthOfTarget <= 0
        || trackingOrientation._halfHeightOfTarget <= 0)
		return false;
	return true;
}

/***************************************************************
 * 初始化系统
 * const Orientation &initialOrientation      初始给定的图像目标区域方位
 * unsigned char * imageData：                图像数据，灰度形式
****************************************************************/
bool Tracker::Initialize(const Orientation &initialOrientation, unsigned short *imageData)
{
	srand(static_cast<unsigned int>(time(nullptr)));

	previousOrientation = initialOrientation;

    // 初始化空间
    if(false == InitSpace())
        return false;

	// 计算目标模板直方图
    CalcuModelHistogram(imageData, this->_modelHist, initialOrientation);

    // 初始化粒子
	GenerateParticles(initialOrientation);

	return true;
}

void Tracker::GenerateParticles(const Orientation &initialOrientation) const
{
	// 初始化粒子状态(以(x0,y0,1,1,Wx,Hy,0.1)为中心呈N(0,0.4)正态分布)
	_particles[0]._orientation = initialOrientation;
	_particles[0].v_xt = static_cast<float>(0.0); // 1.0
	_particles[0].v_yt = static_cast<float>(0.0); // 1.0
	_particles[0].at_dot = static_cast<float>(0.0); // 0.1
	_particleWeights[0] = static_cast<float>(1.0 / _nParticle); // 0.9;

	float randomNumbers[7];
	for (auto i = 1; i < _nParticle; i++)
	{
		// 产生7个随机高斯分布的数
        Utils::RandomGaussian(randomNumbers, 7);

        _particles[i]._orientation._centerX = static_cast<int>(_particles[0]._orientation._centerX + randomNumbers[0] *
																									 initialOrientation._halfWidthOfTarget);
        _particles[i]._orientation._centerY = static_cast<int>(_particles[0]._orientation._centerY + randomNumbers[1] *
																									 initialOrientation._halfHeightOfTarget);
        _particles[i].v_xt = _particles[0].v_xt + randomNumbers[2] * _VELOCITY_DISTURB;
        _particles[i].v_yt = _particles[0].v_yt + randomNumbers[3] * _VELOCITY_DISTURB;
        _particles[i]._orientation._halfWidthOfTarget = static_cast<int>(_particles[0]._orientation._halfWidthOfTarget +
																		 randomNumbers[4] * _SCALE_DISTURB);
        _particles[i]._orientation._halfHeightOfTarget = static_cast<int>(
                _particles[0]._orientation._halfHeightOfTarget + randomNumbers[5] * _SCALE_DISTURB);
        _particles[i].at_dot = _particles[0].at_dot + randomNumbers[6] * _SCALE_CHANGE_D;

		// 权重统一为1/N，让每个粒子有相等的机会
		_particleWeights[i] = static_cast<float>(1.0 / _nParticle);
	}
}

/*****************************************
 * 初始化 空间
 *****************************************/
bool Tracker::InitSpace()
{
    // 申请状态数组的空间
    this->_particles = new SpaceState[_nParticle];
    if(nullptr == this->_particles)
        return false;

    // 申请粒子权重数组的空间
    _particleWeights = new float[_nParticle];
    if(nullptr == this->_particleWeights)
        return false;

    // 申请存放模板的内存
    this->_modelHist = new float[_nBin];
    if (nullptr == this->_modelHist)
        return false;

    return true;
}

void Tracker::ReSelect()
{
	// 存储新的粒子
	SpaceState* newParticles = new SpaceState[this->_nParticle];

	// 统计的随机数所再区间的索引
	auto resampleIndex = new int[this->_nParticle];

	// 根据权重重新采样
    ImportanceSampling(resampleIndex);

	for (auto i = 0; i < this->_nParticle; i++)
		newParticles[i] = this->_particles[resampleIndex[i]];

	for (auto i = 0; i < this->_nParticle; i++)
        this->_particles[i] = newParticles[i];

	delete[] newParticles;
	delete[] resampleIndex;
}

/***************************************************
 * 重新进行重要性采样
 * wights 对应样本权重数组pi(n)
 * ResampleIndex 重采样索引数组(输出)
 * NParticle 权重数组、重采样索引数组元素个数
 ***************************************************/
void Tracker::ImportanceSampling(int *ResampleIndex)
{
	// 申请累计权重数组内存，大小为N+1
	auto cumulateWeight = new float[this->_nParticle + 1];
	// 计算累计权重
    NormalizeCumulatedWeight(this->_particleWeights, cumulateWeight);
	for (auto i = 0; i < this->_nParticle; i++)
	{
		// 随机产生一个[0,1]间均匀分布的数
		auto randomNumber = Utils::rand01();
		// 搜索<=randomNumber的最小索引j
		auto j = BinearySearch(randomNumber, cumulateWeight, this->_nParticle + 1);

		if (j == this->_nParticle)
			j--;
		// 放入重采样索引数组
		ResampleIndex[i] = j;
	}

	delete[] cumulateWeight;
}

/*****************************************************
 * 计算归一化累计概率c'_i
 * float * weight：    为一个有N个权重（概率）的数组
 * float * cumulateWeight： 为一个有N+1个累计权重的数组
*****************************************************/
void Tracker::NormalizeCumulatedWeight(float *weight, float *cumulateWeight)
{
	for (auto i = 0; i < this->_nParticle + 1; i++)
		cumulateWeight[i] = 0;
	for (auto i = 0; i < this->_nParticle; i++)
		cumulateWeight[i + 1] = cumulateWeight[i] + weight[i];
	for (auto i = 0; i < this->_nParticle + 1; i++)
		cumulateWeight[i] = cumulateWeight[i] / cumulateWeight[this->_nParticle];
}

int Tracker::BinearySearch(float v, float* NCumuWeight, int N)
{
	auto left = 0;
	auto right = N - 1;
	while (right >= left)
	{
		auto mid = (left + right) / 2;
		if (v >= NCumuWeight[mid] && v < NCumuWeight[mid + 1])
			return mid;
		if (v < NCumuWeight[mid])
			right = mid - 1;
		else
			left = mid + 1;
	}
	return 0;
}

/**********************************************************************
* 计算特征直方图分布
* 输入参数：
* unsigned char * imgData ：           图像数据，按从左至右，从上至下的顺序扫描
* const Orientation &orientation       目标的方位
* 输出参数：
* float *hist：                        特征直方图
**********************************************************************/
void Tracker::CalcuModelHistogram(unsigned short *imageData, float *hist, const Orientation &orientation)
{
	// 直方图各个值赋0
	for (auto i = 0; i < _nBin; i++)
		hist[i] = 0.0;

	// 考虑特殊情况：centerX, centerY在图像外面，或者，halfWidthOfTarget<=0, halfHeightOfTarget<=0,此时强制直方图为0
    if ((orientation._centerX < 0)
        || (orientation._centerY < 0)
        || (orientation._centerX >= this->_width)
        || (orientation._centerY >= this->_height)
        || (orientation._halfWidthOfTarget <= 0)
        || (orientation._halfHeightOfTarget <= 0))
        return;

	// 计算实际高宽和区域起始点, 超出范围的话就用画的框的边界来赋值粒子的区域
	auto xBeg = orientation._centerX - orientation._halfWidthOfTarget;
	auto yBeg = orientation._centerY - orientation._halfHeightOfTarget;
	if (xBeg < 0) xBeg = 0;
	if (yBeg < 0) yBeg = 0;

	auto xEnd = orientation._centerX + orientation._halfWidthOfTarget;
	auto yEnd = orientation._centerY + orientation._halfHeightOfTarget;
	if (xEnd >= this->_width ) xEnd = this->_width - 1;
	if (yEnd >= this->_height) yEnd = this->_height - 1;

	// 计算目标的外接BoundingBox的宽高
	auto bxBeg = orientation._centerX - 2 * orientation._halfWidthOfTarget;
	auto byBeg = orientation._centerY - 2 * orientation._halfHeightOfTarget;
	if (bxBeg < 0) bxBeg = 0;
	if (byBeg < 0) byBeg = 0;

	auto bxEnd = orientation._centerX + 2 * orientation._halfWidthOfTarget;
	auto byEnd = orientation._centerY + 2 * orientation._halfHeightOfTarget;
	if (bxEnd >= this->_width ) bxEnd = this->_width - 1;
	if (byEnd >= this->_height) byEnd = this->_height - 1;

	// 计算目标区域的像素值均值
	auto sum = 0;
	for(int r = yBeg; r <= yEnd; ++r)
	{
		for(int c = xBeg; c <= xEnd; ++c)
		{
			sum += imageData[r * this->_width + c];
		}
	}
	auto avgInter = (float)(sum * 1.0) / (orientation._halfHeightOfTarget * orientation._halfWidthOfTarget * 4);

	// 计算目标外接BoundingBox的像素均值
	sum = 0;
	for(int r = byBeg; r < byEnd; ++ r)
	{
		for(int c = bxBeg; c < bxEnd; ++ c)
		{
			sum += imageData[r * this->_width + c];
		}
	}
	auto avgBox = (float)(sum * 1.0) / ((yEnd - yBeg + 1) * (yEnd - yBeg + 1));

	// 计算面积
	auto area = 0;
	for(int r = yBeg; r <= yEnd; ++r)
	{
		for(int c = xBeg; c <= xEnd; ++ c)
		{
			if((float)imageData[r * this->_width + c] - avgInter > 0.00001)
				area ++;
		}
	}

	// 计算附加特征
	auto sumHist = 0.0;
	// 对比度
	hist[ BaseBin + 0] = avgInter /avgBox;
	sumHist += hist[BaseBin + 0];
	// 均值（进行归一化）
	hist[BaseBin + 1] = avgInter * 1.0 / (1 << 14);
	sumHist += hist[BaseBin + 1];
	// 面积（归一化）
	hist[BaseBin + 2] = area * 1.0 / (orientation._halfHeightOfTarget * orientation._halfWidthOfTarget * 4);
	sumHist += hist[BaseBin + 2];

	// 计算半径平方a^2
    auto squareOfRadius = orientation._halfWidthOfTarget * orientation._halfWidthOfTarget +
                          orientation._halfHeightOfTarget * orientation._halfHeightOfTarget;

	// 归一化系数
	float f = sumHist;

	for (auto y = yBeg; y <= yEnd; y++)
	{
		for (auto x = xBeg; x <= xEnd; x++)
		{
			auto v = imageData[y * this->_width + x] >> SHIFT;
			//把当前rgb换成一个索引
			auto index = v;

            auto squareOfRadiusFromCurPixelToCenter = (
                    (y - orientation._halfHeightOfTarget) * (y - orientation._halfHeightOfTarget) +
                    (x - orientation._halfWidthOfTarget) * (x - orientation._halfWidthOfTarget));
			// 计算当前像素到中心点的半径平方r^2
			auto r2 = static_cast<float>(squareOfRadiusFromCurPixelToCenter * 1.0 / squareOfRadius);
			// k(r) = 1-r^2, |r| < 1; 其他值 k(r) = 0 ，影响力
			auto k = 1 - r2;
			f = f + k;

			// 计算核密度加权灰度直方图
			hist[index] = hist[index] + k;
		}
	}

	// 归一化直方图
	for (auto i = 0; i < _nBin; i++)
		hist[i] = hist[i] / f;
}

/***********************************************************
 * 根据系统状态方程求取状态预测量,
 * S(t) = A S(t-1) + W(t-1), 其中W(t-1)表示高斯噪声
 ***********************************************************/
void Tracker::Propagate()
{
    float randomNumbers[7];

    // 对每一个状态向量state[i](共N个)进行更新; 加入均值为0的随机高斯噪声
    for (auto i = 0; i < this->_nParticle; i++)
    {
        // 产生7个随机高斯分布的数
		Utils::RandomGaussian(randomNumbers, 7);

        _particles[i]._orientation._centerX = static_cast<int>(_particles[i]._orientation._centerX +
                                                               _particles[i].v_xt * _DELTA_T +
                                                               randomNumbers[0] *
                                                               _particles[i]._orientation._halfWidthOfTarget +
                                                               0.5);
        _particles[i]._orientation._centerY = static_cast<int>(_particles[i]._orientation._centerY +
                                                               _particles[i].v_yt * _DELTA_T +
                                                               randomNumbers[1] *
                                                               _particles[i]._orientation._halfHeightOfTarget +
                                                               0.5);
        _particles[i].v_xt = _particles[i].v_xt + randomNumbers[2] * _VELOCITY_DISTURB;
        _particles[i].v_yt = _particles[i].v_yt + randomNumbers[3] * _VELOCITY_DISTURB;
        _particles[i]._orientation._halfWidthOfTarget = static_cast<int>(_particles[i]._orientation._halfWidthOfTarget +
                                                                         _particles[i]._orientation._halfWidthOfTarget *
                                                                         _particles[i].at_dot +
                                                                         randomNumbers[4] * _SCALE_DISTURB + 0.5);
        _particles[i]._orientation._halfHeightOfTarget = static_cast<int>(
                _particles[i]._orientation._halfHeightOfTarget +
                _particles[i]._orientation._halfHeightOfTarget *
                _particles[i].at_dot +
                randomNumbers[5] * _SCALE_DISTURB + 0.5);
        _particles[i].at_dot = _particles[i].at_dot + randomNumbers[6] * _SCALE_CHANGE_D;

//        circle(_trackingImg, cv::Point(state[i]._orientation._centerX, state[i]._orientation._centerY), 3,
//               cv::Scalar(0, 255, 0), 1, 8, 3);
    }
}


/*************************************************************
 * 观测，根据状态集合St中的每一个采样，观测直方图，然后
 * 更新估计量，获得新的权重概率
 * unsigned char * image：   图像数据，按从左至右，从上至下的顺序扫描
**************************************************************/
void Tracker::Observe(unsigned short *imageData)
{
	float * hist = new float[_nBin];

	for (auto i = 0; i < this->_nParticle; i++)
	{
		// (1) 计算彩色直方图分布
        CalcuModelHistogram(imageData, hist, this->_particles[i]._orientation);
		// (2) Bhattacharyya系数
		float rho = CalcuBhattacharyya(hist, _modelHist);
		// (3) 根据计算得的Bhattacharyya系数计算各个权重值
		this->_particleWeights[i] = CalcuWeightedPi(rho);
	}

	delete[] hist;
}

/*****************************************************************
 * 计算Bhattachryya系数
 * histA 直方图A
 * histB 直方图B
 * Bhattacharyya系数
 *****************************************************************/
float Tracker::CalcuBhattacharyya(float* histA, float* histB) const
{
	float rho = 0.0;
	for (auto i = 0; i < _nBin; i++)
		rho = static_cast<float>(rho + sqrt(histA[i] * histB[i]));
	return rho;
}

float Tracker::CalcuWeightedPi(float rho) const
{
	auto d2 = 1 - rho;
	auto pi_n = static_cast<float>(exp(-d2 / SIGMA2));
	return pi_n;
}

/**********************************************
 * 估计，根据权重，估计一个状态量作为跟踪输出
 * SPACESTATE * EstState：   估计出的状态量
**********************************************/
void Tracker::Estimation(SpaceState &EstState)
{
	float at_dot = 0;
	float halfWidthOfTarget = 0;
	float halfHeightOfTarget = 0;
	float v_xt = 0;
	float v_yt = 0;
	float centerX = 0;
	float centerY = 0;
	float weight_sum = 0;
	for (auto i = 0; i < this->_nParticle; i++) /* 求和 */
	{
		at_dot += _particles[i].at_dot * _particleWeights[i];
		halfWidthOfTarget += _particles[i]._orientation._halfWidthOfTarget * _particleWeights[i];
		halfHeightOfTarget += _particles[i]._orientation._halfHeightOfTarget * _particleWeights[i];
		v_xt += _particles[i].v_xt * _particleWeights[i];
		v_yt += _particles[i].v_yt * _particleWeights[i];
		centerX += _particles[i]._orientation._centerX * _particleWeights[i];
		centerY += _particles[i]._orientation._centerY * _particleWeights[i];
		weight_sum += _particleWeights[i];
	}

	// 求平均
	if (weight_sum <= 0)
		weight_sum = 1; // 防止被0除，一般不会发生

	EstState.at_dot = at_dot / weight_sum;
	EstState._orientation._halfWidthOfTarget = static_cast<int>(halfWidthOfTarget / weight_sum);
	EstState._orientation._halfHeightOfTarget = static_cast<int>(halfHeightOfTarget / weight_sum);
	EstState.v_xt = v_xt / weight_sum;
	EstState.v_yt = v_yt / weight_sum;
	EstState._orientation._centerX = static_cast<int>(centerX / weight_sum);
	EstState._orientation._centerY = static_cast<int>(centerY / weight_sum);
}

/************************************************************
 * 模型更新
 * SPACESTATE EstState       状态量的估计值
 * unsigned short* imageData 图像数据
************************************************************/
void Tracker::ModelUpdate(SpaceState EstState, unsigned short *imageData)
{
	auto estimatedHist = new float[this->_nBin];

	// (1)在估计值处计算目标直方图
    CalcuModelHistogram(imageData, estimatedHist, EstState._orientation);
	// (2)计算Bhattacharyya系数
	float Bha = CalcuBhattacharyya(estimatedHist, _modelHist);
	// (3)计算概率权重
	float Pi_E = CalcuWeightedPi(Bha);

	if (Pi_E > _piThreshold)
	{
		for (auto i = 0; i < this->_nBin; i++)
		{
            _modelHist[i] = static_cast<float>((1.0 - ALPHA_COEFFICIENT) * _modelHist[i] +
                                               ALPHA_COEFFICIENT * estimatedHist[i]);
		}
	}
	delete[] estimatedHist;
}

/*********************************************************
 * 功能： 初始化设置粒子的数量
 * 参数： 粒子数量
 ********************************************************/
void Tracker::SetParticleCount(const unsigned int particleCount)
{
	this->_nParticle = particleCount;
}

Tracker::~Tracker()
{
    if (this->_particles)
    {
        delete[] this->_particles;
        this->_particles = nullptr;
    }
    if (this->_particleWeights)
    {
        delete[] this->_particleWeights;
        this->_particleWeights = nullptr;
    }
}

bool Tracker::CheckDist(Orientation orientation, Orientation previousOrientation)
{
	if(abs(orientation._centerX - previousOrientation._centerX) < 5 &&
			abs(orientation._centerY - previousOrientation._centerY) < 5)
		return true;
	return false;
}
