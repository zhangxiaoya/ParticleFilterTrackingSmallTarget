#include "Tracker.h"
#include "Utils.h"

/*
 * 粒子滤波跟踪算法
 */
int Tracker::ParticleTracking(unsigned short *imageData, Orientation &trackingOrientation, float &maxWeight)
{
	SpaceState estimateState;

	// 重采样
    ReSelect(_particles, _particleWeights);

	// 传播：采样状态方程，对状态变量进行预测
    Propagate(_particles);

	// 观测：对状态量进行更新
    Observe(_particles, _particleWeights, imageData);

	// 估计：对状态量进行估计，提取位置量
    Estimation(_particles, _particleWeights, estimateState);

    trackingOrientation = estimateState._orientation;

	// 模型更新
    ModelUpdate(estimateState, _modelHist, _piThreshold, imageData);

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
		return -1;
	return 1;
}

/*
初始化系统
int targetCenterX, targetCenterY：         初始给定的图像目标区域坐标
int halfWidthOfTarget, halfHeightOfTarget：目标的半宽高
unsigned char * img：                      图像数据，灰度形式
int width, height：                        图像宽高
*/
int Tracker::Initialize(const Orientation &initialOrientation, unsigned short *imageData)
{
	srand(static_cast<unsigned int>(time(nullptr)));

    // 初始化空间
    if(false == InitSpace())
        return -1;

	// 计算目标模板直方图
    CalcuModelHistogram(imageData, this->_modelHist, initialOrientation);

	// 初始化粒子状态(以(x0,y0,1,1,Wx,Hy,0.1)为中心呈N(0,0.4)正态分布)
    _particles[0]._orientation = initialOrientation;
	_particles[0].v_xt = static_cast<float>(0.0); // 1.0
	_particles[0].v_yt = static_cast<float>(0.0); // 1.0
	_particles[0].at_dot = static_cast<float>(0.0); // 0.1
	_particleWeights[0] = static_cast<float>(1.0 / _nParticle); // 0.9;

	GenerateParticles(initialOrientation);

	return 1;
}

void Tracker::GenerateParticles(const Orientation &initialOrientation) const
{
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

void Tracker::ReSelect(SpaceState *state, float *weight)
{
	// 存储新的粒子
	SpaceState* newParticles = new SpaceState[this->_nParticle];

	// 统计的随机数所再区间的索引
	auto resampleIndex = new int[this->_nParticle];

	// 根据权重重新采样
    ImportanceSampling(weight, resampleIndex);

	for (auto i = 0; i < this->_nParticle; i++)
		newParticles[i] = state[resampleIndex[i]];

	for (auto i = 0; i < this->_nParticle; i++)
		state[i] = newParticles[i];

	delete[] newParticles;
	delete[] resampleIndex;
}

/**
 * \brief 重新进行重要性采样
 * \param wights 对应样本权重数组pi(n)
 * \param ResampleIndex 重采样索引数组(输出)
 * \param NParticle 权重数组、重采样索引数组元素个数
 */
void Tracker::ImportanceSampling(float *wights, int *ResampleIndex)
{
	// 申请累计权重数组内存，大小为N+1
	auto cumulateWeight = new float[this->_nParticle + 1];
	// 计算累计权重
    NormalizeCumulatedWeight(wights, cumulateWeight);
	for (auto i = 0; i < this->_nParticle; i++)
	{
		// 随机产生一个[0,1]间均匀分布的数
		auto randomNumber = rand01();
		// 搜索<=randomNumber的最小索引j
		auto j = BinearySearch(randomNumber, cumulateWeight, this->_nParticle + 1);

		if (j == this->_nParticle)
			j--;
		// 放入重采样索引数组
		ResampleIndex[i] = j;
	}

	delete[] cumulateWeight;
}

/*
计算归一化累计概率c'_i
输入参数：
float * weight：    为一个有N个权重（概率）的数组
int N：             数组元素个数
输出参数：
float * cumulateWeight： 为一个有N+1个累计权重的数组，
cumulateWeight[0] = 0;
*/
void Tracker::NormalizeCumulatedWeight(float *weight, float *cumulateWeight)
{
	for (auto i = 0; i < this->_nParticle + 1; i++)
		cumulateWeight[i] = 0;
	for (auto i = 0; i < this->_nParticle; i++)
		cumulateWeight[i + 1] = cumulateWeight[i] + weight[i];
	for (auto i = 0; i < this->_nParticle + 1; i++)
		cumulateWeight[i] = cumulateWeight[i] / cumulateWeight[this->_nParticle];
}

/**
 * \brief 获得一个[0,1]之间的随机数
 * \return 返回一个0到1之间的随机数
 */
float Tracker::rand01()
{
	return (rand() / float(RAND_MAX));
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

/*
计算一幅图像中某个区域的彩色直方图分布
输入参数：
int targetCenterX, targetCenterY：          指定图像区域的中心点
int halfWidthOfTarget, halfHeightOfTarget： 指定图像区域的半宽和半高
unsigned char * imgData ：                  图像数据，按从左至右，从上至下的顺序扫描，
输出参数：
float * ColorHist：                         彩色直方图，颜色索引按：
i = r * G_BIN * B_BIN + g * B_BIN + b排列
int bins：                                  彩色直方图的条数R_BIN*G_BIN*B_BIN（这里取8x8x8=512）
*/
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
	if (xBeg < 0)
        xBeg = 0;
	if (yBeg < 0)
        yBeg = 0;

	auto xEnd = orientation._centerX + orientation._halfWidthOfTarget;
	auto yEnd = orientation._centerY + orientation._halfHeightOfTarget;
	if (xEnd >= this->_width )
        xEnd = this->_width - 1;
	if (yEnd >= this->_height)
        yEnd = this->_height - 1;

	// 计算半径平方a^2
    auto squareOfRadius = orientation._halfWidthOfTarget * orientation._halfWidthOfTarget +
                          orientation._halfHeightOfTarget * orientation._halfHeightOfTarget;

	// 归一化系数
	float f = 0.0;

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

/**
 * \brief 根据系统状态方程求取状态预测量, S(t) = A S(t-1) + W(t-1), 其中W(t-1)表示高斯噪声
 * \param state 待求的状态量数组
 */
void Tracker::Propagate(SpaceState *state)
{
    float randomNumbers[7];

    // 对每一个状态向量state[i](共N个)进行更新; 加入均值为0的随机高斯噪声
    for (auto i = 0; i < this->_nParticle; i++)
    {
        // 产生7个随机高斯分布的数
		Utils::RandomGaussian(randomNumbers, 7);

        state[i]._orientation._centerX = static_cast<int>(state[i]._orientation._centerX + state[i].v_xt * _DELTA_T +
                                                          randomNumbers[0] * state[i]._orientation._halfWidthOfTarget +
                                                          0.5);
        state[i]._orientation._centerY = static_cast<int>(state[i]._orientation._centerY + state[i].v_yt * _DELTA_T +
                                                          randomNumbers[1] * state[i]._orientation._halfHeightOfTarget +
                                                          0.5);
        state[i].v_xt = state[i].v_xt + randomNumbers[2] * _VELOCITY_DISTURB;
        state[i].v_yt = state[i].v_yt + randomNumbers[3] * _VELOCITY_DISTURB;
        state[i]._orientation._halfWidthOfTarget = static_cast<int>(state[i]._orientation._halfWidthOfTarget +
                                                                    state[i]._orientation._halfWidthOfTarget *
                                                                    state[i].at_dot +
                                                                    randomNumbers[4] * _SCALE_DISTURB + 0.5);
        state[i]._orientation._halfHeightOfTarget = static_cast<int>(state[i]._orientation._halfHeightOfTarget +
                                                                     state[i]._orientation._halfHeightOfTarget *
                                                                     state[i].at_dot +
                                                                     randomNumbers[5] * _SCALE_DISTURB + 0.5);
        state[i].at_dot = state[i].at_dot + randomNumbers[6] * _SCALE_CHANGE_D;

//        circle(_trackingImg, cv::Point(state[i]._orientation._centerX, state[i]._orientation._centerY), 3,
//               cv::Scalar(0, 255, 0), 1, 8, 3);
    }
}


/*
观测，根据状态集合St中的每一个采样，观测直方图，然后
更新估计量，获得新的权重概率
输入参数：
SPACESTATE * state：      状态量数组
unsigned char * image：   图像数据，按从左至右，从上至下的顺序扫描
float * ObjectHist：      目标直方图
int hbins：               目标直方图条数
输出参数：
float * weight：          更新后的权重
*/
void Tracker::Observe(SpaceState *state, float *weight, unsigned short *imageData)
{
	float * hist = new float[_nBin];

	for (auto i = 0; i < this->_nParticle; i++)
	{
		// (1) 计算彩色直方图分布
        CalcuModelHistogram(imageData, hist, state[i]._orientation);
		// (2) Bhattacharyya系数
		float rho = CalcuBhattacharyya(hist, _modelHist);
		// (3) 根据计算得的Bhattacharyya系数计算各个权重值
		weight[i] = CalcuWeightedPi(rho);
	}

	delete[] hist;
}

/**
 * \brief 计算Bhattachryya系数
 * \param histA 直方图A
 * \param histB 直方图B
 * \return Bhattacharyya系数
 */
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

/*
估计，根据权重，估计一个状态量作为跟踪输出
输入参数：
SPACESTATE * state：      状态量数组
float * weight：          对应权重
int N：                   状态量数组维数
输出参数：
SPACESTATE * EstState：   估计出的状态量
*/
void Tracker::Estimation(SpaceState *particles, float *weights, SpaceState &EstState)
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
		at_dot += particles[i].at_dot * weights[i];
		halfWidthOfTarget += particles[i]._orientation._halfWidthOfTarget * weights[i];
		halfHeightOfTarget += particles[i]._orientation._halfHeightOfTarget * weights[i];
		v_xt += particles[i].v_xt * weights[i];
		v_yt += particles[i].v_yt * weights[i];
		centerX += particles[i]._orientation._centerX * weights[i];
		centerY += particles[i]._orientation._centerY * weights[i];
		weight_sum += weights[i];
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
模型更新
输入参数：
SPACESTATE EstState：   状态量的估计值
float * TargetHist：    目标直方图
int bins：              直方图条数
float PiT：             阈值（权重阈值）
unsigned char * img：   图像数据，RGB形式
输出：
float * TargetHist：    更新的目标直方图
************************************************************/
void Tracker::ModelUpdate(SpaceState EstState, float *TargetHist, float PiT, unsigned short *imageData)
{
	auto estimatedHist = new float[this->_nBin];

	// (1)在估计值处计算目标直方图
    CalcuModelHistogram(imageData, estimatedHist, EstState._orientation);
	// (2)计算Bhattacharyya系数
	float Bha = CalcuBhattacharyya(estimatedHist, TargetHist);
	// (3)计算概率权重
	float Pi_E = CalcuWeightedPi(Bha);

	if (Pi_E > PiT)
	{
		for (auto i = 0; i < this->_nBin; i++)
		{
            TargetHist[i] = static_cast<float>((1.0 - ALPHA_COEFFICIENT) * TargetHist[i] +
                                               ALPHA_COEFFICIENT * estimatedHist[i]);
		}
	}
	delete[] estimatedHist;
}

/*********************************************************
 * * 功能： 初始化设置粒子的数量
 * * 参数： 粒子数量
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
