#include "Tracker.h"

int Tracker::ParticleTracking(unsigned short *imageData, Orientation &trackingOrientation, float &max_weight)
{
	SpaceState estimateState;

	// 重采样
	ReSelect(_particles, _particleWeights, _nParticle);

	// 传播：采样状态方程，对状态变量进行预测
	Propagate(_particles, _nParticle);

	// 观测：对状态量进行更新
	Observe(_particles, _particleWeights, _nParticle, imageData, this->_width, this->_height);

	// 估计：对状态量进行估计，提取位置量
	Estimation(_particles, _particleWeights, _nParticle, estimateState);

	trackingOrientation._centerX = estimateState.centerX;
	trackingOrientation._centerY = estimateState.centerY;
	trackingOrientation._halfWidthOfTarget = estimateState._halfWidthOfTarget;
	trackingOrientation._halfHeightOfTarget = estimateState._halfHeightOfTarget;

	// 模型更新
	ModelUpdate(estimateState, _modelHist, _nBin, _piThreshold, imageData, this->_width, this->_height);

	// 计算最大权重值
	max_weight = _particleWeights[0];
	for (auto i = 1; i < _nParticle; i++)
		max_weight = max_weight < _particleWeights[i] ? _particleWeights[i] : max_weight;

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
int Tracker::Initialize(const Orientation &initialOrientation, unsigned short *imgData)
{
	srand(static_cast<unsigned int>(time(nullptr)));

	// 申请状态数组的空间
	_particles = new SpaceState[_nParticle];
	// 申请粒子权重数组的空间
	_particleWeights = new float[_nParticle];
	// 确定直方图条数
	_nBin = BIN;
	// 申请存放模板的内存
	_modelHist = new float[_nBin];
	if (_modelHist == nullptr)
		return (-1);

	// 计算目标模板直方图
	CalcuModelHistogram(initialOrientation._centerX,
						initialOrientation._centerY,
						initialOrientation._halfWidthOfTarget,
						initialOrientation._halfHeightOfTarget,
						imgData,
						this->_width,
						this->_height,
						_modelHist);

	// 初始化粒子状态(以(x0,y0,1,1,Wx,Hy,0.1)为中心呈N(0,0.4)正态分布)
	_particles[0].centerX = initialOrientation._centerX;
	_particles[0].centerY = initialOrientation._centerY;
	_particles[0].v_xt = static_cast<float>(0.0); // 1.0
	_particles[0].v_yt = static_cast<float>(0.0); // 1.0
	_particles[0]._halfWidthOfTarget = initialOrientation._halfWidthOfTarget;
	_particles[0]._halfHeightOfTarget = initialOrientation._halfHeightOfTarget;
	_particles[0].at_dot = static_cast<float>(0.0); // 0.1
	_particleWeights[0] = static_cast<float>(1.0 / _nParticle); // 0.9;

	float randomNumbers[7];
	for (auto i = 1; i < _nParticle; i++)
	{
		// 产生7个随机高斯分布的数
		for (auto j = 0; j < 7; j++)
			randomNumbers[j] = randGaussian(0, static_cast<float>(0.6));

		_particles[i].centerX = static_cast<int>(_particles[0].centerX + randomNumbers[0] * initialOrientation._halfWidthOfTarget);
		_particles[i].centerY = static_cast<int>(_particles[0].centerY + randomNumbers[1] * initialOrientation._halfHeightOfTarget);
		_particles[i].v_xt = static_cast<float>(_particles[0].v_xt + randomNumbers[2] * _VELOCITY_DISTURB);
		_particles[i].v_yt = static_cast<float>(_particles[0].v_yt + randomNumbers[3] * _VELOCITY_DISTURB);
		_particles[i]._halfWidthOfTarget = static_cast<int>(_particles[0]._halfWidthOfTarget + randomNumbers[4] * _SCALE_DISTURB);
		_particles[i]._halfHeightOfTarget = static_cast<int>(_particles[0]._halfHeightOfTarget + randomNumbers[5] * _SCALE_DISTURB);
		_particles[i].at_dot = static_cast<float>(_particles[0].at_dot + randomNumbers[6] * _SCALE_CHANGE_D);

		// 权重统一为1/N，让每个粒子有相等的机会
		_particleWeights[i] = static_cast<float>(1.0 / _nParticle);
	}
	return 1;
}

void Tracker::ReSelect(SpaceState* state, float* weight, int nParticle)
{
	// 存储新的粒子
	SpaceState* newParticles = new SpaceState[nParticle];

	// 统计的随机数所再区间的索引
	int* resampleIndex = new int[nParticle];

	// 根据权重重新采样
	ImportanceSampling(weight, resampleIndex, nParticle);

	for (auto i = 0; i < nParticle; i++)
		newParticles[i] = state[resampleIndex[i]];

	for (auto i = 0; i < nParticle; i++)
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
void Tracker::ImportanceSampling(float* wights, int* ResampleIndex, int NParticle)
{
	// 申请累计权重数组内存，大小为N+1
	auto cumulateWeight = new float[NParticle + 1];
	// 计算累计权重
	NormalizeCumulatedWeight(wights, cumulateWeight, NParticle);
	for (auto i = 0; i < NParticle; i++)
	{
		// 随机产生一个[0,1]间均匀分布的数
		auto randomNumber = rand01();
		// 搜索<=randomNumber的最小索引j
		auto j = BinearySearch(randomNumber, cumulateWeight, NParticle + 1);

		if (j == NParticle)
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
void Tracker::NormalizeCumulatedWeight(float* weight, float* cumulateWeight, int N)
{
	for (auto i = 0; i < N + 1; i++)
		cumulateWeight[i] = 0;
	for (auto i = 0; i < N; i++)
		cumulateWeight[i + 1] = cumulateWeight[i] + weight[i];
	for (auto i = 0; i < N + 1; i++)
		cumulateWeight[i] = cumulateWeight[i] / cumulateWeight[N];
}

/**
 * \brief 获得一个[0,1]之间的随机数
 * \return 返回一个0到1之间的随机数
 */
float Tracker::rand01()
{
	return (rand() / float(RAND_MAX));
}

/**
 * \brief 产生一个高斯分布的随机数
 * \param u 高斯分布的均值
 * \param sigma 高斯分布的标准差
 * \return 高斯分布的随机数
 */
float Tracker::randGaussian(float u, float sigma) const
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
	auto y = static_cast<float>(sqrt(-2.0 * log(s) / s) * v1);
	/*
	根据公式
	z = sigma * y + u
	将y变量转换成N(u,sigma)分布
	*/
	return(sigma * y + u);
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
int width, height：                         图像的宽和高
输出参数：
float * ColorHist：                         彩色直方图，颜色索引按：
i = r * G_BIN * B_BIN + g * B_BIN + b排列
int bins：                                  彩色直方图的条数R_BIN*G_BIN*B_BIN（这里取8x8x8=512）
*/
void Tracker::CalcuModelHistogram(int targetCenterX, int targetCenterY,
								  int halfWidthOfTarget, int halfHeightOfTarget,
								  unsigned short* imgData,
								  int width, int height,
								  float* hist)
{
	// 直方图各个值赋0
	for (auto i = 0; i < _nBin; i++)
		hist[i] = 0.0;

	// 考虑特殊情况：centerX, centerY在图像外面，或者，halfWidthOfTarget<=0, halfHeightOfTarget<=0,此时强制令彩色直方图为0
	if ((targetCenterX < 0) || (targetCenterX >= width) || (targetCenterY < 0) || (targetCenterY >= height) || (halfWidthOfTarget <= 0) || (halfHeightOfTarget <= 0))
		return;

	// 计算实际高宽和区域起始点, 超出范围的话就用画的框的边界来赋值粒子的区域
	auto xBeg = targetCenterX - halfWidthOfTarget;
	auto yBeg = targetCenterY - halfHeightOfTarget;
	if (xBeg < 0) xBeg = 0;
	if (yBeg < 0) yBeg = 0;

	auto xEnd = targetCenterX + halfWidthOfTarget;
	auto yEnd = targetCenterY + halfHeightOfTarget;
	if (xEnd >= width) xEnd = width - 1;
	if (yEnd >= height) yEnd = height - 1;

	// 计算半径平方a^2
	auto squareOfRadius = halfWidthOfTarget*halfWidthOfTarget + halfHeightOfTarget*halfHeightOfTarget;

	// 归一化系数
	float f = 0.0;

	for (auto y = yBeg; y <= yEnd; y++)
	{
		for (auto x = xBeg; x <= xEnd; x++)
		{
			auto v = imgData[y * width + x] >> SHIFT;
			//把当前rgb换成一个索引
			auto index = v;

			auto squareOfRadiusFromCurPixelToCenter = ((y - targetCenterY) * (y - targetCenterY) + (x - targetCenterX) * (x - targetCenterX));
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
 * \param NParticle 待求状态个数
 */
void Tracker::Propagate(SpaceState* state, int NParticle)
{
	float randomNumbers[7];

	// 对每一个状态向量state[i](共N个)进行更新; 加入均值为0的随机高斯噪声
	for (auto i = 0; i < NParticle; i++)
	{
		// 产生7个随机高斯分布的数
		for (auto j = 0; j < 7; j++)
			randomNumbers[j] = randGaussian(0, static_cast<float>(0.6));

		state[i].centerX = static_cast<int>(state[i].centerX + state[i].v_xt * _DELTA_T + randomNumbers[0] * state[i]._halfWidthOfTarget + 0.5);
		state[i].centerY = static_cast<int>(state[i].centerY + state[i].v_yt * _DELTA_T + randomNumbers[1] * state[i]._halfHeightOfTarget + 0.5);
		state[i].v_xt = static_cast<float>(state[i].v_xt + randomNumbers[2] * _VELOCITY_DISTURB);
		state[i].v_yt = static_cast<float>(state[i].v_yt + randomNumbers[3] * _VELOCITY_DISTURB);
		state[i]._halfWidthOfTarget = static_cast<int>(state[i]._halfWidthOfTarget + state[i]._halfWidthOfTarget * state[i].at_dot + randomNumbers[4] * _SCALE_DISTURB + 0.5);
		state[i]._halfHeightOfTarget = static_cast<int>(state[i]._halfHeightOfTarget + state[i]._halfHeightOfTarget * state[i].at_dot + randomNumbers[5] * _SCALE_DISTURB + 0.5);
		state[i].at_dot = static_cast<float>(state[i].at_dot + randomNumbers[6] * _SCALE_CHANGE_D);

		circle(_trackingImg, cv::Point(state[i].centerX, state[i].centerY), 3, cv::Scalar(0, 255, 0), 1, 8, 3);
	}
}


/*
观测，根据状态集合St中的每一个采样，观测直方图，然后
更新估计量，获得新的权重概率
输入参数：
SPACESTATE * state：      状态量数组
int N：                   状态量数组维数
unsigned char * image：   图像数据，按从左至右，从上至下的顺序扫描，
颜色排列次序：RGB, RGB, ...
int width, height：                图像的宽和高
float * ObjectHist：      目标直方图
int hbins：               目标直方图条数
输出参数：
float * weight：          更新后的权重
*/
void Tracker::Observe(SpaceState* state, float* weight, int NParticle, unsigned short* imgData, int W, int H)
{
	float * hist = new float[_nBin];

	for (auto i = 0; i < NParticle; i++)
	{
		// (1) 计算彩色直方图分布
		CalcuModelHistogram(state[i].centerX, state[i].centerY, state[i]._halfWidthOfTarget, state[i]._halfHeightOfTarget, imgData, W, H, hist);
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
void Tracker::Estimation(SpaceState* particles, float* weights, int NParticle, SpaceState& EstState)
{
	float at_dot = 0;
	float halfWidthOfTarget = 0;
	float halfHeightOfTarget = 0;
	float v_xt = 0;
	float v_yt = 0;
	float centerX = 0;
	float centerY = 0;
	float weight_sum = 0;
	for (auto i = 0; i < NParticle; i++) /* 求和 */
	{
		at_dot += particles[i].at_dot * weights[i];
		halfWidthOfTarget += particles[i]._halfWidthOfTarget * weights[i];
		halfHeightOfTarget += particles[i]._halfHeightOfTarget * weights[i];
		v_xt += particles[i].v_xt * weights[i];
		v_yt += particles[i].v_yt * weights[i];
		centerX += particles[i].centerX * weights[i];
		centerY += particles[i].centerY * weights[i];
		weight_sum += weights[i];
	}

	// 求平均
	if (weight_sum <= 0)
		weight_sum = 1; // 防止被0除，一般不会发生

	EstState.at_dot = at_dot / weight_sum;
	EstState._halfWidthOfTarget = static_cast<int>(halfWidthOfTarget / weight_sum);
	EstState._halfHeightOfTarget = static_cast<int>(halfHeightOfTarget / weight_sum);
	EstState.v_xt = v_xt / weight_sum;
	EstState.v_yt = v_yt / weight_sum;
	EstState.centerX = static_cast<int>(centerX / weight_sum);
	EstState.centerY = static_cast<int>(centerY / weight_sum);
}

/************************************************************
模型更新
输入参数：
SPACESTATE EstState：   状态量的估计值
float * TargetHist：    目标直方图
int bins：              直方图条数
float PiT：             阈值（权重阈值）
unsigned char * img：   图像数据，RGB形式
int width, height：     图像宽高
输出：
float * TargetHist：    更新的目标直方图
************************************************************/
void Tracker::ModelUpdate(SpaceState EstState, float* TargetHist, int bins, float PiT, unsigned short* imgData, int width, int height)
{
	auto estimatedHist = new float[bins];

	// (1)在估计值处计算目标直方图
	CalcuModelHistogram(EstState.centerX, EstState.centerY, EstState._halfWidthOfTarget, EstState._halfHeightOfTarget, imgData, width, height, estimatedHist);
	// (2)计算Bhattacharyya系数
	float Bha = CalcuBhattacharyya(estimatedHist, TargetHist);
	// (3)计算概率权重
	float Pi_E = CalcuWeightedPi(Bha);

	if (Pi_E > PiT)
	{
		for (auto i = 0; i < bins; i++)
		{
			TargetHist[i] = static_cast<float>((1.0 - ALPHA_COEFFICIENT) * TargetHist[i] + ALPHA_COEFFICIENT * estimatedHist[i]);
		}
	}
	delete[] estimatedHist;
}

/*********************************************************
 * * 功能： 初始化设置粒子的数量
 * * 参数： 粒子数量
 ********************************************************/
void Tracker::SetParticleCount(unsigned int particleCount)
{
	this->_nParticle = particleCount;
}
