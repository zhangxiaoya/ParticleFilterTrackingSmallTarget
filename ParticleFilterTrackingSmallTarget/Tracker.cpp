#include "Tracker.h"
#include "State.h"
#include <ctime>
#include <opencv2/core/core.hpp>

int Tracker::ParticleTracking(unsigned short* image, int width, int height, int& centerX, int& centerY, int& halfWidthOfTarget, int& halfHeightOfTarget, float& max_weight)
{
	SpaceState estimateState;

	// �ز���
	ReSelect(_particles, _particleWeights, _nParticle);

	// ����������״̬���̣���״̬��������Ԥ��
	Propagate(_particles, _nParticle);

	// �۲⣺��״̬�����и���
	Observe(_particles, _particleWeights, _nParticle, image, width, height);

	// ���ƣ���״̬�����й��ƣ���ȡλ����
	Estimation(_particles, _particleWeights, _nParticle, estimateState);

	centerX = estimateState.centerX;
	centerY = estimateState.centerY;
	halfWidthOfTarget = estimateState._halfWidthOfTarget;
	halfHeightOfTarget = estimateState._halfHeightOfTarget;

	// ģ�͸���
	ModelUpdate(estimateState, _modelHist, _nbin, _piThreshold, image, width, height);

	// �������Ȩ��ֵ
	max_weight = _particleWeights[0];
	for (auto i = 1; i < _nParticle; i++)
		max_weight = max_weight < _particleWeights[i] ? _particleWeights[i] : max_weight;
	// ���кϷ��Լ��飬���Ϸ�����-1
	if (centerX < 0 || centerY < 0 || centerX >= width || centerY >= height || halfWidthOfTarget <= 0 || halfHeightOfTarget <= 0)
		return -1;
	return 1;
}

/*
��ʼ��ϵͳ
int targetCenterX, targetCenterY��         ��ʼ������ͼ��Ŀ����������
int halfWidthOfTarget, halfHeightOfTarget��Ŀ��İ���
unsigned char * img��                      ͼ�����ݣ��Ҷ���ʽ
int width, height��                        ͼ����
*/
int Tracker::Initialize(int targetCenterX, int targetCenterY, int halfWidthOfTarget, int halfHeightOfTarget, unsigned short* imgData, int width, int height)
{
	srand(static_cast<unsigned int>(time(nullptr)));

	// ����״̬����Ŀռ�
	_particles = new SpaceState[_nParticle];
	// ��������Ȩ������Ŀռ�
	_particleWeights = new float[_nParticle];
	// ȷ��ֱ��ͼ����
	_nbin = BIN;
	// ����ֱ��ͼ�ڴ�
	_modelHist = new float[_nbin];
	if (_modelHist == nullptr)
		return (-1);

	// ����Ŀ��ģ��ֱ��ͼ
	CalcuModelHistogram(targetCenterX, targetCenterY, halfWidthOfTarget, halfHeightOfTarget, imgData, width, height, _modelHist);

	// ��ʼ������״̬(��(x0,y0,1,1,Wx,Hy,0.1)Ϊ���ĳ�N(0,0.4)��̬�ֲ�)
	_particles[0].centerX = targetCenterX;
	_particles[0].centerY = targetCenterY;
	_particles[0].v_xt = static_cast<float>(0.0); // 1.0
	_particles[0].v_yt = static_cast<float>(0.0); // 1.0
	_particles[0]._halfWidthOfTarget = halfWidthOfTarget;
	_particles[0]._halfHeightOfTarget = halfHeightOfTarget;
	_particles[0].at_dot = static_cast<float>(0.0); // 0.1
	_particleWeights[0] = static_cast<float>(1.0 / _nParticle); // 0.9;

	float randomNumbers[7];
	for (auto i = 1; i < _nParticle; i++)
	{
		// ����7�������˹�ֲ�����
		for (auto j = 0; j < 7; j++)
			randomNumbers[j] = randGaussian(0, static_cast<float>(0.6));

		_particles[i].centerX = static_cast<int>(_particles[0].centerX + randomNumbers[0] * halfWidthOfTarget);
		_particles[i].centerY = static_cast<int>(_particles[0].centerY + randomNumbers[1] * halfHeightOfTarget);
		_particles[i].v_xt = static_cast<float>(_particles[0].v_xt + randomNumbers[2] * _VELOCITY_DISTURB);
		_particles[i].v_yt = static_cast<float>(_particles[0].v_yt + randomNumbers[3] * _VELOCITY_DISTURB);
		_particles[i]._halfWidthOfTarget = static_cast<int>(_particles[0]._halfWidthOfTarget + randomNumbers[4] * _SCALE_DISTURB);
		_particles[i]._halfHeightOfTarget = static_cast<int>(_particles[0]._halfHeightOfTarget + randomNumbers[5] * _SCALE_DISTURB);
		_particles[i].at_dot = static_cast<float>(_particles[0].at_dot + randomNumbers[6] * _SCALE_CHANGE_D);

		// Ȩ��ͳһΪ1/N����ÿ����������ȵĻ���
		_particleWeights[i] = static_cast<float>(1.0 / _nParticle);
	}
	return 1;
}

void Tracker::ReSelect(SpaceState* state, float* weight, int nParticle)
{
	// �洢�µ�����
	SpaceState* newParticles = new SpaceState[nParticle];

	// ͳ�Ƶ�������������������
	int* resampleIndex = new int[nParticle];

	// ����Ȩ�����²���
	ImportanceSampling(weight, resampleIndex, nParticle);

	for (auto i = 0; i < nParticle; i++)
		newParticles[i] = state[resampleIndex[i]];

	for (auto i = 0; i < nParticle; i++)
		state[i] = newParticles[i];

	delete[] newParticles;
	delete[] resampleIndex;
}

/**
 * \brief ���½�����Ҫ�Բ���
 * \param wights ��Ӧ����Ȩ������pi(n)
 * \param ResampleIndex �ز�����������(���)
 * \param NParticle Ȩ�����顢�ز�����������Ԫ�ظ���
 */
void Tracker::ImportanceSampling(float* wights, int* ResampleIndex, int NParticle)
{
	// �����ۼ�Ȩ�������ڴ棬��СΪN+1
	auto cumulateWeight = new float[NParticle + 1];
	// �����ۼ�Ȩ��
	NormalizeCumulatedWeight(wights, cumulateWeight, NParticle);
	for (auto i = 0; i < NParticle; i++)
	{
		// �������һ��[0,1]����ȷֲ�����
		auto randomNumber = rand01();
		// ����<=randomNumber����С����j
		auto j = BinearySearch(randomNumber, cumulateWeight, NParticle + 1);

		if (j == NParticle)
			j--;
		// �����ز�����������
		ResampleIndex[i] = j;
	}

	delete[] cumulateWeight;
}

/*
�����һ���ۼƸ���c'_i
���������
float * weight��    Ϊһ����N��Ȩ�أ����ʣ�������
int N��             ����Ԫ�ظ���
���������
float * cumulateWeight�� Ϊһ����N+1���ۼ�Ȩ�ص����飬
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
 * \brief ���һ��[0,1]֮��������
 * \return ����һ��0��1֮��������
 */
float Tracker::rand01()
{
	return (rand() / float(RAND_MAX));
}

/**
 * \brief ����һ����˹�ֲ��������
 * \param u ��˹�ֲ��ľ�ֵ
 * \param sigma ��˹�ֲ��ı�׼��
 * \return ��˹�ֲ��������
 */
float Tracker::randGaussian(float u, float sigma) const
{
	float v1;
	float s = 100.0;
	/*
	ʹ��ɸѡ��������̬�ֲ�N(0,1)�������(Box-Mulles����)
	1. ����[0,1]�Ͼ����������X1,X2
	2. ����V1=2*X1-1,V2=2*X2-1,s=V1^2+V2^2
	3. ��s<=1,ת����4������ת1
	4. ����A=(-2ln(s)/s)^(1/2),y1=V1*A, y2=V2*A
	y1,y2ΪN(0,1)�������
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
	���ݹ�ʽ
	z = sigma * y + u
	��y����ת����N(u,sigma)�ֲ�
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
����һ��ͼ����ĳ������Ĳ�ɫֱ��ͼ�ֲ�
���������
int targetCenterX, targetCenterY��          ָ��ͼ����������ĵ�
int halfWidthOfTarget, halfHeightOfTarget�� ָ��ͼ������İ��Ͱ��
unsigned char * imgData ��                  ͼ�����ݣ����������ң��������µ�˳��ɨ�裬
int width, height��                         ͼ��Ŀ�͸�
���������
float * ColorHist��                         ��ɫֱ��ͼ����ɫ��������
i = r * G_BIN * B_BIN + g * B_BIN + b����
int bins��                                  ��ɫֱ��ͼ������R_BIN*G_BIN*B_BIN������ȡ8x8x8=512��
*/
void Tracker::CalcuModelHistogram(int targetCenterX, int targetCenterY,
                                  int halfWidthOfTarget, int halfHeightOfTarget,
                                  unsigned short* imgData,
                                  int width, int height,
                                  float* hist)
{
	// ֱ��ͼ����ֵ��0
	for (auto i = 0; i < _nbin; i++)
		hist[i] = 0.0;

	// �������������centerX, centerY��ͼ�����棬���ߣ�halfWidthOfTarget<=0, halfHeightOfTarget<=0,��ʱǿ�����ɫֱ��ͼΪ0
	if ((targetCenterX < 0) || (targetCenterX >= width) || (targetCenterY < 0) || (targetCenterY >= height) || (halfWidthOfTarget <= 0) || (halfHeightOfTarget <= 0))
		return;

	// ����ʵ�ʸ߿��������ʼ��, ������Χ�Ļ����û��Ŀ�ı߽�����ֵ���ӵ�����
	auto xBeg = targetCenterX - halfWidthOfTarget;
	auto yBeg = targetCenterY - halfHeightOfTarget;
	if (xBeg < 0) xBeg = 0;
	if (yBeg < 0) yBeg = 0;

	auto xEnd = targetCenterX + halfWidthOfTarget;
	auto yEnd = targetCenterY + halfHeightOfTarget;
	if (xEnd >= width) xEnd = width - 1;
	if (yEnd >= height) yEnd = height - 1;

	// ����뾶ƽ��a^2
	auto squareOfRadius = halfWidthOfTarget*halfWidthOfTarget + halfHeightOfTarget*halfHeightOfTarget;

	// ��һ��ϵ��
	float f = 0.0;

	for (auto y = yBeg; y <= yEnd; y++)
	{
		for (auto x = xBeg; x <= xEnd; x++)
		{
			auto v = imgData[y * width + x] >> SHIFT;
			//�ѵ�ǰrgb����һ������
			auto index = v;

			auto squareOfRadiusFromCurPixelToCenter = ((y - targetCenterY) * (y - targetCenterY) + (x - targetCenterX) * (x - targetCenterX));
			// ���㵱ǰ���ص����ĵ�İ뾶ƽ��r^2
			auto r2 = static_cast<float>(squareOfRadiusFromCurPixelToCenter * 1.0 / squareOfRadius);
			// k(r) = 1-r^2, |r| < 1; ����ֵ k(r) = 0 ��Ӱ����
			auto k = 1 - r2;
			f = f + k;

			// ������ܶȼ�Ȩ�Ҷ�ֱ��ͼ
			hist[index] = hist[index] + k;
		}
	}

	// ��һ��ֱ��ͼ
	for (auto i = 0; i < _nbin; i++)
		hist[i] = hist[i] / f;
}

/**
 * \brief ����ϵͳ״̬������ȡ״̬Ԥ����, S(t) = A S(t-1) + W(t-1), ����W(t-1)��ʾ��˹����
 * \param state �����״̬������
 * \param NParticle ����״̬����
 */
void Tracker::Propagate(SpaceState* state, int NParticle)
{
	float randomNumbers[7];

	// ��ÿһ��״̬����state[i](��N��)���и���; �����ֵΪ0�������˹����
	for (auto i = 0; i < NParticle; i++)
	{
		// ����7�������˹�ֲ�����
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
�۲⣬����״̬����St�е�ÿһ���������۲�ֱ��ͼ��Ȼ��
���¹�����������µ�Ȩ�ظ���
���������
SPACESTATE * state��      ״̬������
int N��                   ״̬������ά��
unsigned char * image��   ͼ�����ݣ����������ң��������µ�˳��ɨ�裬
��ɫ���д���RGB, RGB, ...
int width, height��                ͼ��Ŀ�͸�
float * ObjectHist��      Ŀ��ֱ��ͼ
int hbins��               Ŀ��ֱ��ͼ����
���������
float * weight��          ���º��Ȩ��
*/
void Tracker::Observe(SpaceState* state, float* weight, int NParticle, unsigned short* imgData, int W, int H)
{
	float * hist = new float[_nbin];

	for (auto i = 0; i < NParticle; i++)
	{
		// (1) �����ɫֱ��ͼ�ֲ�
		CalcuModelHistogram(state[i].centerX, state[i].centerY, state[i]._halfWidthOfTarget, state[i]._halfHeightOfTarget, imgData, W, H, hist);
		// (2) Bhattacharyyaϵ��
		float rho = CalcuBhattacharyya(hist, _modelHist);
		// (3) ���ݼ���õ�Bhattacharyyaϵ���������Ȩ��ֵ
		weight[i] = CalcuWeightedPi(rho);
	}

	delete[] hist;
}

/**
 * \brief ����Bhattachryyaϵ��
 * \param histA ֱ��ͼA
 * \param histB ֱ��ͼB
 * \return Bhattacharyyaϵ��
 */
float Tracker::CalcuBhattacharyya(float* histA, float* histB) const
{
	float rho = 0.0;
	for (auto i = 0; i < _nbin; i++)
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
���ƣ�����Ȩ�أ�����һ��״̬����Ϊ�������
���������
SPACESTATE * state��      ״̬������
float * weight��          ��ӦȨ��
int N��                   ״̬������ά��
���������
SPACESTATE * EstState��   ���Ƴ���״̬��
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
	for (auto i = 0; i < NParticle; i++) /* ��� */
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

	// ��ƽ��
	if (weight_sum <= 0)
		weight_sum = 1; // ��ֹ��0����һ�㲻�ᷢ��

	EstState.at_dot = at_dot / weight_sum;
	EstState._halfWidthOfTarget = static_cast<int>(halfWidthOfTarget / weight_sum);
	EstState._halfHeightOfTarget = static_cast<int>(halfHeightOfTarget / weight_sum);
	EstState.v_xt = v_xt / weight_sum;
	EstState.v_yt = v_yt / weight_sum;
	EstState.centerX = static_cast<int>(centerX / weight_sum);
	EstState.centerY = static_cast<int>(centerY / weight_sum);
}

/************************************************************
ģ�͸���
���������
SPACESTATE EstState��   ״̬���Ĺ���ֵ
float * TargetHist��    Ŀ��ֱ��ͼ
int bins��              ֱ��ͼ����
float PiT��             ��ֵ��Ȩ����ֵ��
unsigned char * img��   ͼ�����ݣ�RGB��ʽ
int width, height��     ͼ����
�����
float * TargetHist��    ���µ�Ŀ��ֱ��ͼ
************************************************************/
void Tracker::ModelUpdate(SpaceState EstState, float* TargetHist, int bins, float PiT, unsigned short* imgData, int width, int height)
{
	auto estimatedHist = new float[bins];

	// (1)�ڹ���ֵ������Ŀ��ֱ��ͼ
	CalcuModelHistogram(EstState.centerX, EstState.centerY, EstState._halfWidthOfTarget, EstState._halfHeightOfTarget, imgData, width, height, estimatedHist);
	// (2)����Bhattacharyyaϵ��
	float Bha = CalcuBhattacharyya(estimatedHist, TargetHist);
	// (3)�������Ȩ��
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
