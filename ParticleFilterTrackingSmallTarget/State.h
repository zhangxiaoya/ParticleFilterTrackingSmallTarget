#pragma once

#include "Orientation.h"

/* 状态空间变量 */
class SpaceState
{
public:
	explicit SpaceState(Orientation orientation = Orientation(),
						float v_xt = 0.0,
						float v_yt = 0.0,
						float at_dot = 0.0)
			: _orientation(orientation),
			  v_xt(v_xt),
			  v_yt(v_yt),
			  at_dot(at_dot) {}

	Orientation _orientation;
	float v_xt;           /* x方向运动速度 */
	float v_yt;           /* y方向运动速度 */
	float at_dot;         /* 尺度变换速度，粒子所代表的那一片区域的尺度变化速度 */
};