#pragma once

struct SpaceState
{						  /* 状态空间变量 */
	int xt;               /* x坐标位置 */
	int yt;               /* y坐标位置 */
	float v_xt;           /* x方向运动速度 */
	float v_yt;           /* y方向运动速度 */
	int Hxt;              /* x方向半窗宽 */
	int Hyt;              /* y方向半窗宽 */
	float at_dot;         /* 尺度变换速度，粒子所代表的那一片区域的尺度变化速度 */
};