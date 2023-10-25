#pragma once
#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include <vector>

constexpr double M_PI = 3.14159265358979323846;

struct Point
{
	double x, y;
	Point() :x(0), y(0) {}
	Point(double _x, double _y) { x = _x; y = _y; }
};

/// <summary>
/// 任务 for A
/// 根据人体轮廓投影控制探头运动到离人最近位置
/// 输入：轮廓向量集，单个探头的坐标
/// 处理：
/// 1. 求出两个探头出发的视野半开夹角向量
/// 2. 遍历出最长正投影
/// 3. 求出d,d'
/// 输出：步进min{d,d'}后的探头坐标
/// 注意点：如果投影已经超出视野呢？
/// </summary>
class DetectorMovementControl
{
public:
	/// <summary>
	/// 获取探头reasonable的步进后坐标
	/// </summary>
	/// <param name="vec_contour_list">轮廓投影集</param>
	/// <param name="vec_detector">探头向量</param>
	/// <param name="full_radian">探头全开角</param>
	/// <returns>步进后探头坐标</returns>
	Point GetDetectorMoveDistance(const std::vector<Point>& vec_contour_list, const Point& vec_detector, const double& full_radian)
	{
		/* 探头向量平移量 */
		double delta_x = abs(vec_detector.x), delta_y = abs(vec_detector.y);

		/* 求两条探头视野边界单位向量，从俯视角看顺时针获取的视野边界向量与逆时针获取的做叉乘方向指向俯视者 */
		double half_radian = full_radian * 0.5; // 半开角弧度
		double length = sqrt(vec_detector.x * vec_detector.x + vec_detector.y * vec_detector.y); // 探头向量模长
		Point unit_vec_dectector(-1 * vec_detector.x / length,	-1 * vec_detector.y / length); // 反向探头单位向量

		// Tricky Point：直接对探头向量反向，并顺时针和逆时针分别旋转半开弧度就行
		double count_clockwise_x = unit_vec_dectector.x * cos(half_radian) - unit_vec_dectector.y * sin(half_radian),
			   count_clockwise_y = unit_vec_dectector.y * cos(half_radian) + unit_vec_dectector.x * sin(half_radian),
			   clockwise_x = unit_vec_dectector.x * cos(half_radian) + unit_vec_dectector.y * sin(half_radian),
			   clockwise_y = unit_vec_dectector.y * cos(half_radian) - unit_vec_dectector.x * sin(half_radian);
		Point clockwise_vec_dectector(clockwise_x, clockwise_y), counter_clockwise_vec_detector(count_clockwise_x, count_clockwise_y);

		/* 求投影轴，在下方的朝下，在上方的朝上 */
		Point clockwise_prj_vec(clockwise_y, -1 * clockwise_x), count_clockwise_prj_vec(-1 * count_clockwise_y, count_clockwise_x);

		/* 求最长正投影量 */
		double clockwise_prj_max = 0, count_clockwise_prj_max = 0; // 因为求最长正投影所以最大值可以从0开始
		for (const auto& vec_contour : vec_contour_list)
		{
			auto prj_clockwise = ProjectOn(vec_contour, clockwise_prj_vec);
			auto prj_count_clockwise = ProjectOn(vec_contour, count_clockwise_prj_vec);
			clockwise_prj_max = std::max(clockwise_prj_max, prj_clockwise);
			count_clockwise_prj_max = std::max(count_clockwise_prj_max, prj_count_clockwise);
		}

		/* 直接套建模公式：d = OD - Lmax / sin(φ) */
		double d1 = length - clockwise_prj_max / sin(half_radian), d2 = length - count_clockwise_prj_max / sin(half_radian);
		
		/* 获取较小值 */
		double min_d = std::min(d1, d2);

		/* 返回移动后的探头位置 */
		double new_detector_x = vec_detector.x - min_d * vec_detector.x / length, new_detector_y = vec_detector.y - min_d * vec_detector.y / length;
		Point moved_detector(new_detector_x, new_detector_y);
		return moved_detector;
	}

private:
	/// <summary>
	/// vec1到vec2的投影值
	/// </summary>
	/// <param name="vec1">投影向量</param>
	/// <param name="vec2">背投影向量</param>
	/// <returns>投影值</returns>
	double ProjectOn(Point vec1, Point vec2)
	{
		return vec1.x * vec2.x + vec1.y * vec2.y;
	}
};


#endif // !MOVEMENT_CONTROLLER_H
