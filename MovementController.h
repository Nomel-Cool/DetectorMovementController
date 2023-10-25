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
/// ���� for A
/// ������������ͶӰ����̽ͷ�˶����������λ��
/// ���룺����������������̽ͷ������
/// ����
/// 1. �������̽ͷ��������Ұ�뿪�н�����
/// 2. ���������ͶӰ
/// 3. ���d,d'
/// ���������min{d,d'}���̽ͷ����
/// ע��㣺���ͶӰ�Ѿ�������Ұ�أ�
/// </summary>
class DetectorMovementControl
{
public:
	/// <summary>
	/// ��ȡ̽ͷreasonable�Ĳ���������
	/// </summary>
	/// <param name="vec_contour_list">����ͶӰ��</param>
	/// <param name="vec_detector">̽ͷ����</param>
	/// <param name="full_radian">̽ͷȫ����</param>
	/// <returns>������̽ͷ����</returns>
	Point GetDetectorMoveDistance(const std::vector<Point>& vec_contour_list, const Point& vec_detector, const double& full_radian)
	{
		/* ̽ͷ����ƽ���� */
		double delta_x = abs(vec_detector.x), delta_y = abs(vec_detector.y);

		/* ������̽ͷ��Ұ�߽絥λ�������Ӹ��ӽǿ�˳ʱ���ȡ����Ұ�߽���������ʱ���ȡ������˷���ָ������ */
		double half_radian = full_radian * 0.5; // �뿪�ǻ���
		double length = sqrt(vec_detector.x * vec_detector.x + vec_detector.y * vec_detector.y); // ̽ͷ����ģ��
		Point unit_vec_dectector(-1 * vec_detector.x / length,	-1 * vec_detector.y / length); // ����̽ͷ��λ����

		// Tricky Point��ֱ�Ӷ�̽ͷ�������򣬲�˳ʱ�����ʱ��ֱ���ת�뿪���Ⱦ���
		double count_clockwise_x = unit_vec_dectector.x * cos(half_radian) - unit_vec_dectector.y * sin(half_radian),
			   count_clockwise_y = unit_vec_dectector.y * cos(half_radian) + unit_vec_dectector.x * sin(half_radian),
			   clockwise_x = unit_vec_dectector.x * cos(half_radian) + unit_vec_dectector.y * sin(half_radian),
			   clockwise_y = unit_vec_dectector.y * cos(half_radian) - unit_vec_dectector.x * sin(half_radian);
		Point clockwise_vec_dectector(clockwise_x, clockwise_y), counter_clockwise_vec_detector(count_clockwise_x, count_clockwise_y);

		/* ��ͶӰ�ᣬ���·��ĳ��£����Ϸ��ĳ��� */
		Point clockwise_prj_vec(clockwise_y, -1 * clockwise_x), count_clockwise_prj_vec(-1 * count_clockwise_y, count_clockwise_x);

		/* �����ͶӰ�� */
		double clockwise_prj_max = 0, count_clockwise_prj_max = 0; // ��Ϊ�����ͶӰ�������ֵ���Դ�0��ʼ
		for (const auto& vec_contour : vec_contour_list)
		{
			auto prj_clockwise = ProjectOn(vec_contour, clockwise_prj_vec);
			auto prj_count_clockwise = ProjectOn(vec_contour, count_clockwise_prj_vec);
			clockwise_prj_max = std::max(clockwise_prj_max, prj_clockwise);
			count_clockwise_prj_max = std::max(count_clockwise_prj_max, prj_count_clockwise);
		}

		/* ֱ���׽�ģ��ʽ��d = OD - Lmax / sin(��) */
		double d1 = length - clockwise_prj_max / sin(half_radian), d2 = length - count_clockwise_prj_max / sin(half_radian);
		
		/* ��ȡ��Сֵ */
		double min_d = std::min(d1, d2);

		/* �����ƶ����̽ͷλ�� */
		double new_detector_x = vec_detector.x - min_d * vec_detector.x / length, new_detector_y = vec_detector.y - min_d * vec_detector.y / length;
		Point moved_detector(new_detector_x, new_detector_y);
		return moved_detector;
	}

private:
	/// <summary>
	/// vec1��vec2��ͶӰֵ
	/// </summary>
	/// <param name="vec1">ͶӰ����</param>
	/// <param name="vec2">��ͶӰ����</param>
	/// <returns>ͶӰֵ</returns>
	double ProjectOn(Point vec1, Point vec2)
	{
		return vec1.x * vec2.x + vec1.y * vec2.y;
	}
};


#endif // !MOVEMENT_CONTROLLER_H
