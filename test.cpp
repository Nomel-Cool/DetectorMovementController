#include "MovementController.h"
#include <iostream>
int main()
{
	DetectorMovementControl dmc;
	std::vector<Point> contour_list =
	{
		{1.6326,-2.1635},
		{3.0547,3.1065},
		{-0.7,-2.38},
		{-1.76,-1.22},
		{0.7956,-2.4401},
		{-2.18,0.68},
		{-1.36,2.08},
		{2.5556,0.8399},
		{0.6356,2.0399}
	};
	Point vec_detector(9.9142, -2.4982);
	double radian = 77.32 * M_PI / 180;
	Point result = dmc.GetDetectorMoveDistance(contour_list, vec_detector, radian);
	std::cout << result.x << ", " << result.y << std::endl;
	return 0;
}