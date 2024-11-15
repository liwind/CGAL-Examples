#include <iostream>
#include <CGAL/Simple_cartesian.h> // 笛卡尔坐标相关头文件

using std::cout;
typedef CGAL::Simple_cartesian<double> Kernel;  // 内核使用双精度浮点数作为笛卡尔坐标
typedef Kernel::Point_2 Point2;  // 二维点
typedef Kernel::Segment_2 Segment2;  // 二维线段

int main()
{
	// 定义2个二维点
	Point2 p(1, 1), q(10, 10);
	cout << "p = " << p << "\n";
	cout << "q = " << q.x() << " " << q.y() << "\n";
	// 计算两点之间的平方距离
	cout << "p,q两点之间的平方距离 = " << CGAL::squared_distance(p, q) << "\n";
	// 计算点到线段的平方距离
	Segment2 s(p, q);
	Point2 m(5, 9);
	cout << "m = " << m << "\n";
	cout << "点m到线段pq的平方距离 = " << CGAL::squared_distance(s, m) << "\n";
	// 判断三点之间的位置关系
	cout << "p到q再到m三点的位置关系 = ";
	switch (CGAL::orientation(p, q, m))
	{
	case CGAL::COLLINEAR:
		std::cout << "三点共线\n";
		break;
	case CGAL::LEFT_TURN:
		std::cout << "三点构成左转\n";
		break;
	case CGAL::RIGHT_TURN:
		std::cout << "三点构成右转\n";
		break;
	}
	// 计算线段的中点
	cout << "p与q的中点 = " << CGAL::midpoint(p, q) << "\n";

	return 0;
}