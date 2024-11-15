#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

using std::cout;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;

int main()
{
	//使用数组存储二维点集
	Point_2 points[5] = { Point_2(0,0), Point_2(10,0), Point_2(10,10), Point_2(6,5), Point_2(4,1) };
	Point_2 result[5];  //接受凸包上的点
	//CGAL凸包算法
	Point_2* ptr = CGAL::convex_hull_2(points, points + 5, result);
	cout << ptr - result << " 个凸包上的点: \n";
	for (int i = 0; i < ptr - result; i++) {
		cout << result[i] << "\n";
	}

	return 0;
}
