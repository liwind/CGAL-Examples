#include <iostream>
#include <CGAL/Simple_cartesian.h>

using std::cout;
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point2;

int main()
{
	{
		Point2 p(0, 0.3), q(1, 0.6), r(2, 0.9);
		cout << (CGAL::collinear(p, q, r) ? "共线\n" : "非共线\n");
	}
	{
		Point2 p(0, 1.0 / 3.0), q(1, 2.0 / 3.0), r(2, 1);
		cout << (CGAL::collinear(p, q, r) ? "共线\n" : "非共线\n");
	}
	{
		Point2 p(0, 0), q(1, 1), r(2, 2);
		cout << (CGAL::collinear(p, q, r) ? "共线\n" : "非共线\n");
	}

	return 0;
}