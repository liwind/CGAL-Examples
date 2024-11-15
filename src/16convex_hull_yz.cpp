#include <iostream>
#include <iterator>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_yz_3.h>
#include <CGAL/convex_hull_2.h>

using std::cin; using std::cout;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K3;
typedef CGAL::Projection_traits_yz_3<K3> K;
typedef K::Point_2 Point_2;

int main()
{
	/* cin points
	12 0 0
	33 1 1
	22 0 1
	123 1 0
	32 0.5 0.2
	12 0.1 0.2
	*/
	std::istream_iterator< Point_2 >  input_begin(cin);
	std::istream_iterator< Point_2 >  input_end;
	std::ostream_iterator< Point_2 >  output(cout, "\n");
	CGAL::convex_hull_2(input_begin, input_end, output, K());

	return 0;
}
