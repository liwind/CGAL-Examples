#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

#include <vector>

using std::cout;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef std::vector<Point_2> Points;

int main()
{
  Points points, result;
  points.push_back(Point_2(0,0));
  points.push_back(Point_2(10,0));
  points.push_back(Point_2(10,10));
  points.push_back(Point_2(6,5));
  points.emplace_back(4, 1);


  CGAL::convex_hull_2( points.begin(), points.end(), std::back_inserter(result) );
  cout << result.size() << " 个凸包上的点: \n";
  for (Point_2 pt : result) {
	  cout << pt << "\n";
  }

  return 0;
}
