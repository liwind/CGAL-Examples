#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h> //精确浮点型
#include <sstream>

using std::cout;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;

int main()
{
  Point_2 p(0, 0.3), q, r(2, 0.9);
  {
    q  = Point_2(1, 0.6);
    cout << (CGAL::collinear(p,q,r) ? "共线\n" : "非共线\n");
  }

  {
    std::istringstream input("0 0.3   1 0.6   2 0.9");
    input >> p >> q >> r;
    cout << (CGAL::collinear(p,q,r) ? "共线\n" : "非共线\n");
  }

  {
    q = CGAL::midpoint(p,r);
    cout << (CGAL::collinear(p,q,r) ? "共线\n" : "非共线\n");
  }

  return 0;
}
