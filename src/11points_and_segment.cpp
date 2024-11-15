#include <iostream>
#include <CGAL/Simple_cartesian.h> // �ѿ����������ͷ�ļ�

using std::cout;
typedef CGAL::Simple_cartesian<double> Kernel;  // �ں�ʹ��˫���ȸ�������Ϊ�ѿ�������
typedef Kernel::Point_2 Point2;  // ��ά��
typedef Kernel::Segment_2 Segment2;  // ��ά�߶�

int main()
{
	// ����2����ά��
	Point2 p(1, 1), q(10, 10);
	cout << "p = " << p << "\n";
	cout << "q = " << q.x() << " " << q.y() << "\n";
	// ��������֮���ƽ������
	cout << "p,q����֮���ƽ������ = " << CGAL::squared_distance(p, q) << "\n";
	// ����㵽�߶ε�ƽ������
	Segment2 s(p, q);
	Point2 m(5, 9);
	cout << "m = " << m << "\n";
	cout << "��m���߶�pq��ƽ������ = " << CGAL::squared_distance(s, m) << "\n";
	// �ж�����֮���λ�ù�ϵ
	cout << "p��q�ٵ�m�����λ�ù�ϵ = ";
	switch (CGAL::orientation(p, q, m))
	{
	case CGAL::COLLINEAR:
		std::cout << "���㹲��\n";
		break;
	case CGAL::LEFT_TURN:
		std::cout << "���㹹����ת\n";
		break;
	case CGAL::RIGHT_TURN:
		std::cout << "���㹹����ת\n";
		break;
	}
	// �����߶ε��е�
	cout << "p��q���е� = " << CGAL::midpoint(p, q) << "\n";

	return 0;
}