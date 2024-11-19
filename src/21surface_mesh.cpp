#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Core>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

using std::cout; using std::vector; using std::array; using std::string;
namespace PMP = CGAL::Polygon_mesh_processing;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point3;
typedef CGAL::Surface_mesh<Point3> Mesh;
typedef Mesh::Vertex_index VertIds;
typedef Mesh::Face_index FaceIds;

bool CGtoIGL(Mesh smesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
	size_t nVerts = smesh.number_of_vertices();
	size_t nFaces = smesh.number_of_faces();
	V.setZero(nVerts, 3);
	F.setZero(nFaces, 3);
	// get vertex position
	for (VertIds vi : smesh.vertices()) {
		Point3 pt = smesh.point(vi);
		V.row(vi.idx()) << pt.x(), pt.y(), pt.z();
	}
	// get face vertex-list
	for (FaceIds fi : smesh.faces()) {
		size_t j = 0;
		for (VertIds vi : smesh.vertices_around_face(smesh.halfedge(fi)))
			F(fi.idx(), j++) = vi.idx();
	}

	return true;
}

int main(int argc, char* argv[]) 
{
	Mesh smesh;
	// build mesh from points and face list
	VertIds v0 = smesh.add_vertex(K::Point_3(0, 0, 0));
	VertIds v1 = smesh.add_vertex(K::Point_3(1, 0, 0));
	VertIds v2 = smesh.add_vertex(K::Point_3(0, 1, 0));
	VertIds v3 = smesh.add_vertex(K::Point_3(0, 0, 1));
	FaceIds f1 = smesh.add_face(v0, v1, v2);
	smesh.add_face(v1, v0, v3);

	// the vertex iterator type is a nested type of the Vertex_range
	Mesh::Vertex_range::iterator  vb;
	Mesh::Vertex_range r = smesh.vertices();
	for (vb = r.begin(); vb != r.end(); ++vb) {
		// Print vertex index and vertex coordinates
		cout << "point " << *vb << ": " << smesh.point(*vb) << "\n";
	}
	cout << "face " << f1 << ": ";
	for (VertIds vi : smesh.vertices_around_face(smesh.halfedge(f1))) {
		cout << vi << ",";
	}

	//read mesh form file
	string filename = "../data/spot.ply";
	if (!CGAL::IO::read_polygon_mesh(filename, smesh)) {
		cout << "Invalid input file: " << filename << "\n";
		return 1;
	}
	size_t nVerts = smesh.number_of_vertices();
	size_t nFaces = smesh.number_of_faces();
	cout << "\nnverts=" << nVerts << " , nfaces=" << nFaces << "\n";

	// get mesh point and face to std vector
	vector<array<double, 3>> vertexPos;
	vector<array<size_t, 3>> faceList;
	vector<double> elevation;
	// get vertex position
	for (VertIds vi : smesh.vertices()) {
		Point3 pt = smesh.point(vi);
		array<double, 3> pos = { pt.x(), pt.y(), pt.z() };
		vertexPos.emplace_back(pos);
	}
	cout << "vector points:" << vertexPos.size() << " , ";
	// get face vertex-list
	array<size_t, 3> faceIndices = { 0,0,0 };
	for (FaceIds fi : smesh.faces()) {
		size_t i = 0;
		for (VertIds vi : smesh.vertices_around_face(smesh.halfedge(fi))) {
			faceIndices[i++] = vi.id();
		}
		faceList.emplace_back(faceIndices);
	}
	cout << "vector faces" << faceList.size() << "\n";

	// get mesh point and face to Eigen Matrix
	Eigen::MatrixXd meshV;
	Eigen::MatrixXi meshF;
	CGtoIGL(smesh, meshV, meshF);
	//cout << meshV;
	//cout << meshF;

	return 0;
}