#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/interpolated_corrected_curvatures.h>
#include <CGAL/Heat_method_3/Surface_mesh_geodesic_distances_3.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>
#include "portable-file-dialogs.h"
#include "glm/glm.hpp"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

using std::string; using std::vector; using::std::array; using std::cout;
using polyscope::guessNiceNameFromPath;
namespace PMP = CGAL::Polygon_mesh_processing;
namespace Heat = CGAL::Heat_method_3;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2         Point2;
typedef K::Iso_rectangle_2 Rectangle2;
typedef K::Segment_2       Segment2;
typedef K::Ray_2           Ray2;
typedef K::Line_2          Line2;
typedef K::Point_3         Point3;
typedef K::FT              Scalar;
typedef K::Vector_3        Vector3;
typedef CGAL::Surface_mesh<Point3> Mesh;
typedef Mesh::Vertex_index         VertIds;
typedef Mesh::Face_index           FaceIds;
typedef Heat::Surface_mesh_geodesic_distances_3<Mesh, Heat::Direct> HeatDistance;
typedef CGAL::Surface_mesh_shortest_path_traits<K, Mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits>         ExactDistance;
typedef CGAL::Delaunay_triangulation_2<K> DT2;
typedef DT2::Finite_edges_iterator        DFEI;

struct QuantityName
{
	string vElevation = "vertex elevation";
	string vNormal = "vertex normal";
	string fNormal = "face normal";
	string gCurvature = "gauss curvature";
	string mCurvature = "mean curvature";
	string nCurvature = "normal mean curvature";
	string sCurvature = "square mean curvature";
	string vDistance = "distance map";
	string gPath = "geodesic path";
	string pVertex = "picked vertex";
}name;

//- mesh data
Mesh cgMesh;
polyscope::SurfaceMesh* psMesh = nullptr;
void myMeshTools();

int main(int argc, char* argv[]) {
	// polyscope scene options config
	polyscope::options::programName = "Mesh Processing";
	polyscope::options::printPrefix = "[Log] ";
	polyscope::options::usePrefsFile = false;
	polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
	polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Free);
	polyscope::view::upDir = polyscope::UpDir::YUp;             // set +Y as up direction
	polyscope::view::frontDir = polyscope::FrontDir::ZFront; // set +z as front direction

	// initialize polycope, creating opengl context and constructing a window
	polyscope::init();

	// Specify the callback function
	polyscope::state::userCallback = myMeshTools;

	// build data visualization

	// pass control flow to polycope, displaying the interactive window
	polyscope::show();

	return 0;
}

void GetCGALMesh(Mesh cmesh, vector<glm::vec3>& V, vector<vector<size_t>>& F)
{
	V = vector<glm::vec3>(cmesh.number_of_vertices());
	F = vector<vector<size_t>>(cmesh.number_of_faces());
	// get vertex position
	for (VertIds vi : cmesh.vertices()) {
		Point3 pt = cmesh.point(vi);
		V[vi.idx()] = glm::vec3(pt.x(), pt.y(), pt.z());
	}
	// get face vertex-list
	for (FaceIds fi : cmesh.faces()) {
		for (VertIds vi : cmesh.vertices_around_face(cmesh.halfedge(fi)))
			F[fi.idx()].emplace_back(vi.idx());
	}
}

template <class LRS>
void CropVoronoiEdge(LRS lrs, Rectangle2 bbox, vector<glm::vec2> & vdEdge)
{
	CGAL::Object cve = CGAL::intersection(lrs, bbox);
	const Segment2* s = CGAL::object_cast<Segment2>(&cve);
	if (s) {
		vdEdge.emplace_back(s->point(0).x(), s->point(0).y());
		vdEdge.emplace_back(s->point(1).x(), s->point(1).y());
	}
}

void myMeshTools()
{
	// ImGui Config begin
	ImGui::PushItemWidth(110);
	//ImGui::SetNextItemOpen(true, ImGuiCond_Once);

	//-File Menu-
	ImGui::TextUnformatted("File");
	static string meshFilePath;
	static vector<glm::vec3> meshV;
	static vector<vector<size_t>> meshF;
	static vector<VertIds> pickedVIds; //picked vertex id
	static vector<glm::vec3> pickedVerts; // picked vertex
	static vector<glm::vec3> pathLines; // geodesic path lines
	// Import Mesh File
	if (ImGui::Button("Import Mesh")) {
		vector<string> result = pfd::open_file("Open a Mesh File", "../data/",
			{ "PLY Mesh(*.ply)", "*.ply", "OBJ Mesh(*.obj)", "*.obj", "All Files(*.*)", "*.*" }).result();
		if (!result.empty()) {
			meshFilePath = result[0];
			if (psMesh != nullptr) {
				polyscope::removeAllStructures(); // clear psMesh
				cgMesh.clear(); meshV.clear(); meshF.clear();
				pathLines.clear(); pickedVerts.clear(); pickedVIds.clear();
			}
			if (PMP::IO::read_polygon_mesh(meshFilePath, cgMesh)) {
				GetCGALMesh(cgMesh, meshV, meshF);
				psMesh = polyscope::registerSurfaceMesh(guessNiceNameFromPath(meshFilePath), meshV, meshF);
				polyscope::view::resetCameraToHomeView();
			}
			else
				polyscope::error("Please load valid manifold surface mesh! \n");
		}

	}
	// Save Screenshot Image
	ImGui::SameLine();
	if (ImGui::Button("Save Screenshot")) {
		string imgFilePath = pfd::save_file("Save a Image File", "../data/",
			{ "PNG Image(*.png)", "*.png", "JPG Image(*.jpg)", "*.jpg", "All Files(*.*)", "*.*" }).result();
		if (!imgFilePath.empty())
			polyscope::screenshot(imgFilePath);
	}

	//-Feature menu-
	ImGui::Separator();
	ImGui::TextUnformatted("Feature");
	// Mesh Property
	if (ImGui::Button("Vertex Normal")) {
		if (psMesh != nullptr) {
			Mesh::Property_map<VertIds, Vector3> vNormals = cgMesh.add_property_map<VertIds, Vector3>("v:normal", CGAL::NULL_VECTOR).first;
			PMP::compute_vertex_normals(cgMesh, vNormals);
			vector<Vector3> vn(vNormals.begin(), vNormals.end());
			psMesh->addVertexVectorQuantity(name.vNormal, vn)->setEnabled(true);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Face Normal")) {
		if (psMesh != nullptr) {
			Mesh::Property_map<FaceIds, Vector3> fNormals = cgMesh.add_property_map<FaceIds, Vector3>("f:normal", CGAL::NULL_VECTOR).first;
			PMP::compute_face_normals(cgMesh, fNormals);
			vector<Vector3> fn(fNormals.begin(), fNormals.end());
			psMesh->addFaceVectorQuantity(name.fNormal, fn)->setEnabled(true);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Vertex Elevation")) {
		if (psMesh != nullptr) {
			vector<double> ev;
			for (glm::vec3 pt : meshV)
				ev.emplace_back(pt.z);
			psMesh->addVertexScalarQuantity(name.vElevation, ev)->setEnabled(true);
		}
	}

	if (ImGui::Button("Gaussian Curvature")) {
		if (psMesh != nullptr) {
			Mesh::Property_map<VertIds, Scalar> gCurvature = cgMesh.add_property_map<VertIds, Scalar>("v:gaussian_curvature", 0).first;
			PMP::interpolated_corrected_curvatures(cgMesh, CGAL::parameters::vertex_Gaussian_curvature_map(gCurvature));
			vector<double> gc(gCurvature.begin(), gCurvature.end());
			psMesh->addVertexScalarQuantity(name.gCurvature, gc)->setEnabled(true);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Mean Curvature")) {
		if (psMesh != nullptr) {
			Mesh::Property_map<VertIds, Scalar> mCurvature = cgMesh.add_property_map<VertIds, Scalar>("v:mean_curvature", 0).first;
			PMP::interpolated_corrected_curvatures(cgMesh, CGAL::parameters::vertex_mean_curvature_map(mCurvature));
			vector<double> mc(mCurvature.begin(), mCurvature.end());
			psMesh->addVertexScalarQuantity(name.mCurvature, mc)->setEnabled(true);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Square Mean Curvature")) {
		if (psMesh != nullptr) {
			Mesh::Property_map<VertIds, Scalar> mCurvature = cgMesh.add_property_map<VertIds, Scalar>("v:mean_curvature", 0).first;
			PMP::interpolated_corrected_curvatures(cgMesh, CGAL::parameters::vertex_mean_curvature_map(mCurvature));
			vector<double> mcq(mCurvature.begin(), mCurvature.end());
			std::for_each(mcq.begin(), mcq.end(), [](double& c) { c *= c; });
			psMesh->addVertexScalarQuantity(name.sCurvature, mcq)->setEnabled(true);
		}
	}

	//-Edit Menu-
	ImGui::Separator();
	ImGui::TextUnformatted("Edit");
	// Select Mesh Vertex
	if (ImGui::Button("Pick Vertex")) {
		if (psMesh != nullptr) {
			size_t pickedVertexId = -1;
			do {
				pickedVertexId = psMesh->selectVertex();
			} while (pickedVertexId == -1);
			pickedVIds.emplace_back(pickedVertexId);
			pickedVerts.emplace_back(meshV[pickedVertexId]);
			polyscope::registerPointCloud(name.pVertex, pickedVerts);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear Picked")) {
		if (!pickedVIds.empty()) {
			if (polyscope::hasPointCloud(name.pVertex))
				polyscope::removePointCloud(name.pVertex);
			if (polyscope::hasCurveNetwork(name.gPath))
				polyscope::removeCurveNetwork(name.gPath);
			pathLines.clear();
			pickedVerts.clear();
			pickedVIds.clear();
		}
	}
	ImGui::SameLine();
	ImGui::Text(" PickedVertsNum = %d", pickedVIds.size());

	// Geodesic Distance
	if (ImGui::Button("Heat Distance Map")) {
		if (psMesh != nullptr) {
			if (pickedVIds.size() > 0) {
				Mesh::Property_map<VertIds, double> vDistance = cgMesh.add_property_map<VertIds, double>("v:distance", 0).first;
				HeatDistance hmd(cgMesh);
				hmd.add_sources(pickedVIds);
				hmd.estimate_geodesic_distances(vDistance);
				vector<double> vd(vDistance.begin(), vDistance.end());
				psMesh->addVertexDistanceQuantity(name.vDistance, vd)->setEnabled(true);
			}
			else
				polyscope::warning("Please select at least one mesh vertex!");
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Compute Geodesic Path")) {
		if (psMesh != nullptr) {
			if (pickedVIds.size() > 1) {
				ExactDistance dPaths(cgMesh);
				for (size_t i = 0; i < pickedVIds.size() - 1; ++i) {
					dPaths.add_source_point(pickedVIds[i]);
					vector<Point3> points;
					dPaths.shortest_path_points_to_source_points(pickedVIds[i + 1], std::back_inserter(points));
					for (size_t j = points.size() - 1; j << points.size() > 0; --j)
						pathLines.emplace_back(points[j].x(), points[j].y(), points[j].z());
					if (i == pickedVIds.size() - 2) pathLines.emplace_back(points.front().x(), points.front().y(), points.front().z());
					dPaths.remove_all_source_points();
				}
				polyscope::registerCurveNetworkLine(name.gPath, pathLines);
			}
			else
				polyscope::warning("Please select at least two mesh vertexes!");
		}
	}

	//-Demo Menu-
	ImGui::Separator();
	ImGui::TextUnformatted("Demo");
	if (ImGui::Button("Voronoi Daigram")) {
		// generate random points in rectangle
		vector<glm::vec2> vdSites;
		Rectangle2 rt(-0.1, -0.1, 4.1, 4.1);
		vector<Point2> points;
		CGAL::Random_points_in_iso_rectangle_2<Point2> generator(Point2(0, 0), Point2(4, 4));
		for (size_t i = 0; i < 50; ++i) {
			vdSites.emplace_back(generator->x(), generator->y());
			points.push_back(*generator++);
		}
		polyscope::registerPointCloud2D("Sites", vdSites);

		DT2 dt2;
		dt2.insert(points.begin(), points.end());
		// draw triangulation
		vector<glm::vec2> dtEdgePoints;
		for (DFEI ei = dt2.finite_edges_begin(); ei != dt2.finite_edges_end(); ++ei) {
			Segment2 e = dt2.segment(ei);
			dtEdgePoints.push_back({ e.point(0).x(),e.point(0).y() });
			dtEdgePoints.push_back({ e.point(1).x(),e.point(1).y() });
		}
		polyscope::view::setNavigateStyle(polyscope::NavigateStyle::Planar);
		polyscope::view::projectionMode = polyscope::ProjectionMode::Orthographic;
		polyscope::registerCurveNetworkSegments2D("Delaunay2", dtEdgePoints);
		polyscope::view::resetCameraToHomeView();
		// draw voronoi
		vector<glm::vec2> vdEdgePoints;
		for (DFEI ei = dt2.finite_edges_begin(); ei != dt2.finite_edges_end(); ++ei) {
			Line2 l; Ray2 r; Segment2 s;
			CGAL::Object ve = dt2.dual(ei);
			if (ve.assign(l)) CropVoronoiEdge(l, rt, vdEdgePoints);
			else if (ve.assign(r)) CropVoronoiEdge(r, rt, vdEdgePoints);
			else if (ve.assign(s)) CropVoronoiEdge(s, rt, vdEdgePoints);
			else cout << "voronoi edge error";
		}
		polyscope::registerCurveNetworkSegments2D("Voronoi", vdEdgePoints);
	}

	// ImGui Config end
	ImGui::PopItemWidth();
}