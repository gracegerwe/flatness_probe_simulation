#include <BRepClass_FaceClassifier.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <fstream>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <iostream>

void write_simple_ply(const std::string& filename,
                      const std::vector<gp_Pnt>& points) {
  std::ofstream ply(filename);

  if (!ply.is_open()) {
    std::cerr << "❌ Failed to open " << filename << " for writing.\n";
    return;
  }

  ply << "ply\nformat ascii 1.0\n";
  ply << "element vertex " << points.size() << "\n";
  ply << "property float x\nproperty float y\nproperty float z\n";
  ply << "end_header\n";
  for (const auto& p : points) {
    ply << p.X() << " " << p.Y() << " " << p.Z() << "\n";
  }
  ply.close();

  std::cout << "✅ Wrote " << points.size() << " points to " << filename
            << "\n";
}

int main() {
  std::cout << "📦 Loading STEP file...\n";

  STEPControl_Reader reader;
  if (reader.ReadFile("/Users/gracegerwe/Downloads/Servo_Horn.step") !=
      IFSelect_RetDone) {
    std::cerr << "Failed to load STEP.\n";
    return 1;
  }
  reader.TransferRoots();
  TopoDS_Shape shape = reader.OneShape();

  std::cout << "✅ STEP file loaded.\n";

  TopoDS_Face face;
  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    TopoDS_Face f = TopoDS::Face(exp.Current());
    Handle(Geom_Surface) surf = BRep_Tool::Surface(f);
    if (!surf.IsNull() && Handle(Geom_Plane)::DownCast(surf)) {
      face = f;
      std::cout << "✅ Found planar face.\n";
      break;
    }
  }

  if (face.IsNull()) {
    std::cerr << "No flat face found.\n";
    return 1;
  }

  // Sample grid points on face
  Standard_Real umin, umax, vmin, vmax;
  BRepTools::UVBounds(face, umin, umax, vmin, vmax);

  std::cout << "📐 UV bounds: u=[" << umin << ", " << umax << "] v=[" << vmin
            << ", " << vmax << "]\n";

  int N = 10;
  std::vector<gp_Pnt> points;
  Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

  for (int i = 0; i <= N; ++i) {
    for (int j = 0; j <= N; ++j) {
      double u = umin + (umax - umin) * i / double(N);
      double v = vmin + (vmax - vmin) * j / double(N);
      gp_Pnt p = surf->Value(u, v);
      points.push_back(p);
    }
  }

  write_simple_ply("../data/probed_points.ply", points);
  std::cout << "Done. View in MeshLab.\n";
  return 0;
}
