#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <algorithm>
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

  std::vector<std::pair<double, TopoDS_Face>> face_areas;

  int face_counter = 0;
  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    TopoDS_Face face = TopoDS::Face(exp.Current());
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    double area = props.Mass();
    face_areas.emplace_back(area, face);
    std::cout << "🔍 Face " << face_counter++ << " → Area: " << area << "\n";
  }

  std::cout << "📊 Total faces found: " << face_areas.size() << "\n";

  std::sort(face_areas.begin(), face_areas.end(),
            [](const std::pair<double, TopoDS_Face>& a,
               const std::pair<double, TopoDS_Face>& b) {
              return a.first > b.first;
            });

  int N = 30;  // Sampling grid resolution

  for (size_t i = 0; i < std::min(size_t(5), face_areas.size()); ++i) {
    const auto& [area, face] = face_areas[i];
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);

    Standard_Real umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);

    std::cout << "🧩 Sampling Face " << i << " (Area: " << area << ")\n";
    std::cout << "   → UV bounds: u=[" << umin << ", " << umax << "] v=["
              << vmin << ", " << vmax << "]\n";

    std::vector<gp_Pnt> points;

    for (int u = 0; u <= N; ++u) {
      for (int v = 0; v <= N; ++v) {
        double uu = umin + (umax - umin) * u / static_cast<double>(N);
        double vv = vmin + (vmax - vmin) * v / static_cast<double>(N);

        BRepClass_FaceClassifier classifier(face, gp_Pnt2d(uu, vv), 1e-6);
        if (classifier.State() == TopAbs_IN ||
            classifier.State() == TopAbs_ON) {
          gp_Pnt p = surf->Value(uu, vv);
          points.push_back(p);
        }
      }
    }

    std::cout << "   → Sampled " << points.size() << " valid points.\n";

    std::string filename = "../data/top_face_" + std::to_string(i) + ".ply";
    write_simple_ply(filename, points);
    std::cout << "📤 Exported to " << filename << "\n\n";
  }

  std::cout << "🎉 Done! Open the PLYs in MeshLab to explore.\n";
  return 0;
}
