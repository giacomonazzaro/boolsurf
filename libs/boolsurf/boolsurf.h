#pragma once

#include "boolsurf_utils.h"
//
#include "../mesh_spline_editing/spline_editing.h"

using namespace yocto;
using namespace std;

const static int null_label = -999;

struct facet {
  std::array<vec2f, 3> corners = {};  // barycentric coordinated of vertices
  int                  id      = -1;  // id in mesh.triangles
};

struct bool_mesh : spline_mesh {
  // Boolsurf output
  vector<bool> is_edge_on_boundary = {};

  struct shape_segment {  // TODO(giacomo): Move outside of mesh.
    int   shape_id;
    int   boundary_id;
    int   curve_id;
    float t;
    int   face_in;
    int   face_out;
  };
  vector<shape_segment>        polygon_borders    = {};
  vector<int>                  face_tags          = {};
  hash_map<int, vector<facet>> triangulated_faces = {};

  // Backup
  vector<vec3i> old_adjacencies = {};
  int           num_triangles   = 0;
  int           num_positions   = 0;
};

struct bool_point {
  int   shape_id;
  int   boundary_id = 0;
  int   curve_id;
  float t;
};

struct mesh_segment {
  vec2f start   = {};
  vec2f end     = {};
  float t_start = 0.0f;
  float t_end   = 0.0f;
  int   face    = -1;
};

vector<mesh_segment> mesh_segments(
    const vector<vec3i>& triangles, const geodesic_path& path);

vector<mesh_segment> make_curve_segments(
    const bool_mesh& mesh, const anchor_point& start, const anchor_point& end);

struct bool_shape {
  vector<int> generators = {};
  bool        is_root    = true;
  vec3f       color      = {0, 0, 0};
  // hash_set<int> cells      = {};
};

struct bool_cell {
  struct bool_cell_arc {
    int  cell_id, shape_id;
    bool entering;
  };
  hash_set<vec3i> adjacency = {};  // {cell_id, crossed_polygon_id, entering}
};

// TODO(giacomo): This is a pair of bool_point.
struct bool_shape_intersection {
  vec2i shape_ids    = {-1, -1};
  vec2i boundary_ids = {-1, -1};
  vec2i curve_ids    = {-1, -1};
  vec2f t            = {0.0f, 0.0f};
};

namespace yocto {
struct BSH_patch {
  int         adj_cell_0  = -1;
  int         adj_cell_1  = -1;
  float       weight      = 1;
  int         shape_id    = -1;
  int         num_samples = 0;
  vector<int> adj_patches = {};
};

struct BSH_graph {
  int               num_cells        = 0;
  vector<BSH_patch> patches          = {};
  vector<int>       segment_to_patch = {};
};
}  // namespace yocto

struct bool_state {
  vector<bool_shape> shapes = {};

  vector<bool_shape_intersection> intersections = {};

  vector<bool_cell>   cells           = {};
  vector<vector<int>> labels          = {};
  vector<int>         shape_from_cell = {};
  hash_set<int>       invalid_shapes  = {};
  vector<int>         shapes_sorting  = {};

  BSH_graph    bsh_input  = {};
  vector<bool> bsh_output = {};
};

inline void add_intersection(vector<bool_shape_intersection>& intersections,
    int shape_id0, int boundary_id0, int curve_id0, float t0, int shape_id1,
    int boundary_id1, int curve_id1, float t1) {
  auto& isec        = intersections.emplace_back();
  isec.shape_ids    = {shape_id0, shape_id1};
  isec.boundary_ids = {boundary_id0, boundary_id1};
  isec.curve_ids    = {curve_id0, curve_id1};
  isec.t            = {t0, t1};
}

inline vector<vector<int>> make_cell_faces(
    const vector<int>& face_tags, int num_cells) {
  auto cell_faces = vector<vector<int>>(num_cells);
  for (int i = 0; i < face_tags.size(); i++) {
    if (face_tags[i] == -1) continue;
    cell_faces[face_tags[i]].push_back(i);
  }
  return cell_faces;
}

inline vector<vector<vec3i>> make_cell_triangles(const vector<int>& face_tags,
    const vector<vec3i>& triangles, int num_cells) {
  auto cell_triangles = vector<vector<vec3i>>(num_cells);
  for (int i = 0; i < face_tags.size(); i++) {
    if (face_tags[i] == -1) continue;
    cell_triangles[face_tags[i]].push_back(triangles[i]);
  }
  return cell_triangles;
}

vector<vector<vec3i>> shapes_triangles(
    const bool_state& state, const bool_mesh& mesh);

namespace yocto {  // TODO(giacomo): Fix this.
struct bool_operation {
  enum struct Type { op_union, op_difference, op_intersection, op_xor };
  int  shape_a = -1;
  int  shape_b = -1;
  Type type    = Type::op_union;

  inline static const auto type_names = vector<string>{
      "op_union", "op_difference", "op_intersection", "op_xor"};
};
}  // namespace yocto

void init_mesh(bool_mesh& mesh);
void reset_mesh(bool_mesh& mesh);

void update_polygon(bool_state& state, const bool_mesh& mesh, int polygon_id);

void              slice_mesh(bool_mesh& mesh, bool_state& state,
                 const vector<vector<vector<vector<mesh_segment>>>>& shapes);
vector<bool_cell> make_cell_graph(bool_mesh& mesh);
void              compute_cell_labels(bool_state& state);

bool compute_cells(bool_mesh& mesh, bool_state& state,
    const vector<vector<vector<vector<mesh_segment>>>>& shapes);

BSH_graph    make_bsh_input(bool_state& state, bool_mesh& mesh,
       const vector<vector<vector<vector<mesh_segment>>>>& shape_segments);
vector<bool> run_bsh(const BSH_graph& bsh);

vector<int> make_shape_from_cell(const bool_state& state);
// void compute_shapes(const bool_state& state);
// void       compute_shape_borders(const bool_mesh& mesh, bool_state& state);
// bool_state compute_border_polygons(const bool_state& state);
void compute_bool_operation(bool_state& state, const bool_operation& op);
void compute_bool_operations(
    bool_state& state, const vector<bool_operation>& ops);

template <typename Add_Shape>
void insert_anchor_points(Splinesurf& splinesurf, const spline_mesh& mesh,
    const vector<bool_shape_intersection>& intersections,
    Add_Shape&&                            add_shape) {
  auto isec_points = vector<bool_point>{};
  for (int i = 0; i < intersections.size(); i++) {
    auto& point0       = isec_points.emplace_back();
    point0.shape_id    = intersections[i].shape_ids[0];
    point0.boundary_id = intersections[i].boundary_ids[0];
    point0.curve_id    = intersections[i].curve_ids[0];
    point0.t           = intersections[i].t[0];
    auto& point1       = isec_points.emplace_back();
    point1.shape_id    = intersections[i].shape_ids[1];
    point1.boundary_id = intersections[i].boundary_ids[1];
    point1.curve_id    = intersections[i].curve_ids[1];
    point1.t           = intersections[i].t[1];
  }
  std::sort(isec_points.begin(), isec_points.end(), [](auto& a, auto& b) {
    if (a.shape_id != b.shape_id) return a.shape_id < b.shape_id;
    if (a.boundary_id != b.boundary_id) return a.boundary_id < b.boundary_id;
    if (a.curve_id != b.curve_id)
      return a.curve_id > b.curve_id;  // starting from end!
    return a.t > b.t;                  // starting from end!
  });

  for (int i = 0; i < isec_points.size(); i++) {
    auto point  = isec_points[i];
    auto spline = splinesurf.get_spline_view(point.shape_id);
    auto cp     = spline.input.control_polygon(point.curve_id);
    auto t      = point.t;

    // Adjust t of following intersections.
    for (int k = i + 1; k < isec_points.size(); k++) {
      if (isec_points[k].shape_id != point.shape_id) break;
      if (isec_points[k].boundary_id != point.boundary_id) break;
      if (isec_points[k].curve_id != point.curve_id) break;
      isec_points[k].t /= t;
    }

    auto [left, right] = insert_bezier_point(mesh, cp, t);

    // Edit previous handle.
    spline.input.control_points[point.curve_id].handles[1] = left[1];

    // Edit next handle.
    auto next = point.curve_id + 1;
    if (next >= (int)spline.input.control_points.size()) next = 0;
    spline.input.control_points[next].handles[0] = right[2];

    // Inserted anchor point.
    auto p = anchor_point{right[0], {left[2], right[1]}};
    insert_anchor_point(spline, point.curve_id + 1, p, add_shape);
    // TODO(giacomo): Trigger update only of instersected curves.
  }
}

void compute_symmetrical_difference(
    bool_state& state, const vector<int>& shapes);

vec3f get_cell_color(const bool_state& state, int cell_id, bool color_shapes);

struct scope_timer {
  string  message    = "";
  int64_t start_time = -1;
  bool    print      = true;
  scope_timer(const string& msg);
  ~scope_timer();  // print time when scope ends
};
#define _PROFILE_SCOPE(name) ;
// #define _PROFILE_SCOPE(name) auto _profile = scope_timer(string(name));
#define _PROFILE() _PROFILE_SCOPE(__FUNCTION__)

inline bool_state*& global_state() {
  static bool_state* state = nullptr;
  return state;
}

inline bool_mesh*& global_mesh() {
  static bool_mesh* mesh = nullptr;
  return mesh;
}
