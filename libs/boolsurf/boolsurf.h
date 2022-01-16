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
  int   boundary_id;
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

struct bool_shape {
  vector<int>   generators = {};
  bool          is_root    = true;
  vec3f         color      = {0, 0, 0};
  hash_set<int> cells      = {};
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

// Informazioni per la triangolazione di una faccia della mesh
// Contiene: UV coords dei nodi locali di un triangolo.
// Indici globali della mesh corrispondenti ai nodi locali
// Edges con indici locali per vincolare la triangolazione
// Mappa che va da lato del triangolo k = 1, 2, 3 e a lista di nodi e lerp
// corrispondenti su quel lato (serve per creare ulteriori vincoli)
struct triangulation_info {
  int face = -1;

  vector<vec2f>                      nodes   = {};
  vector<int>                        indices = {};
  vector<vec2i>                      edges   = {};
  array<vector<pair<int, float>>, 3> edgemap = {};
};

inline bool_state*& global_state() {
  static bool_state* state = nullptr;
  return state;
}

inline bool_mesh*& global_mesh() {
  static bool_mesh* mesh = nullptr;
  return mesh;
}

namespace yocto {  // TODO(giacomo): Fix this.
struct bool_operation {
  enum struct Type {
    op_union,
    op_difference,
    op_intersection,
    op_symmetrical_difference
  };
  int  shape_a = -1;
  int  shape_b = -1;
  Type type    = Type::op_union;

  inline static const auto type_names = vector<string>{"op_union",
      "op_difference", "op_intersection", "op_symmetrical_difference"};
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

void       compute_shapes(bool_state& state);
void       compute_shape_borders(const bool_mesh& mesh, bool_state& state);
bool_state compute_border_polygons(const bool_state& state);
void       compute_bool_operation(bool_state& state, const bool_operation& op);
void       compute_bool_operations(
          bool_state& state, const vector<bool_operation>& ops);

void compute_symmetrical_difference(
    bool_state& state, const vector<int>& shapes);

vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
    const vector<vec3f>& positions, const geodesic_path& path);

vector<mesh_segment> make_curve_segments(
    const bool_mesh& mesh, const anchor_point& start, const anchor_point& end);
// vector<vector<mesh_segment>> make_boundary_segments(
//     const bool_mesh& mesh, const vector<anchor_point>& polygon);

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

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *     DEBUGGING STUFF
 *
 */

template <typename F>
static vector<int> flood_fill(const bool_mesh& mesh, const vector<int>& start,
    const int polygon, F&& check) {
  auto visited = vector<bool>(mesh.adjacencies.size(), false);

  auto result = vector<int>();
  auto stack  = start;

  while (!stack.empty()) {
    auto face = stack.back();
    stack.pop_back();

    if (visited[face]) continue;
    visited[face] = true;

    result.push_back(face);

    for (auto neighbor : mesh.adjacencies[face]) {
      if (neighbor < 0 || visited[neighbor])
        continue;
      else if (check(face, -polygon) && check(neighbor, -polygon))
        // Check if "face" is not inner and "neighbor" is outer
        stack.push_back(neighbor);
      else if (check(neighbor, polygon))
        stack.push_back(neighbor);
    }
  }

  return result;
}

template <typename F>
static vector<int> flood_fill(
    const bool_mesh& mesh, const vector<int>& start, F&& check) {
  auto visited = vector<bool>(mesh.adjacencies.size(), false);

  auto result = vector<int>();
  auto stack  = start;

  while (!stack.empty()) {
    auto face = stack.back();
    stack.pop_back();

    if (visited[face]) continue;
    visited[face] = true;

    result.push_back(face);

    for (auto neighbor : mesh.adjacencies[face]) {
      if (neighbor < 0 || visited[neighbor]) continue;
      if (check(face, neighbor)) stack.push_back(neighbor);
    }
  }

  return result;
}

template <typename F>
static void flood_fill_debug(
    const bool_mesh& mesh, const vector<int>& start, F&& check) {
  int face = -1;
  if (debug_stack().empty()) {
    debug_restart() = true;
    return;
  }
  while (!debug_stack().empty()) {
    auto f = debug_stack().back();
    debug_stack().pop_back();
    if (debug_visited()[f]) continue;
    face = f;
    break;
  }
  if (face == -1) return;

  debug_visited()[face] = true;

  debug_result().push_back(face);

  // auto tag = mesh.borders.tags[face];
  // auto adj = mesh.adjacencies[face];
  //  printf("\nfrom %d: tag(%d %d %d) adj(%d %d %d)\n", face, tag[0], tag[1],
  //      tag[2], adj[0], adj[1], adj[2]);

  // for (auto neighbor : mesh.adjacencies[face]) {
  for (int k = 0; k < 3; k++) {
    auto neighbor = mesh.adjacencies[face][k];
    if (neighbor < 0 || debug_visited()[neighbor]) continue;
    if (check(face, k)) {
      debug_stack().push_back(neighbor);
    }
    // auto tag = mesh.borders.tags[neighbor];
    // auto adj = mesh.adjacencies[neighbor];
    //      printf("ok   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0],
    //      tag[1],
    //          tag[2], adj[0], adj[1], adj[2]);
    //    printf("no   %d: tag(%d %d %d) adj(%d %d %d)\n", neighbor, tag[0],
    //    tag[1],
    //        tag[2], adj[0], adj[1], adj[2]);
  }
}
