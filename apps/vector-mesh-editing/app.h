#pragma once
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include "utils.h"

using namespace yocto;  // TODO(giacomo): Remove this.

struct Spline_Input {
  vector<mesh_point> control_points   = {};
  vector<bool>       is_smooth        = {};
  int                num_subdivisions = 4;

  inline int num_curves() const {
    return max((int)(control_points.size() - 1) / 3, 0);
  }
};
struct Spline_Output {
  vector<mesh_point> points = {};
};
struct Spline_Cache {
  vector<vec3f> positions        = {};
  geodesic_path tangent0         = {};  // TODO(giacomo): use mesh_paths?
  geodesic_path tangent1         = {};  // TODO(giacomo): use mesh_paths?
  vector<int>   curve_shapes     = {};  // id of shape in scene_data
  hash_set<int> curves_to_update = {};
};
// struct Spline_Draw {
//   vector<int>   curve_shapes     = {};  // id of shape in scene_data
//   hash_set<int> curves_to_update = {};
// };
struct Spline_View {
  Spline_Input&  input;
  Spline_Output& output;
  Spline_Cache&  cache;
  // Spline_Draw&   draw;
};

struct Splinesurf {
  vector<Spline_Input>  spline_input  = {};
  vector<Spline_Output> spline_output = {};
  vector<Spline_Cache>  spline_cache  = {};
  // vector<Spline_Draw>   spline_draw   = {};

  Spline_View get_spline_view(int id) {
    return Spline_View{spline_input[id], spline_output[id], spline_cache[id]};
  }
  inline int num_splines() const { return spline_input.size(); }
};

struct Editing {
  int         selected_spline_id        = -1;
  int         selected_control_point_id = -1;
  vector<int> selected_curves_id        = {};
};

struct App {
  struct Mesh : shape_data {
    vector<vec3i>        adjacencies = {};
    dual_geodesic_solver dual_solver = {};
    // bool_borders         borders     = {};

    // shape_bvh                  bvh                = {};
    // bbox3f                     bbox               = {};
    // int                        num_triangles      = 0;
    // int                        num_positions      = 0;
    // hash_map<int, vector<int>> triangulated_faces = {};
    // geodesic_solver            graph              = {};
  };

  scene_data scene = {};
  Mesh       mesh  = {};
  shape_bvh  bvh   = {};
  float      time  = 0;

  Editing                 editing           = {};
  Splinesurf              splinesurf        = {};
  std::unordered_set<int> splines_to_update = {};

  inline int add_spline() {
    auto id = (int)splinesurf.spline_input.size();
    splinesurf.spline_input.push_back({});
    splinesurf.spline_output.push_back({});
    splinesurf.spline_cache.push_back({});
    // splinesurf.spline_draw.push_back({});
    return id;
  }

  inline Spline_View get_spline_view(int id) { return splinesurf.get_spline_view(id); }

  inline Spline_View selected_spline() {
    if (splinesurf.spline_input.empty()) {
      add_spline();
      editing.selected_spline_id = 0;
    }
    return get_spline_view(editing.selected_spline_id);
  }
};
