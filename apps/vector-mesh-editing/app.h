#pragma once
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <functional>

#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif

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
  vector<int>   point_shapes     = {};  // id of shape in scene_data
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
  inline int num_splines() const { return (int)spline_input.size(); }
};

inline int add_spline(Splinesurf& splinesurf) {
  auto id = (int)splinesurf.spline_input.size();
  splinesurf.spline_input.push_back({});
  splinesurf.spline_output.push_back({});
  splinesurf.spline_cache.push_back({});
  // splinesurf.spline_draw.push_back({});
  return id;
}

struct Editing {
  struct Selection {
    int spline_id        = -1;
    int control_point_id = -1;
  };
  Selection selection             = {};
  bool      holding_control_point = false;
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

  std::vector<std::function<void()>> jobs       = {};
  bool                               update_bvh = false;

  inline Spline_View get_spline_view(int id) {
    return splinesurf.get_spline_view(id);
  }

  inline Spline_View selected_spline() {
    if (splinesurf.num_splines() == 0) {
      editing.selection.spline_id = add_spline(splinesurf);
    }
    return get_spline_view(editing.selection.spline_id);
  }
};

template <typename Params>
void init_app(App& app, const Params& params) {
  // loading shape
  auto error = string{};
  if (!load_shape(params.shape, app.mesh, error)) print_fatal(error);
  auto bbox = invalidb3f;
  for (auto& pos : app.mesh.positions) bbox = merge(bbox, pos);
  for (auto& pos : app.mesh.positions)
    pos = (pos - center(bbox)) / max(size(bbox));

  app.mesh.adjacencies = face_adjacencies(app.mesh.triangles);
  app.mesh.dual_solver = make_dual_geodesic_solver(
      app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies);
  app.bvh = make_triangles_bvh(app.mesh.triangles, app.mesh.positions, {});

  // make scene
  app.scene = make_shape_scene(app.mesh, params.addsky);

  // Add line material.
  auto line_material  = app.scene.materials[0];
  line_material.color = {1, 0, 0};
  app.scene.materials.push_back(line_material);
}

inline mesh_point intersect_mesh(App& app, const glinput_state& input) {
  auto& shape    = app.scene.shapes.at(0);
  auto& camera   = app.scene.cameras.at(0);
  auto  mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
      input.mouse_pos.y / float(input.window_size.y)};
  auto  ray      = camera_ray(
      camera.frame, camera.lens, camera.aspect, camera.film, mouse_uv);
  auto isec = intersect_triangles_bvh(
      app.bvh, shape.triangles, shape.positions, ray, false);
  if (isec.hit) {
    // if (stroke.empty() || stroke.back().element != isec.element ||
    // stroke.back().uv != isec.uv) {
    // stroke.push_back({isec.element, isec.uv});
    // printf("point: %d, %f %f\n", isec.element, isec.uv.x, isec.uv.y);
    // updated = true;
    // }
    return mesh_point{isec.element, isec.uv};
  }
  return {};
}

inline void process_mouse(App& app, const glinput_state& input) {
  if (!input.mouse_left) {
    app.editing.holding_control_point = false;
    return;
  }
  auto point = intersect_mesh(app, input);
  if (point.face == -1) {
    app.editing.selection = {};
    return;
  }
  auto selection = app.editing.selection;
  if (selection.spline_id == -1) return;
  //  if (selection.curve_id == -1) return;
  if (selection.control_point_id == -1) return;
  auto spline = app.selected_spline();
  spline.input.control_points[selection.control_point_id] = point;
  auto shape_id = spline.cache.point_shapes[selection.control_point_id];
  app.jobs.push_back([shape_id, point, &app]() {
    app.scene.instances[shape_id].frame.o = eval_position(
        app.mesh.triangles, app.mesh.positions, point);
    app.update_bvh = true;
  });
}

inline void process_click(App& app, scene_data& scene, glscene_state& glscene,
    vector<int>& updated_shapes, vector<shape_data>& new_shapes,
    vector<instance_data>& new_instances, const glinput_state& input) {
  if (input.mouse_left_click) {
    auto add_shape =
        [](scene_data& scene, glscene_state& glscene, const shape_data& shape,
            vector<shape_data>&    new_shapes,
            vector<instance_data>& new_instances,
            const frame3f& frame = identity3x4f, int material = 1) {
          auto id = (int)scene.shapes.size() + (int)new_shapes.size();
          // scene.shapes.push_back({});
          new_shapes.push_back(shape);
          new_instances.push_back({frame, id, material});
          glscene.shapes.push_back({});
          return id;
        };

    if (app.splinesurf.num_splines() == 0) {
      app.editing.selection.spline_id = add_spline(app.splinesurf);
    }
    if (app.editing.selection.spline_id == -1) return;

    auto point = intersect_mesh(app, input);
    if (point.face == -1) return;

    auto spline   = app.selected_spline();
    auto point_id = (int)spline.input.control_points.size();
    spline.input.control_points.push_back(point);
    app.editing.selection.control_point_id = point_id;

    {
      auto frame = frame3f{};
      frame.o    = eval_position(app.mesh.triangles, app.mesh.positions, point);
      auto  shape_id = add_shape(scene, glscene, make_sphere(8, 0.005, 1),
          new_shapes, new_instances, frame);
      auto& shape    = scene.shapes[shape_id];
      spline.cache.point_shapes.push_back(shape_id);
      updated_shapes.push_back(shape_id);
    }

    if ((spline.input.control_points.size() - 1) % 3 == 0 &&
        spline.input.control_points.size() >= 4) {
      printf("cp: %ld\n", app.splinesurf.spline_input[0].control_points.size());
      auto curve_id = (spline.input.control_points.size() - 1) / 3 - 1;
      spline.cache.curves_to_update.insert((int)curve_id);

      auto shape_id = add_shape(
          scene, glscene, {}, new_shapes, new_instances, {});
      spline.cache.curve_shapes.push_back(shape_id);
    }
  }
}

// TODO(giacomo): Put following stuff in splinesurf.h
inline vector<mesh_point> bezier_spline(const App::Mesh& mesh,
    const std::array<mesh_point, 4>& control_points, int subdivisions) {
  return compute_bezier_path(mesh.dual_solver, mesh.triangles, mesh.positions,
      mesh.adjacencies, control_points, subdivisions);
}
inline vector<mesh_point> bezier_spline(const App::Mesh& mesh,
    const vector<mesh_point>& control_points, int subdivisions) {
  return compute_bezier_path(mesh.dual_solver, mesh.triangles, mesh.positions,
      mesh.adjacencies, control_points, subdivisions);
}

void update_output(Spline_Output& output, const Spline_Input& input,
    const App::Mesh& mesh, const hash_set<int>& curves_to_update) {
  if (curves_to_update.size()) {
    output.points = bezier_spline(
        mesh, input.control_points, input.num_subdivisions);
  }
  // for (auto& curve_id : spline.cache.curves_to_update) {
  // auto id             = curve_id * 3 + 1;
  // auto control_points = *(
  //     std::array<mesh_point, 4>*)&spline.input.control_points[id];
  // auto curve = bezier_spline(app.mesh, spline.input.control_points,
  //     spline.input.num_subdivisions);
  // auto points_per_curve = 3 * (1 << num_subdivisions) + 1;
  // for (int i = 0; i < curve.size(); i++) {
  //   spline.output.points[id + i] = curve[i];
  // }
  // }
}

void update_cache(Spline_Cache& cache, const Spline_Output& output,
    scene_data& scene, vector<int>& updated_shapes) {
  auto& mesh = scene.shapes[0];
  if (cache.curves_to_update.size()) {
    cache.positions.resize(output.points.size());
    for (int i = 0; i < output.points.size(); i++) {
      cache.positions[i] = eval_position(
          mesh.triangles, mesh.positions, output.points[i]);
    }
  }

  for (auto& c : cache.curves_to_update) {
    auto  id             = cache.curve_shapes[c];
    auto& shape          = scene.shapes[id];
    auto  line_thickness = 0.005;
    shape         = polyline_to_cylinders(cache.positions, 16, line_thickness);
    shape.normals = compute_normals(shape);
    updated_shapes += cache.curve_shapes[c];
  }
  cache.curves_to_update.clear();
}

inline void update_splines(
    App& app, scene_data& scene, vector<int>& updated_shapes) {
  // Update bezier outputs of edited curves.
  for (int i = 0; i < app.splinesurf.num_splines(); i++) {
    auto spline = app.get_spline_view(i);
    update_output(
        spline.output, spline.input, app.mesh, spline.cache.curves_to_update);
  }

  // Update bezier positions of edited curves.
  for (int i = 0; i < app.splinesurf.num_splines(); i++) {
    auto spline = app.get_spline_view(i);
    update_cache(spline.cache, spline.output, scene, updated_shapes);
  }
}
