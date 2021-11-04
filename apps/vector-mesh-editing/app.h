#pragma once
#include <boolsurf/boolsurf.h>
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
  bool               is_closed        = false;

  inline std::array<mesh_point, 4> control_polygon(int curve_id) const {
    if (is_closed && curve_id == (control_points.size() - 1) / 3 - 1) {
      auto ptr = &control_points.back();
      return {*(ptr - 3), *(ptr - 2), *(ptr - 1), control_points[0]};
    }
    return *(std::array<mesh_point, 4>*)&control_points[curve_id * 3];
  }
  inline int num_curves() const {
    // TODO(giacomo): if (is_closed) { }
    return max((int)(control_points.size() - 1) / 3, 0);
  }
  inline vector<int> curves_from_control_point(int point_id) {
    if (point_id == 0) {
      if (is_closed) {
        return {0, num_curves() - 1};
      } else {
        return {0};
      }
    }

    // anchor
    if (point_id % 3 == 0) {
      if (point_id == control_points.size() - 1) {
        return {point_id / 3 - 1};
      }
      return {point_id / 3 - 1, point_id / 3};
    } else
      return {point_id / 3};
  }
};

struct Spline_Output {
  vector<vector<mesh_point>> points = {};
};

struct Spline_Cache {
  struct Tangent {
    geodesic_path path     = {};  // TODO(giacomo): use mesh_paths?
    int           shape_id = -1;
  };
  struct Curve {
    vector<vec3f>          positions = {};
    int                    shape_id  = -1;
    std::array<Tangent, 2> tangents  = {};  // TODO(giacomo): use mesh_paths?
  };

  std::vector<Curve> curves           = {};  // TODO(giacomo): use mesh_paths?
  hash_set<int>      curves_to_update = {};
  vector<int>        point_shapes     = {};  // id of shape in scene_data
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
  // struct Mesh : shape_data {
  //   vector<vec3i>        adjacencies = {};
  //   dual_geodesic_solver dual_solver = {};
  // bool_borders         borders     = {};

  // shape_bvh                  bvh                = {};
  // bbox3f                     bbox               = {};
  // int                        num_triangles      = 0;
  // int                        num_positions      = 0;
  // hash_map<int, vector<int>> triangulated_faces = {};
  // geodesic_solver            graph              = {};
  // };

  scene_data scene = {};
  bool_mesh  mesh  = {};
  shape_bvh  bvh   = {};
  float      time  = 0;

  Editing    editing    = {};
  Splinesurf splinesurf = {};
  // std::unordered_set<int> splines_to_update = {};
  vector<shape_data>    new_shapes;
  vector<instance_data> new_instances;

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

inline int add_shape(scene_data& scene, const shape_data& shape,
    vector<shape_data>& new_shapes, vector<instance_data>& new_instances,
    const frame3f& frame = identity3x4f, int material = 1) {
  auto id = (int)scene.shapes.size() + (int)new_shapes.size();
  // scene.shapes.push_back({});
  new_shapes.push_back(shape);
  new_instances.push_back({frame, id, material});
  //  glscene.shapes.push_back({});
  return id;
}

inline void process_mouse(
    App& app, vector<int>& updated_shapes, const glinput_state& input) {
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
  app.editing.holding_control_point = true;

  auto spline = app.selected_spline();
  spline.input.control_points[selection.control_point_id] = point;
  auto shape_id = spline.cache.point_shapes[selection.control_point_id];

  // app.jobs.push_back([shape_id, point, &app, &updated_shapes]() {
  app.scene.instances[shape_id].frame.o = eval_position(
      app.mesh.triangles, app.mesh.positions, point);
  updated_shapes += shape_id;

  auto touched_curves = spline.input.curves_from_control_point(
      selection.control_point_id);
  for (auto curve : touched_curves) {
    if (curve >= 0 && curve < spline.input.num_curves())
      spline.cache.curves_to_update.insert(curve);
  }
  // });
}

inline int add_curve(App& app, Spline_Cache& cache) {
  auto curve_id = (int)cache.curves.size();
  cache.curves.emplace_back();
  cache.curves_to_update.insert((int)curve_id);
  cache.curves[curve_id].shape_id = add_shape(
      app.scene, {}, app.new_shapes, app.new_instances, {});
  return curve_id;
}

inline void process_click(
    App& app, vector<int>& updated_shapes, const glinput_state& input) {
  if (!input.mouse_left_click) return;
  auto& scene = app.scene;
  if (app.splinesurf.num_splines() == 0) {
    app.editing.selection.spline_id = add_spline(app.splinesurf);
  }
  if (app.editing.selection.spline_id == -1) return;

  auto point = intersect_mesh(app, input);
  if (point.face == -1) return;

  for (int spline_id = 0; spline_id < app.splinesurf.spline_input.size();
       spline_id++) {
    for (int i = 0;
         i < app.splinesurf.spline_input[spline_id].control_points.size();
         i++) {
      auto  point    = app.splinesurf.spline_input[spline_id].control_points[i];
      auto  mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
          input.mouse_pos.y / float(input.window_size.y)};
      auto& camera   = app.scene.cameras[0];
      auto  ray      = camera_ray(
          camera.frame, camera.lens, camera.aspect, camera.film, mouse_uv);
      auto  uv = vec2f{};
      float dist;
      auto  line_thickness = 0.005 * 2;
      auto  center         = eval_position(
          app.mesh.triangles, app.mesh.positions, point);
      auto hit = intersect_sphere(ray, center, line_thickness, uv, dist);
      if (hit) {
        app.editing.selection.spline_id        = spline_id;
        app.editing.selection.control_point_id = i;
        return;
      }
    }
  }

  auto spline   = app.selected_spline();
  auto point_id = (int)spline.input.control_points.size();
  spline.input.control_points.push_back(point);
  app.editing.selection.control_point_id = point_id;

  {
    auto frame = frame3f{};
    frame.o    = eval_position(app.mesh.triangles, app.mesh.positions, point);
    auto  shape_id = add_shape(scene, make_sphere(8, 0.005, 1), app.new_shapes,
        app.new_instances, frame);
    auto& shape    = scene.shapes[shape_id];
    spline.cache.point_shapes.push_back(shape_id);
    updated_shapes.push_back(shape_id);
  }

  if ((spline.input.control_points.size() - 1) % 3 == 0 &&
      spline.input.control_points.size() >= 4) {
    printf("cp: %ld\n", app.splinesurf.spline_input[0].control_points.size());
    //      auto curve_id = (spline.input.control_points.size() - 1) / 3 - 1;
    //      spline.cache.curves_to_update.insert((int)curve_id);
    add_curve(app, spline.cache);
  }
}

// TODO(giacomo): Put following stuff in splinesurf.h
inline vector<mesh_point> bezier_spline(const bool_mesh& mesh,
    const std::array<mesh_point, 4>& control_points, int subdivisions) {
  return compute_bezier_path(mesh.dual_solver, mesh.triangles, mesh.positions,
      mesh.adjacencies, control_points, subdivisions);
}
inline vector<mesh_point> bezier_spline(const bool_mesh& mesh,
    const vector<mesh_point>& control_points, int subdivisions) {
  return compute_bezier_path(mesh.dual_solver, mesh.triangles, mesh.positions,
      mesh.adjacencies, control_points, subdivisions);
}

void update_output(Spline_Output& output, const Spline_Input& input,
    const bool_mesh& mesh, const hash_set<int>& curves_to_update) {
  for (auto curve_id : curves_to_update) {
    if (output.points.size() <= curve_id) output.points.resize(curve_id + 1);
    auto control_polygon    = input.control_polygon(curve_id);
    output.points[curve_id] = bezier_spline(
        mesh, control_polygon, input.num_subdivisions);
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
  for (auto curve_id : cache.curves_to_update) {
    cache.curves[curve_id].positions.resize(output.points[curve_id].size());
    for (int i = 0; i < output.points[curve_id].size(); i++) {
      cache.curves[curve_id].positions[i] = eval_position(
          mesh.triangles, mesh.positions, output.points[curve_id][i]);
    }
  }

  for (auto curve_id : cache.curves_to_update) {
    auto  shape_id       = cache.curves[curve_id].shape_id;
    auto& shape          = scene.shapes[shape_id];
    auto  line_thickness = 0.005;
    shape                = polyline_to_cylinders(
        cache.curves[curve_id].positions, 16, line_thickness);
    shape.normals = compute_normals(shape);
    updated_shapes += cache.curves[curve_id].shape_id;
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
