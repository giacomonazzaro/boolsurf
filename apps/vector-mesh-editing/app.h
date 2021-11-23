#pragma once
#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>

#include <functional>

#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif

#include "spline_editing.h"
#include "utils.h"

using namespace yocto;  // TODO(giacomo): Remove this.

struct App {
  scene_data scene      = {};
  bool_mesh  mesh       = {};
  bool_state bool_state = {};
  shape_bvh  bvh        = {};
  float      time       = 0;

  Editing    editing    = {};
  Splinesurf splinesurf = {};

  vector<shape_data>    new_shapes;
  vector<instance_data> new_instances;

  std::vector<std::function<void()>> jobs       = {};
  bool                               update_bvh = false;

  bool  flag           = true;
  float line_thickness = 0.005;

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

void add_mesh_edges(scene_data& scene, const shape_data& mesh) {
  auto& instance     = scene.instances.emplace_back();
  instance.shape     = (int)scene.shapes.size();
  instance.material  = (int)scene.materials.size();
  auto& material     = scene.materials.emplace_back();
  material.color     = {0, 0, 0};
  material.type      = scene_material_type::glossy;
  material.roughness = 0.5;
  auto& edges        = scene.shapes.emplace_back();
  for (auto& tr : mesh.triangles) {
    for (int k = 0; k < 3; k++) {
      auto a = tr[k];
      auto b = tr[(k + 1) % 3];
      if (a > b) continue;
      auto index = (int)edges.positions.size();
      edges.radius.push_back(0.001);
      edges.radius.push_back(0.001);
      edges.lines.push_back({index, index + 1});
      edges.positions.push_back(mesh.positions[a]);
      edges.positions.push_back(mesh.positions[b]);
    }
  }
}

template <typename Params>
void init_app(App& app, const Params& params) {
  // loading shape
  auto error = string{};

  auto test = bool_test{};
  load_test(test, params.shape);

  if (!load_shape(test.model, app.mesh, error)) print_fatal(error);
  init_mesh(app.mesh);

  app.bool_state = state_from_test(app.mesh, test, 0.0, false);

  compute_cells(app.mesh, app.bool_state);
  //  compute_shapes(app.bool_state);
  app.mesh.triangles.resize(app.mesh.num_triangles);
  app.mesh.positions.resize(app.mesh.num_positions);
  app.mesh.normals = compute_normals(app.mesh);
  app.bvh = make_triangles_bvh(app.mesh.triangles, app.mesh.positions, {});

  // make scene
  app.scene = make_shape_scene(app.mesh, params.addsky);

  // Add line material.
  auto spline_material  = app.scene.materials[0];
  spline_material.color = {1, 0, 0};
  app.scene.materials.push_back(spline_material);

  auto tangent_material  = spline_material;
  tangent_material.color = {0, 0, 1};
  app.scene.materials.push_back(tangent_material);

  // add_mesh_edges(app.scene, app.mesh);
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
    return mesh_point{isec.element, isec.uv};
  }
  return {};
}

inline int add_shape(App& app, const shape_data& shape = {},
    const frame3f& frame = identity3x4f, int material = 1) {
  auto id = (int)app.scene.shapes.size() + (int)app.new_shapes.size();
  app.new_shapes.push_back(shape);
  app.new_instances.push_back({frame, id, material});
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
    app.editing.selection.spline_id = {};
    return;
  }

  auto selection = app.editing.selection;
  if (selection.spline_id == -1) return;
  if (selection.control_point_id == -1) return;
  app.editing.holding_control_point = true;

  auto spline = app.selected_spline();
  move_selected_point(app.splinesurf, app.editing.selection, app.mesh, point);

  updated_shapes +=
      spline.cache.points[app.editing.selection.control_point_id].anchor_id;
  updated_shapes +=
      spline.cache.points[app.editing.selection.control_point_id].handle_ids[0];
  updated_shapes +=
      spline.cache.points[app.editing.selection.control_point_id].handle_ids[1];

  // TODO(giacomo): Cleanup.
  auto touched_curves = vector<int>{};
  auto prev           = selection.control_point_id - 1;
  if (prev < 0) {
    if (spline.input.is_closed)
      prev = spline.input.control_points.size() - 1;
    else
      prev = 0;
  }
  touched_curves = {selection.control_point_id, prev};
  for (auto curve : touched_curves) {
    if (curve >= 0 && curve < spline.input.num_curves())
      spline.cache.curves_to_update.insert(curve);
  }
}

inline bool intersect_mesh_point(const bool_mesh& mesh, const ray3f& ray,
    const mesh_point& point, float point_size) {
  auto  uv = vec2f{};
  float dist;
  auto  center = eval_position(mesh.triangles, mesh.positions, point);
  auto  hit    = intersect_sphere(ray, center, point_size, uv, dist);
  return hit;
}

inline void toggle_handle_visibility(App& app, bool visible) {
  auto selection = app.editing.selection;
  if (selection.spline_id == -1) return;
  for (int k = 0; k < 2; k++) {
    auto handle_id = app.selected_spline()
                         .cache.points[selection.control_point_id]
                         .handle_ids[k];
    app.scene.instances[handle_id].visible = visible;
    auto tangent_id                        = app.selected_spline()
                          .cache.points[selection.control_point_id]
                          .tangents[k]
                          .shape_id;
    app.scene.instances[tangent_id].visible = visible;
  }
}

inline bool update_selection(App& app, const vec2f& mouse_uv) {
  auto& selection = app.editing.selection;
  auto& camera    = app.scene.cameras[0];
  auto  radius    = 2 * app.line_thickness;
  auto  ray       = camera_ray(
      camera.frame, camera.lens, camera.aspect, camera.film, mouse_uv);

  for (int spline_id = 0; spline_id < app.splinesurf.spline_input.size();
       spline_id++) {
    auto& spline = app.splinesurf.spline_input[spline_id];
    for (int i = 0; i < spline.control_points.size(); i++) {
      auto& anchor = spline.control_points[i];
      for (int k = 0; k < 2; k++) {
        auto hit = intersect_mesh_point(
            app.mesh, ray, anchor.handles[k], radius);
        if (hit) {
          selection.spline_id        = spline_id;
          selection.control_point_id = i;
          selection.handle_id        = k;
          return true;
        }
      }

      auto hit = intersect_mesh_point(app.mesh, ray, anchor.point, radius);
      if (hit) {
        selection.spline_id        = spline_id;
        selection.control_point_id = i;
        selection.handle_id        = -1;
        return true;
      }
    }
  }
  return false;
}

inline void process_click(
    App& app, vector<int>& updated_shapes, const glinput_state& input) {
  if (!input.mouse_left_click) return;

  // Compute clicked point and exit if it mesh was not clicked.
  auto point = intersect_mesh(app, input);
  if (point.face == -1) return;

  // Update selection. If it was changed, don't add new points.
  auto mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
      input.mouse_pos.y / float(input.window_size.y)};
  toggle_handle_visibility(app, false);
  if (update_selection(app, mouse_uv)) {
    toggle_handle_visibility(app, true);
    return;
  }

  // If there are no splines, create the first one and select it.
  if (app.splinesurf.num_splines() == 0) {
    app.editing.selection           = {};
    app.editing.selection.spline_id = add_spline(app.splinesurf);
  }

  // If no spline is selected, do nothing.
  if (app.editing.selection.spline_id == -1) return;

  // Add new anchor point.
  auto spline        = app.selected_spline();
  auto add_app_shape = [&]() -> int { return add_shape(app, {}); };
  auto anchor_id     = add_anchor_point(spline, point, add_app_shape);
  app.editing.selection.control_point_id = anchor_id;
  app.editing.selection.handle_id        = 1;
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
}

void update_cache(const App& app, Spline_Cache& cache,
    const Spline_Input& input, const Spline_Output& output, scene_data& scene,
    vector<int>& updated_shapes) {
  auto& mesh = scene.shapes[0];
  for (auto curve_id : cache.curves_to_update) {
    cache.curves[curve_id].positions.resize(output.points[curve_id].size());
    for (int i = 0; i < output.points[curve_id].size(); i++) {
      cache.curves[curve_id].positions[i] = eval_position(
          mesh.triangles, mesh.positions, output.points[curve_id][i]);
    }
  }

  for (auto curve_id : cache.curves_to_update) {
    auto  shape_id = cache.curves[curve_id].shape_id;
    auto& shape    = scene.shapes[shape_id];
    shape          = polyline_to_cylinders(
        cache.curves[curve_id].positions, 16, app.line_thickness);
    shape.normals = compute_normals(shape);
    updated_shapes += cache.curves[curve_id].shape_id;
  }

  for (auto point_id : cache.points_to_update) {
    auto& anchor = cache.points[point_id];
    for (int k = 0; k < 2; k++) {
      auto& tangent   = anchor.tangents[k];
      auto  shape_id  = tangent.shape_id;
      auto  positions = path_positions(tangent.path, app.mesh.triangles,
          app.mesh.positions, app.mesh.adjacencies);

      auto& instance    = scene.instances[shape_id];
      auto& shape       = scene.shapes[shape_id];
      instance.material = 2;
      shape = polyline_to_cylinders(positions, 16, app.line_thickness * 0.6);
      shape.normals = compute_normals(shape);
      updated_shapes += shape_id;

      auto& handle_instance    = scene.instances[anchor.handle_ids[k]];
      handle_instance.material = 2;
      handle_instance.frame.o  = positions.back();
      auto& handle_shape       = scene.shapes[handle_instance.shape];
      if (handle_shape.triangles.empty()) {
        auto radius  = app.line_thickness * 2;
        handle_shape = make_sphere(8, radius, 1);
        updated_shapes += anchor.handle_ids[k];
      }
    }
    auto& anchor_instance    = scene.instances[anchor.anchor_id];
    anchor_instance.material = 1;
    anchor_instance.frame.o  = eval_position(
        app.mesh, input.control_points[point_id].point);
    auto& anchor_shape = scene.shapes[anchor_instance.shape];
    if (anchor_shape.triangles.empty()) {
      auto radius  = app.line_thickness * 2;
      anchor_shape = make_sphere(8, radius, 1);
      updated_shapes += anchor.anchor_id;
    }
  }
  cache.points_to_update.clear();
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
    update_cache(
        app, spline.cache, spline.input, spline.output, scene, updated_shapes);
  }
}
