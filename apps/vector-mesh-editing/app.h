#pragma once
#include <boolsurf/boolsurf.h>
#include <boolsurf/boolsurf_io.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_shape.h>
#include <yocto_gui/yocto_opengl.h>

#include <functional>
#include <utility>  // std::move?

#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif

#include "render.h"
#include "spline_editing.h"
#include "utils.h"

using namespace yocto;  // TODO(giacomo): Remove this.

struct Shape_Entry {
  int        id       = -1;
  shape_data shape    = {};
  frame3f    frame    = {};
  int        material = -1;
};

struct App {
  Render     render;
  scene_data scene      = {};
  bool_mesh  mesh       = {};
  bool_state bool_state = {};
  shape_bvh  bvh        = {};
  float      time       = 0;

  Editing    editing    = {};
  Splinesurf splinesurf = {};

  vector<ogl_shape>   spline_shapes  = {};
  vector<Shape_Entry> new_shapes     = {};
  int                 num_new_shapes = 0;
  // vector<instance_data> new_instances;

  std::vector<std::function<void()>> jobs       = {};
  bool                               update_bvh = false;

  bool  flag           = true;
  float line_thickness = 0.001;

  inline Spline_View get_spline_view(int id) {
    return splinesurf.get_spline_view(id);
  }
  inline const Const_Spline_View get_spline_view(int id) const {
    return splinesurf.get_spline_view(id);
  }

  inline Spline_View selected_spline() {
    return get_spline_view(editing.selection.spline_id);
  }
  inline const Const_Spline_View selected_spline() const {
    assert(splinesurf.num_splines());
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

  // if (!load_shape(test.model, app.mesh, error)) print_fatal(error);
  if (!load_shape(params.shape, app.mesh, error)) print_fatal(error);
  init_mesh(app.mesh);

  app.bool_state   = state_from_test(app.mesh, test, 0.0, false);
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

inline void set_shape(App& app, int id, const shape_data& shape,
    const frame3f& frame = identity3x4f, int material = 1) {
  app.new_shapes.push_back({id, (shape), frame, material});
  // app.new_shapes.push_back({id, std::move(shape), frame, material});
}

inline int add_shape(App& app, const shape_data& shape = {},
    const frame3f& frame = identity3x4f, int material = 1) {
  auto id = (int)app.scene.shapes.size() + app.num_new_shapes;
  set_shape(app, id, shape, frame, material);
  app.num_new_shapes += 1;
  return id;
}

inline void update_new_shapes(App& app) {
  auto& scene = app.scene;
  for (auto& [id, shape, frame, material] : app.new_shapes) {
    scene.shapes.resize(max((int)scene.shapes.size(), id + 1));
    scene.instances.resize(max((int)scene.instances.size(), id + 1));
    scene.shapes[id]    = shape;
    scene.instances[id] = {frame, id, material, true};
  }

  app.new_shapes.clear();
  app.num_new_shapes = 0;
}

void update_boolsurf_input(bool_state& state, const App& app) {
  auto& mesh = app.mesh;
  for (int i = 0; i < app.splinesurf.num_splines(); i++) {
    auto spline = app.splinesurf.get_spline_view(i);
    if (spline.input.control_points.size() < 2) continue;
    // Add new 1-polygon shape to state
    // if (test_polygon.empty()) continue;

    auto& bool_shape = state.bool_shapes.emplace_back();
    auto& polygon    = bool_shape.polygons.emplace_back();
    //      polygon.points   = test_polygon;
    for (auto& anchor : spline.input.control_points) {
      polygon.points.push_back(
          {anchor.point, {anchor.handles[0], anchor.handles[1]}});
    }
    recompute_polygon_segments(mesh, polygon);
  }
}

inline void update_cell_graphics(
    App& app, const bool_state& state, vector<int>& updated_shapes) {
  static auto cell_to_shape = hash_map<int, int>{};
  auto&       mesh          = app.mesh;

  auto timer = scope_timer("update graphics");
  for (int i = 0; i < state.cells.size(); i++) {
    auto& cell         = state.cells[i];
    auto  material_id  = app.scene.materials.size();
    auto& material     = app.scene.materials.emplace_back();
    material.type      = scene_material_type::glossy;
    material.roughness = 0.4;
    if (state.labels.size())
      material.color = get_cell_color(state, i, false);
    else
      material.color = vec3f{0.8, 0.8, 0.8};

    material.type      = scene_material_type::glossy;
    material.roughness = 0.5;

    auto shape = shape_data{};
    // TODO(giacomo): Too many copies of positions.
    shape.positions = mesh.positions;
    shape.triangles.resize(cell.faces.size());
    for (int i = 0; i < cell.faces.size(); i++) {
      shape.triangles[i] = mesh.triangles[cell.faces[i]];
    }

    auto shape_id = -1;
    if (auto it = cell_to_shape.find(i); it == cell_to_shape.end()) {
      shape_id         = add_shape(app, shape, {}, material_id);
      cell_to_shape[i] = shape_id;
    } else {
      shape_id = it->second;
      set_shape(app, shape_id, shape, {}, material_id);
    }
    updated_shapes += shape_id;
  }

  for (auto& [cell_id, shape_id] : cell_to_shape) {
    if (cell_id >= state.cells.size()) {
      set_shape(app, shape_id, {});
      updated_shapes += shape_id;
    }
  }

  app.scene.instances[0].visible = false;
}

inline void toggle_handle_visibility(App& app, bool visible) {
  auto selection = app.editing.selection;
  if (selection.spline_id == -1) return;
  if (selection.control_point_id == -1) return;
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

inline void set_selected_spline(App& app, int spline_id) {
  toggle_handle_visibility(app, false);
  app.editing.selection           = {};
  app.editing.selection.spline_id = spline_id;
}

inline void set_selected_point(
    App& app, int spline_id, int anchor_id, int handle_id) {
  toggle_handle_visibility(app, false);
  app.editing.selection                  = {};
  app.editing.selection.spline_id        = spline_id;
  app.editing.selection.control_point_id = anchor_id;
  app.editing.selection.handle_id        = handle_id;
  toggle_handle_visibility(app, true);
}

inline void process_mouse(
    App& app, vector<int>& updated_shapes, const glinput_state& input) {
  if (input.modifier_alt) return;
  if (!input.mouse_left) {
    app.editing.holding_control_point = false;
    app.editing.creating_new_point    = false;
    return;
  }

  auto point = intersect_mesh(app, input);
  if (point.face == -1) {
    // app.editing.selection.spline_id = {};
    set_selected_spline(app, -1);
    return;
  }

  {
    auto timer     = scope_timer("update boolsurf");
    app.bool_state = {};
    update_boolsurf_input(app.bool_state, app);
    compute_cells(app.mesh, app.bool_state);
    // compute_shapes(app.bool_state);
    update_cell_graphics(app, app.bool_state, updated_shapes);
    reset_mesh(app.mesh);
  }

  auto selection = app.editing.selection;
  if (selection.spline_id == -1) return;
  if (selection.control_point_id == -1) return;
  app.editing.holding_control_point = true;

  auto spline = app.selected_spline();
  move_selected_point(app.splinesurf, app.editing.selection, app.mesh, point,
      app.editing.creating_new_point);

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
          set_selected_point(app, spline_id, i, k);
          // selection.spline_id        = spline_id;
          // selection.control_point_id = i;
          // selection.handle_id        = k;
          return true;
        }
      }

      auto hit = intersect_mesh_point(app.mesh, ray, anchor.point, radius);
      if (hit) {
        set_selected_point(app, spline_id, i, -1);
        // selection.spline_id        = spline_id;
        // selection.control_point_id = i;
        // selection.handle_id        = -1;
        return true;
      }
    }
  }
  return false;
}

inline void process_click(
    App& app, vector<int>& updated_shapes, const glinput_state& input) {
  if (input.modifier_alt) return;
  if (input.mouse_right_click) {
    auto click_timer = scope_timer("click-timer");
    app.render.stop_render();

    // auto t              = app.mesh.triangles;
    // auto p              = app.mesh.positions;
    // app.mesh            = {};
    // app.mesh.triangles  = t;
    // app.mesh.positions  = p;
    // app.scene.shapes[0] = app.mesh;
    // init_mesh(app.mesh);

    //
    //    struct Intersection {
    //      int   curve_id = -1;
    //      float t        = 0;
    //    };
    //    auto intersection_map = hash_map<int, vector<Intersection>>{};
    //
    //    for (auto& isec : state.intersections) {
    //      for (int k = 0; k < 2; k++) {
    //        auto& item     = isec.locations[k];
    //        auto  shape_id = item.shape_id;
    //        intersection_map[shape_id].push_back({item.curve_id, item.t});
    //      }
    //    }
    //    auto stuff = vector<std::function<void()>>{};
    //    for (auto& [shape_id, insertions] : intersection_map) {
    //      auto num_insertions = 0;
    //      sort(insertions.begin(), insertions.end(),
    //          [](auto& a, auto& b) { return a.curve_id < b.curve_id; });
    //
    //      // TODO(giacomo): [null polygon refactor].
    //      auto spline = app.splinesurf.get_spline_view(shape_id - 1);
    //
    //      auto old_input = spline.input;
    //      for (auto& insertion : insertions) {
    //        auto curve_id = insertion.curve_id + num_insertions;
    //        auto cp       = spline.input.control_polygon(insertion.curve_id);
    //
    //        auto [left, right] = insert_bezier_point(app.mesh.dual_solver,
    //            app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies,
    //            cp, insertion.t, false, -1);
    //        spline.input.control_points[curve_id].handles[1] = left[1];
    //        auto p = anchor_point{right[0], {left[2], right[1]}};
    //        spline.input.control_points[curve_id + 1].handles[0] = right[2];
    //        stuff.push_back([&, p, curve_id]() {
    //          auto add_app_shape = [&]() -> int { return add_shape(app, {});
    //          }; insert_anchor_point(spline, p, curve_id + 1, app.mesh,
    //          add_app_shape);
    //        });
    //        num_insertions += 1;
    //      }
    //
    //      for (auto& s : stuff) s();
    //      stuff.clear();
    //
    //      // Update graphics.
    //      for (int i = 0; i < spline.cache.points.size(); i++) {
    //        spline.cache.points_to_update.insert(i);
    //      }
    //      for (int i = 0; i < spline.cache.curves.size(); i++) {
    //        spline.cache.curves_to_update.insert(i);
    //      }
    app.render.restart();
  }

  if (!input.mouse_left_click) return;

  // Compute clicked point and exit if it mesh was not clicked.
  auto point = intersect_mesh(app, input);
  if (point.face == -1) return;

  // Update selection. If it was changed, don't add new points.
  auto mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
      input.mouse_pos.y / float(input.window_size.y)};
  // toggle_handle_visibility(app, false);
  if (update_selection(app, mouse_uv)) {
    // toggle_handle_visibility(app, true);
    return;
  }

  // If there are no splines, create the first one and select it.
  if (app.splinesurf.num_splines() == 0) {
    auto spline_id = add_spline(app.splinesurf);
    set_selected_spline(app, spline_id);
  }

  // If no spline is selected, do nothing.
  if (app.editing.selection.spline_id == -1) return;

  // Add new anchor point.
  auto spline          = app.selected_spline();
  auto add_point_shape = [&]() -> int {
    auto radius   = app.line_thickness * 2;
    auto shape_id = add_shape(app, make_sphere(8, radius, 1), {}, 1);
    updated_shapes += shape_id;
    return shape_id;
  };
  auto add_path_shape = [&]() -> int { return add_shape(app, {}); };
  auto anchor_id      = add_anchor_point(
      spline, point, add_point_shape, add_path_shape);
  set_selected_point(app, app.editing.selection.spline_id, anchor_id, 1);
  // app.editing.selection.control_point_id = anchor_id;
  // app.editing.selection.handle_id        = 1;
  app.editing.creating_new_point = true;
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

inline vector<mesh_segment> make_segments(
    const bool_mesh& mesh, const mesh_point& start, const mesh_point& end) {
  auto path      = compute_geodesic_path(mesh, start, end);
  auto threshold = 0.001f;
  for (auto& l : path.lerps) {
    l = yocto::clamp(l, 0 + threshold, 1 - threshold);
  }
  auto segments = mesh_segments(mesh.triangles, mesh.positions, path);
  auto t        = path_parameters(path, mesh.triangles, mesh.positions);
  for (int i = 0; i < segments.size(); i++) {
    segments[i].t_start = t[i];
    segments[i].t_end   = t[i + 1];
  }
  return segments;
};

vector<mesh_segment> make_bezier_segments(const bool_mesh& mesh,
    const array<mesh_point, 4>& control_polygon, int num_subdivisions) {
  auto result = vector<mesh_segment>{};
  auto points = bezier_spline(mesh, control_polygon, num_subdivisions);
  for (int k = 0; k < points.size() - 1; k++) {
    auto segments = make_segments(mesh, points[k], points[k + 1]);
    auto min      = float(k) / points.size();
    auto max      = float(k + 1) / points.size();
    for (auto& s : segments) {
      s.t_start = s.t_start * (max - min) + min;
      s.t_end   = s.t_end * (max - min) + min;
    }
    result += segments;
  }
  return result;
}

void update_output(Spline_Output& output, const Spline_Input& input,
    const bool_mesh& mesh, const hash_set<int>& curves_to_update) {
  for (auto curve_id : curves_to_update) {
    if (output.segments.size() <= curve_id)
      output.segments.resize(curve_id + 1);
    auto control_polygon = input.control_polygon(curve_id);
    if (control_polygon[0] == control_polygon[3]) {
      output.segments[curve_id] = make_segments(
          mesh, control_polygon[0], control_polygon[3]);
    } else {
      output.segments[curve_id] = make_bezier_segments(
          mesh, control_polygon, input.num_subdivisions);
    }
  }
}

void update_cache(App& app, Spline_Cache& cache, const Spline_Input& input,
    const Spline_Output& output, scene_data& scene,
    vector<int>& updated_shapes) {
  auto& mesh = scene.shapes[0];
  for (auto curve_id : cache.curves_to_update) {
    cache.curves[curve_id].positions.resize(output.segments[curve_id].size());
    for (int i = 0; i < output.segments[curve_id].size(); i++) {
      auto& segment                       = output.segments[curve_id][i];
      cache.curves[curve_id].positions[i] = eval_position(
          mesh.triangles, mesh.positions, {segment.face, segment.start});
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
      auto  positions = path_positions(
          tangent.path, app.mesh.triangles, app.mesh.positions);

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

inline void draw_widgets(App& app, const glinput_state& input) {
  // auto names    = vector<string>{name};
  // auto selected = 0;
  auto& render = app.render;
  auto  edited = 0;

  //  draw_glcombobox("name", selected, names);
  auto current = (int)render.render_current;
  draw_glprogressbar("sample", current, render.params_ptr->samples);

  draw_gllabel("selected spline", app.editing.selection.spline_id);
  draw_gllabel(
      "selected control_point", app.editing.selection.control_point_id);
  if (draw_glbutton("add spline")) {
    auto spline_id = add_spline(app.splinesurf);
    set_selected_spline(app, spline_id);
    // app.editing.selection           = {};
    // app.editing.selection.spline_id = spline_id;
  }
  if (draw_glbutton("close spline")) {
    auto add_app_shape = [&]() -> int { return add_shape(app, {}); };
    close_spline(app.selected_spline(), add_app_shape);
  }
  if (begin_glheader("render")) {
    auto edited  = 0;
    auto tparams = *render.params_ptr;
    //    edited += draw_glcombobox("camera", tparams.camera, camera_names);
    edited += draw_glslider("resolution", tparams.resolution, 180, 4096);
    edited += draw_glslider("samples", tparams.samples, 16, 4096);
    edited += draw_glcombobox(
        "tracer", (int&)tparams.sampler, trace_sampler_names);
    edited += draw_glcombobox(
        "false color", (int&)tparams.falsecolor, trace_falsecolor_names);
    edited += draw_glslider("bounces", tparams.bounces, 1, 128);
    edited += draw_glslider("batch", tparams.batch, 1, 16);
    edited += draw_glslider("clamp", tparams.clamp, 10, 1000);
    edited += draw_glcheckbox("envhidden", tparams.envhidden);
    continue_glline();
    edited += draw_glcheckbox("filter", tparams.tentfilter);
    edited += draw_glslider("pratio", tparams.pratio, 1, 64);
    // edited += draw_glslider("exposure", tparams.exposure, -5, 5);
    end_glheader();
    if (edited) {
      render.stop_render();
      *render.params_ptr = tparams;
      render.restart();
    }
  }
  auto& params = *render.params_ptr;
  if (begin_glheader("tonemap")) {
    edited += draw_glslider("exposure", params.exposure, -5, 5);
    edited += draw_glcheckbox("filmic", params.filmic);
    edited += draw_glcheckbox("denoise", params.denoise);
    end_glheader();
    if (edited) {
      tonemap_image_mt(
          render.display, render.image, params.exposure, params.filmic);
      set_image(render.glimage, render.display);
    }
  }
}

static bool uiupdate_image_params(
    const glinput_state& input, glimage_params& glparams) {
  // handle mouse
  if (input.mouse_left && input.modifier_alt && !input.widgets_active) {
    if (input.modifier_ctrl) {
      glparams.scale *= yocto::pow(
          2.0f, (input.mouse_pos.y - input.mouse_last.y) * 0.001f);
      return true;
    } else {
      glparams.center += input.mouse_pos - input.mouse_last;
      return true;
    }
  }
  return false;
}

inline void update(
    App& app, Render& render, bvh_data& bvh, const glinput_state& input) {
  static auto updated_shapes = vector<int>{};

  auto& scene         = app.scene;
  auto& params        = *render.params_ptr;
  auto  edited_camera = scene.cameras[params.camera];
  if (uiupdate_camera_params(input, edited_camera)) {
    render.stop_render();
    scene.cameras[params.camera] = edited_camera;
    render.restart();
  }

  {
    process_click(app, updated_shapes, input);
    process_mouse(app, updated_shapes, input);

    if (app.jobs.size()) {
      render.stop_render();
      for (auto& job : app.jobs) job();
      app.jobs.clear();
      render.restart();
    }

    if (app.new_shapes.size()) {
      render.stop_render();
      update_new_shapes(app);
      // scene.shapes += app.new_shapes;
      // scene.instances += app.new_instances;
      update_splines(app, scene, updated_shapes);
      updated_shapes.clear();

      bvh = make_bvh(scene, params);
      render.restart();
    }
  }
}

static void update_image_params(const glinput_state& input,
    const image_data& image, glimage_params& glparams) {
  glparams.window                           = input.window_size;
  glparams.framebuffer                      = input.framebuffer_viewport;
  std::tie(glparams.center, glparams.scale) = camera_imview(glparams.center,
      glparams.scale, {image.width, image.height}, glparams.window,
      glparams.fit);
}

// Open a window and show an scene via path tracing
void view_raytraced_scene(App& app, const string& title, const string& name,
    scene_data& scene, const trace_params& params_ = {}, bool print = true,
    bool edit = false) {
  // copy params and camera
  auto params = params_;

  // build bvh
  if (print) print_progress_begin("build bvh");
  auto bvh = make_bvh(scene, params);
  if (print) print_progress_end();

  // init renderer
  if (print) print_progress_begin("init lights");
  auto lights = make_lights(scene, params);
  if (print) print_progress_end();

  // fix renderer type if no lights
  if (lights.lights.empty() && is_sampler_lit(params)) {
    if (print) print_info("no lights presents --- switching to eyelight");
    params.sampler = trace_sampler_type::eyelight;
  }

  // init state
  if (print) print_progress_begin("init state");
  auto& render = app.render;
  render.init(scene, bvh, lights, params);
  if (print) print_progress_end();

  // callbacks
  auto callbacks    = glwindow_callbacks{};
  callbacks.init_cb = [&](const glinput_state& input) {
    auto lock = std::lock_guard{render.render_mutex};
    init_image(render.glimage);
    set_image(render.glimage, render.display);
  };
  callbacks.clear_cb = [&](const glinput_state& input) {
    clear_image(render.glimage);
  };
  callbacks.draw_cb = [&](const glinput_state& input) {
    // update image
    if (render.render_update) {
      auto lock = std::lock_guard{render.render_mutex};
      set_image(render.glimage, render.display);
      render.render_update = false;
    }
    update_image_params(input, render.image, render.glparams);
    draw_image(render.glimage, render.glparams);
  };
  callbacks.widgets_cb = [&](const glinput_state& input) {
    draw_widgets(app, input);
  };
  callbacks.uiupdate_cb = [&](const glinput_state& input) {
    update(app, render, bvh, input);
  };

  // run ui
  run_ui({1280 + 320, 720}, title, callbacks);

  // done
  render.stop_render();
}
