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

struct Anchor_Point {
  mesh_point point      = {};
  mesh_point handles[2] = {{}, {}};
  bool       is_smooth  = true;
};

struct Spline_Input {
  vector<Anchor_Point> control_points   = {};
  int                  num_subdivisions = 4;
  bool                 is_closed        = false;

  inline std::array<mesh_point, 4> control_polygon(int curve_id) const {
    if (is_closed && curve_id == control_points.size() - 1) {
      auto a = control_points[curve_id].point;
      auto b = control_points[curve_id].handles[1];
      auto c = control_points[0].handles[0];
      auto d = control_points[0].point;
      return {a, b, c, d};
    }
    auto a = control_points[curve_id].point;
    auto b = control_points[curve_id].handles[1];
    auto c = control_points[curve_id + 1].handles[0];
    auto d = control_points[curve_id + 1].point;
    return {a, b, c, d};
  }
  inline int num_curves() const {
    if (control_points.size() == 1) return 0;
    if (is_closed)
      return control_points.size();
    else
      return control_points.size() - 1;
  }
};

struct Spline_Output {
  vector<vector<mesh_point>> points = {};
};

struct Spline_Cache {
  struct Curve {
    vector<vec3f> positions = {};
    int           shape_id  = -1;
  };
  struct Tangent {
    geodesic_path path     = {};
    int           shape_id = -1;
  };
  struct Point {
    int                    anchor_id     = -1;
    int                    handle_ids[2] = {-1, -1};
    std::array<Tangent, 2> tangents      = {};
  };
  std::vector<Curve> curves           = {};
  std::vector<Point> points           = {};
  hash_set<int>      curves_to_update = {};
  hash_set<int>      points_to_update = {};
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
    int handle_id        = -1;
  };
  Selection  selection             = {};
  mesh_point clicked_point         = {};
  bool       holding_control_point = false;
};

inline geodesic_path shortest_path(
    const bool_mesh& mesh, const mesh_point& start, const mesh_point& end) {
  //  check_point(start);
  //  check_point(end);
  auto path = geodesic_path{};
  if (start.face == end.face) {
    path.start = start;
    path.end   = end;
    path.strip = {start.face};
    return path;
  }
  auto strip = compute_strip(
      mesh.dual_solver, mesh.triangles, mesh.positions, end, start);
  path = shortest_path(
      mesh.triangles, mesh.positions, mesh.adjacencies, start, end, strip);
  return path;
}

inline vec2f tangent_path_direction(
    const bool_mesh& mesh, const geodesic_path& path) {
  auto find = [](const vec3i& vec, int x) {
    for (int i = 0; i < size(vec); i++)
      if (vec[i] == x) return i;
    return -1;
  };

  auto direction = vec2f{};
  auto start_tr  = triangle_coordinates(
      mesh.triangles, mesh.positions, path.start);

  if (path.lerps.empty()) {
    direction = interpolate_triangle(
        start_tr[0], start_tr[1], start_tr[2], path.end.uv);
  } else {
    auto x    = path.lerps[0];
    auto k    = find(mesh.adjacencies[path.strip[0]], path.strip[1]);
    direction = lerp(start_tr[k], start_tr[(k + 1) % 3], x);
  }
  return normalize(direction);
}

struct App {
  scene_data scene = {};
  bool_mesh  mesh  = {};
  shape_bvh  bvh   = {};
  float      time  = 0;

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

inline void move_selected_point(App& app, Splinesurf& splinesurf,
    const Editing::Selection& selection, const bool_mesh& mesh,
    const mesh_point& point) {
  assert(selection.spline_id != -1);
  assert(selection.control_point_id != -1);
  auto  spline = splinesurf.get_spline_view(selection.spline_id);
  auto& anchor = spline.input.control_points[selection.control_point_id];
  if (selection.handle_id == -1) {
    auto& point_cache = spline.cache.points[selection.control_point_id];
    auto  offset      = shortest_path(mesh, anchor.point, point);

#if 0
    // Old implementation. Probably faster but less stable.
    auto rot = parallel_transport_rotation(
        mesh.triangles, mesh.positions, mesh.adjacencies, offset);

    for (int k = 0; k < 2; k++) {
      auto& tangent = point_cache.tangents[k].path;
      auto  len     = path_length(
          tangent, mesh.triangles, mesh.positions, mesh.adjacencies);
      auto dir          = tangent_path_direction(mesh, tangent);
      dir               = rot * dir;
      tangent           = straightest_path(mesh, point, dir, len);
      anchor.handles[k] = point_cache.tangents[k].path.end;
    }
#endif

    auto offset_dir = tangent_path_direction(mesh, offset);
    auto offset_len = path_length(
        offset, mesh.triangles, mesh.positions, mesh.adjacencies);
    for (int k = 0; k < 2; k++) {
      auto& tangent = point_cache.tangents[k].path;
      auto  rot     = parallel_transport_rotation(
          mesh.triangles, mesh.positions, mesh.adjacencies, tangent);

      auto dir          = rot * offset_dir;
      auto p            = straightest_path(mesh, tangent.end, dir, offset_len);
      tangent           = shortest_path(app.mesh, point, p.end);
      anchor.handles[k] = tangent.end;
    }
    anchor.point = point;

  } else {
    auto& handle = anchor.handles[selection.handle_id];
    handle       = point;

    // Update tangents and other handle
    int   k           = selection.handle_id;
    auto& point_cache = spline.cache.points[selection.control_point_id];
    point_cache.tangents[k].path = shortest_path(
        mesh, anchor.point, anchor.handles[k]);

    if (anchor.is_smooth) {
      auto dir = tangent_path_direction(mesh, point_cache.tangents[k].path);
      auto len = path_length(point_cache.tangents[k].path, mesh.triangles,
          mesh.positions, mesh.adjacencies);
      point_cache.tangents[1 - k].path = straightest_path(
          mesh, anchor.point, -dir, len);
      anchor.handles[1 - k] = point_cache.tangents[1 - k].path.end;
    }
  }

  spline.cache.points_to_update.insert(selection.control_point_id);
}

template <typename Params>
void init_app(App& app, const Params& params) {
  // loading shape
  auto error = string{};
  if (!load_shape(params.shape, app.mesh, error)) print_fatal(error);
  init_mesh(app.mesh);

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

inline int add_shape(scene_data& scene, const shape_data& shape,
    vector<shape_data>& new_shapes, vector<instance_data>& new_instances,
    const frame3f& frame = identity3x4f, int material = 1) {
  auto id = (int)scene.shapes.size() + (int)new_shapes.size();
  new_shapes.push_back(shape);
  new_instances.push_back({frame, id, material});
  return id;
}

inline int add_curve(App& app, Spline_Cache& cache, const Spline_Input& input) {
  auto curve_id = (int)cache.curves.size();
  cache.curves_to_update.insert((int)curve_id);

  auto& curve    = cache.curves.emplace_back();
  curve.shape_id = add_shape(
      app.scene, {}, app.new_shapes, app.new_instances, {});
  return curve_id;
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
  move_selected_point(
      app, app.splinesurf, app.editing.selection, app.mesh, point);

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

inline int add_control_point(App& app, Spline_View& spline,
    vector<int>& updated_shapes, const Anchor_Point& anchor) {
  auto point_id = (int)spline.input.control_points.size();
  spline.input.control_points.push_back(anchor);

  auto& cache = spline.cache.points.emplace_back();
  auto  frame = frame3f{};
  frame.o = eval_position(app.mesh.triangles, app.mesh.positions, anchor.point);
  auto radius     = app.line_thickness * 2;
  cache.anchor_id = add_shape(app.scene, make_sphere(8, radius, 1),
      app.new_shapes, app.new_instances, frame);
  updated_shapes.push_back(cache.anchor_id);

  for (int k = 0; k < 2; k++) {
    auto radius = app.line_thickness * 0.6 * 2;
    frame.o     = eval_position(
        app.mesh.triangles, app.mesh.positions, anchor.handles[k]);
    cache.handle_ids[k] = add_shape(app.scene, make_sphere(8, radius, 1),
        app.new_shapes, app.new_instances, frame);
    updated_shapes.push_back(cache.handle_ids[k]);
  }

  cache.tangents[0].shape_id = add_shape(
      app.scene, {}, app.new_shapes, app.new_instances, {}, 2);
  cache.tangents[1].shape_id = add_shape(
      app.scene, {}, app.new_shapes, app.new_instances, {}, 2);

  spline.cache.points_to_update.insert(point_id);
  return point_id;
}

inline void process_click(
    App& app, vector<int>& updated_shapes, const glinput_state& input) {
  // Compute clicked point and exit if it mesh was not clicked.
  if (input.mouse_left_click) {
    auto point = intersect_mesh(app, input);
    if (point.face == -1) return;

    auto mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
        input.mouse_pos.y / float(input.window_size.y)};
    if (update_selection(app, mouse_uv)) {
      return;
    }

    // If there are no splines, create the first one and select it.
    if (app.splinesurf.num_splines() == 0) {
      app.editing.selection           = {};
      app.editing.selection.spline_id = add_spline(app.splinesurf);
    }

    // app.editing.clicked_point = point;

    // If no spline is selected, do nothing.
    if (app.editing.selection.spline_id == -1) return;
    auto spline       = app.selected_spline();
    auto anchor       = Anchor_Point{};
    anchor.point      = point;
    anchor.handles[0] = point;
    anchor.handles[1] = point;
    auto a            = add_control_point(app, spline, updated_shapes, anchor);
    app.editing.selection.control_point_id = a;
    app.editing.selection.handle_id        = 1;

    if (spline.input.control_points.size() > 1) {
      add_curve(app, spline.cache, spline.input);
    }
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

      auto handle_id                     = anchor.handle_ids[k];
      scene.instances[handle_id].frame.o = positions.back();

      auto& shape = scene.shapes[shape_id];
      shape = polyline_to_cylinders(positions, 16, app.line_thickness * 0.6);
      shape.normals = compute_normals(shape);
      updated_shapes += shape_id;
    }
    scene.instances[anchor.anchor_id].frame.o = eval_position(
        app.mesh, input.control_points[point_id].point);
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
