#pragma once
#include <boolsurf/boolsurf.h>
#include <yocto/yocto_mesh.h>

using namespace yocto;

using Anchor_Point = anchor_point;

struct Spline_Input {
  vector<Anchor_Point> control_points   = {};
  vector<bool>         is_smooth        = {};
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
    if (control_points.size() <= 1) return 0;
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

struct Spline_View {
  Spline_Input&  input;
  Spline_Output& output;
  Spline_Cache&  cache;
};

struct Const_Spline_View {
  const Spline_Input&  input;
  const Spline_Output& output;
  const Spline_Cache&  cache;
};

struct Splinesurf {
  vector<Spline_Input>  spline_input  = {};
  vector<Spline_Output> spline_output = {};
  vector<Spline_Cache>  spline_cache  = {};

  Spline_View get_spline_view(int id) {
    return Spline_View{spline_input[id], spline_output[id], spline_cache[id]};
  }
    const Const_Spline_View get_spline_view(int id) const {
      return Const_Spline_View{spline_input[id], spline_output[id], spline_cache[id]};
    }
  inline int num_splines() const { return (int)spline_input.size(); }
};

struct Editing {
  struct Selection {
    int spline_id        = -1;
    int control_point_id = -1;
    int handle_id        = -1;
  };
  Selection  selection             = {};
  mesh_point clicked_point         = {};
  bool       holding_control_point = false;
  bool       creating_new_point    = false;
};

inline int add_spline(Splinesurf& splinesurf) {
  auto id = (int)splinesurf.spline_input.size();
  splinesurf.spline_input.push_back({});
  splinesurf.spline_output.push_back({});
  splinesurf.spline_cache.push_back({});
  return id;
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

template <typename Add_Shape>
inline int add_curve(
    Spline_Cache& cache, Add_Shape& add_shape, int _curve_id = -1) {
  if (_curve_id != -1) {
    auto curve_id = _curve_id;
    insert(cache.curves, curve_id, {});
    auto& curve    = cache.curves[curve_id];
    curve.shape_id = add_shape();
    cache.curves_to_update.insert((int)curve_id);
    return curve_id;
  }
  auto  curve_id = (int)cache.curves.size();
  auto& curve    = cache.curves.emplace_back();
  curve.shape_id = add_shape();
  cache.curves_to_update.insert((int)curve_id);
  return curve_id;
}

template <typename Add_Shape>
inline int add_anchor_point(
    Spline_View& spline, const anchor_point& anchor, Add_Shape& add_shape) {
  // Add point to input.
  auto point_id = (int)spline.input.control_points.size();
  spline.input.control_points.push_back(anchor);
  spline.input.is_smooth.push_back(true);

  // Add shapes for point, handles and tangents to cache.
  auto& cache                = spline.cache.points.emplace_back();
  cache.anchor_id            = add_shape();
  cache.tangents[0].shape_id = add_shape();
  cache.tangents[1].shape_id = add_shape();
  for (int k = 0; k < 2; k++) cache.handle_ids[k] = add_shape();

  // Add curve
  if (point_id > 0) add_curve(spline.cache, add_shape);

  // Trigger update of this point.
  spline.cache.points_to_update.insert(point_id);
  return point_id;
}

template <typename Add_Shape>
inline int insert_anchor_point(Spline_View& spline, const anchor_point& anchor,
    int point_id, const bool_mesh& mesh, Add_Shape& add_shape) {
  // Add point to input.
  // auto point_id = (int)spline.input.control_points.size();
  insert(spline.input.control_points, point_id, anchor);
  insert(spline.input.is_smooth, point_id, true);

  // Add shapes for point, handles and tangents to cache.
  insert(spline.cache.points, point_id, {});
  auto& cache                = spline.cache.points[point_id];
  cache.anchor_id            = add_shape();
  cache.tangents[0].shape_id = add_shape();
  cache.tangents[1].shape_id = add_shape();
  for (int k = 0; k < 2; k++) cache.handle_ids[k] = add_shape();
  for (int k = 0; k < 2; k++) {
    cache.tangents[k].path = shortest_path(
        mesh, anchor.point, anchor.handles[k]);
  }

  // Add curve
  if (point_id > 0) add_curve(spline.cache, add_shape, point_id);

  // Trigger update of this point.
  spline.cache.points_to_update.insert(point_id);
  return point_id;
}

template <typename Add_Shape>
inline int add_anchor_point(
    Spline_View& spline, const mesh_point& point, Add_Shape& add_shape) {
  // Create anchor point with zero-length tangents.
  auto anchor = Anchor_Point{point, {point, point}};
  return add_anchor_point(spline, anchor, add_shape);
}

inline void move_selected_point(Splinesurf& splinesurf,
    const Editing::Selection& selection, const bool_mesh& mesh,
    const mesh_point& point, bool symmetric_length) {
  assert(selection.spline_id != -1);
  assert(selection.control_point_id != -1);
  auto  spline    = splinesurf.get_spline_view(selection.spline_id);
  auto& anchor    = spline.input.control_points[selection.control_point_id];
  auto  is_smooth = spline.input.is_smooth[selection.control_point_id];

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
    auto offset_len = path_length(offset, mesh.triangles, mesh.positions);
    for (int k = 0; k < 2; k++) {
      auto& tangent = point_cache.tangents[k].path;
      auto  rot     = parallel_transport_rotation(
          mesh.triangles, mesh.positions, mesh.adjacencies, tangent);

      auto dir          = rot * offset_dir;
      auto p            = straightest_path(mesh, tangent.end, dir, offset_len);
      tangent           = shortest_path(mesh, point, p.end);
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

    if (is_smooth) {
      auto dir = tangent_path_direction(mesh, point_cache.tangents[k].path);
      auto kk  = symmetric_length ? k : 1 - k;
      auto len = path_length(
          point_cache.tangents[kk].path, mesh.triangles, mesh.positions);
      point_cache.tangents[1 - k].path = straightest_path(
          mesh, anchor.point, -dir, len);
      anchor.handles[1 - k] = point_cache.tangents[1 - k].path.end;
    }
  }

  spline.cache.points_to_update.insert(selection.control_point_id);
}

template <typename Add_Shape>
inline int close_spline(Spline_View spline, Add_Shape& add_shape) {
  spline.input.is_closed = true;
  return add_curve(spline.cache, add_shape);
}
