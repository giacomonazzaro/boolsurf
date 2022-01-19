#pragma once
#include "spline_mesh.h"

struct anchor_point {
  mesh_point point      = {};
  mesh_point handles[2] = {{}, {}};
};

struct Spline_Input {
  vector<anchor_point> control_points = {};
  vector<bool>         is_smooth      = {};
  bool                 is_closed      = true;

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
      return (int)control_points.size();
    else
      return (int)control_points.size() - 1;
  }
};

inline vec2i curves_adjacent_to_point(const Spline_Input& input, int point_id) {
  auto result = vec2i{-1, -1};
  auto prev   = point_id - 1;
  if (prev < 0) {
    if (input.is_closed)
      prev = (int)input.control_points.size() - 1;
    else
      prev = -1;
  }
  result = {prev, point_id};
  return result;
}

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
  Spline_Input& input;
  Spline_Cache& cache;
};

struct Const_Spline_View {
  const Spline_Input& input;
  const Spline_Cache& cache;
};

struct Splinesurf {
  vector<Spline_Input> spline_input = {};
  vector<Spline_Cache> spline_cache = {};

  Spline_View get_spline_view(int id) {
    return Spline_View{spline_input[id], spline_cache[id]};
  }
  const Const_Spline_View get_spline_view(int id) const {
    return Const_Spline_View{spline_input[id], spline_cache[id]};
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
  splinesurf.spline_cache.push_back({});
  return id;
}

inline vec2f tangent_path_direction(
    const spline_mesh& mesh, const geodesic_path& path) {
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

template <typename Delete_Shape>
inline void delete_curve(
    Spline_Cache& cache, int curve_id, Delete_Shape& delete_shape) {
  delete_shape(cache.curves[curve_id].shape_id);
  cache.curves.erase(cache.curves.begin() + curve_id);
}

template <typename Add_Point_Shape, typename Add_Path_Shape>
inline int add_anchor_point(Spline_View& spline, const anchor_point& anchor,
    Add_Point_Shape& add_point_shape, Add_Path_Shape& add_path_shape) {
  // Add point to input.
  auto point_id = (int)spline.input.control_points.size();
  spline.input.control_points.push_back(anchor);
  spline.input.is_smooth.push_back(true);

  // Add shapes for point, handles and tangents to cache.
  auto& cache                = spline.cache.points.emplace_back();
  cache.anchor_id            = add_point_shape();
  cache.tangents[0].shape_id = add_point_shape();
  cache.tangents[1].shape_id = add_point_shape();
  for (int k = 0; k < 2; k++) cache.handle_ids[k] = add_path_shape();

  // Add curve
  if (point_id > 0) add_curve(spline.cache, add_path_shape);
  if (spline.cache.curves.size() == 1) add_curve(spline.cache, add_path_shape);

  // Trigger update of this point.
  spline.cache.points_to_update.insert(point_id);

  return point_id;
}

// TODO(giacomo): Very similar too add_anchor_point. Merge?
template <typename Add_Shape>
inline int insert_anchor_point(Spline_View& spline, int point_id,
    const anchor_point& anchor, Add_Shape& add_shape) {
  // Add point to input.
  insert(spline.input.control_points, point_id, anchor);
  insert(spline.input.is_smooth, point_id, true);

  // Add shapes for point, handles and tangents to cache.
  insert(spline.cache.points, point_id, {});
  auto& cache                = spline.cache.points[point_id];
  cache.anchor_id            = add_shape();
  cache.tangents[0].shape_id = add_shape();
  cache.tangents[1].shape_id = add_shape();
  for (int k = 0; k < 2; k++) cache.handle_ids[k] = add_shape();

  // Add curve
  if (point_id > 0) add_curve(spline.cache, add_shape, point_id);

  // Trigger update of this point.
  spline.cache.points_to_update.insert(point_id);
  return point_id;
}

template <typename Add_Point_Shape, typename Add_Path_Shape>
inline int add_anchor_point(Spline_View& spline, const mesh_point& point,
    Add_Point_Shape& add_point_shape, Add_Path_Shape& add_path_shape) {
  // Create anchor point with zero-length tangents.
  auto anchor = anchor_point{point, {point, point}};
  return add_anchor_point(spline, anchor, add_point_shape, add_path_shape);
}

template <typename Delete_Shape>
inline void delete_anchor_point(
    Spline_View& spline, int point_id, Delete_Shape& delete_shape) {
  // Add shapes for point, handles and tangents to cache.
  auto& cache = spline.cache.points[point_id];
  delete_shape(cache.anchor_id);
  delete_shape(cache.tangents[0].shape_id);
  delete_shape(cache.tangents[1].shape_id);
  for (int k = 0; k < 2; k++) delete_shape(cache.handle_ids[k]);

  delete_curve(spline.cache, point_id, delete_shape);

  spline.input.control_points.erase(
      spline.input.control_points.begin() + point_id);
  spline.input.is_smooth.erase(spline.input.is_smooth.begin() + point_id);
}

template <typename Delete_Shape>
inline void delete_spline(
    Splinesurf& splinesurf, int spline_id, Delete_Shape& delete_shape) {
  auto spline = splinesurf.get_spline_view(spline_id);
  for (int i = (int)spline.input.control_points.size() - 1; i >= 0; i--) {
    delete_anchor_point(spline, i, delete_shape);
  }
  splinesurf.spline_input.erase(splinesurf.spline_input.begin() + spline_id);
  splinesurf.spline_cache.erase(splinesurf.spline_cache.begin() + spline_id);
}

inline void move_selected_point(Splinesurf& splinesurf,
    const Editing::Selection& selection, const spline_mesh& mesh,
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
