#pragma once
#include "../yocto/yocto_mesh.h"  // TODO(giacomo): Solve include situation.
using namespace yocto;

struct spline_mesh : shape_data {
  vector<vec3i>        adjacencies = {};
  dual_geodesic_solver dual_solver = {};
};

inline geodesic_path shortest_path(
    const spline_mesh& mesh, const mesh_point& start, const mesh_point& end) {
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

inline mesh_point eval_path(
    const spline_mesh& mesh, const geodesic_path& path, float t) {
  return eval_path_point(
      path, mesh.triangles, mesh.positions, mesh.adjacencies, t);
}

inline geodesic_path straightest_path(const spline_mesh& mesh,
    const mesh_point& start, const vec2f& direction, float length) {
  return straightest_path(mesh.triangles, mesh.positions, mesh.adjacencies,
      start, direction, length);
}

inline geodesic_path straightest_path(
    const spline_mesh& mesh, const mesh_point& start, const vec2f& coord) {
  auto len = length(coord);
  return straightest_path(mesh.triangles, mesh.positions, mesh.adjacencies,
      start, coord / len, len);
}

inline vec3f eval_position(const spline_mesh& mesh, const mesh_point& point) {
  return eval_position(mesh.triangles, mesh.positions, point);
}

inline vec3f eval_normal(const spline_mesh& mesh, const mesh_point& point) {
  return eval_normal(mesh.triangles, mesh.normals, point);
}

inline vec3f eval_normal(const spline_mesh& mesh, int face) {
  auto [x, y, z] = mesh.triangles[face];
  return triangle_normal(
      mesh.positions[x], mesh.positions[y], mesh.positions[z]);
}

inline vector<mesh_point> bezier_spline(const spline_mesh& mesh,
    const std::array<mesh_point, 4>& control_points, int subdivisions) {
  return compute_bezier_path(mesh.dual_solver, mesh.triangles, mesh.positions,
      mesh.adjacencies, control_points, subdivisions);
}
