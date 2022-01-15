#include "boolsurf.h"

#include <cassert>
#include <deque>

#include "ext/CDT/CDT/include/CDT.h"

constexpr auto adjacent_to_nothing = -2;

#define DEBUG_DATA 0
#if DEBUG_DATA
#define add_debug_triangle(face, triangle) debug_triangles()[face] = triangle
#else
#define add_debug_triangle(face, triangle) ;
#endif
#if DEBUG_DATA
#define add_debug_edge(face, edge) debug_edges()[face] = edge
#else
#define add_debug_edge(face, edge) ;
#endif
#if DEBUG_DATA
#define add_debug_node(face, node) debug_nodes()[face] = node
#else
#define add_debug_node(face, node) ;
#endif
#if DEBUG_DATA
#define add_debug_index(face, index) debug_indices()[face] = index
#else
#define add_debug_index(face, index) ;
#endif

// get time in nanoseconds - useful only to compute difference of times
inline int64_t get_time_() {
  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

static int scope_timer_indent = 0;
scope_timer::scope_timer(const string& msg) {
  if (scope_timer_indent == 0) printf("       \n");
  printf("[timer]");
  printf(" %.*s", scope_timer_indent, "|||||||||||||||||||||||||");
  printf("%s started\n", msg.c_str());
  scope_timer_indent += 1;
  message    = msg;
  start_time = get_time_();
}

// Format duration string from nanoseconds
static string format_duration(int64_t duration) {
  auto elapsed = duration / 1000000;  // milliseconds
  auto hours   = (int)(elapsed / 3600000);
  elapsed %= 3600000;
  auto mins = (int)(elapsed / 60000);
  elapsed %= 60000;
  auto secs  = (int)(elapsed / 1000);
  auto msecs = (int)(elapsed % 1000);
  char buffer[256];
  snprintf(
      buffer, sizeof(buffer), "%02d:%02d:%02d.%03d", hours, mins, secs, msecs);
  return buffer;
}

scope_timer::~scope_timer() {
  scope_timer_indent -= 1;
  if (start_time < 0) return;
  auto elapsed = get_time_() - start_time;
  printf("[timer]");
  printf(" %.*s", scope_timer_indent, "|||||||||||||||||||||||||");
  // printf("%d", scope_timer_indent);
  printf("%s %s\n", message.c_str(), format_duration(elapsed).c_str());
}

void init_mesh(bool_mesh& mesh) {
  // Make triangle mesh inside [-1, 1]^ cube.
  if (mesh.quads.size()) {
    mesh.triangles = quads_to_triangles(mesh.quads);
    mesh.quads.clear();
  }
  fit_into_cube(mesh.positions);

  // Adjacency data
  mesh.adjacencies = face_adjacencies(mesh.triangles);
  mesh.dual_solver = make_dual_geodesic_solver(
      mesh.triangles, mesh.positions, mesh.adjacencies);

  // Additional data for computing cells
  mesh.is_edge_on_boundary = vector<bool>(3 * mesh.triangles.size(), false);
  mesh.face_tags           = vector<int>(mesh.triangles.size(), -1);
  mesh.polygon_borders.clear();

  // Backup information for fast reset
  mesh.old_adjacencies = mesh.adjacencies;
  mesh.num_triangles   = (int)mesh.triangles.size();
  mesh.num_positions   = (int)mesh.positions.size();

  // @MUTLITHREAD_DANGER
  mesh.triangles.reserve(mesh.triangles.size() * 2);
  mesh.adjacencies.reserve(mesh.adjacencies.size() * 2);
}

void reset_mesh(bool_mesh& mesh) {
  mesh.triangles.resize(mesh.num_triangles);
  mesh.positions.resize(mesh.num_positions);
  mesh.triangulated_faces.clear();
  mesh.polygon_borders.clear();
  mesh.adjacencies = mesh.old_adjacencies;

  // mesh.is_edge_on_boundary    = vector<bool>(3 * mesh.triangles.size(),
  // false); mesh.face_tags       = vector<int>(mesh.triangles.size(), -1);

  // mesh.dual_solver.graph.resize(mesh.num_triangles);
  // mesh.triangulated_faces.clear();

  // for (auto& [face, _] : mesh.triangulated_faces) {
  //   for (int k = 0; k < 3; k++) {
  //     auto neighbor = mesh.adjacencies[face][k];
  //     if (neighbor == -1) continue;
  //     auto kk = find_adjacent_triangle(
  //         mesh.triangles[neighbor], mesh.triangles[face]);

  //     // Fix adjacencies.
  //     mesh.adjacencies[neighbor][kk] = face;
  //   }
  // }

  // assert(mesh.old_adjacencies == mesh.adjacencies);
  // mesh.adjacencies.resize(mesh.num_triangles);
}

geodesic_path compute_geodesic_path(
    const bool_mesh& mesh, const mesh_point& start, const mesh_point& end) {
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

mesh_point eval_geodesic_path(
    const bool_mesh& mesh, const geodesic_path& path, float t) {
  return eval_path_point(
      path, mesh.triangles, mesh.positions, mesh.adjacencies, t);
}

vector<mesh_segment> mesh_segments(const vector<vec3i>& triangles,
    const vector<vec3f>& positions, const geodesic_path& path) {
  auto result = vector<mesh_segment>{};
  result.reserve(path.strip.size());

  for (int i = 0; i < path.strip.size(); ++i) {
    vec2f start_uv;
    if (i == 0) {
      start_uv = path.start.uv;
    } else {
      vec2f uvw[3] = {{0, 0}, {1, 0}, {0, 1}};
      auto  k      = find_adjacent_triangle(
          triangles[path.strip[i]], triangles[path.strip[i - 1]]);
      auto a   = uvw[mod3(k)];
      auto b   = uvw[mod3(k + 1)];
      start_uv = lerp(a, b, 1 - path.lerps[i - 1]);
    }

    vec2f end_uv;
    if (i == path.strip.size() - 1) {
      end_uv = path.end.uv;
    } else {
      vec2f uvw[3] = {{0, 0}, {1, 0}, {0, 1}};
      auto  k      = find_adjacent_triangle(
          triangles[path.strip[i]], triangles[path.strip[i + 1]]);
      auto a = uvw[k];
      auto b = uvw[mod3(k + 1)];
      end_uv = lerp(a, b, path.lerps[i]);
    }
    if (start_uv == end_uv) continue;

    result.push_back({start_uv, end_uv, 0, 0, path.strip[i]});
  }
  return result;
}

vector<mesh_segment> make_curve_segments(
    const bool_mesh& mesh, const anchor_point& start, const anchor_point& end) {
  auto curve = vector<mesh_segment>{};

  auto get_segments = [&](const mesh_point& start, const mesh_point& end) {
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

  if (start.point == start.handles[1] && end.handles[0] == end.point) {
    curve = get_segments(start.point, end.point);
  } else {
    auto control_points = array<mesh_point, 4>{
        start.point, start.handles[1], end.handles[0], end.point};
    auto points = compute_bezier_path(mesh.dual_solver, mesh.triangles,
        mesh.positions, mesh.adjacencies, control_points, 4);
    for (int k = 0; k < points.size() - 1; k++) {
      auto segments = get_segments(points[k], points[k + 1]);
      auto min      = float(k) / points.size();
      auto max      = float(k + 1) / points.size();
      for (auto& s : segments) {
        s.t_start = s.t_start * (max - min) + min;
        s.t_end   = s.t_end * (max - min) + min;
      }
      curve += segments;
    }
  }
  return curve;
}

vector<vector<mesh_segment>> make_boundary_segments(
    const bool_mesh& mesh, const vector<anchor_point>& boundary) {
  auto result = vector<vector<mesh_segment>>{};
  for (int i = 0; i < boundary.size(); i++) {
    auto& start = boundary[i];
    auto  end   = boundary[(i + 1) % boundary.size()];

    auto& curve = result.emplace_back();
    curve       = make_curve_segments(mesh, start, end);
  }
  return result;
}

struct hashgrid_polyline {
  int           shape_id    = -1;
  int           boundary_id = 0;
  vector<vec2f> points      = {};
  vector<int>   vertices    = {};
  vector<int>   curve_ids   = {};
  vector<float> t           = {};

  bool is_closed = false;
};

inline int num_segments(const hashgrid_polyline& polyline) {
  if (polyline.is_closed) return (int)polyline.points.size();
  return (int)polyline.points.size() - 1;
}

inline pair<vec2f, vec2f> get_segment(
    const hashgrid_polyline& polyline, int i) {
  if (polyline.is_closed) {
    return {
        polyline.points[i], polyline.points[(i + 1) % polyline.points.size()]};
  } else {
    return {polyline.points[i], polyline.points[i + 1]};
  }
}

inline vec2i get_segment_vertices(const hashgrid_polyline& polyline, int i) {
  if (polyline.is_closed) {
    return {polyline.vertices[i],
        polyline.vertices[(i + 1) % polyline.vertices.size()]};
  } else {
    return {polyline.vertices[i], polyline.vertices[i + 1]};
  }
}

using mesh_hashgrid = hash_map<int, vector<hashgrid_polyline>>;
// struct mesh_hashgrid : hash_map<int, vector<hashgrid_polyline>> {};

inline int add_vertex(bool_mesh& mesh, mesh_hashgrid& hashgrid,
    const mesh_point& point, int polyline_id, int curve_id, float t,
    int vertex = -1) {
  float eps = 0.00001;

  auto update_polyline = [&](int v) {
    if (polyline_id == -1) return;  // @DONT_ADD_TO_POLYLINE. -1 is a signal
    auto& polyline = hashgrid[point.face][polyline_id];
    polyline.vertices.push_back(v);
    polyline.curve_ids.push_back(curve_id);
    polyline.t.push_back(t);
    polyline.points.push_back(point.uv);
  };

  {  // Maybe collapse with original mesh vertices.
    auto uv = point.uv;
    auto tr = mesh.triangles[point.face];
    if (uv.x < eps && uv.y < eps) {
      update_polyline(tr.x);
      return tr.x;
    }
    if (uv.x > 1 - eps && uv.y < eps) {
      update_polyline(tr.y);
      return tr.y;
    }
    if (uv.y > 1 - eps && uv.x < eps) {
      update_polyline(tr.z);
      return tr.z;
    }
  }

  {  // Maybe collapse with already added vertices.
    auto& polylines = hashgrid[point.face];
    for (auto& polyline : polylines) {
      for (int i = 0; i < polyline.vertices.size(); i++) {
        if (length(point.uv - polyline.points[i]) < eps) {
          update_polyline(polyline.vertices[i]);
          return polyline.vertices[i];
        }
      }
    }
  }

  // No collapse. Add new vertex to mesh.
  if (vertex == -1) {
    vertex   = (int)mesh.positions.size();
    auto pos = eval_position(mesh.triangles, mesh.positions, point);
    mesh.positions.push_back(pos);
  }

  update_polyline(vertex);
  return vertex;
}

static mesh_hashgrid compute_hashgrid(bool_mesh&        mesh,
    const vector<vector<vector<vector<mesh_segment>>>>& shapes) {
  // }, hash_map<int, int>& control_points) {
  _PROFILE();
  // La hashgrid associa ad ogni faccia una lista di polilinee.
  // Ogni polilinea è definita da una sequenza punti in coordinate
  // baricentriche, ognuno di essi assiociato al corrispondente vertice della
  // mesh.
  auto hashgrid = mesh_hashgrid{};

  for (auto shape_id = 0; shape_id < shapes.size(); shape_id++) {
    //    auto& polygons = shapes[shape_id].polygons;
    for (auto boundary_id = 0; boundary_id < shapes[shape_id].size();
         boundary_id++) {
      //      auto& polygon = polygons[boundary_id];
      if (shapes[shape_id][boundary_id].empty()) continue;
      if (shapes[shape_id][boundary_id][0].empty()) continue;
      auto& boundary = shapes[shape_id][boundary_id];
      // La polilinea della prima faccia del poligono viene processata alla fine
      // (perché si trova tra il primo e l'ultimo edge)
      int  first_face   = boundary[0][0].face;
      int  first_vertex = -1;
      auto indices      = vec2i{-1, -1};  // edge_id, segment_id

      int last_face   = -1;
      int last_vertex = -1;

      for (auto curve_id = 0; curve_id < boundary.size(); curve_id++) {
        auto& curve = boundary[curve_id];

        for (auto segment_id = 0; segment_id < curve.size(); segment_id++) {
          auto& segment = curve[segment_id];

          // Iniziamo a riempire l'hashgrid a partire da quando troviamo una
          // faccia diversa da quella iniziale del poligono (il primo tratto
          // verrà aggiunto a posteriori per evitare inconsistenza)
          if (segment.face == first_face && indices == vec2i{-1, -1}) continue;
          if (indices == vec2i{-1, -1}) indices = {curve_id, segment_id};

          auto& entry = hashgrid[segment.face];

          // Se la faccia del segmento che stiamo processando è diversa
          // dall'ultima salvata allora creiamo una nuova polilinea, altrimenti
          // accodiamo le nuove informazioni.
          if (segment.face != last_face) {
            auto  polyline_id    = (int)entry.size();
            auto& polyline       = entry.emplace_back();
            polyline.shape_id    = shape_id;
            polyline.boundary_id = boundary_id;

            last_vertex = add_vertex(mesh, hashgrid,
                {segment.face, segment.start}, polyline_id, curve_id,
                segment.t_start, last_vertex);
            if (first_vertex == -1) first_vertex = last_vertex;

            last_vertex = add_vertex(mesh, hashgrid,
                {segment.face, segment.end}, polyline_id, curve_id,
                segment.t_end);

          } else {
            auto  polyline_id = (int)entry.size() - 1;
            auto& polyline    = entry.back();
            assert(segment.end != polyline.points.back());

            last_vertex = add_vertex(mesh, hashgrid,
                {segment.face, segment.end}, polyline_id, curve_id,
                segment.t_end);
          }

          last_face = segment.face;
        }
      }

      if (indices == vec2i{-1, -1}) {
        auto& entry          = hashgrid[first_face];
        auto  polyline_id    = (int)entry.size();
        auto& polyline       = entry.emplace_back();
        polyline.shape_id    = shape_id;
        polyline.boundary_id = boundary_id;
        polyline.is_closed   = true;

        for (auto curve_id = 0; curve_id < boundary.size(); curve_id++) {
          auto& curve = boundary[curve_id];
          for (int segment_id = 0; segment_id < curve.size(); segment_id++) {
            auto& segment = curve[segment_id];

            last_vertex = add_vertex(mesh, hashgrid,
                {segment.face, segment.start}, polyline_id, curve_id,
                segment.t_start);
          }
        }
      };

      // Ripetiamo parte del ciclo (fino a indices) perché il primo tratto di
      // polilinea non è stato inserito nell'hashgrid
      auto vertex = -1;
      for (auto curve_id = 0; curve_id <= indices.x; curve_id++) {
        auto end_idx = (curve_id < indices.x) ? boundary[curve_id].size()
                                              : indices.y;
        for (auto s = 0; s < end_idx; s++) {
          auto& segment     = boundary[curve_id][s];
          auto& entry       = hashgrid[segment.face];
          auto  polyline_id = (int)entry.size() - 1;

          if (curve_id == indices.x && s == indices.y - 1)
            vertex = first_vertex;

          if (segment.face != last_face) {
            printf("\n\n  error in %s, %s, line %d\n", __FILE__, __FUNCTION__,
                __LINE__);
            printf("  segment.face != last_face, %d != %d]\n\n\n", segment.face,
                last_face);
          }
          last_vertex = add_vertex(mesh, hashgrid, {segment.face, segment.end},
              polyline_id, curve_id, segment.t_end, vertex);
        }
      }
    }
  }
  return hashgrid;
}

//[[maybe_unused]] static hash_map<int, int> compute_control_points(
//    vector<mesh_polygon>&             polygons,
//    const vector<vector<vector<int>>> vertices) {
//  auto control_points = hash_map<int, int>();
//  for (auto p = 0; p < vertices.size(); p++) {
//    for (auto e = 0; e < vertices[p].size(); e++) {
//      auto control_point_idx            = vertices[p][e][0];
//      auto mesh_point_idx               = polygons[p].points[e];
//      control_points[control_point_idx] = mesh_point_idx;
//    }
//  }
//  return control_points;
//}

void save_tree_png(const bool_state& state, string filename,
    const string& extra, bool color_shapes);

static vector<bool_cell> make_mesh_cells(vector<int>& cell_tags,
    const vector<vec3i>& adjacencies, const vector<bool>& is_edge_on_boundary) {
  auto result = vector<bool_cell>{};
  cell_tags   = vector<int>(adjacencies.size(), -1);

  // consume task stack
  // Iniziamo dall'ultimo nodo che sicuramente corrisponde a un triangolo
  auto starts = vector<int>{(int)adjacencies.size() - 1};

  while (starts.size()) {
    auto start = starts.back();
    starts.pop_back();
    if (cell_tags[start] >= 0) continue;

    // pop element from task stack
    auto first_face = start;

    auto  cell_id    = (int)result.size();
    auto& cell       = result.emplace_back();
    auto  face_stack = vector<int>{first_face};

    while (!face_stack.empty()) {
      auto face = face_stack.back();
      face_stack.pop_back();

      if (cell_tags[face] >= 0) continue;
      cell_tags[face] = cell_id;

      for (int k = 0; k < 3; k++) {
        auto neighbor = adjacencies[face][k];
        if (neighbor < 0) continue;

        auto neighbor_cell = cell_tags[neighbor];
        if (neighbor_cell >= 0) continue;
        if (is_edge_on_boundary[3 * face + k]) {
          starts.push_back(neighbor);
        } else {
          face_stack.push_back(neighbor);
        }
      }
    }
  }

  return result;
}

vector<bool_cell> make_cell_graph(bool_mesh& mesh) {
  _PROFILE();
  auto cells = make_mesh_cells(
      mesh.face_tags, mesh.adjacencies, mesh.is_edge_on_boundary);

  {
    _PROFILE_SCOPE("tag_cell_edges");
    for (auto& segment : mesh.polygon_borders) {
      if (segment.face_in < 0 || segment.face_out < 0) continue;
      auto a = mesh.face_tags[segment.face_in];
      auto b = mesh.face_tags[segment.face_out];
      cells[a].adjacency.insert({b, segment.shape_id, 0});
      cells[b].adjacency.insert({a, segment.shape_id, 1});
    }
  }

  return cells;
}

static vector<int> find_roots(const vector<bool_cell>& cells) {
  // Trova le celle non hanno archi entranti con segno di poligono positivo.
  auto adjacency = vector<int>(cells.size(), 0);
  for (auto& cell : cells) {
    for (auto& [neighbor, shape_id, entering] : cell.adjacency) {
      if (entering > 0) adjacency[neighbor] += 1;
    }
  }

  auto result = vector<int>{};
  for (int i = 0; i < adjacency.size(); i++) {
    if (adjacency[i] == 0) result.push_back(i);
  }
  return result;
}

static vector<bool_cell> compute_shape_macrograph(
    const vector<bool_cell>& cells, int shape_id) {
  auto components      = vector<vector<int>>();
  auto shape_component = hash_map<int, int>();
  auto visited         = vector<bool>(cells.size(), false);

  for (auto c = 0; c < cells.size(); c++) {
    if (visited[c]) continue;

    auto  component_id = (int)components.size();
    auto& component    = components.emplace_back();

    auto stack = vector<int>();
    stack.push_back(c);

    while (!stack.empty()) {
      auto cell_id = stack.back();
      stack.pop_back();

      if (visited[cell_id]) continue;
      visited[cell_id] = true;
      component.push_back(cell_id);
      shape_component[cell_id] = component_id;

      auto& cell = cells[cell_id];
      for (auto& [neighbor, n_shape_id, entering] : cell.adjacency) {
        if (n_shape_id == shape_id) continue;
        if (visited[neighbor]) continue;
        stack.push_back(neighbor);
      }
    }
  }

  auto macrograph = vector<bool_cell>((int)components.size());
  for (auto c = 0; c < (int)components.size(); c++) {
    for (auto id : components[c]) {
      for (auto& [neighbor, n_shape_id, entering] : cells[id].adjacency) {
        if (n_shape_id != shape_id) continue;

        auto neighbor_component = shape_component.at(neighbor);
        macrograph[c].adjacency.insert(
            {neighbor_component, n_shape_id});  // Now missing sign.
      }
    }
  }

  return macrograph;
}

static void compute_cycles(const vector<bool_cell>& cells, int node,
    vec2i parent, vector<int> visited, vector<vec2i> parents,
    vector<vector<vec2i>>& cycles) {
  // Se il nodo il considerazione è già stato completamente visitato allora
  // terminiamo la visita
  if (visited[node] == 2) return;

  // Se il nodo in considerazione non è stato completamente visitato e lo
  // stiamo rivisitando ora allora abbiamo trovato un ciclo
  if (visited[node] == 1) {
    auto  cycle   = vector<vec2i>();
    auto& current = parent;
    cycle.push_back(current);

    // Risalgo l'albero della visita fino a che non trovo lo stesso nodo e
    // salvo il ciclo individuato
    while (current.x != node) {
      auto prev = parents[current.x];

      // (marzia) check: è vero che ho un ciclo corretto se il verso
      // (entrante/uscente) è lo stesso per tutti gli archi?
      // if (sign(prev.y) != sign(current.y)) return;
      current = prev;
      cycle.push_back(current);
    }

    cycles.push_back(cycle);
    return;
  }

  // Settiamo il padre del nodo attuale e iniziamo ad esplorare i suoi vicini
  parents[node] = parent;
  visited[node] = 1;

  for (auto& [neighbor, shape_id, entering] : cells[node].adjacency) {
    // Se stiamo percorrendo lo stesso arco ma al contrario allora continuo,
    // altrimenti esploriamo il vicino
    // if (shape_id > 0) continue;
    // if (neighbor == parent.x && shape_id == -parent.y) continue;
    compute_cycles(cells, neighbor, {node, shape_id}, visited, parents, cycles);
  }

  // Settiamo il nodo attuale come completamente visitato
  visited[node] = 2;
}

inline vector<vector<vec2i>> compute_graph_cycles(
    const vector<bool_cell>& cells) {
  // _PROFILE();
  auto visited        = vector<int>(cells.size(), 0);
  auto parents        = vector<vec2i>(cells.size(), {0, 0});
  auto cycles         = vector<vector<vec2i>>();
  auto start_node     = 0;
  auto invalid_parent = vec2i{-1, -1};
  compute_cycles(cells, start_node, invalid_parent, visited, parents, cycles);
  return cycles;
}

hash_set<int> compute_invalid_shapes(
    const vector<bool_cell>& cells, int num_shapes) {
  _PROFILE();
  auto invalid_shapes = hash_set<int>();
  for (auto s = 1; s < num_shapes; s++) {
    auto shape_graph = compute_shape_macrograph(cells, s);
    auto cycles      = compute_graph_cycles(shape_graph);

    for (auto cycle : cycles)
      if (cycle.size() % 2 == 1) invalid_shapes.insert(s);
  }
  return invalid_shapes;
}

inline vector<vector<int>> compute_components(
    const bool_state& state, const bool_shape& bool_shape) {
  // Calcoliamo le componenti tra le celle presenti in una bool_shape
  // (per calcolarne i bordi in maniera più semplice)
  auto cells   = vector<int>(bool_shape.cells.begin(), bool_shape.cells.end());
  auto visited = hash_map<int, bool>();
  for (auto cell : cells) visited[cell] = false;

  auto components = vector<vector<int>>();

  for (auto cell : cells) {
    if (visited[cell]) continue;

    auto& component = components.emplace_back();

    auto stack = vector<int>();
    stack.push_back(cell);

    while (!stack.empty()) {
      auto cell_idx = stack.back();
      stack.pop_back();

      if (visited[cell_idx]) continue;
      visited[cell_idx] = true;
      component.push_back(cell_idx);

      auto& cell = state.cells[cell_idx];
      for (auto& [neighbor, shape_id, entering] : cell.adjacency) {
        if (find_idx(cells, neighbor) == -1) continue;
        if (visited[neighbor]) continue;
        stack.push_back(neighbor);
      }
    }
  }
  return components;
}

static vector<vector<int>> propagate_cell_labels(bool_state& state) {
  _PROFILE();
  // Inizializziamo le label delle celle a 0.
  auto  num_shapes     = (int)state.shapes.size();
  auto& cells          = state.cells;
  auto& labels         = state.labels;
  auto& invalid_shapes = state.invalid_shapes;

  labels = vector<vector<int>>(
      cells.size(), vector<int>(num_shapes, null_label));

  // TODO: Initialization of visit (is it ok?)
  auto new_start = vector<int>();
  new_start.push_back((int)cells.size() - 1);
  for (auto cell_id : new_start) {
    auto& cell      = cells[cell_id];
    labels[cell_id] = vector<int>(num_shapes, 0);
    for (auto& [neighbor_id, shape_id, entering] : cell.adjacency) {
      if (entering > 0) continue;
      labels[cell_id][shape_id] = 1;
    }
  }

  auto queue   = deque<int>(new_start.begin(), new_start.end());
  auto visited = vector<bool>(cells.size(), false);
  for (auto& s : new_start) visited[s] = true;

  while (!queue.empty()) {
    // print("queue", queue);
    auto cell_id = queue.front();
    queue.pop_front();
    // static int c = 0;
    // save_tree_png(
    //     *global_state(), "data/tests/" + to_string(c) + ".png", "", false);
    // c += 1;

    auto& cell = cells[cell_id];
    for (auto& [neighbor_id, shape_id, entering] : cell.adjacency) {
      auto& neighbor_labels = labels[neighbor_id];
      auto  updated_labels  = labels[cell_id];
      updated_labels[shape_id] += entering ? 1 : -1;  // yocto::sign(shape_id);

      auto updated = false;
      for (int s = 0; s < neighbor_labels.size(); s++) {
        if (neighbor_labels[s] == null_label) {
          neighbor_labels[s] = updated_labels[s];
          updated            = true;
        }
      }

      for (int s = 0; s < neighbor_labels.size(); s++) {
        if (updated_labels[s] % 2 != neighbor_labels[s] % 2) {
          invalid_shapes.insert(s);
        }
      }

      if (updated) {
        if (!contains(queue, neighbor_id)) {
          queue.push_back(neighbor_id);
        }
      }

      visited[neighbor_id] = true;
    }
  }
  return labels;
}

static void add_polygon_intersection_points(bool_state& state,
    hash_map<int, vector<hashgrid_polyline>>& hashgrid, bool_mesh& mesh) {
  _PROFILE();
  // Calcoliamo sia le intersezioni che le self-intersections, aggiungendo i
  // vertici nuovi alla mesh.

  for (auto& [face, polylines] : hashgrid) {
    // Check for polyline self interesctions
    for (auto p0 = 0; p0 < polylines.size(); p0++) {
      auto& poly = polylines[p0];

      for (int s0 = 0; s0 < num_segments(poly) - 1; s0++) {
        auto [start0, end0] = get_segment(poly, s0);
        int num_added       = 0;  // number of points added to poly

        for (int s1 = s0 + 1; s1 < num_segments(poly); s1++) {
          // Skip adjacent segments.
          if (poly.is_closed) {
            if (yocto::abs(s0 - s1) % num_segments(poly) <= 1) continue;
          } else {
            if (yocto::abs(s0 - s1) <= 1) continue;
          }

          auto [start1, end1] = get_segment(poly, s1);

          auto l = intersect_segments(start0, end0, start1, end1);
          if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
            continue;
          }

          auto uv    = lerp(start1, end1, l.y);
          auto point = mesh_point{face, uv};
          // Don't add vertex to polyline. See @DONT_ADD_TO_POLYLINE.
          auto vertex = add_vertex(mesh, hashgrid, point, -1, -1, -1);

          auto t0 = lerp(poly.t[s0], poly.t[s0 + 1], l.x);
          auto t1 = lerp(poly.t[s1], poly.t[s1 + 1], l.y);
          add_intersection(state.intersections, poly.shape_id, poly.boundary_id,
              poly.curve_ids[s0], t0, poly.shape_id, poly.boundary_id,
              poly.curve_ids[s1], t1);

          insert(poly.points, s0 + 1, uv);
          insert(poly.vertices, s0 + 1, vertex);
          insert(poly.t, s0 + 1, t0);
          insert(poly.curve_ids, s0 + 1, poly.curve_ids[s0]);

          insert(poly.points, s1 + 2, uv);
          insert(poly.vertices, s1 + 2, vertex);
          insert(poly.t, s1 + 2, t1);
          insert(poly.curve_ids, s1 + 2, poly.curve_ids[s1]);
          num_added += 1;
          s1 += 2;
        }
        s0 += num_added;
      }
    }

    // Check for intersections between different polylines
    for (auto p0 = 0; p0 < polylines.size() - 1; p0++) {
      for (auto p1 = p0 + 1; p1 < polylines.size(); p1++) {
        auto& poly0 = polylines[p0];
        auto& poly1 = polylines[p1];
        for (int s0 = 0; s0 < num_segments(poly0); s0++) {
          auto [start0, end0] = get_segment(poly0, s0);
          int num_added       = 0;  // number of points added to poly0

          for (int s1 = 0; s1 < num_segments(poly1); s1++) {
            auto [start1, end1] = get_segment(poly1, s1);

            auto l = intersect_segments(start0, end0, start1, end1);
            if (l.x <= 0.0f || l.x >= 1.0f || l.y <= 0.0f || l.y >= 1.0f) {
              continue;
            }

            auto uv    = lerp(start1, end1, l.y);
            auto point = mesh_point{face, uv};
            // Don't add vertex to polyline. See @DONT_ADD_TO_POLYLINE.
            auto vertex = add_vertex(mesh, hashgrid, point, -1, -1, -1);

            // state.isecs_generators[vertex] = {poly0.shape_id,
            // poly1.shape_id};

            auto t0 = lerp(poly0.t[s0], poly0.t[s0 + 1], l.x);
            auto t1 = lerp(poly1.t[s1], poly1.t[s1 + 1], l.y);
            add_intersection(state.intersections, poly0.shape_id,
                poly0.boundary_id, poly0.curve_ids[s0], t0, poly1.shape_id,
                poly1.boundary_id, poly1.curve_ids[s1], t1);

            insert(poly0.points, s0 + 1, uv);
            insert(poly0.vertices, s0 + 1, vertex);
            insert(poly0.t, s0 + 1, t0);
            insert(poly0.curve_ids, s0 + 1, poly0.curve_ids[s0]);

            insert(poly1.points, s1 + 1, uv);
            insert(poly1.vertices, s1 + 1, vertex);
            insert(poly1.t, s1 + 1, t1);
            insert(poly1.curve_ids, s1 + 1, poly1.curve_ids[s1]);

            num_added += 1;
            s1 += 1;
          }
          s0 += num_added;
        }
      }
    }
  }
}

triangulation_info triangulation_constraints(const bool_mesh& mesh, int face,
    const vector<hashgrid_polyline>& polylines) {
  auto info    = triangulation_info{};
  info.face    = face;
  info.nodes   = vector<vec2f>{{0, 0}, {1, 0}, {0, 1}};
  info.indices = vector<int>(
      &mesh.triangles[face][0], &mesh.triangles[face][3]);

  for (auto& polyline : polylines) {
    // Per ogni segmento della polyline, aggiorniamo triangulation_info,
    // aggiungendo nodi, indici, e edge constraints.
    for (auto i = 0; i < num_segments(polyline); i++) {
      vec2f uvs[2];
      tie(uvs[0], uvs[1]) = get_segment(polyline, i);
      auto vertices       = get_segment_vertices(polyline, i);
      assert(vertices[0] < mesh.positions.size());
      assert(vertices[1] < mesh.positions.size());

      // TODO(giacomo): Set to -1 or 'invalid'.
      auto local_vertices = vec2i{-7, -8};
      for (int k = 0; k < 2; k++) {
        local_vertices[k] = find_idx(info.indices, vertices[k]);
        if (local_vertices[k] == -1) {
          info.indices.push_back(vertices[k]);
          info.nodes.push_back(uvs[k]);
          local_vertices[k] = (int)info.indices.size() - 1;
        }
      }

      // Aggiungiamo l'edge ai constraints della triangolazione.
      info.edges.push_back({local_vertices[0], local_vertices[1]});

      // Extra: Se i nodi sono su un edge del triangolo, allora li
      // salviamo nella edgemap.
      for (int j = 0; j < 2; j++) {
        auto [k, lerp] = get_edge_lerp_from_uv(uvs[j]);
        if (k != -1) {
          info.edgemap[k].push_back({local_vertices[j], lerp});
        }
      }
    }
  }
  return info;
}

void add_boundary_edge_constraints(
    array<vector<pair<int, float>>, 3>& edgemap, vector<vec2i>& edges) {
  // Aggiungiamo gli edge di vincolo per i lati del triangolo.
  for (int k = 0; k < 3; k++) {
    auto  edge        = get_triangle_edge_from_index(k);
    auto& edge_points = edgemap[k];

    // Se sul lato non ci sono punti aggiuntivi, allora lo aggiungiamo ai
    // vincoli cosi' come e'.
    if (edge_points.empty()) {
      edges.push_back(edge);
      continue;
    }

    // Ordiniamo i punti che giacciono sul lato per lerp crescente.
    if (edge_points.size() > 1) {
      sort(edge_points.begin(), edge_points.end(),
          [](auto& a, auto& b) { return a.second < b.second; });
    }

    // Creiamo primo e ultimo vincolo di questo lato.
    edges.push_back({edge.x, edge_points[0].first});
    edges.push_back({edge.y, edge_points.back().first});

    // Creiamo i rimanenti vincoli contenuti nel lato.
    for (auto i = 0; i < edge_points.size() - 1; i++) {
      edges.push_back({edge_points[i].first, edge_points[i + 1].first});
    }
  }
}

static pair<vector<vec3i>, vector<vec3i>> single_split_triangulation(
    const vector<vec2f>& nodes, const vec2i& edge) {
  // Calcoliamo la triangolazione con un singolo segmento all'interno del
  // triangolo.
  auto start_edge = get_edge_from_uv(nodes[edge.x]);
  auto end_edge   = get_edge_from_uv(nodes[edge.y]);

  auto triangles   = vector<vec3i>(3);
  auto adjacencies = vector<vec3i>(3);

  // image: libs/boolsurf/notes/sinlge-split-adjacency.jpg
  if (edge.x < 3) {
    // Se il segmento ha come inizio un punto in un lato e come fine il
    // vertice del triangolo opposto
    triangles[0] = {edge.x, end_edge.x, edge.y};
    triangles[1] = {edge.x, edge.y, end_edge.y};
    triangles.resize(2);

    adjacencies[0] = {adjacent_to_nothing, adjacent_to_nothing, 1};
    adjacencies[1] = {0, adjacent_to_nothing, adjacent_to_nothing};
    adjacencies.resize(2);

  } else if (edge.y < 3) {
    // Se il segmento ha come inizio un vertice di un triangolo e come fine un
    // punto punto nel lato opposto
    triangles[0] = {edge.y, start_edge.x, edge.x};
    triangles[1] = {edge.y, edge.x, start_edge.y};
    triangles.resize(2);

    adjacencies[0] = {adjacent_to_nothing, adjacent_to_nothing, 1};
    adjacencies[1] = {0, adjacent_to_nothing, adjacent_to_nothing};
    adjacencies.resize(2);

  } else {
    // Se il segmento ha inizio e fine su due lati del triangolo
    auto [x, y] = start_edge;
    if (start_edge.y == end_edge.x) {
      auto z       = end_edge.y;
      triangles[0] = {x, edge.x, z};
      triangles[1] = {edge.x, edge.y, z};
      triangles[2] = {edge.x, y, edge.y};

      adjacencies[0] = {adjacent_to_nothing, 1, adjacent_to_nothing};
      adjacencies[1] = {2, adjacent_to_nothing, 0};
      adjacencies[2] = {adjacent_to_nothing, adjacent_to_nothing, 1};

    } else if (start_edge.x == end_edge.y) {
      auto z       = end_edge.x;
      triangles[0] = {x, edge.x, edge.y};
      triangles[1] = {edge.x, z, edge.y};
      triangles[2] = {edge.x, y, z};

      adjacencies[0] = {adjacent_to_nothing, 1, adjacent_to_nothing};
      adjacencies[1] = {2, adjacent_to_nothing, 0};
      adjacencies[2] = {adjacent_to_nothing, adjacent_to_nothing, 1};

    } else {
      assert(0);
    }
  }

  return {triangles, adjacencies};
}

// Constrained Delaunay Triangulation
static pair<vector<vec3i>, vector<vec3i>> constrained_triangulation(
    const vector<vec2f>& _nodes, const vector<vec2i>& edges, int face) {
  auto nodes = _nodes;
  // Questo purtroppo serve.
  for (auto& n : nodes) n *= 1e9;

  auto  result      = pair<vector<vec3i>, vector<vec3i>>{};
  auto& triangles   = result.first;
  auto& adjacencies = result.second;

  // (marzia): qui usiamo float, ma si possono usare anche i double
  using Float = float;
  auto cdt = CDT::Triangulation<Float>(CDT::FindingClosestPoint::ClosestRandom);
  cdt.insertVertices(
      nodes.begin(), nodes.end(),
      [](const vec2f& point) -> Float { return point.x; },
      [](const vec2f& point) -> Float { return point.y; });
  cdt.insertEdges(
      edges.begin(), edges.end(),
      [](const vec2i& edge) -> int { return edge.x; },
      [](const vec2i& edge) -> int { return edge.y; });

  cdt.eraseOuterTriangles();
  adjacencies.reserve(cdt.triangles.size());

  triangles.reserve(cdt.triangles.size());

  for (auto& tri : cdt.triangles) {
    auto verts = vec3i{
        (int)tri.vertices[0], (int)tri.vertices[1], (int)tri.vertices[2]};

    auto adjacency = vec3i{};
    for (auto k = 0; k < 3; k++) {
      auto neigh = tri.neighbors[k];
      if (neigh == CDT::noNeighbor)
        adjacency[k] = adjacent_to_nothing;
      else
        adjacency[k] = (int)neigh;
    }

#if DEBUG_DATA
    // TODO: serve? (marzia): Forse no!
    auto& a           = nodes[verts.x];
    auto& b           = nodes[verts.y];
    auto& c           = nodes[verts.z];
    auto  orientation = cross(b - a, c - b);
    if (fabs(orientation) < 0.00001) {
      global_state()->failed = true;
      printf("[%s]: Collinear in face : %d\n", __FUNCTION__, face);
      return {};
    }
#endif

    triangles.push_back(verts);
    adjacencies.push_back(adjacency);
  }
  return result;
}

static void update_face_adjacencies(bool_mesh& mesh) {
  _PROFILE();
  // Aggiorniamo le adiacenze per i triangoli che sono stati processati
  auto border_edgemap = hash_map<vec2i, int>{};
  border_edgemap.reserve(mesh.triangulated_faces.size() * 6);

  // Per ogni triangolo processato elaboro tutti i suoi sottotriangoli
  for (auto& [face, faces] : mesh.triangulated_faces) {
    // Converto il triangolo in triplette di vertici globali.
    auto triangles = vector<vec3i>(faces.size());
    for (int i = 0; i < faces.size(); i++) {
      triangles[i] = mesh.triangles[faces[i].id];
    }

    for (int i = 0; i < faces.size(); i++) {
      // Guardo se nell'adiacenza ci sono dei triangoli mancanti
      // (segnati con adjacent_to_nothing per non confonderli i -1 già
      // presenti nell'adiacenza della mesh originale).
      auto& adj = mesh.adjacencies[faces[i].id];
      for (int k = 0; k < 3; k++) {
        if (adj[k] != adjacent_to_nothing) continue;

        // Prendo l'edge di bordo corrispondente ad un adjacent_to_nothing
        auto edge = get_mesh_edge_from_index(triangles[i], k);

        // Se è un arco della mesh originale lo processo subito
        if (edge.x < mesh.num_positions && edge.y < mesh.num_positions) {
          // Cerco il triangolo adiacente al triangolo originale su quel lato
          for (int kk = 0; kk < 3; kk++) {
            auto edge0 = get_mesh_edge_from_index(mesh.triangles[face], kk);
            if (make_edge_key(edge) == make_edge_key(edge0)) {
              // Aggiorno direttamente l'adiacenza nel nuovo triangolo e del
              // vicino
              auto neighbor = mesh.adjacencies[face][kk];
              if (neighbor == -1) continue;

              mesh.adjacencies[faces[i].id][k] = neighbor;

              auto it = find_in_vec(mesh.adjacencies[neighbor], face);
              mesh.adjacencies[neighbor][it] = faces[i].id;
            }
          }
          continue;
        }

        // Se non è un arco della mesh originale
        auto edge_key = make_edge_key(edge);
        auto it       = border_edgemap.find(edge_key);

        // Se non l'ho mai incontrato salvo in una mappa l'edge e la
        // faccia corrispondente. Se l'ho già incontrato ricostruisco
        // l'adiacenza tra il triangolo corrente e il neighbor già trovato.
        if (it == border_edgemap.end()) {
          // border_edgemap.insert(it, {edge_key, faces[i]});
          border_edgemap[edge_key] = faces[i].id;
        } else {
          auto neighbor                    = it->second;
          mesh.adjacencies[faces[i].id][k] = neighbor;
          for (int kk = 0; kk < 3; ++kk) {
            auto edge2 = get_mesh_edge_from_index(mesh.triangles[neighbor], kk);
            edge2      = make_edge_key(edge2);
            if (edge2 == edge_key) {
              mesh.adjacencies[neighbor][kk] = faces[i].id;
              break;
            }
          }
        }
      }
    }
  }
}

inline bool check_tags(
    const bool_mesh& mesh, const vector<vec3i>& is_edge_on_boundary) {
  for (int i = 0; i < mesh.triangles.size(); i++) {
    if (mesh.triangulated_faces.find(i) != mesh.triangulated_faces.end()) {
      continue;
    }
    auto face = i;
    auto tr   = mesh.triangles[face];
    if (tr == vec3i{0, 0, 0}) continue;
    for (int k = 0; k < 3; k++) {
      auto neighbor = mesh.adjacencies[face][k];
      if (neighbor < 0) continue;
      // auto n0 = mesh.adjacencies[face];
      // auto n1 = mesh.adjacencies[neighbor];
      auto kk = find_in_vec(mesh.adjacencies[neighbor], face);
      assert(kk != -1);

      auto tags0 = is_edge_on_boundary[face];
      auto tags1 = is_edge_on_boundary[neighbor];
      auto tag0  = tags0[k];
      auto tag1  = tags1[kk];
      assert(tag0 == -tag1);
    }
  }
  return true;
}

#include <yocto/yocto_parallel.h>

template <typename F>
inline void parallel_for_batch(int num_threads, size_t size, F&& f) {
  auto threads    = vector<std::thread>(num_threads);
  auto batch_size = size / num_threads;
  auto batch_job  = [&](size_t k) {
    auto from = k * batch_size;
    auto to   = std::min(from + batch_size, size);
    for (auto i = from; i < to; i++) f(i);
  };

  for (auto k = 0; k < num_threads; k++) {
    threads[k] = std::thread(batch_job, k);
  }
  for (auto k = 0; k < num_threads; k++) {
    threads[k].join();
  }
}

static void triangulate(bool_mesh& mesh, const mesh_hashgrid& hashgrid) {
  _PROFILE();
  // auto mesh_triangles_size = atomic<size_t>{mesh.triangles.size()};
  auto mesh_mutex = std::mutex{};
  auto i          = 0;
  auto faces      = vector<int>(hashgrid.size());
  for (auto& [key, _] : hashgrid) {
    faces[i++] = key;
  }
  mesh.triangulated_faces.reserve(hashgrid.size());

  // for (auto& [face, polylines] : hashgrid) {
  auto f = [&](size_t index) {
    auto  face      = faces[index];
    auto& polylines = hashgrid.at(face);

    // Calcola le info per la triangolazione, i.e. (nodi ed edge
    // constraints).
    auto info = triangulation_constraints(mesh, face, polylines);

    //    add_debug_node(face, info.nodes);
    //    add_debug_index(face, info.indices);

    // Se la faccia contiene solo segmenti corrispondenti ad edge del
    // triangolo stesso, non serve nessuna triangolazione.
    if (info.nodes.size() == 3) {
      auto nodes = std::array<vec2f, 3>{vec2f{0, 0}, vec2f{1, 0}, vec2f{0, 1}};
      mesh.triangulated_faces[face] = {facet{nodes, face}};
      return;
    }

    auto triangulated_faces = vector<facet>{};
    auto triangles          = vector<vec3i>();
    auto adjacency          = vector<vec3i>();

    // Se il triangolo ha al suo interno un solo segmento allora chiamiamo
    // la funzione di triangolazione più semplice, altrimenti chiamiamo CDT
    if (info.edges.size() == 1) {
      tie(triangles, adjacency) = single_split_triangulation(
          info.nodes, info.edges[0]);
    } else {
      add_boundary_edge_constraints(info.edgemap, info.edges);
      tie(triangles, adjacency) = constrained_triangulation(
          info.nodes, info.edges, face);
    }
    // Converti triangli locali in globali.
    for (int i = 0; i < triangles.size(); i++) {
      auto& tr = triangles[i];
      auto  n  = std::array<vec2f, 3>{
          info.nodes[tr[0]], info.nodes[tr[1]], info.nodes[tr[2]]};
      triangulated_faces.push_back({n, -1});
      tr = {info.indices[tr.x], info.indices[tr.y], info.indices[tr.z]};
    }

    auto polygon_faces = vector<bool_mesh::shape_segment>{};
    struct Entry {
      int   shape_id, boundary_id, curve_id;
      float t;
    };
    auto border_map = hash_map<vec2i, Entry>{};
    for (auto& p : polylines) {
      for (auto i = 0; i < num_segments(p); i++) {
        auto edge = get_segment_vertices(p, i);
        assert(p.points.size() == p.points.size());
        assert(p.vertices.size() == p.points.size());
        assert(p.curve_ids.size() == p.points.size());
        assert(p.t.size() == p.points.size());
        border_map[edge] = Entry{
            p.shape_id, p.boundary_id, p.curve_ids[i], p.t[i]};
      }
    }

    for (int i = 0; i < triangles.size(); i++) {
      for (int k = 0; k < 3; k++) {
        auto edge = get_mesh_edge_from_index(triangles[i], k);
        if (auto it = border_map.find(edge); it != border_map.end()) {
          auto& entry         = it->second;
          auto& segment       = polygon_faces.emplace_back();
          segment.shape_id    = entry.shape_id;
          segment.boundary_id = entry.boundary_id;
          segment.curve_id    = entry.curve_id;
          segment.t           = entry.t;
          segment.face_in     = i;
          segment.face_out    = adjacency[i][k];
        }
      }
    }

    add_debug_edge(face, info.edges);
    add_debug_triangle(face, triangles);

    // TODO(giacomo): Pericoloso: se resize() innesca una riallocazione, il
    // codice dopo l'unlock che sta eseguendo su un altro thread continua a
    // scrivere su memoria vecchia e deallocata.
    // In @MUTLITHREAD_DANGER preallochiamo piu' spazio per evitare questo.
    // Soluzione: si mette tutto dentro il lock, anche il pezzetto dopo. Si
    // abbassano un po' le performance (quanto? boh, poco).
    auto mesh_triangles_old_size = 0;
    {
      auto lock               = std::lock_guard{mesh_mutex};
      mesh_triangles_old_size = (int)mesh.adjacencies.size();
      mesh.triangles.resize(mesh_triangles_old_size + triangles.size());
      mesh.adjacencies.resize(mesh_triangles_old_size + triangles.size());
      for (int i = 0; i < triangles.size(); i++) {
        triangulated_faces[i].id = mesh_triangles_old_size + i;
      }
      mesh.triangulated_faces[face] = triangulated_faces;
      for (auto& segment : polygon_faces) {
        if (segment.face_in >= 0) segment.face_in += mesh_triangles_old_size;
        if (segment.face_out >= 0) segment.face_out += mesh_triangles_old_size;
      }
      mesh.polygon_borders += polygon_faces;
    }

    for (int i = 0; i < triangles.size(); i++) {
      mesh.triangles[mesh_triangles_old_size + i] = triangles[i];
    }
    for (int i = 0; i < triangles.size(); i++) {
      for (auto& x : adjacency[i])
        if (x != adjacent_to_nothing) x += mesh_triangles_old_size;
      mesh.adjacencies[mesh_triangles_old_size + i] = adjacency[i];
    }
  };

// parallel_for_batch(8, hashgrid.size(), f);
#if DEBUG_DATA
  for (int i = 0; i < hashgrid.size(); i++) f(i);
#else
  parallel_for(hashgrid.size(), f);
#endif
}

static vector<bool> make_edges_boundary_tags(const bool_mesh& mesh) {
  _PROFILE();
  auto result = vector<bool>(3 * mesh.adjacencies.size(), false);
  for (auto& segment : mesh.polygon_borders) {
    if (segment.face_in < 0 || segment.face_out < 0) continue;
    auto k = find_in_vec(mesh.adjacencies[segment.face_in], segment.face_out);
    assert(k != -1);
    result[3 * segment.face_in + k] = true;
    auto kk = find_in_vec(mesh.adjacencies[segment.face_out], segment.face_in);
    assert(kk != -1);
    result[3 * segment.face_out + kk] = true;
  }
  return result;
}

void slice_mesh(bool_mesh& mesh, bool_state& state,
    const vector<vector<vector<vector<mesh_segment>>>>& edges) {
  _PROFILE();

  // Calcoliamo hashgrid e intersezioni tra poligoni,
  // aggiungendo ulteriori vertici nuovi alla mesh
  auto hashgrid = compute_hashgrid(mesh, edges);
  add_polygon_intersection_points(state, hashgrid, mesh);

  // Triangolazione e aggiornamento dell'adiacenza
  triangulate(mesh, hashgrid);
  update_face_adjacencies(mesh);

  // Tagga tutti i mesh edges che si trovano su un boundary.
  mesh.is_edge_on_boundary = make_edges_boundary_tags(mesh);
}

void compute_cell_labels(bool_state& state) {
  _PROFILE();

  propagate_cell_labels(state);

  auto offset = vector<int>((int)state.shapes.size(), 0);
  for (auto& cell_label : state.labels) {
    for (auto l = 0; l < cell_label.size(); l++) {
      offset[l] = min(offset[l], cell_label[l]);
    }
  }

  for (auto& cell_label : state.labels) {
    for (auto l = 0; l < cell_label.size(); l++) {
      cell_label[l] = (cell_label[l] - offset[l]) % 2;
    }
  }
}

#include "boolsurf_io.h"
#include "bsh.h"

struct Curve_Patches {
  struct Patch {
    vector<pair<int, float>> points;
  };
  int                              num_nodes = 0;
  std::unordered_map<vec3i, Patch> edges     = {};
};

Curve_Patches make_curve_patches(
    const vector<vector<vector<vector<mesh_segment>>>>& shape_segments) {
  auto  curve_patches = Curve_Patches{};
  auto& num_nodes     = curve_patches.num_nodes;
  auto& edges         = curve_patches.edges;
  for (int shape_id = 0; shape_id < shape_segments.size(); shape_id++) {
    auto& shape = shape_segments[shape_id];
    for (int boundary_id = 0; boundary_id < shape.size(); boundary_id++) {
      auto& boundary = shape[boundary_id];
      auto  first    = num_nodes;
      // auto  last     = num_nodes + (int)boundary.size() - 1;
      for (int curve_id = 0; curve_id < boundary.size(); curve_id++) {
        // auto& curve = boundary[curve_id];
        // auto  node  = add_node(graph, shape_id, boundary_id, curve_id);
        auto  node  = num_nodes++;
        auto& patch = edges[{shape_id, boundary_id, curve_id}];
        patch.points.push_back({node, 0.0f});
        if (curve_id == (int)boundary.size() - 1)
          patch.points.push_back({first, 1.0f});
        else
          patch.points.push_back({node + 1, 1.0f});
      }
    }
  }
  return curve_patches;
}

void add_intersections(Curve_Patches& curve_patches, const bool_state& state) {
  for (auto& isec : state.intersections) {
    auto node = curve_patches.num_nodes++;
    for (int k = 0; k < 2; k++) {
      auto  shape_id    = isec.shape_ids[k];
      auto  boundary_id = isec.boundary_ids[k];
      auto  curve_id    = isec.curve_ids[k];
      auto  t           = isec.t[k];
      auto  key         = vec3i{shape_id, boundary_id, curve_id};
      auto& points      = curve_patches.edges[key].points;
      points.push_back({node, t});
    }
  }

  for (auto& [key, patch] : curve_patches.edges) {
    auto& points = patch.points;
    if (points.size() <= 2) continue;
    std::sort(points.begin(), points.end(),
        [](auto& a, auto& b) { return a.second < b.second; });
  }
}

vector<vector<int>> make_node_graph(const Curve_Patches& curve_patches) {
  auto nodes = vector<vector<int>>{};

  auto add = [&](int a, int b) {
    // auto id             = (int)patch_to_id.size();
    // patch_to_id[{a, b}] = id;
    if (a >= nodes.size()) nodes.resize(a + 1);
    nodes[a] += b;
  };

  for (auto& [key, patch] : curve_patches.edges) {
    for (int i = 0; i < patch.points.size() - 1; i++) {
      add(patch.points[i].first, patch.points[i + 1].first);
    }
  }
  return nodes;
}

unordered_map<vec2i, int> make_patch_to_id(const vector<vector<int>>& nodes) {
  auto patch_to_id = unordered_map<vec2i, int>{};
  for (int i = 0; i < nodes.size(); i++) {
    for (int k = 0; k < nodes[i].size(); k++) {
      auto neighbor = nodes[i][k];
      if (neighbor < 0) continue;
      auto id  = (int)patch_to_id.size();
      auto key = vec2i{i, neighbor};
      if (patch_to_id.count(key) == 0) patch_to_id[key] = id;
    }
  }
  return patch_to_id;
}

vector<vector<int>> make_node_to_patches(const vector<vector<int>>& nodes,
    const unordered_map<vec2i, int>& patch_to_id) {
  auto node_to_patches = vector<vector<int>>(nodes.size());
  for (int i = 0; i < nodes.size(); i++) {
    for (auto& neighbor : nodes[i]) {
      auto id = patch_to_id.at({i, neighbor});
      node_to_patches[i] += id;
      node_to_patches[neighbor] += id;
    }
  }
  return node_to_patches;
}

void set_patch_adjacency(vector<BSH_patch>& patches,
    const vector<vector<int>>& node_to_patches, int num_patches) {
  patches.resize(num_patches);
  for (int i = 0; i < node_to_patches.size(); i++) {
    for (int k = 0; k < node_to_patches[i].size(); k++) {
      for (int j = 0; j < node_to_patches[i].size(); j++) {
        if (k == j) continue;
        patches[node_to_patches[i][k]].adj_patches += node_to_patches[i][j];
        patches[node_to_patches[i][j]].adj_patches += node_to_patches[i][k];
      }
    }
  }

  // remove duplicates
  for (auto& patch : patches) {
    auto& p  = patch.adj_patches;
    auto  hs = hash_set<int>(p.begin(), p.end());
    p        = vector<int>(hs.begin(), hs.end());
  }
}

// auto patch_to_id = hash_map<vec2i, int>{};

// auto patch_graph = hash_map<vec2i, vec2i>{};
// for (auto& segment : mesh.polygon_borders) {
//   auto  key  = vec3i{segment.shape_id, segment.boundary_id,
//   segment.curve_id}; auto& side = edges.at(key); auto  t    = segment.t;
//   auto  cell_in  = mesh.face_tags[segment.face_in];
//   auto  cell_out = mesh.face_tags[segment.face_out];
//   for (int i = 0; i < side.points.size() - 1; i++) {
//     auto [isec_node, isec_t] = side.points[i + 1];
//     auto patch_key           = vec2i{side.points[i].first, isec_node};
//     if (isec_t >= t) {
//       if (patch_graph.count(patch_key)) {
//         auto& item = patch_graph.at(patch_key);
//         assert(item.x == cell_in);
//         assert(item.y == cell_out);
//         continue;
//       }
//       patch_graph[patch_key] = {cell_in, cell_out};
//     }
//   }
// }

// for (auto& [key, value] : patch_graph) {
//   printf("patch [%d, %d] adjacent to cells [%d, %d]\n", key.x, key.y,
//   value.x,
//       value.y);
// }

BSH_graph make_bsh_input(bool_state& state, bool_mesh& mesh,
    const vector<vector<vector<vector<mesh_segment>>>>& shape_segments) {
  auto curve_patches = make_curve_patches(shape_segments);
  add_intersections(curve_patches, state);
  auto nodes = make_node_graph(curve_patches);
  // save_graph(nodes, "boolsurf/test");

  auto& borders = mesh.polygon_borders;
  // std::sort(borders.begin(), borders.end(), [](auto& a, auto& b) {
  //   if (a.shape_id != b.shape_id) return a.shape_id < b.shape_id;
  //   if (a.boundary_id != b.boundary_id) return a.boundary_id < b.boundary_id;
  //   if (a.curve_id != b.curve_id) return a.curve_id < b.curve_id;
  //   return a.t < b.t;
  // });

  // printf("\n\n");
  // for (auto& border : borders) {
  //   printf("[%d %d]\n", mesh.face_tags[border.face_in],
  //       mesh.face_tags[border.face_out]);
  // }

  auto patch_to_id      = make_patch_to_id(nodes);
  auto node_to_patches  = make_node_to_patches(nodes, patch_to_id);
  auto patch_to_cells   = vector<vec2i>(patch_to_id.size(), {-1, -1});
  auto segment_to_patch = vector<int>(borders.size());

  auto set_patch_to_cell_adjacency = [&](vector<BSH_patch>& bsh_patches) {
    for (int b = 0; b < borders.size(); b++) {
      auto& border = borders[b];
      auto  key = vec3i{border.shape_id, border.boundary_id, border.curve_id};
      auto& points = curve_patches.edges.at(key).points;
      for (int i = 1; i < points.size(); i++) {
        if (border.t < points[i].second) {
          auto k        = vec2i{points[i - 1].first, points[i].first};
          auto id       = patch_to_id.at(k);
          auto cell_in  = mesh.face_tags[border.face_in];
          auto cell_out = mesh.face_tags[border.face_out];
          if (bsh_patches[id].adj_cell_0 != -1) {
            continue;
            assert(bsh_patches[id].adj_cell_0 == cell_in);
            assert(bsh_patches[id].adj_cell_1 == cell_out);
          }
          bsh_patches[id].shape_id   = border.shape_id;
          bsh_patches[id].adj_cell_0 = cell_in;
          bsh_patches[id].adj_cell_1 = cell_out;
          segment_to_patch[b]        = id;
        }
      }
    }
  };

  auto& bsh            = state.bsh_input;
  bsh.segment_to_patch = segment_to_patch;
  bsh.num_cells        = (int)state.cells.size();
  set_patch_adjacency(bsh.patches, node_to_patches, (int)patch_to_id.size());
  set_patch_to_cell_adjacency(bsh.patches);
  return bsh;
}

vector<bool> run_bsh(const BSH_graph& bsh) {
  if (bsh.patches.empty()) return {};

  auto input_name  = "boolsurf/bsh-input.json"s;
  auto output_name = "boolsurf/bsh-output.json"s;
  auto bin_name =
      "/Users/nazzaro/dev/Boundary_Sampled_Halfspaces/build/graph_cut"s;
  save_bsh(bsh, input_name);
  auto cmd = bin_name + " " + input_name + " " + output_name;
  system(cmd.c_str());
  auto output = vector<bool>();
  load_bsh_result(output, output_name);
  return output;
}

bool compute_cells(bool_mesh& mesh, bool_state& state,
    const vector<vector<vector<vector<mesh_segment>>>>& shape_segments) {
  _PROFILE();
  global_state() = &state;
  global_mesh()  = &mesh;
  // Triangola mesh in modo da embeddare tutti i poligoni come mesh-edges.
  slice_mesh(mesh, state, shape_segments);

  state.shapes.resize(shape_segments.size());

  // Trova celle e loro adiacenza via flood-fill.
  state.cells = make_cell_graph(mesh);

  //  auto patches = make_patches(mesh, state);

  // Calcola i label delle celle con una visita sulla loro adiacenza.
  compute_cell_labels(state);

  return true;
}

void compute_shapes(bool_state& state) {
  // Calcoliamo le informazioni sulla shape, come le celle che ne fanno parte
  auto& shapes  = state.shapes;
  auto& sorting = state.shapes_sorting;
  sorting.resize(state.shapes.size());

  // Assign a polygon and a color to each shape.
  for (auto s = 0; s < state.shapes.size(); s++) {
    shapes[s].color = get_color(s);
    sorting[s]      = s;
  }

  // Distribute cells to shapes.
  // La prima shape è relativa alla cella ambiente, che è root per
  // definizione
  //  shapes[0].cells = hash_set<int>(
  //      state.ambient_cells.begin(), state.ambient_cells.end());
  //  shapes[0].is_root = false;

  state.shape_from_cell.resize(state.labels[0].size());
  for (auto c = 0; c < state.cells.size(); c++) {
    auto count = 0;
    for (auto p = 1; p < state.labels[c].size(); p++) {
      if (state.labels[c][p] > 0) {
        shapes[p].cells.insert(c);
        state.shape_from_cell[c] = p;
        count += 1;
      }
    }

    if (count == 0) shapes[0].cells.insert(c);
  }
}

void compute_generator_polygons(
    const bool_state& state, int shape_idx, hash_set<int>& result) {
  // Calcoliamo ricorsivamente i poligoni iniziali coinvolti nelle operazioni
  // che hanno generato la shape corrente
  auto& bool_shape = state.shapes[shape_idx];

  // Se la shape non ha generatori allora corrisponde ad una shape di un
  // poligono
  if (bool_shape.generators.empty()) {
    result.insert(shape_idx);
    return;
  }

  // Calcolo i generatori per le shape che hanno generato quella corrente
  for (auto& generator : bool_shape.generators) {
    if (generator == -1) continue;
    compute_generator_polygons(state, generator, result);
  }
}

void compute_shape_borders(const bool_mesh& mesh, bool_state& state) {
  //   // Calcoliamo tutti i bordi di una shape
  //   for (auto s = 0; s < state.shapes.size(); s++) {
  //     auto& bool_shape = state.shapes[s];

  //     // Calcoliamo il bordo solo per le shape root dell'albero csg
  //     if (!bool_shape.is_root) continue;

  //     // Calcoliamo i poligoni iniziali coinvolti nelle operazioni che hanno
  //     // generato la root (ci serve successivamente per salvare nel bordo
  //     // solamente i punti corretti)
  //     auto generator_polygons = hash_set<int>();
  //     compute_generator_polygons(state, s, generator_polygons);

  //     auto components = compute_components(state, bool_shape);

  //     for (auto& component : components) {
  //       // Step 1: Calcoliamo gli edges che stanno sul bordo
  //       auto edges = hash_set<vec2i>();

  //       for (auto c : component) {
  //         auto& cell = state.cells[c];
  //         // Per ogni cella che compone la shape calcolo il bordo a partire
  //         // dalle facce che ne fanno parte

  //         for (auto face : cell.faces) {
  //           // Se è una faccia interna allora non costituirà il bordo
  //           for (auto k = 0; k < 3; k++) {
  //             if (mesh.is_edge_on_boundary[3 * face + k] == false) continue;
  //           }

  //           // Per ogni lato del triangolo considero solamente quelli che
  //           sono
  //           // di bordo (tag != 0)
  //           auto& tri = mesh.triangles[face];
  //           for (auto k = 0; k < 3; k++) {
  //             auto tag = mesh.is_edge_on_boundary[3 * face + k];
  //             if (tag == false) continue;
  //             auto edge     = get_mesh_edge_from_index(tri, k);
  //             auto rev_edge = vec2i{edge.y, edge.x};

  //             // Se 'edge' è già stato incontrato allora esso è un bordo tra
  //             due
  //             // celle che fanno parte dela stessa shape, quindi lo elimino
  //             dal
  //             // set.
  //             auto it = edges.find(rev_edge);
  //             if (it == edges.end())
  //               edges.insert(edge);
  //             else
  //               edges.erase(it);
  //           }
  //         }
  //       }

  //       // Step 2: Riordiniamo i bordi
  //       // Per ogni vertice salviamo il proprio successivo
  //       auto next_vert = hash_map<int, int>();
  //       for (auto& edge : edges) next_vert[edge.x] = edge.y;

  //       for (auto& [key, value] : next_vert) {
  //         // Se il valore è -1 abbiamo già processato il punto
  //         if (value == -1) continue;

  //         // Aggiungiamo un nuovo bordo
  //         auto border_points = vector<int>();

  //         auto current = key;

  //         while (true) {
  //           auto next = next_vert.at(current);
  //           if (next == -1) break;

  //           next_vert.at(current) = -1;

  //           // Se il vertice corrente è un punto di controllo lo aggiungo al
  //           // bordo
  //           //          if (contains(state.control_points, current)) {
  //           //            // Se è un punto di intersezione controlliamo che i
  //           //            poligoni che
  //           //            // lo hanno generato siano entrambi compresi nei
  //           //            poligoni che
  //           //            // hanno generato anche la shape.
  //           //            if (contains(state.isecs_generators, current)) {
  //           //              auto& isec_generators =
  //           //              state.isecs_generators.at(current);
  //           //
  //           //              if (contains(generator_polygons,
  //           isec_generators.x)
  //           //              &&
  //           //                  contains(generator_polygons,
  //           isec_generators.y))
  //           //                border_points.push_back(current);
  //           //            } else
  //           //              border_points.push_back(current);
  //           //          }

  //           // Se un bordo è stato chiuso correttamente lo inseriamo tra i
  //           bordi
  //           // della shape
  //           if (next == key) {
  //             bool_shape.border_points.push_back(border_points);
  //             break;
  //           } else
  //             current = next;
  //         }
  //       }
  //     }
  //   }
}

bool_state compute_border_polygons(const bool_state& state) {
  auto new_state = bool_state{};
  //  new_state.points = state.points;
  //
  //  for (auto& bool_shape : state.bool_shapes) {
  //    if (!bool_shape.is_root) continue;
  //    auto& test_shape = new_state.bool_shapes.emplace_back();
  //    for (auto& border : bool_shape.border_points) {
  //      auto& polygon = test_shape.polygons.emplace_back();
  //      for (auto v : border) {
  //        auto id = state.control_points.at(v);
  //        polygon.points.push_back(id);
  //      }
  //    }
  //  }
  return new_state;
}

void compute_bool_operation(bool_state& state, const bool_operation& op) {
  auto& a = state.shapes[op.shape_a];
  auto& b = state.shapes[op.shape_b];

  // Convertiamo il vettore di interi in bool per semplificare le operazioni
  auto aa = vector<bool>(state.cells.size(), false);
  for (auto& c : a.cells) aa[c] = true;

  auto bb = vector<bool>(state.cells.size(), false);
  for (auto& c : b.cells) bb[c] = true;

  if (op.type == bool_operation::Type::op_union) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] || bb[i];
  } else if (op.type == bool_operation::Type::op_intersection) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] && bb[i];
  } else if (op.type == bool_operation::Type::op_difference) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] && !bb[i];
  } else if (op.type == bool_operation::Type::op_symmetrical_difference) {
    for (auto i = 0; i < aa.size(); i++) aa[i] = aa[i] != bb[i];
  }

  // Le shape 'a' e 'b' sono state usate nell'operazione,
  // quindi non sono root del csg tree
  a.is_root = false;
  b.is_root = false;

  // Creiamo una nuova shape risultato, settando come generatori le shape 'a'
  // e 'b' e riconvertendo il vettore di bool a interi
  auto  shape_id = state.shapes.size();
  auto& c        = state.shapes.emplace_back();
  c.generators   = {op.shape_a, op.shape_b};
  c.color        = state.shapes[op.shape_a].color;
  auto sorting   = find_idx(state.shapes_sorting, op.shape_a);

  insert(state.shapes_sorting, sorting, (int)shape_id);

  for (auto i = 0; i < aa.size(); i++)
    if (aa[i]) c.cells.insert(i);
}

void compute_bool_operations(
    bool_state& state, const vector<bool_operation>& ops) {
  _PROFILE();
  for (auto& op : ops) {
    compute_bool_operation(state, op);
  }
}

vec3f get_cell_color(const bool_state& state, int cell_id, bool color_shapes) {
  if (state.shapes.empty() && state.labels.empty()) return {1, 1, 1};
  if (color_shapes) {
    for (int s = (int)state.shapes_sorting.size() - 1; s >= 0; s--) {
      auto& bool_shape = state.shapes[state.shapes_sorting[s]];
      if (bool_shape.cells.count(cell_id) && bool_shape.is_root) {
        return bool_shape.color;
      }
    }
    return {1, 1, 1};
  } else {
    auto color = vec3f{0, 0, 0};
    int  count = 0;
    for (int p = 0; p < state.labels[cell_id].size(); p++) {
      auto label = state.labels[cell_id][p];
      if (label > 0) {
        color += get_color(p);
        count += 1;
      }
    }
    if (count > 0) {
      color /= count;
    } else {
      color = {0.9, 0.9, 0.9};
    }
    return color;
  }
}

hash_map<int, vector<vec3i>>& debug_triangles() {
  static hash_map<int, vector<vec3i>> result = {};
  return result;
}

hash_map<int, vector<vec2i>>& debug_edges() {
  static hash_map<int, vector<vec2i>> result = {};
  return result;
}

hash_map<int, vector<vec2f>>& debug_nodes() {
  static hash_map<int, vector<vec2f>> result = {};
  return result;
}

hash_map<int, vector<int>>& debug_indices() {
  static hash_map<int, vector<int>> result = {};
  return result;
}

vector<int>& debug_result() {
  static vector<int> result = {};
  return result;
}
vector<bool>& debug_visited() {
  static vector<bool> result = {};
  return result;
}
vector<int>& debug_stack() {
  static vector<int> result = {};
  return result;
}
bool& debug_restart() {
  static bool result = {};
  return result;
}
