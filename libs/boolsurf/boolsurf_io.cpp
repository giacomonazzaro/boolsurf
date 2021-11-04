#include <stdio.h>
#define NANOSVG_ALL_COLOR_KEYWORDS
#define NANOSVG_IMPLEMENTATION
#include "ext/nanosvg/src/nanosvg.h"
//
#include <yocto/yocto_modelio.h>
#include <yocto/yocto_sceneio.h>

#include "boolsurf_io.h"

bool load_json(const string& filename, json& js) {
  // error helpers
  auto error = ""s;
  auto text  = ""s;
  if (!load_text(filename, text, error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  try {
    js = json::parse(text);
    return true;
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return false;
  }
}

bool save_test(const bool_test& test, const string& filename) {
  auto js           = json{};
  js["points"]      = test.points;
  js["shapes"]      = test.shapes;
  js["polygons"]    = test.polygons;
  js["model"]       = test.model;
  js["operations"]  = test.operations;
  js["camera"]      = test.camera;
  js["cell_colors"] = test.cell_colors;

  auto error = ""s;
  if (!save_text(filename, js.dump(2), error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  return true;
}

bool load_test(bool_test& test, const string& filename) {
  auto js = json{};
  if (!load_json(filename, js)) {
    return false;
  }

  if (js.find("cell_colors") != js.end()) {
    test.cell_colors = js["cell_colors"].get<vector<vec3f>>();
  }

  try {
    if (js.find("screenspace") != js.end()) {
      test.screenspace = js["screenspace"].get<bool>();
    }

    if (test.screenspace) {
      test.polygons_screenspace = js["polygons"].get<vector<vector<vec2f>>>();

      for (auto& polygon : test.polygons_screenspace) {
        auto area = 0.0f;
        for (int p = 0; p < polygon.size(); p++) {
          auto& point = polygon[p];
          auto& next  = polygon[(p + 1) % polygon.size()];
          area += cross(next, point);
        }

        if (area < 0) reverse(polygon.begin(), polygon.end());
      }

      auto bbox = bbox2f{};
      for (auto& polygon : test.polygons_screenspace)
        for (auto& p : polygon) bbox = merge(bbox, p);

      for (auto& polygon : test.polygons_screenspace)
        for (auto& p : polygon) p = (p - center(bbox)) / max(size(bbox));

    } else {
      test.points   = js["points"].get<vector<mesh_point>>();
      test.polygons = js["polygons"].get<vector<vector<int>>>();
    }

    if (js.find("shapes") != js.end()) {
      test.shapes = js["shapes"].get<vector<vector<int>>>();
    }

    if (js.find("operations") != js.end()) {
      test.operations = js["operations"].get<vector<bool_operation>>();
    }

    if (js.find("camera") != js.end()) {
      test.camera     = js["camera"].get<scene_camera>();
      test.has_camera = true;
    }

    if (js.find("model") != js.end()) {
      test.model = js["model"].get<string>();
    }
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return false;
  }
  return true;
}

bool_state state_from_test(const bool_mesh& mesh, const bool_test& test,
    float drawing_size, bool use_projection) {
  auto state   = bool_state{};
  state.points = test.points;
  state.bool_shapes.clear();
  // state.polygons.clear();

  if (test.screenspace) {
    auto camera = make_camera(mesh);
    return make_test_state(
        test, mesh, mesh.bvh, camera, drawing_size, use_projection);
  }

  if (test.shapes.empty()) {
    for (auto& test_polygon : test.polygons) {
      // Add new 1-polygon shape to state
      // if (test_polygon.empty()) continue;

      auto& bool_shape = state.bool_shapes.emplace_back();
      auto& polygon    = bool_shape.polygons.emplace_back();
      polygon.points   = test_polygon;
      recompute_polygon_segments(mesh, state, polygon);
    }
  } else {
    for (auto& test_shape : test.shapes) {
      auto& bool_shape = state.bool_shapes.emplace_back();
      for (auto& polygon_id : test_shape) {
        auto& polygon  = bool_shape.polygons.emplace_back();
        polygon.points = test.polygons[polygon_id];
        recompute_polygon_segments(mesh, state, polygon);
      }
    }
  }

  return state;
}

bool_state state_from_screenspace_test(
    bool_mesh& mesh, bool_test& test, float drawing_size, bool use_projection) {
  int  seed          = 0;
  auto stop          = false;
  auto rng           = make_rng(seed);
  auto state         = bool_state{};
  auto mesh_original = mesh;

  while (!stop) {
    for (auto trial = 0; trial < 20; trial++) {
      state    = {};
      auto cam = scene_camera{};
      auto eye = sample_sphere(rand2f(rng)) * 2.5;

      auto position = vec3f{0, 0, 0};
      cam.frame     = lookat_frame(eye, position, {0, 1, 0});
      cam.focus     = length(eye - position);

      auto center = intersect_mesh(mesh, cam, vec2f{0.5, 0.5});
      test.camera = make_camera(mesh, seed);

      if (center.face == -1) continue;

      add_polygons(state, mesh, test.camera, test, center, drawing_size, false);
      test.camera = cam;

      try {
        stop = compute_cells(mesh, state);
        compute_shapes(state);
      } catch (const std::exception&) {
        stop = true;
      }

      mesh = mesh_original;
      if (stop) break;
    }

    drawing_size -= 0.001f;
  }
  return state;
}

void add_polygons(bool_state& state, const bool_mesh& mesh,
    const scene_camera& camera, const bool_test& test, const mesh_point& center,
    float svg_size, bool screenspace, bool straight_up) {
  auto polygons = test.polygons_screenspace;

  auto make_straight_up = [&](vec2f& uv) {
    if (!straight_up) return;
    uv         = -uv;
    auto frame = mat3f{};

    auto p0  = eval_position(mesh, {center.face, {0, 0}});
    auto p1  = eval_position(mesh, {center.face, {1, 0}});
    frame.x  = normalize(p1 - p0);
    frame.z  = eval_normal(mesh, center.face);
    frame.y  = normalize(cross(frame.z, frame.x));
    auto up  = normalize(vec3f{0, 1, -0.35});
    auto v   = normalize(vec2f{dot(up, frame.x), dot(up, frame.y)});
    auto rot = mat2f{{v.x, v.y}, {-v.y, v.x}};
    uv       = rot * uv;
  };

  for (auto& polygon : polygons) {
    for (auto& uv : polygon) {
      uv *= svg_size;
      uv.x = -uv.x;
    }
  }

  auto get_projected_point = [&](vec2f uv) {
    uv.x /= camera.film;                    // input.window_size.x;
    uv.y /= (camera.film / camera.aspect);  // input.window_size.y;
    uv.x = -uv.x;
    uv += vec2f{0.5, 0.5};
    auto cam      = scene_camera{};
    auto position = eval_position(mesh, center);
    auto normal   = eval_normal(mesh, center);
    auto eye      = position + normal * 0.2;
    cam.frame     = lookat_frame(eye, position, {0, 1, 0});
    cam.focus     = length(eye - position);
    return intersect_mesh(mesh, cam, uv);
  };

  auto get_mapped_point = [&](vec2f uv) {
    make_straight_up(uv);
    uv /= camera.film;
    auto path     = straightest_path(mesh, center, uv);
    path.end.uv.x = clamp(path.end.uv.x, 0.0f, 1.0f);
    path.end.uv.y = clamp(path.end.uv.y, 0.0f, 1.0f);
    return path.end;
  };

  for (auto& test_shape : test.shapes) {
    auto bool_shape = shape{};
    for (auto id = 0; id < test_shape.size(); id++) {
      auto  bool_polygon = mesh_polygon{};
      auto& test_polygon = polygons[test_shape[id]];

      for (auto uv : test_polygon) {
        auto point = screenspace ? get_projected_point(uv)
                                 : get_mapped_point(uv);
        if (point.face == -1) continue;

        // Add point to state.
        bool_polygon.points.push_back((int)state.points.size());
        state.points.push_back(point);
      }

      if (bool_polygon.points.size() <= 2) {
        assert(0);
        bool_polygon.points.clear();
        continue;
      }

      bool_shape.polygons.push_back(bool_polygon);
      recompute_polygon_segments(mesh, state, bool_polygon);
    }
    if (bool_shape.polygons.size()) state.bool_shapes.push_back(bool_shape);
  }
}

scene_shape create_polygon_shape(
    const vector<vec3f>& positions, float thickness) {
  //  auto positions = eval_positions(triangles, positions, path);
  auto shape = scene_shape{};
  for (auto idx = 0; idx < positions.size(); idx++) {
    auto sphere = make_sphere(8, thickness);
    for (auto& p : sphere.positions) p += positions[idx];
    merge_quads(shape.quads, shape.positions, shape.normals, shape.texcoords,
        sphere.quads, sphere.positions, sphere.normals, sphere.texcoords);

    if (idx < (int)positions.size() - 1) {
      auto cylinder = make_uvcylinder({32, 1, 1},
          {thickness, length(positions[idx] - positions[idx + 1]) / 2});
      auto frame    = frame_fromz((positions[idx] + positions[idx + 1]) / 2,
          normalize(positions[idx + 1] - positions[idx]));

      for (auto& p : cylinder.positions) p = transform_point(frame, p);
      for (auto& n : cylinder.normals) n = transform_direction(frame, n);
      merge_quads(shape.quads, shape.positions, shape.normals, shape.texcoords,
          cylinder.quads, cylinder.positions, cylinder.normals,
          cylinder.texcoords);
    }
  }
  return shape;
}

scene_model make_scene(const bool_mesh& mesh, const bool_state& state,
    const scene_camera& camera, bool color_shapes, bool color_hashgrid,
    bool save_edges, bool save_polygons, float line_width,
    const vector<vec3f>& cell_colors) {
  auto scene = scene_model{};
  scene.cameras.push_back(camera);

  if (color_hashgrid) {
    auto& hashgrid_instance     = scene.instances.emplace_back();
    hashgrid_instance.material  = (int)scene.materials.size();
    auto& hashgrid_material     = scene.materials.emplace_back();
    hashgrid_material.type      = scene_material_type::glossy;
    hashgrid_material.roughness = 0.5;
    hashgrid_material.color     = vec3f{0.4f, 0.4f, 0.4f};
    hashgrid_instance.shape     = (int)scene.shapes.size();
    auto hashgrid_shape         = scene_shape{};
    hashgrid_shape.positions    = mesh.positions;

    auto& mesh_instance     = scene.instances.emplace_back();
    mesh_instance.material  = (int)scene.materials.size();
    auto& mesh_material     = scene.materials.emplace_back();
    mesh_material.type      = scene_material_type::glossy;
    mesh_material.roughness = 0.5;
    mesh_material.color     = vec3f{1.0f, 1.0f, 1.0f};
    mesh_instance.shape     = (int)scene.shapes.size() + 1;
    auto mesh_shape         = scene_shape{};
    mesh_shape.positions    = mesh.positions;

    // TODO(giacomo): Too many copies of positions.

    for (auto f = 0; f < mesh.num_triangles; f++) {
      if (contains(mesh.triangulated_faces, f)) {
        auto& new_faces = mesh.triangulated_faces.at(f);
        for (auto face : new_faces)
          hashgrid_shape.triangles.push_back(mesh.triangles[face]);
      } else {
        printf("Here\n");
        mesh_shape.triangles.push_back(mesh.triangles[f]);
      }
    }

    scene.shapes.push_back(hashgrid_shape);
    scene.shapes.push_back(mesh_shape);
  } else {
    for (int i = 0; i < state.cells.size(); i++) {
      auto& cell = state.cells[i];

      auto& instance    = scene.instances.emplace_back();
      instance.material = (int)scene.materials.size();
      auto& material    = scene.materials.emplace_back();

      if (cell_colors.size()) {
        material.color = cell_colors[i];
      } else {
        if (state.labels.size())
          material.color = get_cell_color(state, i, color_shapes);
        else
          material.color = vec3f{1.0f, 1.0f, 1.0f};
      }

      material.type      = scene_material_type::glossy;
      material.roughness = 0.5;
      instance.shape     = (int)scene.shapes.size();
      auto shape         = scene_shape{};

      // TODO(giacomo): Too many copies of positions.
      shape.positions = mesh.positions;
      for (auto face : cell.faces) {
        shape.triangles.push_back(mesh.triangles[face]);
      }

      scene.shapes.push_back(shape);
    }
  }

  if (save_edges) {
    auto& instance     = scene.instances.emplace_back();
    instance.shape     = (int)scene.shapes.size();
    instance.material  = (int)scene.materials.size();
    auto& material     = scene.materials.emplace_back();
    material.color     = {0.25, 0.25, 0.25};
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

  if (!state.bool_shapes.size()) {
    auto& instance    = scene.instances.emplace_back();
    instance.material = (int)scene.materials.size();

    auto& material     = scene.materials.emplace_back();
    material.color     = {0.5, 0.5, 0.5};
    material.type      = scene_material_type::glossy;
    material.roughness = 0.5;

    instance.shape = (int)scene.shapes.size();
    auto& shape    = scene.shapes.emplace_back();

    shape.positions = mesh.positions;
    shape.triangles = mesh.triangles;
  }

  if (save_polygons) {
    for (int s = 0; s < state.bool_shapes.size(); s++) {
      if (state.bool_shapes[s].polygons.empty()) continue;

      auto& bool_shape = state.bool_shapes[s];
      for (int i = 0; i < bool_shape.polygons.size(); i++) {
        auto& polygon   = bool_shape.polygons[i];
        auto  positions = vector<vec3f>();
        positions.reserve(polygon.length + 1);

        for (auto& edge : polygon.edges) {
          for (auto& segment : edge) {
            positions.push_back(
                eval_position(mesh, {segment.face, segment.start}));
          }
        }

        if (polygon.edges.size() && polygon.edges.back().size()) {
          auto& segment = polygon.edges.back().back();
          positions.push_back(eval_position(mesh, {segment.face, segment.end}));
        }

        if (positions.empty()) continue;

        auto& instance    = scene.instances.emplace_back();
        instance.material = (int)scene.materials.size();
        auto& material    = scene.materials.emplace_back();
        material.color    = get_color(s);
        material.type     = scene_material_type::matte;
        instance.shape    = (int)scene.shapes.size();

        auto shape = create_polygon_shape(positions, line_width);
        scene.shapes.push_back(shape);
      }
    }
  }

  return scene;
}

void export_model(
    const bool_state& state, const bool_mesh& mesh, const string& filename) {
  auto error = string{};
  auto model = obj_model{};

  auto obj = obj_shape{};
  add_positions(obj, mesh.positions);
  add_normals(obj, mesh.normals);

  for (int c = 0; c < state.cells.size(); c++) {
    auto& cell       = state.cells[c];
    auto  cell_color = get_cell_color(state, c, false);

    auto material    = obj_material();
    material.name    = to_string(c);
    material.diffuse = cell_color;
    model.materials.push_back(material);

    auto triangles = vector<vec3i>();
    for (auto& face : cell.faces) triangles.push_back(mesh.triangles[face]);
    add_triangles(
        obj, triangles, c, !mesh.normals.empty(), !mesh.texcoords.empty());
  }

  model.shapes.push_back(obj);
  save_obj(filename, model, error);
}

#include <yocto/yocto_color.h>

string tree_to_string(const bool_state& state, bool color_shapes) {
  auto&  cells  = state.cells;
  string result = "digraph {\n";
  result += "forcelabels=true\n";

  for (int i = 0; i < cells.size(); i++) {
    auto& cell  = cells[i];
    auto  color = get_cell_color(state, i, color_shapes);
    color       = rgb_to_hsv(color);
    char str[1024];
    auto label = string{};
    if (state.labels.empty())
      label = "";
    else {
      for (int k = 1; k < state.labels[i].size(); k++) {
        if (state.labels[i][k] == null_label) {
          label += "0 ";
          continue;
        }
        label += to_string(state.labels[i][k]) + " ";
      }
    }
    sprintf(str, "%d [label=\"%d\n%s\" style=filled fillcolor=\"%f %f %f\"]\n",
        i, i, label.c_str(), color.x, color.y, color.z);
    result += std::string(str);

    for (auto [neighbor, polygon] : cell.adjacency) {
      if (polygon < 0) continue;
      int  c     = neighbor;
      auto color = rgb_to_hsv(get_color(polygon));
      sprintf(str, "%d -> %d [ label=\"%d\" color=\"%f %f %f\"]\n", i, c,
          polygon, color.x, color.y, color.z);
      result += std::string(str);
    }
  }
  result += "}\n";
  return result;
}

void save_tree_png(const bool_state& state, string filename,
    const string& extra, bool color_shapes) {
  if (filename.empty()) filename = "data/tests/test.json";
  auto  graph = replace_extension(filename, extra + ".txt");
  FILE* file  = fopen(graph.c_str(), "w");
  fprintf(file, "%s", tree_to_string(state, color_shapes).c_str());
  fclose(file);

  auto image = replace_extension(filename, extra + ".png");
  auto cmd   = "dot -Tpng "s + graph + " > " + image;
  printf("%s\n", cmd.c_str());
  system(cmd.c_str());
  cmd = "rm "s + graph;
  system(cmd.c_str());
}

vector<Svg_Shape> load_svg(const string& filename) {
  struct NSVGimage* image;
  image = nsvgParseFromFile(filename.c_str(), "px", 96);
  printf("svg loaded, size: %f x %f\n", image->width, image->height);
  auto size = vec2f{image->width, image->height};

  auto svg = vector<Svg_Shape>{};
  for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
    auto& svg_shape = svg.emplace_back();

    unsigned int c;
    if (shape->fill.type == NSVG_PAINT_COLOR) {
      c = shape->fill.color;
    } else if (shape->fill.type >= NSVG_PAINT_LINEAR_GRADIENT) {
      c = shape->fill.gradient->stops[0].color;
    } else {
      c = 0;
    }
    float r         = ((c >> 16) & 0xFF) / 255.0;  // Extract the RR byte
    float g         = ((c >> 8) & 0xFF) / 255.0;   // Extract the GG byte
    float b         = ((c)&0xFF) / 255.0;
    svg_shape.color = yocto::pow(vec3f{b, g, r}, 2.2f);

    for (auto path = shape->paths; path != NULL; path = path->next) {
      auto& svg_path = svg_shape.paths.emplace_back();
      auto  area     = 0.0f;

      for (int i = 0; i < path->npts - 1; i += 3) {
        float* p     = &path->pts[i * 2];
        auto&  curve = svg_path.emplace_back();
        curve[0]     = vec2f{p[0], size.y - p[1]} / size.y;
        curve[1]     = vec2f{p[2], size.y - p[3]} / size.y;
        curve[2]     = vec2f{p[4], size.y - p[5]} / size.y;
        curve[3]     = vec2f{p[6], size.y - p[7]} / size.y;

        area += cross(curve[0], curve[1]);
        area += cross(curve[1], curve[2]);
        area += cross(curve[2], curve[3]);
        // printf("(%f %f) (%f %f) (%f %f) (%f %f)\n", curve[0].x, curve[0].y,
        //     curve[1].x, curve[1].y, curve[2].x, curve[2].y, curve[3].x,
        //     curve[3].y);
      }

      if (area < 0.0f) {
        // std::reverse(svg_shape.paths.begin(), svg_shape.paths.end());
        // for (auto& path : svg_shape.paths) {
        std::reverse(svg_path.begin(), svg_path.end());
        for (auto& curve : svg_path) {
          curve = {curve[3], curve[2], curve[1], curve[0]};
        }
        //}
      }
      printf("area: %f\n", area);
    }
  }

  nsvgDelete(image);
  return svg;
}

void init_from_svg(bool_state& state, const bool_mesh& mesh,
    const mesh_point& center, const vector<Svg_Shape>& svg, float svg_size,
    int svg_subdivs) {
  auto p0  = eval_position(mesh, {center.face, {0, 0}});
  auto p1  = eval_position(mesh, {center.face, {1, 0}});
  auto rot = mat2f{};
  {
    auto frame = mat3f{};
    frame.x    = normalize(p1 - p0);
    frame.z    = eval_normal(mesh, center.face);
    frame.y    = normalize(cross(frame.z, frame.x));

    auto up = vec3f{0, 1, 0};
    auto v  = normalize(vec2f{dot(up, frame.x), dot(up, frame.y)});
    rot     = mat2f{{v.x, v.y}, {-v.y, v.x}};
  }

  for (auto& shape : svg) {
    for (auto& path : shape.paths) {
      auto& polygon = state.polygons.emplace_back();

      auto control_points = vector<mesh_point>{};
      // polygon.center = center;
      // polygon.frame  = mat2f{rot, vec2f{-rot.y, rot.x}};
      // polygon.color  = shape.color;
      for (auto& segment : path) {
        // polygon.curves.push_back({});
        for (int i = 0; i < 3; i++) {
          // vec2f uv = clamp(segment[i], 0.0f, 1.0f);
          vec2f uv = segment[i];
          uv -= vec2f{0.5, 0.5};
          uv = rot * uv;
          uv *= svg_size;
          auto line = straightest_path(mesh, center, uv);
          control_points += line.end;
        }
      }
      auto bezier = compute_bezier_path(mesh.dual_solver, mesh.triangles,
          mesh.positions, mesh.adjacencies, control_points, svg_subdivs);

      for (int i = 0; i < bezier.size() - 1; i++) {
        if (i > 0 && bezier[i] == bezier[i - 1]) continue;
        polygon.points += (int)state.points.size();
        state.points += bezier[i];
      }
    }
  }
}
