#include <yocto/yocto_cli.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>

#include <unordered_set>
template <typename Key, typename Value>
using hash_map = std::unordered_map<Key, Value>;

template <typename Key>
using hash_set = std::unordered_set<Key>;

#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif
using namespace yocto;

// Vector append and concatenation
template <typename T>
inline void operator+=(vector<T>& a, const vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}
template <typename T>
inline void operator+=(vector<T>& a, const T& b) {
  a.push_back(b);
}

struct Spline_Input {
  vector<mesh_point> control_points   = {};
  vector<bool>       is_smooth        = {};
  int                num_subdivisions = 4;

  int num_curves() const {
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
  hash_set<int> curves_to_update = {};
};
struct Spline_Draw {
  vector<int>   curve_shapes     = {};  // id of shape in scene_data
  hash_set<int> curves_to_update = {};
};
struct Spline_View {
  Spline_Input&  input;
  Spline_Output& output;
  Spline_Cache&  cache;
  Spline_Draw&   draw;
};

struct Splinesurf {
  vector<Spline_Input>  spline_input  = {};
  vector<Spline_Output> spline_output = {};
  vector<Spline_Cache>  spline_cache  = {};
  vector<Spline_Draw>   spline_draw   = {};
};

struct Editing {
  int selected_spline_id = -1;
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

  Editing                 editing = {};
  Splinesurf              splinesurf;
  std::unordered_set<int> splines_to_update = {};

  int add_spline() {
    auto id = (int)splinesurf.spline_input.size();
    splinesurf.spline_input.push_back({});
    splinesurf.spline_output.push_back({});
    splinesurf.spline_cache.push_back({});
    splinesurf.spline_draw.push_back({});
    return id;
  }

  Spline_View get_spline_view(int id) {
    return Spline_View{splinesurf.spline_input[id],
        splinesurf.spline_output[id], splinesurf.spline_cache[id],
        splinesurf.spline_draw[id]};
  }

  Spline_View selected_spline() {
    if (splinesurf.spline_input.empty()) {
      add_spline();
      editing.selected_spline_id = 0;
    }
    return get_spline_view(editing.selected_spline_id);
  }
};

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

// view params
struct view_params {
  string shape  = "shape.ply";
  string output = "out.ply";
  bool   addsky = false;
};

void add_options(const cli_command& cli, view_params& params) {
  add_argument(cli, "shape", params.shape, "Input shape.");
  add_option(cli, "output", params.output, "Output shape.");
  add_option(cli, "addsky", params.addsky, "Add sky.");
}

#ifndef YOCTO_OPENGL

// view shapes
void run_view(const view_params& params) { print_fatal("Opengl not compiled"); }

#else

// inline shape_data make_polygon_shape(
//    const App::Mesh& mesh, const vector<vec3f>& positions, float line_width) {
//  auto shape = shape_data{};
//
//  {
//    auto froms = vector<vec3f>();
//    auto tos   = vector<vec3f>();
//    froms.reserve(positions.size() - 1);
//    tos.reserve(positions.size() - 1);
//    for (int i = 0; i < positions.size() - 1; i++) {
//      auto from = positions[i];
//      auto to   = positions[i + 1];
//      if (from == to) continue;
//      froms.push_back(from);
//      tos.push_back(to);
//    }
//
//    auto cylinder = make_uvcylinder({16, 1, 1}, {line_width, 1});
//    for (auto& p : cylinder.positions) {
//      p.z = p.z * 0.5 + 0.5;
//    }
//
//    shape.quads     = cylinder.quads;
//    shape.positions = cylinder.positions;
//    shape.normals   = cylinder.normals;
//    shape.froms     = froms;
//    shape.tos       = tos;
//  }
//  return shape;
//}

// view shapes
void run_view(const view_params& params) {
  // load shape
  auto error = string{};
  auto shape = shape_data{};
  if (!load_shape(params.shape, shape, error, true)) print_fatal(error);

  // make scene
  auto scene = make_shape_scene(shape, params.addsky);

  // run view
  view_scene("yshape", params.shape, scene);
}

#endif

struct glview_params {
  string shape  = "shape.ply";
  bool   addsky = false;
};

// Cli
void add_options(const cli_command& cli, glview_params& params) {
  add_argument(cli, "shape", params.shape, "Input shape.");
  add_option(cli, "addsky", params.addsky, "Add sky.");
}

#ifndef YOCTO_OPENGL

// view shapes
void run_glview(const glview_params& params) {
  print_fatal("Opengl not compiled");
}

#else

using app_callback = std::function<void(const glinput_state& input, App& app)>;

mesh_point intersect_mesh(App& app, const glinput_state& input) {
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
    printf("point: %d, %f %f\n", isec.element, isec.uv.x, isec.uv.y);
    // updated = true;
    // }
    return mesh_point{isec.element, isec.uv};
  }
  return {};
}

void run_app(App& app, const string& name, const glscene_params& params_,
    const app_callback& widgets_callback  = {},
    const app_callback& uiupdate_callback = {},
    const app_callback& update_callback   = {}) {
  // glscene
  auto  glscene = glscene_state{};
  auto& scene   = app.scene;

  // draw params
  auto params = params_;

  // top level combo
  auto names    = vector<string>{name};
  auto selected = 0;

  // camera names
  auto camera_names = scene.camera_names;
  if (camera_names.empty()) {
    for (auto idx = 0; idx < (int)scene.cameras.size(); idx++) {
      camera_names.push_back("camera" + std::to_string(idx + 1));
    }
  }

  // gpu updates
  auto updated_shapes   = vector<int>{};
  auto updated_textures = vector<int>{};

  // callbacks
  auto callbacks    = glwindow_callbacks{};
  callbacks.init_cb = [&](const glinput_state& input) {
    init_glscene(glscene, scene);
  };
  callbacks.clear_cb = [&](const glinput_state& input) {
    clear_scene(glscene);
  };
  callbacks.draw_cb = [&](const glinput_state& input) {
    draw_scene(glscene, scene, input.framebuffer_viewport, params);
  };
  callbacks.widgets_cb = [&](const glinput_state& input) {
    draw_glcombobox("name", selected, names);
    if (begin_glheader("shade")) {
      draw_glcombobox("camera", params.camera, camera_names);
      draw_glcheckbox("wireframe", params.wireframe);
      continue_glline();
      draw_glcheckbox("faceted", params.faceted);
      continue_glline();
      draw_glcheckbox("double sided", params.double_sided);
      draw_glcombobox(
          "lighting", (int&)params.lighting, glscene_lighting_names);
      draw_glslider("exposure", params.exposure, -10, 10);
      draw_glslider("gamma", params.gamma, 0.1f, 4);
      draw_glslider("near", params.near, 0.01f, 1.0f);
      draw_glslider("far", params.far, 1000.0f, 10000.0f);
      draw_glcoloredit("background", params.background);
      end_glheader();
    }
    // draw_scene_editor(scene, selection, {});
    if (widgets_callback) {
      widgets_callback(input, app);
      if (!updated_shapes.empty() || !updated_textures.empty()) {
        update_glscene(glscene, scene, updated_shapes, updated_textures);
        updated_shapes.clear();
        updated_textures.clear();
      }
    }
  };
  callbacks.update_cb = [&](const glinput_state& input) {
    if (update_callback) {
      update_callback(input, app);
      if (!updated_shapes.empty() || !updated_textures.empty()) {
        update_glscene(glscene, scene, updated_shapes, updated_textures);
        updated_shapes.clear();
        updated_textures.clear();
      }
    }
  };
  callbacks.uiupdate_cb = [&](const glinput_state& input) {
    // handle mouse and keyboard for navigation
    if (uiupdate_callback) {
      uiupdate_callback(input, app);
      if (!updated_shapes.empty() || !updated_textures.empty()) {
        update_glscene(glscene, scene, updated_shapes, updated_textures);
        updated_shapes.clear();
        updated_textures.clear();
      }
    }
    auto camera = scene.cameras.at(params.camera);
    if (uiupdate_camera_params(input, camera)) {
      scene.cameras.at(params.camera) = camera;
    }

    if (input.mouse_left_click) {
      auto point = intersect_mesh(app, input);
      if (point.face != -1) {
        auto spline = app.selected_spline();
        spline.input.control_points.push_back(point);

        printf(
            "cp: %ld\n", app.splinesurf.spline_input[0].control_points.size());
        if ((spline.input.control_points.size() - 1) % 3 == 0 &&
            spline.input.control_points.size() >= 4) {
          auto curve_id = (spline.input.control_points.size() - 1) / 3 - 1;
          spline.cache.curves_to_update.insert((int)curve_id);
          auto shape_id = (int)scene.shapes.size();
          spline.draw.curve_shapes.push_back(shape_id);
          scene.shapes.push_back({});
          scene.instances.push_back({{}, shape_id, 1});
          glscene.shapes.push_back({});
        }
      }
    }

    for (int i = 0; i < app.splinesurf.spline_input.size(); i++) {
      auto spline = app.get_spline_view(i);
      if (spline.cache.curves_to_update.size()) {
        spline.output.points = bezier_spline(app.mesh,
            spline.input.control_points, spline.input.num_subdivisions);

        spline.cache.positions.resize(spline.output.points.size());
        for (int i = 0; i < spline.output.points.size(); i++) {
          spline.cache.positions[i] = eval_position(
              app.mesh.triangles, app.mesh.positions, spline.output.points[i]);
        }
      }
      spline.draw.curves_to_update = spline.cache.curves_to_update;
      spline.cache.curves_to_update.clear();
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

    for (int i = 0; i < app.splinesurf.spline_input.size(); i++) {
      auto spline = app.get_spline_view(i);
      for (auto& c : spline.draw.curves_to_update) {
        auto  id        = spline.draw.curve_shapes[c];
        auto& shape     = scene.shapes[id];
        shape.positions = spline.cache.positions;
        shape.lines.resize(spline.cache.positions.size() - 1);
        for (int i = 0; i < spline.cache.positions.size() - 1; i++) {
          shape.lines[i] = {i, i + 1};
        }
        updated_shapes += spline.draw.curve_shapes[c];
      }
      spline.draw.curves_to_update.clear();
    }
    // update_glscene(glscene, scene, updated_shapes, updated_textures);
  };
  // run ui
  run_ui({1280 + 320, 720}, "yshade", callbacks);
}

void widgets_callback(const glinput_state& input, App& app) {
  auto time = app.time;
  // if (input.widgets_active) {
  // printf("time: %f, function: %s\n", time, __FUNCTION__);
  // }
}

void update_callback(const glinput_state& input, App& app) {
  app.time =
      std::chrono::high_resolution_clock::now().time_since_epoch().count() *
      1e-9;
};

void run_glview(const glview_params& params) {
  // loading shape
  auto error = string{};

  auto app = App{};
  if (!load_shape(params.shape, app.mesh, error)) print_fatal(error);

  // make scene
  app.mesh.adjacencies = face_adjacencies(app.mesh.triangles);
  app.mesh.dual_solver = make_dual_geodesic_solver(
      app.mesh.triangles, app.mesh.positions, app.mesh.adjacencies);
  app.bvh   = make_triangles_bvh(app.mesh.triangles, app.mesh.positions, {});
  app.scene = make_shape_scene(app.mesh, params.addsky);
  auto line_material  = material_data{};
  line_material.color = {1, 1, 1};
  app.scene.materials.push_back(line_material);

  // run viewer
  run_app(app, "yshape", {}, widgets_callback, update_callback);
  // , update_callback);
}

#endif

struct app_params {
  string        command = "view";
  view_params   view    = {};
  glview_params glview  = {};
};

// Cli
void add_options(const cli_command& cli, app_params& params) {
  set_command_var(cli, params.command);
  // add_command(cli, "convert", params.convert, "Convert shapes.");
  // add_command(
  //     cli, "fvconvert", params.fvconvert, "Convert face-varying shapes.");
  add_command(cli, "view", params.view, "View shapes.");
  // add_command(cli, "heightfield", params.heightfield, "Create an
  // heightfield."); add_command(cli, "hair", params.hair, "Grow hairs on a
  // shape."); add_command(cli, "sample", params.sample, "Sample shapepoints on
  // a shape.");
  add_command(cli, "glview", params.glview, "View shapes with OpenGL.");
}

// Run
void run(const vector<string>& args) {
  // command line parameters
  auto error  = string{};
  auto params = app_params{};
  auto cli    = make_cli("yshape", params, "Process and view shapes.");
  if (!parse_cli(cli, args, error)) print_fatal(error);

  // dispatch commands
  if (params.command == "view") {
    return run_view(params.view);
  } else if (params.command == "glview") {
    return run_glview(params.glview);
  } else {
    print_fatal("yshape: unknown command");
  }
}

// Main
int main(int argc, const char* argv[]) { run(make_cli_args(argc, argv)); }
