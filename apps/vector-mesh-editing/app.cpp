#include <yocto/yocto_cli.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_sceneio.h>
//
#include "app.h"
#include "render.h"
#include "utils.h"

#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif
using namespace yocto;

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
  view_raytraced_scene("yshape", params.shape, scene);
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

void update_output(Spline_Output& output, const Spline_Input& input,
    const App::Mesh& mesh, const hash_set<int>& curves_to_update) {
  if (curves_to_update.size()) {
    output.points = bezier_spline(
        mesh, input.control_points, input.num_subdivisions);
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
  if (cache.curves_to_update.size()) {
    cache.positions.resize(output.points.size());
    for (int i = 0; i < output.points.size(); i++) {
      cache.positions[i] = eval_position(
          mesh.triangles, mesh.positions, output.points[i]);
    }
  }

  for (auto& c : cache.curves_to_update) {
    auto  id             = cache.curve_shapes[c];
    auto& shape          = scene.shapes[id];
    auto  line_thickness = 0.005;
    shape         = polyline_to_cylinders(cache.positions, 8, line_thickness);
    shape.normals = compute_normals(shape);
    updated_shapes += cache.curve_shapes[c];
  }
  cache.curves_to_update.clear();
}

void run_app(App& app, const string& name, const glscene_params& params_,
    const app_callback& widgets_callback  = {},
    const app_callback& uiupdate_callback = {},
    const app_callback& update_callback   = {}) {
  // glscene
  auto  glscene = glscene_state{};
  auto& scene   = app.scene;

  auto add_shape = [](scene_data& scene, glscene_state& glscene,
                       const frame3f& frame = identity3x4f, int material = 1) {
    auto id = (int)scene.shapes.size();
    scene.shapes.push_back({});
    scene.instances.push_back({frame, id, material});
    glscene.shapes.push_back({});
    return id;
  };

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

        {
          auto frame = frame3f{};
          frame.o    = eval_position(
              app.mesh.triangles, app.mesh.positions, point);
          auto  shape_id = add_shape(scene, glscene, frame);
          auto& shape    = scene.shapes[shape_id];
          shape          = make_sphere(8, 0.005, 1);
          updated_shapes.push_back(shape_id);
        }

        printf(
            "cp: %ld\n", app.splinesurf.spline_input[0].control_points.size());
        if ((spline.input.control_points.size() - 1) % 3 == 0 &&
            spline.input.control_points.size() >= 4) {
          auto curve_id = (spline.input.control_points.size() - 1) / 3 - 1;
          spline.cache.curves_to_update.insert((int)curve_id);

          auto shape_id = add_shape(scene, glscene);
          spline.cache.curve_shapes.push_back(shape_id);
        }
      }
    }

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
  app.scene           = make_shape_scene(app.mesh, params.addsky);
  auto line_material  = material_data{};
  line_material.color = {1, 0, 0};
  app.scene.materials.push_back(line_material);
}

void run_glview(const glview_params& params) {
  auto app = App{};
  init_app(app, params);

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
