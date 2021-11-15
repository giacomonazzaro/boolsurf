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
  //  auto error = string{};
  //  auto shape = shape_data{};
  //  if (!load_shape(params.shape, shape, error, true)) print_fatal(error);
  //
  //  // make scene
  //  auto scene = make_shape_scene(shape, params.addsky);

  auto app = App{};
  init_app(app, params);

  // run view
  view_raytraced_scene(app, "yshape", params.shape, app.scene);
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

  // scene
  auto& new_instances = app.new_instances;
  auto& new_shapes    = app.new_shapes;

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

    process_click(app, updated_shapes, input);
    scene.shapes += new_shapes;
    scene.instances += new_instances;
    process_mouse(app, updated_shapes, input);

    for (auto& _ : new_shapes) {
      glscene.shapes.emplace_back();
    }

    update_splines(app, scene, updated_shapes);

    if (!updated_shapes.empty() || !updated_textures.empty()) {
      update_glscene(glscene, scene, updated_shapes, updated_textures);
      updated_shapes.clear();
      updated_textures.clear();
    }

    new_shapes.clear();
    new_instances.clear();
    // updated_shapes.clear();
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
