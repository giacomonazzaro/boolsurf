#include <yocto/yocto_cli.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_mesh.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif
using namespace yocto;

struct Spline_Input {
  vector<mesh_point> control_points = {};
  vector<bool>       is_smooth      = {};
};
struct Spline_Output {
  vector<mesh_point> points = {};
};
struct Spline_Cache {
  vector<vec3f> positions = {};
  geodesic_path tangent0  = {};  // TODO(giacomo): use mesh_paths?
  geodesic_path tangent1  = {};  // TODO(giacomo): use mesh_paths?
};
struct Spline_Draw {
  int         shape_id         = {};  // id of shape in scene_data
  int         instance_id      = {};  // id of instance in scene_data
  vector<int> curves_to_update = {};
};
struct Spline_View {
  Spline_Input&  input;
  Spline_Output& ioutput;
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
  scene_data scene = {};
  shape_bvh  bvh   = {};
  float      time  = 0;

  Editing    editing = {};
  Splinesurf splinesurf;

  int add_spline() {
    auto id = (int)splinesurf.spline_input.size();
    splinesurf.spline_input.push_back({});
    splinesurf.spline_output.push_back({});
    splinesurf.spline_cache.push_back({});
    splinesurf.spline_draw.push_back({});
    return id;
  }

  Spline_View selected_spline_input() {
    if (splinesurf.spline_input.empty()) {
      add_spline();
      editing.selected_spline_id = 0;
    }
    auto id = editing.selected_spline_id;
    return Spline_View{splinesurf.spline_input[id],
        splinesurf.spline_output[id], splinesurf.spline_cache[id],
        splinesurf.spline_draw[id]};
  }
};

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
        auto spline = app.selected_spline_input();
        spline.input.control_points.push_back(point);
        printf("xxx: %ld\n", app.splinesurf.spline_input.size());
        printf(
            "cp: %ld\n", app.splinesurf.spline_input[0].control_points.size());
      }
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

void run_glview(const glview_params& params) {
  // loading shape
  auto error = string{};
  auto shape = shape_data{};
  if (!load_shape(params.shape, shape, error)) print_fatal(error);

  // make scene
  auto app  = App{};
  app.bvh   = make_triangles_bvh(shape.triangles, shape.positions, {});
  app.scene = make_shape_scene(shape, params.addsky);

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
