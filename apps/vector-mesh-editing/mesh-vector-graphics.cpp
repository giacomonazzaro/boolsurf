#include <yocto/yocto_cli.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_sceneio.h>
//
#include <boolsurf/bsh.h>
#include <yocto_gui/yocto_shade.h>

#include "app.h"
#include "render.h"

using namespace yocto;

struct glview_params {
  string shape            = "shape.ply";
  string svg              = "";
  string envlight_texture = "boolsurf/scenes/doge2.hdr";
  float  line_thickness   = 0.001;
  bool   envlight         = false;

  string ao_output      = "";
  int    ao_num_samples = 1;
  int    gui_width      = 320;
};

// Cli
void add_options(const cli_command& cli, glview_params& params) {
  add_argument(cli, "shape", params.shape, "Input shape.");
  add_option(cli, "svg", params.svg, "Load svg.");
  add_option(cli, "line-thickness", params.line_thickness, "Line thickness.");
  add_option(cli, "envlight", params.envlight, "Environment lighting.");
  add_option(cli, "envlight-texture", params.envlight_texture,
      "Environment lighting texture.");
  add_option(cli, "ao-output", params.ao_output,
      "Filename of output shape with baked ambient occlusion.");
  add_option(cli, "ao-samples", params.ao_num_samples,
      "Number of samples for baked ao.");
  add_option(cli, "gui-width", params.gui_width, "Gui width.");
}

// view shapes
void run_view(const glview_params& params) {
  auto app = App{};
  init_app(app, params);
  view_raytraced_scene(app, "yshape", params.shape, app.scene);
}

void update_glscene(shade_scene& glscene, const scene_data& scene,
    const hash_set<int>& updated_shapes) {
  PROFILE();
  for (auto shape_id : updated_shapes) {
    if(shape_id == -1) continue;
    set_shape(glscene.shapes.at(shape_id), scene.shapes[shape_id]);
  }
  // TODO(giacomo): Update textures.
}

using app_callback = std::function<void(const glinput_state& input, App& app)>;

void run_app(App& app) {
  auto& glscene = app.glscene;
  auto& scene   = app.scene;

  // draw params
  auto& params = app.shade_params;
  if (scene.environments.size()) {
    params.lighting         = shade_lighting_type::envlight;
    params.hide_environment = true;
  }

  // camera names
  auto camera_names = scene.camera_names;
  if (camera_names.empty()) {
    for (auto idx = 0; idx < (int)scene.cameras.size(); idx++) {
      camera_names.push_back("camera" + std::to_string(idx + 1));
    }
  }

  // scene
  auto& new_shapes = app.new_shapes;

  // callbacks
  auto callbacks    = glwindow_callbacks{};
  callbacks.init_cb = [&](const glinput_state& input) {
    init_scene(glscene, scene, false, true);
  };
  callbacks.clear_cb = [&](const glinput_state& input) {
    clear_scene(glscene);
  };
  callbacks.draw_cb = [&](const glinput_state& input) {
    draw_scene(glscene, scene, input.framebuffer_viewport, params);
  };

  // top level combo
  auto selected        = 0;
  callbacks.widgets_cb = [&](const glinput_state& input) {
    draw_gllabel("frame time (ms)", app.frame_time_ms);
    draw_glcheckbox("flag", app.flag);
    if (draw_glcheckbox("splines visibility", app.are_splines_visible)) {
      toggle_splines_visibility(app);
    }

    auto edited = 0;
    edited += draw_glcheckbox("preview", app.preview_booleans);

    auto op = (int)app.bool_operation.type;
    edited += draw_glcombobox("boolean", op, bool_operation::type_names);
    app.bool_operation.type = (bool_operation::Type)op;

    {
      auto names = vector<string>{};
      for (int i = 0; i < app.shapes.size(); i++) {
        names.push_back(std::to_string(i));
      }
      if (names.size() >= 2) {
        edited += draw_glcombobox("a", app.bool_operation.shape_a, names);
        edited += draw_glcombobox("b", app.bool_operation.shape_b, names);
      }
    }

    if (edited) update_boolsurf(app);

    if (draw_glslider(
            "patch-id", app.patch_id, 0, (int)app.bsh_input.patches.size())) {
      update_boolsurf(app);
    }

    static auto svg_size = 0.1f;
    draw_glslider("svg size", svg_size, 0, 1);
    static auto svg_rotation = 0.1f;
    draw_glslider("svg rotation", svg_rotation, 0, 360);

    if (draw_glbutton("Load SVG")) {
      auto svg    = load_svg(app.svg_filename);
      auto center = intersect_mesh(app, vec2f{0.5, 0.5});
      auto angle  = radians(svg_rotation);
      init_from_svg(
          app, app.splinesurf, app.mesh, center, svg, svg_size, angle, 4);
      update_all_splines(app);
    }
    if (draw_glbutton("Save Scene")) {
      auto scene_dir = "boolsurf/scenes";

      auto error = string{};
      if (!make_directory(scene_dir, error)) {
        printf("%s\n", error.c_str());
      }
      if (!make_directory(path_join(scene_dir, "shapes"), error)) {
        printf("%s\n", error.c_str());
      }
      if (!make_directory(path_join(scene_dir, "textures"), error)) {
        printf("%s\n", error.c_str());
      }
      if (scene.environments.empty()) {
        add_environment(scene, app.envlight_texture, error);
      }
      auto scene_path = path_join(scene_dir, "scene.json");
      auto result     = save_scene(scene_path, app.scene);

      if (!result) {
        printf("[Save Scene]: %s\n", result.error.c_str());
      } else {
        printf("[Save Scene]: Success!\n");
      }
    }
    if (draw_glbutton("Add Shape")) {
      auto spline_id = add_spline(app.splinesurf);
      set_selected_spline(app, spline_id);
    }

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
      draw_glslider(
          "ambient roughness", app.ambient_shape_roughness, 0.0f, 1.0f);
      draw_glslider("shapes roughness", app.shapes_roughness, 0.0f, 1.0f);
      draw_glcoloredit("background", params.background);
      end_glheader();
    }
  };

  callbacks.uiupdate_cb = [&](const glinput_state& input) {
    auto timer = simple_timer();
    start_timer(timer);

    auto camera = scene.cameras.at(params.camera);
    if (uiupdate_camera_params(input, camera)) {
      scene.cameras.at(params.camera) = camera;
    }

    int edited = 0;
    edited += process_key(app, input);

    process_click(app, app.updated_shapes, input);
    process_mouse(app, app.updated_shapes, input);

    for (auto& entry : app.new_shapes) {
      glscene.shapes.resize(max((int)glscene.shapes.size(), entry.id + 1));
    }
    add_new_shapes(app);
    new_shapes.clear();

    edited += update_splines(app, scene, app.updated_shapes);
    if (edited) {
      update_boolsurf(app);
    }

    for (auto& entry : app.new_shapes) {
      glscene.shapes.resize(max((int)glscene.shapes.size(), entry.id + 1));
    }
    add_new_shapes(app);
    new_shapes.clear();

    if (!app.updated_shapes.empty()) {
      update_glscene(glscene, scene, app.updated_shapes);
      app.updated_shapes.clear();
    }

    // printf("mouse %f %f \n", input.mouse_pos.x, input.mouse_pos.y);
    // printf("window %d %d \n", input.window_size.x, input.window_size.y);
    // printf("fb %d %d %d %d\n", input.framebuffer_viewport.x,
    //     input.framebuffer_viewport.y, input.framebuffer_viewport.z,
    //     input.framebuffer_viewport.w);
    // auto mouse_uv = vec2f{input.mouse_pos.x / float(input.window_size.x),
    //     input.mouse_pos.y / float(input.window_size.y)};
    // printf("mouse_uv %f %f \n\n", mouse_uv.x, mouse_uv.y);
    // app.scene.cameras[0].aspect = input.window_size.x / input.window_size.y;
    {
      auto& camera  = app.scene.cameras[0];
      auto  w       = float(input.window_size.x);
      auto  h       = float(input.window_size.y);
      camera.film   = (0.036f * w) / 1280;
      camera.aspect = w / h;
    }
    app.frame_time_ms = elapsed_seconds(timer) * 1000;
  };
  // run ui
  run_ui({1280 + app.widgets_width, 720}, "Boolsurf", callbacks,
      app.widgets_width, true);
}

// Main
int main(int argc, const char* argv[]) {
  auto args   = make_cli_args(argc, argv);
  auto error  = string{};
  auto params = glview_params{};
  auto cli    = make_cli("yshape", params, "Process and view shapes.");
  if (!parse_cli(cli, args, error)) print_fatal(error);

  auto app = App{};
  init_app(app, params);

  // run viewer
  run_app(app);
}
