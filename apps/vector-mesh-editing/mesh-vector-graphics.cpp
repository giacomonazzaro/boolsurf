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
  string envlight_texture = "boolsurf/scenes/uffizi.hdr";
  float  line_thickness   = 0.001;
  bool   envlight         = false;

  string ao_output      = "";
  int    ao_num_samples = 1;
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
    set_shape(glscene.shapes.at(shape_id), scene.shapes[shape_id]);
  }
  // TODO(giacomo): Update textures.
}

using app_callback = std::function<void(const glinput_state& input, App& app)>;

void run_app(App& app) {
  auto& glscene = app.glscene;
  auto& scene   = app.scene;

  // draw params
  auto params = shade_params{};
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
      update_all_splines(app);
      for (int i = 0; i < app.splinesurf.num_splines(); i++) {
        auto spline = app.get_spline_view(i);
        for (auto& curve : spline.cache.curves) {
          app.scene.instances[curve.shape_id].visible = app.are_splines_visible;
        }
      }
    }

    if (draw_glslider(
            "patch-id", app.patch_id, 0, (int)app.bsh_input.patches.size())) {
      update_boolsurf(app, input);
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
      draw_glcoloredit("background", params.background);
      end_glheader();
    }
  };

  callbacks.uiupdate_cb = [&](const glinput_state& input) {
    auto timer = simple_timer();
    start_timer(timer);

    if (app.envlight) params.exposure = -1;

    auto camera = scene.cameras.at(params.camera);
    if (uiupdate_camera_params(input, camera)) {
      scene.cameras.at(params.camera) = camera;
    }

    if (input.modifier_ctrl && input.modifier_shift &&
        app.selected_spline().input.control_points.size() > 1) {
      auto spline_id = add_spline(app.splinesurf);
      set_selected_spline(app, spline_id);
    }

    if (input.key_pressed[(int)gui_key::enter]) {
      auto add_app_shape = [&]() -> int { return add_shape(app, {}, {}, 1); };
      insert_anchor_points(app.splinesurf, app.mesh,
          app.bool_state.intersections, add_app_shape);
      update_all_splines(app);
    }

    process_click(app, app.updated_shapes, input);
    process_mouse(app, app.updated_shapes, input);

    for (auto& entry : app.new_shapes) {
      glscene.shapes.resize(max((int)glscene.shapes.size(), entry.id + 1));
    }
    add_new_shapes(app);
    new_shapes.clear();

    auto edited = update_splines(app, scene, app.updated_shapes);
    if (edited) {
      update_boolsurf(app, input);
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

    app.frame_time_ms = elapsed_seconds(timer) * 1000;
  };
  // run ui
  run_ui({1280 + 320, 720}, "yshade", callbacks);
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
