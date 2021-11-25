#pragma once
#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif

#include "app.h"

struct scene_selection {
  int camera      = 0;
  int instance    = 0;
  int environment = 0;
  int shape       = 0;
  int texture     = 0;
  int material    = 0;
  int subdiv      = 0;
};

// renderer update
struct Render {
  const scene_data&   scene;
  const bvh_data&     bvh;
  const trace_lights& lights;
  trace_params&       params;

  trace_state state   = {};
  image_data  image   = {};
  image_data  display = {};
  image_data  render  = {};

  // opengl image
  glimage_state  glimage  = {};
  glimage_params glparams = {};

  std::atomic<bool> render_update  = {};
  std::atomic<int>  render_current = {};
  std::mutex        render_mutex   = {};
  future<void>      render_worker  = {};
  atomic<bool>      render_stop    = {};

  Render(const scene_data& _scene, const bvh_data& _bvh,
      const trace_lights& _lights, trace_params& _params)
      : scene(_scene), bvh(_bvh), lights(_lights), params(_params) {
    state   = make_state(scene, params);
    image   = make_image(state.width, state.height, true);
    display = make_image(state.width, state.height, false);
    render  = make_image(state.width, state.height, true);
  }

  auto render_scene() {
    for (auto sample = 0; sample < params.samples; sample += params.batch) {
      if (render_stop) return;
      parallel_for(state.width, state.height, [&](int i, int j) {
        for (auto s = 0; s < params.batch; s++) {
          if (render_stop) return;
          trace_sample(state, scene, bvh, lights, i, j, params);
        }
      });
      state.samples += params.batch;
      if (!render_stop) {
        auto lock      = std::lock_guard{render_mutex};
        render_current = state.samples;
        if (!params.denoise || render_stop) {
          get_render(render, state);
        } else {
          get_denoised(render, state);
        }
        image = render;
        tonemap_image_mt(display, image, params.exposure, params.filmic);
        render_update = true;
      }
    }
  }

  auto reset_display() {
    // stop render
    render_stop = true;
    if (render_worker.valid()) render_worker.get();

    state   = make_state(scene, params);
    image   = make_image(state.width, state.height, true);
    display = make_image(state.width, state.height, false);
    render  = make_image(state.width, state.height, true);

    render_worker = {};
    render_stop   = false;

    // preview
    auto pparams = params;
    pparams.resolution /= params.pratio;
    pparams.samples = 1;
    auto pstate     = make_state(scene, pparams);
    trace_samples(pstate, scene, bvh, lights, pparams);
    auto preview = get_render(pstate);
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % render.width, j = idx / render.width;
      auto pi            = clamp(i / params.pratio, 0, preview.width - 1),
           pj            = clamp(j / params.pratio, 0, preview.height - 1);
      render.pixels[idx] = preview.pixels[pj * preview.width + pi];
    }
    // if (current > 0) return;
    {
      auto lock      = std::lock_guard{render_mutex};
      render_current = 0;
      image          = render;
      tonemap_image_mt(display, image, params.exposure, params.filmic);
      render_update = true;
    }

    // start renderer
    render_worker = std::async(
        std::launch::async, [this]() { this->render_scene(); });
  }

  // stop render
  void stop_render() {
    render_stop = true;
    if (render_worker.valid()) render_worker.get();
  }
};

static void update_image_params(const glinput_state& input,
    const image_data& image, glimage_params& glparams) {
  glparams.window                           = input.window_size;
  glparams.framebuffer                      = input.framebuffer_viewport;
  std::tie(glparams.center, glparams.scale) = camera_imview(glparams.center,
      glparams.scale, {image.width, image.height}, glparams.window,
      glparams.fit);
}

static bool uiupdate_image_params(
    const glinput_state& input, glimage_params& glparams) {
  // handle mouse
  if (input.mouse_left && input.modifier_alt && !input.widgets_active) {
    if (input.modifier_ctrl) {
      glparams.scale *= yocto::pow(
          2.0f, (input.mouse_pos.y - input.mouse_last.y) * 0.001f);
      return true;
    } else {
      glparams.center += input.mouse_pos - input.mouse_last;
      return true;
    }
  }
  return false;
}

inline void draw_widgets(App& app, Render& render, const glinput_state& input);

inline void update(
    App& app, Render& render, bvh_data& bvh, const glinput_state& input) {
  static auto updated_shapes = vector<int>{};

  auto& scene  = app.scene;
  auto& params = render.params;
  auto  camera = scene.cameras[params.camera];
  if (uiupdate_camera_params(input, camera)) {
    render.stop_render();
    scene.cameras[params.camera] = camera;
    render.reset_display();
  }

  {
    process_click(app, updated_shapes, input);
    process_mouse(app, updated_shapes, input);

    if (input.mouse_right_click) {
      render.stop_render();
      auto& state = app.bool_state;
      state       = {};
      auto& mesh  = app.mesh;
      for (int i = 0; i < app.splinesurf.num_splines(); i++) {
        auto spline = app.splinesurf.get_spline_view(i);

        // Add new 1-polygon shape to state
        // if (test_polygon.empty()) continue;

        auto& bool_shape = state.bool_shapes.emplace_back();
        auto& polygon    = bool_shape.polygons.emplace_back();
        //      polygon.points   = test_polygon;
        for (auto& anchor : spline.input.control_points) {
          polygon.points.push_back(
              {anchor.point, {anchor.handles[0], anchor.handles[1]}});
        }
        recompute_polygon_segments(mesh, polygon);
      }

      compute_cells(app.mesh, app.bool_state);
      //  compute_shapes(app.bool_state);
      app.mesh.triangles.resize(app.mesh.num_triangles);
      app.mesh.positions.resize(app.mesh.num_positions);
      render.reset_display();
    }

    if (app.jobs.size()) {
      render.stop_render();
      for (auto& job : app.jobs) job();
      app.jobs.clear();
      render.reset_display();
    }

    if (app.new_instances.size()) {
      render.stop_render();
      scene.shapes += app.new_shapes;
      scene.instances += app.new_instances;
      update_splines(app, scene, updated_shapes);
      updated_shapes.clear();
      app.new_shapes.clear();
      app.new_instances.clear();
      bvh = make_bvh(scene, params);
      render.reset_display();
    }
  }
}

// Open a window and show an scene via path tracing
void view_raytraced_scene(App& app, const string& title, const string& name,
    scene_data& scene, const trace_params& params_ = {}, bool print = true,
    bool edit = false) {
  // copy params and camera
  auto params = params_;

  // build bvh
  if (print) print_progress_begin("build bvh");
  auto bvh = make_bvh(scene, params);
  if (print) print_progress_end();

  // init renderer
  if (print) print_progress_begin("init lights");
  auto lights = make_lights(scene, params);
  if (print) print_progress_end();

  // fix renderer type if no lights
  if (lights.lights.empty() && is_sampler_lit(params)) {
    if (print) print_info("no lights presents --- switching to eyelight");
    params.sampler = trace_sampler_type::eyelight;
  }

  // init state
  if (print) print_progress_begin("init state");
  auto render = Render(scene, bvh, lights, params);
  if (print) print_progress_end();

  // start rendering
  render.reset_display();

  // prepare selection
  auto selection = scene_selection{};

  // callbacks
  auto callbacks    = glwindow_callbacks{};
  callbacks.init_cb = [&](const glinput_state& input) {
    auto lock = std::lock_guard{render.render_mutex};
    init_image(render.glimage);
    set_image(render.glimage, render.display);
  };
  callbacks.clear_cb = [&](const glinput_state& input) {
    clear_image(render.glimage);
  };
  callbacks.draw_cb = [&](const glinput_state& input) {
    // update image
    if (render.render_update) {
      auto lock = std::lock_guard{render.render_mutex};
      set_image(render.glimage, render.display);
      render.render_update = false;
    }
    update_image_params(input, render.image, render.glparams);
    draw_image(render.glimage, render.glparams);
  };
  callbacks.widgets_cb = [&](const glinput_state& input) {
    draw_widgets(app, render, input);
  };
  callbacks.uiupdate_cb = [&](const glinput_state& input) {
    update(app, render, bvh, input);
  };

  // run ui
  run_ui({1280 + 320, 720}, title, callbacks);

  // done
  render.stop_render();
}

static bool draw_image_inspector(const glinput_state& input,
    const image_data& image, const image_data& display,
    glimage_params& glparams) {
  if (begin_glheader("inspect")) {
    draw_glslider("zoom", glparams.scale, 0.1, 10);
    draw_glcheckbox("fit", glparams.fit);
    draw_glcoloredit("background", glparams.background);
    auto [i, j] = image_coords(input.mouse_pos, glparams.center, glparams.scale,
        {image.width, image.height});
    auto ij     = vec2i{i, j};
    draw_gldragger("mouse", ij);
    auto image_pixel   = zero4f;
    auto display_pixel = zero4f;
    if (i >= 0 && i < image.width && j >= 0 && j < image.height) {
      image_pixel   = image.pixels[j * image.width + i];
      display_pixel = image.pixels[j * image.width + i];
    }
    draw_glcoloredit("image", image_pixel);
    draw_glcoloredit("display", display_pixel);
    end_glheader();
  }
  return false;
}

static bool draw_scene_editor(scene_data& scene, scene_selection& selection,
    const function<void()>& before_edit) {
  auto edited = 0;
  if (begin_glheader("cameras")) {
    draw_glcombobox("camera", selection.camera, scene.camera_names);
    auto camera = scene.cameras.at(selection.camera);
    edited += draw_glcheckbox("ortho", camera.orthographic);
    edited += draw_glslider("lens", camera.lens, 0.001, 1);
    edited += draw_glslider("aspect", camera.aspect, 0.1, 5);
    edited += draw_glslider("film", camera.film, 0.1, 0.5);
    edited += draw_glslider("focus", camera.focus, 0.001, 100);
    edited += draw_glslider("aperture", camera.aperture, 0, 1);
    //   frame3f frame        = identity3x4f;
    if (edited) {
      if (before_edit) before_edit();
      scene.cameras.at(selection.camera) = camera;
    }
    end_glheader();
  }
  if (begin_glheader("environments")) {
    draw_glcombobox(
        "environment", selection.environment, scene.environment_names);
    auto environment = scene.environments.at(selection.environment);
    edited += draw_glcoloredithdr("emission", environment.emission);
    edited += draw_glcombobox(
        "emission_tex", environment.emission_tex, scene.texture_names, true);
    //   frame3f frame        = identity3x4f;
    if (edited) {
      if (before_edit) before_edit();
      scene.environments.at(selection.environment) = environment;
    }
    end_glheader();
  }
  if (begin_glheader("instances")) {
    draw_glcombobox("instance", selection.instance, scene.instance_names);
    auto instance = scene.instances.at(selection.instance);
    edited += draw_glcombobox("shape", instance.shape, scene.shape_names);
    edited += draw_glcombobox(
        "material", instance.material, scene.material_names);
    //   frame3f frame        = identity3x4f;
    if (edited) {
      if (before_edit) before_edit();
      scene.instances.at(selection.instance) = instance;
    }
    end_glheader();
  }
  if (begin_glheader("materials")) {
    draw_glcombobox("material", selection.material, scene.material_names);
    auto material = scene.materials.at(selection.material);
    edited += draw_glcoloredithdr("emission", material.emission);
    edited += draw_glcombobox(
        "emission_tex", material.emission_tex, scene.texture_names, true);
    edited += draw_glcoloredithdr("color", material.color);
    edited += draw_glcombobox(
        "color_tex", material.color_tex, scene.texture_names, true);
    edited += draw_glslider("roughness", material.roughness, 0, 1);
    edited += draw_glcombobox(
        "roughness_tex", material.roughness_tex, scene.texture_names, true);
    edited += draw_glslider("metallic", material.metallic, 0, 1);
    edited += draw_glslider("ior", material.ior, 0.1, 5);
    if (edited) {
      if (before_edit) before_edit();
      scene.materials.at(selection.material) = material;
    }
    end_glheader();
  }
  if (begin_glheader("shapes")) {
    draw_glcombobox("shape", selection.shape, scene.shape_names);
    auto& shape = scene.shapes.at(selection.shape);
    draw_gllabel("points", (int)shape.points.size());
    draw_gllabel("lines", (int)shape.lines.size());
    draw_gllabel("triangles", (int)shape.triangles.size());
    draw_gllabel("quads", (int)shape.quads.size());
    draw_gllabel("positions", (int)shape.positions.size());
    draw_gllabel("normals", (int)shape.normals.size());
    draw_gllabel("texcoords", (int)shape.texcoords.size());
    draw_gllabel("colors", (int)shape.colors.size());
    draw_gllabel("radius", (int)shape.radius.size());
    draw_gllabel("tangents", (int)shape.tangents.size());
    end_glheader();
  }
  if (begin_glheader("textures")) {
    draw_glcombobox("texture", selection.texture, scene.texture_names);
    auto& texture = scene.textures.at(selection.texture);
    draw_gllabel("width", texture.width);
    draw_gllabel("height", texture.height);
    draw_gllabel("linear", texture.linear);
    draw_gllabel("byte", !texture.pixelsb.empty());
    end_glheader();
  }
  if (begin_glheader("subdivs")) {
    draw_glcombobox("subdiv", selection.subdiv, scene.subdiv_names);
    auto& subdiv = scene.subdivs.at(selection.subdiv);
    draw_gllabel("quadspos", (int)subdiv.quadspos.size());
    draw_gllabel("quadsnorm", (int)subdiv.quadsnorm.size());
    draw_gllabel("quadstexcoord", (int)subdiv.quadstexcoord.size());
    draw_gllabel("positions", (int)subdiv.positions.size());
    draw_gllabel("normals", (int)subdiv.normals.size());
    draw_gllabel("texcoords", (int)subdiv.texcoords.size());
    end_glheader();
  }
  return (bool)edited;
}

inline void draw_widgets(App& app, Render& render, const glinput_state& input) {
  // auto names    = vector<string>{name};
  // auto selected = 0;
  auto edited = 0;

  //  draw_glcombobox("name", selected, names);
  auto current = (int)render.render_current;
  draw_glprogressbar("sample", current, render.params.samples);

  draw_gllabel("selected spline", app.editing.selection.spline_id);
  draw_gllabel(
      "selected control_point", app.editing.selection.control_point_id);
  if (draw_glbutton("add spline")) {
    auto spline_id                  = add_spline(app.splinesurf);
    app.editing.selection           = {};
    app.editing.selection.spline_id = spline_id;
  }
  if (draw_glbutton("close spline")) {
    auto add_app_shape = [&]() -> int { return add_shape(app, {}); };
    close_spline(app.selected_spline(), add_app_shape);
  }
  if (begin_glheader("render")) {
    auto edited  = 0;
    auto tparams = render.params;
    //    edited += draw_glcombobox("camera", tparams.camera, camera_names);
    edited += draw_glslider("resolution", tparams.resolution, 180, 4096);
    edited += draw_glslider("samples", tparams.samples, 16, 4096);
    edited += draw_glcombobox(
        "tracer", (int&)tparams.sampler, trace_sampler_names);
    edited += draw_glcombobox(
        "false color", (int&)tparams.falsecolor, trace_falsecolor_names);
    edited += draw_glslider("bounces", tparams.bounces, 1, 128);
    edited += draw_glslider("batch", tparams.batch, 1, 16);
    edited += draw_glslider("clamp", tparams.clamp, 10, 1000);
    edited += draw_glcheckbox("envhidden", tparams.envhidden);
    continue_glline();
    edited += draw_glcheckbox("filter", tparams.tentfilter);
    edited += draw_glslider("pratio", tparams.pratio, 1, 64);
    // edited += draw_glslider("exposure", tparams.exposure, -5, 5);
    end_glheader();
    if (edited) {
      render.stop_render();
      render.params = tparams;
      render.reset_display();
    }
  }
  auto& params = render.params;
  if (begin_glheader("tonemap")) {
    edited += draw_glslider("exposure", params.exposure, -5, 5);
    edited += draw_glcheckbox("filmic", params.filmic);
    edited += draw_glcheckbox("denoise", params.denoise);
    end_glheader();
    if (edited) {
      tonemap_image_mt(
          render.display, render.image, params.exposure, params.filmic);
      set_image(render.glimage, render.display);
    }
  }
}
