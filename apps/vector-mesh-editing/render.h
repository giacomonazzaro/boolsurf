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
