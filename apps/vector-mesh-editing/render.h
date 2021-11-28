#pragma once
#if YOCTO_OPENGL == 1
#include <yocto_gui/yocto_glview.h>
#endif

#include "app.h"

struct Render_View {
  const scene_data&   scene;
  const bvh_data&     bvh;
  const trace_lights& lights;
  trace_params&       params;
  Render_View(const scene_data& _scene, const bvh_data& _bvh,
      const trace_lights& _lights, trace_params& _params)
      : scene(_scene), bvh(_bvh), lights(_lights), params(_params) {}
};

// renderer update
struct Render {
  const scene_data*   scene_ptr;
  const bvh_data*     bvh_ptr;
  const trace_lights* lights_ptr;
  trace_params*       params_ptr;
  trace_state         state     = {};
  image_data          image     = {};
  image_data          display   = {};
  image_data          rendering = {};

  // opengl image
  glimage_state  glimage  = {};
  glimage_params glparams = {};

  std::atomic<bool> render_update  = {};
  std::atomic<int>  render_current = {};
  std::mutex        render_mutex   = {};
  future<void>      render_worker  = {};
  atomic<bool>      render_stop    = {};

  void init(const scene_data& scene, const bvh_data& bvh, const trace_lights& lights,
      trace_params& params) {
    scene_ptr  = &scene;
    bvh_ptr    = &bvh;
    lights_ptr = &lights;
    params_ptr = &params;
    restart();
  }

  void render() {
    const auto& scene  = *scene_ptr;
    const auto& bvh    = *bvh_ptr;
    const auto& lights = *lights_ptr;
    auto&       params = *params_ptr;

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
          get_render(rendering, state);
        } else {
          get_denoised(rendering, state);
        }
        image = rendering;
        tonemap_image_mt(display, image, params.exposure, params.filmic);
        render_update = true;
      }
    }
  }

  void render_async() {
    // start renderer
    render_worker = std::async(
        std::launch::async, [this]() { this->render(); });
  }

  // stop render
  void stop_render() {
    render_stop = true;
    if (render_worker.valid()) render_worker.get();
  }

  void reset() {
    state     = make_state(*scene_ptr, *params_ptr);
    image     = make_image(state.width, state.height, true);
    display   = make_image(state.width, state.height, false);
    rendering = make_image(state.width, state.height, true);
  }

  void restart() {
    stop_render();
    reset();

    render_worker = {};
    render_stop   = false;

    const auto& scene  = *scene_ptr;
    const auto& bvh    = *bvh_ptr;
    const auto& lights = *lights_ptr;
    auto&       params = *params_ptr;

    // render preview
    auto pparams = params;
    pparams.resolution /= params.pratio;
    pparams.samples = 1;
    auto pstate     = make_state(scene, pparams);
    trace_samples(pstate, scene, bvh, lights, pparams);
    auto preview = get_render(pstate);
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % rendering.width, j = idx / rendering.width;
      auto pi               = clamp(i / params.pratio, 0, preview.width - 1),
           pj               = clamp(j / params.pratio, 0, preview.height - 1);
      rendering.pixels[idx] = preview.pixels[pj * preview.width + pi];
    }
    // if (current > 0) return;
    {
      auto lock      = std::lock_guard{render_mutex};
      render_current = 0;
      image          = rendering;
      tonemap_image_mt(display, image, params.exposure, params.filmic);
      render_update = true;
    }

    render_async();
  }
};
