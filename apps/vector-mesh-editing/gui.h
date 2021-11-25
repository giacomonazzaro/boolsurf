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
