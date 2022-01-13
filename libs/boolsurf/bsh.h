#pragma once

//#include <yocto/yocto_sceneio.h>  // TODO(giacomo): Serve solo per funzioni inline temporanee

#include "boolsurf.h"  // TODO(giacomo): Serve solo per funzioni inline temporanee
#include "ext/json.hpp"
using namespace yocto;

namespace yocto {

using json = nlohmann::json;

inline void to_json(json& js, const BSH_patch& patch) {
  js["adj_cell_0"]  = patch.adj_cell_0;
  js["adj_cell_1"]  = patch.adj_cell_1;
  js["weight"]      = patch.weight;
  js["implicit_id"] = patch.shape_id;
  js["num_samples"] = patch.num_samples;
  js["adj_patches"] = patch.adj_patches;
}

inline void from_json(const json& js, BSH_patch& patch) {
  js.at("adj_cell_0").get_to(patch.adj_cell_0);
  js.at("adj_cell_1").get_to(patch.adj_cell_1);
  js.at("weight").get_to(patch.weight);
  js.at("implicit_id").get_to(patch.shape_id);
  js.at("num_samples").get_to(patch.num_samples);
  js.at("adj_patches").get_to(patch.adj_patches);
}
}  // namespace yocto

inline bool save_bsh(const BSH_graph& graph, const string& filename) {
  auto js         = json{};
  js["num_cells"] = graph.num_cells;
  js["patches"]   = graph.patches;

  auto error = ""s;
  if (!save_text(filename, js.dump(2), error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  return true;
}

inline bool bsh_load_json(const string& filename, json& js) {
  // error helpers
  auto error = ""s;
  auto text  = ""s;
  if (!load_text(filename, text, error)) {
    printf("[%s]: %s\n", __FUNCTION__, error.c_str());
    return false;
  }
  try {
    js = json::parse(text);
    return true;
  } catch (std::exception& e) {
    printf("[%s]: %s\n", __FUNCTION__, e.what());
    return false;
  }
}

inline bool load_bsh_result(vector<bool>& labels, const string& filename) {
  auto js = json{};
  if (!bsh_load_json(filename, js)) {
    return false;
  }

  if (js.find("labels") != js.end()) {
    labels = js["labels"].get<vector<bool>>();
  } else {
    printf("[%s]: no labels found!\n", __FUNCTION__);
  }

  printf("[%s]: labels\n", __FUNCTION__);
  for (auto l : labels) {
    printf("%d ", (int)l);
  }
  printf("\n");

  return true;
}
