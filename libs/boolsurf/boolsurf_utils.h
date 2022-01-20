#pragma once

#ifdef _WIN32
#undef near
#undef far
#endif

#include "../yocto/yocto_mesh.h"
#include "../yocto/yocto_scene.h"
#include "../yocto/yocto_shape.h"  // hashing vec2i
//
#include <cassert>
#include <deque>
namespace yocto {
struct ogl_texture;
}

using namespace yocto;

#define _PRINT_CALL(function, file, line) \
  printf("%s() at %s, line %d\n", function, file, line)

#define PRINT_CALL() _PRINT_CALL(__FUNCTION__, __FILE__, __LINE__)

inline int mod3(int i) { return (i > 2) ? i - 3 : i; }

#if 1
#include "ext/robin_hood.h"
template <typename Key, typename Value>
using hash_map = robin_hood::unordered_flat_map<Key, Value>;

template <typename Key>
using hash_set = robin_hood::unordered_flat_set<Key>;
#else

#include <unordered_set>
template <typename Key, typename Value>
using hash_map = std::unordered_map<Key, Value>;

template <typename Key>
using hash_set = std::unordered_set<Key>;
#endif

// Vector append and concatenation
template <typename T>
inline void operator+=(vector<T>& a, const vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}
template <typename T>
inline void operator+=(vector<T>& a, const T& b) {
  a.push_back(b);
}
template <typename T>
inline vector<T> operator+(const vector<T>& a, const vector<T>& b) {
  auto c = a;
  c += b;
  return c;
}
template <typename T>
inline void operator+=(hash_set<T>& a, const T& b) {
  a.insert(b);
}

template <typename T>
inline void insert(vector<T>& vec, size_t i, const T& x) {
  vec.insert(vec.begin() + i, x);
}

// namespace std {
// size_t hash(const vector<int>& V) {
//   auto h = V.size();
//   for (auto& i : V) {
//     h ^= i + 0x9e3779b9 + (h << 6) + (h >> 2);
//   }
//   return h;
// }
// }  // namespace std

// TODO(giacomo): Expose this function in yocto_mesh.h
inline int find_in_vec(const vec3i& vec, int x) {
  for (auto i = 0; i < 3; i++)
    if (vec[i] == x) return i;
  return -1;
}

template <class T>
inline int find_idx(const vector<T>& vec, const T& x) {
  for (auto i = 0; i < vec.size(); i++)
    if (vec[i] == x) return i;
  return -1;
}

// TODO(gicomo): rename
// (marzia): check name
template <class T, typename F>
inline int find_where(const vector<T>& vec, F&& f) {
  for (auto i = 0; i < vec.size(); i++)
    if (f(vec[i])) return i;
  return -1;
}

// TODO(giacomo): Expose this function in yocto_mesh.h
inline int find_adjacent_triangle(
    const vec3i& triangle, const vec3i& adjacent) {
  for (int i = 0; i < 3; i++) {
    auto k = find_in_vec(adjacent, triangle[i]);
    if (k != -1) {
      if (find_in_vec(adjacent, triangle[mod3(i + 1)]) != -1) {
        return i;
      } else {
        return mod3(i + 2);
      }
    }
  }
  // assert(0 && "input triangles are not adjacent");
  return -1;
}

// From yocto_mesh.h + small update
inline vec2f intersect_segments(const vec2f& start1, const vec2f& end1,
    const vec2f& start2, const vec2f& end2) {
  if (end1 == start2) return zero2f;
  if (end2 == start1) return one2f;
  if (start2 == start1) return zero2f;
  if (end2 == end1) return one2f;

  auto a = end1 - start1;    // direction of line a
  auto b = start2 - end2;    // direction of line b, reversed
  auto d = start2 - start1;  // right-hand side

  auto det = a.x * b.y - a.y * b.x;
  if (det == 0) return {-1, -1};

  auto r = (d.x * b.y - d.y * b.x) / det;
  auto s = (a.x * d.y - a.y * d.x) / det;
  return {r, s};
}

inline vec2i make_edge_key(const vec2i& edge) {
  if (edge.x > edge.y) return {edge.y, edge.x};
  return edge;
};

inline vec2i get_mesh_edge_from_index(const vec3i& triangle, int k) {
  if (k == 0) return {triangle.x, triangle.y};
  if (k == 1) return {triangle.y, triangle.z};
  if (k == 2) return {triangle.z, triangle.x};

  assert(0);
  return {-1, -1};
}

inline vec2i get_triangle_edge_from_index(int k) {
  if (k == 0) return {0, 1};
  if (k == 1) return {1, 2};
  if (k == 2) return {2, 0};

  assert(0);
  return {-1, -1};
}

inline vec2i get_edge_from_uv(const vec2f& uv) {
  if (uv.y == 0) return {0, 1};
  if (fabs(uv.x + uv.y - 1.0f) < 0.00001)
    return {1, 2};  // (marzia): cambiata epsilon, occhio!
  if (uv.x == 0) return {2, 0};

  assert(0);
  return {-1, -1};
};

inline pair<int, float> get_edge_lerp_from_uv(const vec2f& uv) {
  if (uv.y == 0) return {0, uv.x};
  if (uv.x == 0) return {2, 1.0f - uv.y};
  if (fabs(uv.x + uv.y - 1.0f) < 0.00001) return {1, uv.y};

  return {-1, -1};
}

template <typename T>
void smooth_out_signal(vector<T>& values, const shape_data& shape) {
  auto result  = vector<T>(values.size(), T{0});
  auto weights = vector<float>(values.size(), 0.0f);
  for (int i = 0; i < shape.triangles.size(); i++) {
    auto [x, y, z] = shape.triangles[i];
    auto area      = triangle_area(1e6 * shape.positions[x],
        1e6 * shape.positions[y], 1e6 * shape.positions[z]);
    result[x] += area * values[x];
    result[y] += area * values[y];
    result[z] += area * values[z];
    weights[x] += area;
    weights[y] += area;
    weights[z] += area;
  }
  for (int i = 0; i < result.size(); i++) result[i] /= weights[i];
  swap(values, result);
}

// template <int N>
// struct hash<std::array<int, N>> {
//  size_t operator()(const std::array<int, N>& v) const {
//    static const auto hasher = std::hash<int>();
//    auto              h      = (size_t)0;
//    for (auto& vv : v) {
//      h ^= hasher(vv) + 0x9e3779b9 + (h << 6) + (h >> 2);
//    }
//    return h;
//  }
//};

// inline vec3f get_color(int i) {
//   static auto colors = vector<vec3f>{
//       {0.5, 0.5, 0.5},
//       {1, 0, 0},
//       {0, 0.5, 0},
//       {0, 0, 1},
//       {0, 0.5, 0.5},
//       {1, 0.5, 0},
//       {0.5, 0, 1},
//       {0.5, 0, 0},
//       {0, 0.5, 0},
//       {0, 0, 0.5},
//       {0, 0.5, 0.5},
//       {0.5, 0.5, 0},
//       {0.5, 0, 0.5},
//   };

//   return colors[i % colors.size()];
// }

inline vec3f get_color(int i) {
  auto colors = vector<vec3f>{
      {0, 0, 0.8},    // blue
      {0.8, 0, 0},    // red
      {0, 0, 0.8},    // blue
      {0, 0.5, 0},    // green
      {0.5, 0.5, 0},  // yellow
      {1, 0, 1},      // purple
      {0, 1, 1},      // cyan
      //      {0.5, 0.5, 0.5},
      {190 / 255.0, 45 / 255.0, 52 / 255.0},
      {0.063, 0.426, 0.127},
      {0.026, 0.087, 0.539},
      {0.270, 0.654, 1.00},
      {246 / 255.0, 217 / 255.0, 69 / 255.0},
      {243 / 255.0, 136 / 255.0, 40 / 255.0},
      {223 / 255.0, 146 / 255.0, 142 / 255.0},
      {130 / 255.0, 185 / 255.0, 80 / 255.0},
      {0, 0, 0.5},
      {0, 0.5, 0.5},
      {0.5, 0.5, 0},
      {0.5, 0, 0.5},
  };

  return colors[i % colors.size()];
}

// namespace std {

// inline string to_string(const vec3i& v) {
//   return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " +
//          std::to_string(v.z) + "}";
// }

// inline string to_string(const vec3f& v) {
//   return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " +
//          std::to_string(v.z) + "}";
// }

// inline string to_string(const vec2i& v) {
//   return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + "}";
// }

// inline string to_string(const vec2f& v) {
//   return "{" + std::to_string(v.x) + ", " + std::to_string(v.y) + "}";
// }

// inline string to_string(const mesh_point& p) {
//   return "{" + std::to_string(p.face) + ", " + std::to_string(p.uv) + "}";
// }

// // TODO(giacomo): doesn't work
// template <typename T>
// string to_string(const vector<T>& vec) {
//   auto max_elements = 100;
//   auto result       = string{};
//   result.reserve(1e5);
//   auto count = 0;
//   auto str   = (char*)result.data();
//   count += sprintf(str, "[size: %lu] ", vec.size());
//   if (vec.empty()) {
//     count += sprintf(str, "]\n");
//     goto end;
//   }
//   for (int i = 0; i < min((int)vec.size() - 1, max_elements); i++) {
//     count += sprintf(str, "%s, ", std::to_string(vec[i]).c_str());
//   }
//   if (vec.size() > max_elements) {
//     count += sprintf(str, "...]");
//   } else {
//     count += sprintf(str, "%s]", std::to_string(vec.back()).c_str());
//   }
// end:
//   count += sprintf(str, "\n");
//   result.resize(count);
//   return result;
// }

// template <typename T>
// string to_string(const hash_set<T>& vec) {
//   auto max_elements = 100;
//   auto result       = string{};
//   result.reserve(1e5);
//   auto count = 0;
//   auto str   = (char*)result.data();
//   count += sprintf(str, "[size: %lu] [", vec.size());

//   int i = 0;
//   for (auto& item : vec) {
//     if (i > max_elements) break;
//     i += 1;
//     count += sprintf(str, "%s, ", std::to_string(item).c_str());
//   }

//   count += sprintf(str, "]\n");
//   result.resize(count);
//   return result;
// }
// }  // namespace std

template <typename T>
void print(const string& name, const T& v) {
  auto s = std::to_string(v);
  printf("%s: %s\n", name.c_str(), s.c_str());
}

template <typename T>
void print(const string& name, const vector<T>& vec, int max_elements = 100) {
  printf("[size: %lu] ", vec.size());
  printf("%s: [", name.c_str());
  if (vec.empty()) {
    printf("]\n");
    return;
  }
  for (int i = 0; i < min((int)vec.size() - 1, max_elements); i++) {
    printf("%s, ", std::to_string(vec[i]).c_str());
  }
  if (vec.size() > max_elements) {
    printf("...]");
  } else {
    printf("%s]", std::to_string(vec.back()).c_str());
  }
  printf("\n");
}

template <typename T>
void print(
    const string& name, const std::deque<T>& vec, int max_elements = 100) {
  printf("[size: %lu] ", vec.size());
  printf("%s: [", name.c_str());
  if (vec.empty()) {
    printf("]\n");
    return;
  }

  for (auto& x : vec) {
    printf("%s ", std::to_string(x).c_str());
  }

  if (vec.size() > max_elements) {
    printf("...]");
  } else {
    printf("]");
  }
  printf("\n");
}

void draw_triangulation(
    ogl_texture* texture, int face, vec2i size = {2048, 2048});

// template <class K, class V, class F>
// inline bool maybe_add(const hash_map<K, V>& map, const K& k, F&& f) {
//  auto it = map.find(i);
//  if (it == map.end()) {
//    key      = f();
//    map[key] = key;
//  } else {
//    key = it->second;
//  }
//  return key;
//}

template <class K, class V>
inline bool contains(const hash_map<K, V>& map, const K& x) {
  return map.find(x) != map.end();
}

template <class T>
inline bool contains(const hash_set<T>& set, const T& x) {
  return set.find(x) != set.end();
}

template <class T>
inline bool contains(const vector<T>& vec, const T& x) {
  return find(vec.begin(), vec.end(), x) != vec.end();
}

template <class T>
inline bool contains(const std::deque<T>& vec, const T& x) {
  return find(vec.begin(), vec.end(), x) != vec.end();
}

template <class T>
inline const T& max(const vector<T>& vec) {
  return *max_element(vec.begin(), vec.end());
}

template <class T, typename F>
inline const T& max(const vector<T>& vec, F&& f) {
  auto max_index = 0;
  for (int i = 1; i < vec.size(); i++) {
    if (f(vec[i], vec[max_index])) max_index = i;
  }
  return vec[max_index];
}

template <class T>
inline T sum(const vector<T>& vec) {
  auto result = T{0};
  for (auto& v : vec) result += v;
  return result;
}

template <typename F>
inline void serial_for(size_t size, F&& f) {
  for (int i = 0; i < size; i++) f(i);
}

// template <>
// struct std::hash<vector<int>> {
//   size_t operator()(const vector<int>& V) const {
//     auto hash = V.size();
//     for (auto& i : V) {
//       hash ^= i + 0x9e3779b9 + (hash << 6) + (hash >> 2);
//     }
//     return hash;
//   }
// };

// template <class T>
// struct std::hash<vector<T>> {
//   size_t operator()(const vector<T>& V) const {
//     auto hash = V.size();
//     for (auto& i : V) {
//       hash ^= std::hash<T>{}(i);
//     }
//     return hash;
//   }
// };

// template <>
// struct std::hash<std::unordered_set<int>> {
//   size_t operator()(const std::unordered_set<int>& V) const {
//     auto hash = V.size();
//     for (auto& i : V) {
//       hash ^= std::hash<int>{}(i);
//     }
//     return hash;
//   }
// };

hash_map<int, vector<vec3i>>& debug_triangles();
hash_map<int, vector<vec2i>>& debug_edges();
hash_map<int, vector<vec2f>>& debug_nodes();
hash_map<int, vector<int>>&   debug_indices();

vector<int>&  debug_result();
vector<bool>& debug_visited();
vector<int>&  debug_stack();
bool&         debug_restart();
