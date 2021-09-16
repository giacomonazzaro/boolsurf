#pragma once
#include <unordered_map>
#include <unordered_set>

template <typename Key, typename Value>
using hash_map = std::unordered_map<Key, Value>;

template <typename Key>
using hash_set = std::unordered_set<Key>;

// Vector append and concatenation
template <typename T>
inline void operator+=(std::vector<T>& a, const std::vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}
template <typename T>
inline void operator+=(std::vector<T>& a, const T& b) {
  a.push_back(b);
}
