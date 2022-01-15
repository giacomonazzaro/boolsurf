#pragma once
#include "bytes.h"
using std::vector;

struct History {
  struct Edit {
    std::function<void(Edit*)> update = {};

    virtual void done() {}
    virtual void undo() = 0;
    virtual void redo() = 0;
    virtual ~Edit() {}
  };

  vector<Edit*> done   = {};
  vector<Edit*> undone = {};
};

// struct Multi_Edit : History::Edit {
//     vector<Edit*> edits = {};
//     Multi_Edit(const vector<Edit*> _edits) : edits(_edits) {}
//     void done() override {
//         for (int i = 0; i < edits.size(); i++) edits[i]->done();
//     }
//     void undo() override {
//         for (int i = edits.size() - 1; i >= 0; i--) edits[i]->undo();
//     }
//     void redo() override {
//         for (int i = 0; i < edits.size(); i++) edits[i]->redo();
//     }
// };

inline bool commit(History& history, History::Edit* edit) {
  edit->done();
  if (edit->update) edit->update(edit);
  history.done.push_back(edit);

  for (auto edit : history.undone) delete edit;
  history.undone.clear();

  return true;
}

inline bool undo(History& history) {
  if (history.done.empty()) return false;

  auto edit = history.done.back();
  edit->undo();
  if (edit->update) edit->update(edit);

  history.done.pop_back();
  history.undone.push_back(edit);
  return true;
}

inline bool redo(History& history) {
  if (history.undone.empty()) return false;

  auto edit = history.undone.back();
  history.undone.pop_back();

  edit->redo();
  if (edit->update) edit->update(edit);

  history.done.push_back(edit);
  return true;
}

inline std::string to_string(const History::Edit& history_edit) {
  return "no string representation for this edit";
}

template <typename T>
struct Value_Edit : History::Edit {
  T  value_copy = {};
  T* location   = nullptr;

  Value_Edit(T& edit) {
    value_copy = edit;
    location   = &edit;
  }
  void undo() override { std::swap(*location, value_copy); }
  void redo() override { std::swap(*location, value_copy); }
};

template <typename T>
struct Buffer_Edit : History::Edit {
  byte*        buffer      = nullptr;
  byte*        buffer_copy = nullptr;
  size_t       buffer_size = 0;
  size_t       window_size = 0;
  vector<byte> diff        = {};

  Buffer_Edit(T* _buffer, size_t _buffer_size, size_t _window_size = 256) {
    buffer      = (byte*)_buffer;
    buffer_size = _buffer_size;
    window_size = _window_size;

    buffer_copy = new byte[buffer_size];
    memcpy(buffer_copy, buffer, buffer_size);
  }

  void done() override {
    diff = make_diff(
        buffer, buffer_size, buffer_copy, buffer_size, window_size);
    delete buffer_copy;
  }
  void undo() override { apply_diff(buffer, diff); }
  void redo() override { apply_diff(buffer, diff); }
};

template <typename T>
struct Vector_Edit : History::Edit {
  vector<T>*   vec         = {};
  vector<T>    vec_copy    = {};
  size_t       window_size = 0;
  vector<byte> diff        = {};
  size_t       old_size    = 0;
  size_t       new_size    = 0;

  Vector_Edit(vector<T>* _vec, size_t _window_size = sizeof(T)) {
    vec         = _vec;
    vec_copy    = *_vec;
    window_size = _window_size;
    old_size    = vec->size();
  }

  void done() override {
    diff     = make_diff((const byte*)vec->data(), sizeof(T) * vec->size(),
        (const byte*)vec_copy.data(), sizeof(T) * vec_copy.size(), window_size);
    new_size = vec->size();

    // TODO(giacomo): free memory of vec_copy
    // https://www.techiedelight.com/delete-vector-free-memory-cpp/
    std::vector<T>().swap(vec_copy);
  }
  void undo() override {
    vec->resize(old_size);
    apply_diff(vec->data(), diff);
  }
  void redo() override {
    vec->resize(new_size);
    apply_diff(vec->data(), diff);
  }
};

template <typename T>
struct Vector_Item_Edit : History::Edit {
  vector<T>* vec       = {};
  T          item_copy = {};
  size_t     item_id   = 0;

  Vector_Item_Edit(vector<T>* _vec, size_t id) {
    vec       = _vec;
    item_copy = (*_vec)[id];
    item_id   = id;
  }

  void undo() override { std::swap((*vec)[item_id], item_copy); }
  void redo() override { std::swap((*vec)[item_id], item_copy); }
};
