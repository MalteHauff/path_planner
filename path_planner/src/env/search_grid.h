#pragma once

#include <vector>
#include <cfloat>
#include <cassert>
#include <iostream>

namespace adore {
namespace fun {

/**
 * 3D grid (Width x Length x Depth) storing pointers to NodeT.
 * - **Non-owning by default**: won’t delete NodeT* (safe with node pools).
 * - Depth-major linearization: dep*(Width*Length) + row*Length + col
 *
 * NodeT must expose:
 *   int index_width, index_length, index_depth;
 *   bool isOpen, isClosed;  double get_G();
 */
template<typename NodeT>
class GRID
{
public:
    GRID()
        : width_(0), length_(0), depth_(1),
          delete_on_init_(false),
          owns_pointers_(false)
    {}

    void resize(int width, int length, int depth = 1)
    {
        if (width <= 0 || length <= 0 || depth <= 0) {
            std::cerr << "[GRID] invalid resize: width=" << width
                      << " length=" << length << " depth=" << depth << "\n";
            return;
        }
        clear_all();
        width_  = width;
        length_ = length;
        depth_  = depth;
        cells_.assign(static_cast<size_t>(width_) * length_ * depth_, nullptr);
    }

    // Always null pointers between plans to avoid dangling pool pointers.
    void initialize()
    {
        for (auto &p : cells_) p = nullptr;
    }

    void set_delete_on_init(bool v) { delete_on_init_ = v; } // kept for compat (noop)
    void set_owns_pointers(bool v)  { owns_pointers_  = v; }
    bool owns_pointers() const      { return owns_pointers_; }

    // Store/replace pointer at its own indices
    void replace(NodeT* new_node, double /*headingResolution*/ = 0.0)
    {
        if (!new_node) return;
        const int r = new_node->index_width;
        const int c = new_node->index_length;
        const int d = new_node->index_depth;
        if (!valid_indices(r,c,d)) return;
        size_t idx = to_linear_index(r,c,d);
        NodeT* old = cells_[idx];
        if (owns_pointers_ && old && old != new_node) delete old;
        cells_[idx] = new_node;
    }

    NodeT* get(int row, int col, int dep) const
    {
        if (!valid_indices(row,col,dep)) return nullptr;
        return cells_[to_linear_index(row,col,dep)];
    }

    // ---- helpers used by A* / Hybrid A* ----
    double get_G(NodeT* node, double /*headingResolution*/ = 0.0) const
    {
        if (!node) return DBL_MAX;
        NodeT* s = get(node->index_width, node->index_length, node->index_depth);
        return s ? s->get_G() : DBL_MAX;
    }

    bool isClosed(NodeT* node, double /*headingResolution*/ = 0.0) const
    {
        if (!node) return false;
        NodeT* s = get(node->index_width, node->index_length, node->index_depth);
        return s ? s->isClosed : false;
    }

    bool isOpen(NodeT* node, double /*headingResolution*/ = 0.0) const
    {
        if (!node) return false;
        NodeT* s = get(node->index_width, node->index_length, node->index_depth);
        return s ? s->isOpen : false;
    }

    // Some code calls this to ensure the node is stored in the grid
    void set_visited(NodeT* node)
    {
        if (!node) return;
        replace(node);
        node->isOpen = node->isOpen; // no-op, kept to mirror legacy semantics
    }

    void set_closed(NodeT* node, double /*headingResolution*/ = 0.0)
    {
        if (!node) return;
        NodeT* s = get(node->index_width, node->index_length, node->index_depth);
        if (s) s->isClosed = true;
        else {
            // if the slot is empty, store the node and mark closed
            replace(node);
            node->isClosed = true;
        }
    }

    void clear_all()
    {
        if (owns_pointers_) {
            for (auto &p : cells_) { if (p) delete p; p = nullptr; }
        }
        cells_.clear();
        width_ = length_ = depth_ = 0;
    }

    int width()  const { return width_; }
    int length() const { return length_; }
    int depth()  const { return depth_; }

private:
    bool valid_indices(int row, int col, int dep) const
    {
        return !(col < 0 || col >= length_ ||
                 row < 0 || row >= width_  ||
                 dep < 0 || dep >= depth_);
    }

    size_t to_linear_index(int row, int col, int dep) const
    {
        return static_cast<size_t>(dep) * static_cast<size_t>(width_) * static_cast<size_t>(length_)
             + static_cast<size_t>(row) * static_cast<size_t>(length_)
             + static_cast<size_t>(col);
    }

    int  width_, length_, depth_;
    bool delete_on_init_;
    bool owns_pointers_;
    std::vector<NodeT*> cells_;
};

} // namespace fun
} // namespace adore
