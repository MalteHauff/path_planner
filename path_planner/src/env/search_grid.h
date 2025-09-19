#pragma once

#include <vector>
#include <cfloat>
#include <cassert>
#include <iostream>

namespace adore {
namespace fun {

/**
 * Simple GRID container that stores pointers to Node objects in a 3D (width x length x depth)
 * layout, with ownership: the grid owns stored pointers and will delete them when replaced.
 *
 * Coordinate conventions:
 *   - x (column/length) index in [0, length)
 *   - y (row/width)    index in [0, width)
 *   - depth index in [0, depth)
 *
 * Indexing into internal 1D vector uses:
 *   idx = depth_idx * (width*length) + y * length + x
 *
 * Usage notes:
 *  - Before calling replace(node, headingResolution) ensure node->update_index(width, length, depth, headingResolution)
 *    has been called (so node->index_length, index_width, index_depth are valid).
 *  - GRID::replace takes ownership of the passed pointer and will delete any previously stored pointer at that cell.
 */
template <typename NodeT>
class GRID
{
public:
    GRID()
        : width_(0), length_(0), depth_(1), delete_on_init_(false)
    {}

    ~GRID()
    {
        clear_all();
    }

    // Resize the grid; previous contents are deleted.
    void resize(int width, int length, int depth = 1)
    {
        if (width <= 0 || length <= 0 || depth <= 0) {
            std::cerr << "[GRID] resize called with non-positive dims: "
                      << "width=" << width << " length=" << length << " depth=" << depth << std::endl;
            return;
        }
        // delete old contents
        clear_all();
        width_ = width;
        length_ = length;
        depth_ = depth;
        cells_.assign(static_cast<size_t>(width_) * length_ * depth_, nullptr);
    }

    // Initialize the grid — optional: delete contents if delete_on_init_ true
    void initialize()
    {
        if (delete_on_init_) {
            clear_all();
            cells_.assign(static_cast<size_t>(width_) * length_ * depth_, nullptr);
        } else {
            // keep pointers but reset flags if desired (no deletion)
            for (size_t i = 0; i < cells_.size(); ++i) {
                if (cells_[i]) {
                    cells_[i]->isOpen = false;
                    cells_[i]->isClosed = false;
                    // do not delete
                }
            }
        }
    }

    // Set whether initialize() should delete stored pointers
    void set_delete_on_init(bool v) { delete_on_init_ = v; }

    // Replace stored pointer for the cell corresponding to new_node's indices.
    // Deletes old pointer if present and not equal to new_node.
    void replace(NodeT* new_node, double /*headingResolution*/)
    {
        if (new_node == nullptr) {
            return;
        }

        // We expect caller to have prepared indices via Node::update_index or setPosition.
        int idx_depth = new_node->index_depth;
        int idx_width = new_node->index_width;   // row (y)
        int idx_length = new_node->index_length; // col (x)

        // Bounds safety
        if (!valid_indices(idx_width, idx_length, idx_depth)) {
            std::cerr << "[GRID] replace: Node indices out of bounds: "
                      << "x=" << idx_length << " y=" << idx_width << " depth=" << idx_depth
                      << " grid(LxW x D)=(" << length_ << "x" << width_ << " x " << depth_ << ")\n";
            return;
        }

        size_t linear_idx = to_linear_index(idx_width, idx_length, idx_depth);
        NodeT* old = cells_[linear_idx];
        if (old != nullptr && old != new_node) {
            delete old;
        }
        cells_[linear_idx] = new_node;
    }

    // ---- Backwards-compatible overloads (no headingResolution) ----
    // These forward to the canonical implementation to avoid changing many call sites.

    // replace(old API)
    void replace(NodeT* new_node)
    {
        replace(new_node, 0.0);
    }

    // get_G with headingResolution (canonical)
    double get_G(NodeT* node, double /*headingResolution*/) const
    {
        if (node == nullptr) return DBL_MAX;
        int idx_depth = node->index_depth;
        int idx_width = node->index_width;
        int idx_length = node->index_length;
        if (!valid_indices(idx_width, idx_length, idx_depth)) {
            return DBL_MAX;
        }
        size_t linear_idx = to_linear_index(idx_width, idx_length, idx_depth);
        NodeT* stored = cells_[linear_idx];
        if (stored == nullptr) return DBL_MAX;
        return stored->get_G();
    }

    // get_G old API (no headingResolution)
    double get_G(NodeT* node) const
    {
        return get_G(node, 0.0);
    }

    // Return whether the stored node at the node's index is marked closed (canonical)
    bool isClosed(NodeT* node, double /*headingResolution*/) const
    {
        if (node == nullptr) return false;
        int idx_depth = node->index_depth;
        int idx_width = node->index_width;
        int idx_length = node->index_length;
        if (!valid_indices(idx_width, idx_length, idx_depth)) {
            return false;
        }
        size_t linear_idx = to_linear_index(idx_width, idx_length, idx_depth);
        NodeT* stored = cells_[linear_idx];
        if (stored == nullptr) return false;
        return stored->isClosed;
    }

    // isClosed old API
    bool isClosed(NodeT* node) const
    {
        return isClosed(node, 0.0);
    }

    // Return whether the stored node at the node's index is open (canonical)
    bool isOpen(NodeT* node, double /*headingResolution*/) const
    {
        if (node == nullptr) return false;
        int idx_depth = node->index_depth;
        int idx_width = node->index_width;
        int idx_length = node->index_length;
        if (!valid_indices(idx_width, idx_length, idx_depth)) {
            return false;
        }
        size_t linear_idx = to_linear_index(idx_width, idx_length, idx_depth);
        NodeT* stored = cells_[linear_idx];
        if (stored == nullptr) return false;
        return stored->isOpen;
    }

    // isOpen old API
    bool isOpen(NodeT* node) const
    {
        return isOpen(node, 0.0);
    }

    // Mark the stored node at the node's index as closed (canonical)
    void set_closed(NodeT* node, double /*headingResolution*/)
    {
        if (node == nullptr) return;
        int idx_depth = node->index_depth;
        int idx_width = node->index_width;
        int idx_length = node->index_length;
        if (!valid_indices(idx_width, idx_length, idx_depth)) {
            return;
        }
        size_t linear_idx = to_linear_index(idx_width, idx_length, idx_depth);
        NodeT* stored = cells_[linear_idx];
        if (stored != nullptr) {
            stored->isClosed = true;
            stored->isOpen = false;
        }
    }

    // set_closed old API
    void set_closed(NodeT* node)
    {
        set_closed(node, 0.0);
    }

    // ---- new helper expected by some legacy callers ----
    // Mark the stored node at the node's index as visited (exists in older code)
    void set_visited(NodeT* node)
    {
        if (node == nullptr) return;
        int idx_depth = node->index_depth;
        int idx_width = node->index_width;
        int idx_length = node->index_length;
        if (!valid_indices(idx_width, idx_length, idx_depth)) {
            return;
        }
        size_t linear_idx = to_linear_index(idx_width, idx_length, idx_depth);
        NodeT* stored = cells_[linear_idx];
        if (stored != nullptr) {
            stored->isVisited = true;
        }
    }

    // For debugging: dump counts of allocated cells (non-null)
    size_t non_null_count() const
    {
        size_t cnt = 0;
        for (auto p : cells_) if (p != nullptr) ++cnt;
        return cnt;
    }

    // Clear and delete all stored pointers
    void clear_all()
    {
        for (auto &p : cells_) {
            if (p != nullptr) {
                delete p;
                p = nullptr;
            }
        }
        cells_.clear();
        width_ = length_ = depth_ = 0;
    }

    int width() const { return width_; }
    int length() const { return length_; }
    int depth() const { return depth_; }

private:
    bool valid_indices(int row, int col, int dep) const
    {
        if (col < 0 || col >= length_) return false;
        if (row < 0 || row >= width_) return false;
        if (dep < 0 || dep >= depth_) return false;
        return true;
    }

    // linear index layout: depth major, then row, then column
    size_t to_linear_index(int row, int col, int dep) const
    {
        // dep * (width * length) + row * length + col
        return static_cast<size_t>(dep) * (static_cast<size_t>(width_) * static_cast<size_t>(length_))
             + static_cast<size_t>(row) * static_cast<size_t>(length_)
             + static_cast<size_t>(col);
    }

    int width_;
    int length_;
    int depth_;
    bool delete_on_init_;

    std::vector<NodeT*> cells_;
};

} // namespace fun
} // namespace adore
