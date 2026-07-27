#ifndef ASTAR_CELL_H
#define ASTAR_CELL_H

/// represents a single pixel in A* grid
class AStarCell {
public:
    int idx;    // flattened index
    float g;    // cost-to-come
    float f;    // priority = g + h

    AStarCell(int i, float g_, float f_)
        : idx(i), g(g_), f(f_) {}

    friend bool operator<(const AStarCell& a, const AStarCell& b) {
        return a.f > b.f;   // min-heap behavior
    }

    friend bool operator==(const AStarCell& a, const AStarCell& b) {
        return a.idx == b.idx;
    }
};

#endif

