/*
 * graph definition
 */
#pragma once
#include "utils.hpp"
#include "../../../freeNav-base/basic_elements/point.h"

namespace LaCAM2 {
    struct Vertex {
        const uint id;     // index for V in Graph
        const uint index;  // index for U, width * y + x, in Graph
        std::vector<Vertex *> neighbor;

        Vertex(uint _id, uint _index);
    };

    using Vertices = std::vector<Vertex *>;
    using Config = std::vector<Vertex *>;  // a set of locations for all agents

    struct Graph {
        Vertices V;                          // without nullptr
        Vertices U;                          // with nullptr
        uint width;                          // grid width
        uint height;                         // grid height
        Graph();

        Graph(const std::string &filename);  // taking map filename
        // yz: for common interfaces
        Graph(freeNav::DimensionLength* dim, const freeNav::IS_OCCUPIED_FUNC<2> & isoc);
        ~Graph();

        uint size() const;  // the number of vertices
    };

    bool is_same_config(
            const Config &C1,
            const Config &C2);  // check equivalence of two configurations

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
    struct ConfigHasher {
        uint operator()(const Config &C) const;
    };

    std::ostream &operator<<(std::ostream &os, const Vertex *v);

    std::ostream &operator<<(std::ostream &os, const Config &config);
}