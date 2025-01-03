#include "../include/graph.hpp"
namespace LaCAM2 {

    Graph * external_graph_ptr = nullptr;


    Vertex::Vertex(//uint _id,
                   uint _index)
            : //id(_id),
            index(_index), neighbor(Vertices()) {
    }

    Graph::Graph() : //V(Vertices()),
    width(0), height(0) {}

    Graph::~Graph() {
//        for (auto &v : V)
//            if (v != nullptr) delete v;
//        V.clear();
        for (auto &v : U)
            if (v != nullptr) { delete v; v = nullptr; }
        U.clear();
    }

// to load graph
    static const std::regex r_height = std::regex(R"(height\s(\d+))");
    static const std::regex r_width = std::regex(R"(width\s(\d+))");
    static const std::regex r_map = std::regex(R"(map)");

    Graph::Graph(const std::string &filename) :// V(Vertices()),
    width(0), height(0) {
        std::ifstream file(filename);
        if (!file) {
            std::cout << "file " << filename << " is not found." << std::endl;
            return;
        }
        std::string line;
        std::smatch results;

        // read fundamental graph parameters
        while (getline(file, line)) {
            // for CRLF coding
            if (*(line.end() - 1) == 0x0d) line.pop_back();

            if (std::regex_match(line, results, r_height)) {
                height = std::stoi(results[1].str());
            }
            if (std::regex_match(line, results, r_width)) {
                width = std::stoi(results[1].str());
            }
            if (std::regex_match(line, results, r_map)) break;
        }

        U = Vertices(width * height, nullptr);

        // create vertices
        uint y = 0;
        while (getline(file, line)) {
            // for CRLF coding
            if (*(line.end() - 1) == 0x0d) line.pop_back();
            for (uint x = 0; x < width; ++x) {
                char s = line[x];
                if (s == 'T' or s == '@') continue;  // object
                auto index = width * y + x;
                auto v = new Vertex(//V.size(),
                                    index);
                //V.push_back(v);
//                total_vertexs ++;
                U[index] = v;
            }
            ++y;
        }
        file.close();

        // create edges
        for (uint y = 0; y < height; ++y) {
            for (uint x = 0; x < width; ++x) {
                auto v = U[width * y + x];
                if (v == nullptr) continue;
                // left
                if (x > 0) {
                    auto u = U[width * y + (x - 1)];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
                // right
                if (x < width - 1) {
                    auto u = U[width * y + (x + 1)];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
                // up
                if (y < height - 1) {
                    auto u = U[width * (y + 1) + x];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
                // down
                if (y > 0) {
                    auto u = U[width * (y - 1) + x];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
            }
        }
    }

    uint Graph::size() const {
        return U.size();
    }

    bool is_same_config(const Config &C1, const Config &C2) {
        const auto N = C1.size();
        for (size_t i = 0; i < N; ++i) {
            if (C1[i]->index != C2[i]->index) return false;
        }
        return true;
    }

    uint ConfigHasher::operator()(const Config &C) const {
        uint hash = C.size();
//        for (auto &v : C) hash ^= v->id + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        for (auto &v : C) hash ^= v->index + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }

    std::ostream &operator<<(std::ostream &os, const Vertex *v) {
        os << v->index;
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const Config &config) {
        os << "<";
        const auto N = config.size();
        for (size_t i = 0; i < N; ++i) {
            if (i > 0) os << ",";
            os << std::setw(5) << config[i];
        }
        os << ">";
        return os;
    }

    Graph::Graph(freeNav::DimensionLength* dim, const freeNav::IS_OCCUPIED_FUNC<2> & isoc) {
//        std::ifstream file(filename);
//        if (!file) {
//            std::cout << "file " << filename << " is not found." << std::endl;
//            return;
//        }
//        std::string line;
//        std::smatch results;
//
//        // read fundamental graph parameters
//        while (getline(file, line)) {
//            // for CRLF coding
//            if (*(line.end() - 1) == 0x0d) line.pop_back();
//
//            if (std::regex_match(line, results, r_height)) {
//                height = std::stoi(results[1].str());
//            }
//            if (std::regex_match(line, results, r_width)) {
//                width = std::stoi(results[1].str());
//            }
//            if (std::regex_match(line, results, r_map)) break;
//        }
        width = dim[0], height = dim[1];
        U = Vertices(width * height, nullptr);

        // create vertices
//        int y = 0;
//        while (getline(file, line)) {
//            // for CRLF coding
//            if (*(line.end() - 1) == 0x0d) line.pop_back();
//            for (int x = 0; x < width; ++x) {
//                char s = line[x];
//                if (s == 'T' or s == '@') continue;  // object
//                auto index = width * y + x;
//                auto v = new Vertex(V.size(), index);
//                V.push_back(v);
//                U[index] = v;
//            }
//            ++y;
//        }
//        file.close();

        //freeNav::Id total_index = freeNav::getTotalIndexOfSpace<2>(dim);
        freeNav::Pointi<2> pt;
        for(int x=0; x<dim[0]; x++) {
            for(int y=0; y<dim[1]; y++) {
                pt = freeNav::Pointi<2>{x, y};
                if(!isoc(pt)) {
                    auto index = width * y + x;
                    auto v = new Vertex(//V.size(),
                                        index);
                    //V.push_back(v);
                    U[index] = v;
                }
            }
        }

        // create edges
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                auto v = U[width * y + x];
                if (v == nullptr) continue;
                // left
                if (x > 0) {
                    auto u = U[width * y + (x - 1)];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
                // right
                if (x < width - 1) {
                    auto u = U[width * y + (x + 1)];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
                // up
                if (y < height - 1) {
                    auto u = U[width * (y + 1) + x];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
                // down
                if (y > 0) {
                    auto u = U[width * (y - 1) + x];
                    if (u != nullptr) v->neighbor.push_back(u);
                }
            }
        }
    }

    Graph * external_grid_ptr = nullptr;

    bool isPtInRange(int x, int y, uint width, uint height) {
        if(x < 0 || x >= width) { return false; }
        if(y < 0 || y >= height) { return false; }
        return true;
    }

    void setStatesToOccupied(Graph * G, const freeNav::Pointis<2>& grids,
                             const freeNav::IS_OCCUPIED_FUNC<2>& raw_isoc,
                             const freeNav::IS_OCCUPIED_FUNC<2>& new_isoc) {
        assert(G != nullptr);

        // set related edge to unpassable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->U[y*G->width + x] != nullptr);
            std::vector<Vertex *>  nearby_nodes;
            // left
            if (isPtInRange(x-1,y,G->width, G->height) && G->U[(x - 1) + y*G->width] != nullptr) nearby_nodes.push_back(G->U[(x - 1) + y*G->width]);
            // right
            if (isPtInRange(x+1,y,G->width, G->height) && G->U[(x + 1) + y*G->width] != nullptr) nearby_nodes.push_back(G->U[(x + 1) + y*G->width]);
            // up
            if (isPtInRange(x,y-1,G->width, G->height) && G->U[x + (y-1)*G->width] != nullptr) nearby_nodes.push_back(G->U[x + (y-1)*G->width]);
            // down
            if (isPtInRange(x,y+1,G->width, G->height) && G->U[x + (y+1)*G->width] != nullptr) nearby_nodes.push_back(G->U[x + (y+1)*G->width]);
            // if (must) nearby node have edge to current grid, remove it
            for(const auto& node : nearby_nodes) {
                // only considering raw passable and st
                int erase_count = 0;
                for(auto iter = node->neighbor.begin(); iter != node->neighbor.end();) {
                    if((*iter)->index % G->width == x && (*iter)->index / G->width == y) {
                        iter = node->neighbor.erase(iter);
                        //std::cout << "delete edge " << node->pos.x << ", " << node->pos.y << " / " << x << ", " << y << std::endl;
                        erase_count ++;
                    } else  {
                        iter ++;
                    }
                }
                freeNav::Pointi<2> near_pt; near_pt[0] = node->index/G->width, near_pt[1] = node->index/G->width;
                //std::cout << "nearyby isoc/new_isoc = " << raw_isoc(near_pt) << "/" << new_isoc(near_pt) << std::endl;
                //std::cout << "node->neighbor size " << node->neighbor.size() << ", erase_count = " << erase_count << std::endl;
                assert(erase_count == 1);
            }
        }

        // set node to unpassable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->U[y*G->width + x] != nullptr); // yz: other agents' start or target must be free in raw graph
            delete G->U[y*G->width + x]; // yz: remove occupied node permanently, but we will restore it
            G->U[y*G->width + x] = nullptr;
        }
        return;
    }

    void restoreStatesToPassable(Graph * G, const freeNav::Pointis<2>& grids,
                                 const freeNav::IS_OCCUPIED_FUNC<2>& raw_isoc,
                                 const freeNav::IS_OCCUPIED_FUNC<2>& new_isoc) {
        assert(G != nullptr);
//        std::cout << "flag 0" << std::endl;
//        std::cout << G->U.size() << std::endl;
        // restore nodes
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->U[y*G->width + x] == nullptr); // yz: other agents' start or target must be free in raw graph
            G->U[y*G->width + x] = new Vertex(y*G->width + x);
        }
//        std::cout << "flag 1" << std::endl;
        // restore edges of restore nodes
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->U[y*G->width + x] != nullptr); // yz: other agents' start or target must be free in raw graph
            // left
            if (isPtInRange(x-1,y,G->width, G->height) && G->U[x - 1 + y*G->width]!=nullptr) G->U[y*G->width + x]->neighbor.push_back(G->U[x - 1 + y*G->width]);
            // right
            if (isPtInRange(x+1,y,G->width, G->height) && G->U[x + 1 + y*G->width]!=nullptr) G->U[y*G->width + x]->neighbor.push_back(G->U[x + 1 + y*G->width]);
            // up
            if (isPtInRange(x,y-1,G->width, G->height) && G->U[x + (y-1)*G->width]!=nullptr) G->U[y*G->width + x]->neighbor.push_back(G->U[x + (y-1)*G->width]);
            // down
            if (isPtInRange(x,y+1,G->width, G->height) && G->U[x + (y+1)*G->width]!=nullptr) G->U[y*G->width + x]->neighbor.push_back(G->U[x + (y+1)*G->width]);
        }
//        std::cout << "flag 2" << std::endl;

        // restore related edge to passable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->U[y*G->width + x] != nullptr);
            std::vector<Vertex *> nearby_nodes;
            // left
            if (isPtInRange(x-1,y,G->width, G->height) && G->U[x - 1 + y*G->width]!=nullptr) nearby_nodes.push_back(G->U[x - 1 + y*G->width]);
            // right
            if (isPtInRange(x+1,y,G->width, G->height) && G->U[x + 1 + y*G->width]!=nullptr) nearby_nodes.push_back(G->U[x + 1 + y*G->width]);
            // up
            if (isPtInRange(x,y-1,G->width, G->height) && G->U[x + (y-1)*G->width]!=nullptr) nearby_nodes.push_back(G->U[x + (y-1)*G->width]);
            // down
            if (isPtInRange(x,y+1,G->width, G->height) && G->U[x + (y+1)*G->width]!=nullptr) nearby_nodes.push_back(G->U[x + (y+1)*G->width]);
            // if (must) nearby node have edge to current grid, remove it
            for(const auto& node : nearby_nodes) {
                freeNav::Pointi<2> pt; pt[0] = node->index%G->width, pt[1] = node->index/G->width;
                if(!raw_isoc(pt) && !new_isoc(pt)) {
                    // do not add edge to new added nodes, which already add in previous
                    int edge_count = 0;
                    // check whether already have related edges (in theory there shouldn't be)
                    for(auto iter = node->neighbor.begin(); iter != node->neighbor.end(); iter++) {
                        if((*iter)->index%G->width == x && (*iter)->index/G->width == y) {
                            iter = node->neighbor.erase(iter);
                            edge_count ++;
                        }
                    }
                    assert(edge_count == 0);
                    node->neighbor.push_back(G->U[y*G->width + x]);
                }
            }
        }
//        std::cout << "flag 3" << std::endl;

        return;
    }


}