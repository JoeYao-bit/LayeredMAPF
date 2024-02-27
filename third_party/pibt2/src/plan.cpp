#include "../include/plan.hpp"
namespace PIBT_2 {

    Config Plan::get(const int t) const {
        const int configs_size = configs.size();
        if (!(0 <= t && t < configs_size)) halt("invalid timestep");
        return configs[t];
    }

    Node *Plan::get(const int t, const int i) const {
        if (empty()) halt("invalid operation");
        if (!(0 <= t && t < (int) configs.size())) halt("invalid timestep");
        if (!(0 <= i && i < (int) configs[0].size())) halt("invalid agent id");
        return configs[t][i];
    }

    path_pathfinding::Path Plan::getPath(const int i) const {
        path_pathfinding::Path path;
        int makespan = getMakespan();
        for (int t = 0; t <= makespan; ++t) path.push_back(get(t, i));
        return path;
    }

    Config Plan::last() const {
        if (empty()) halt("invalid operation");
        return configs[getMakespan()];
    }

    Node *Plan::last(const int i) const {
        if (empty()) halt("invalid operation");
        if (i < 0 || (int) configs[0].size() <= i) halt("invalid operation");
        return configs[getMakespan()][i];
    }

    void Plan::clear() { configs.clear(); }

    void Plan::add(const Config &c) {
        if (!configs.empty() && configs.at(0).size() != c.size()) {
            halt("invalid operation");
        }
        configs.push_back(c);
    }

    bool Plan::empty() const { return configs.empty(); }

    int Plan::size() const { return configs.size(); }

    int Plan::getMakespan() const { return size() - 1; }

    int Plan::getPathCost(const int i) const {
        const int makespan = getMakespan();
        const Node *g = get(makespan, i);
        int c = makespan;
        while (c > 0 && get(c - 1, i) == g) --c;
        return c;
    }

    int Plan::getSOC() const {
        int makespan = getMakespan();
        if (makespan <= 0) return 0;
        int num_agents = get(0).size();
        int soc = 0;
        for (int i = 0; i < num_agents; ++i) soc += getPathCost(i);
        return soc;
    }

    Plan Plan::operator+(const Plan &other) const {
        // check validity
        Config c1 = last();
        Config c2 = other.get(0);
        const int c1_size = c1.size();
        const int c2_size = c2.size();
        if (c1_size != c2_size) halt("invalid operation");
        for (int i = 0; i < c1_size; ++i) {
            if (c1[i] != c2[i]) halt("invalid operation.");
        }
        // merge
        Plan new_plan;
        new_plan.configs = configs;
        for (int t = 1; t < other.size(); ++t) new_plan.add(other.get(t));
        return new_plan;
    }

    void Plan::operator+=(const Plan &other) {
        if (configs.empty()) {
            configs = other.configs;
            return;
        }
        // check validity
        if (!sameConfig(last(), other.get(0))) halt("invalid operation");
        // merge
        for (int t = 1; t < other.size(); ++t) add(other.get(t));
    }

    bool Plan::validate(MAPF_Instance *P) const {
        return validate(P->getConfigStart(), P->getConfigGoal());
    }

    bool Plan::validate(MAPD_Instance *P) const {
        // check tasks
        if ((int) P->getOpenTasks().size() > 0) {
            warn("validation, tasks remain");
            return false;
        }
        auto closed_tasks = P->getClosedTasks();
        if ((int) closed_tasks.size() != P->getTaskNum()) {
            warn("validation, num of closed_tasks is invalid");
            return false;
        }
        if (!std::all_of(closed_tasks.begin(), closed_tasks.end(), [](Task *task) {
            return task->loc_current == task->loc_delivery;
        })) {
            warn("validation, some tasks seem to be invalid");
            return false;
        }

        return validate(P->getConfigStart());
    }

    bool Plan::validate(const Config &starts, const Config &goals) const {
        // check goal
        if (!sameConfig(last(), goals)) {
            warn("validation, invalid goals");
            return false;
        }
        return validate(starts);
    }

    bool Plan::validate(const Config &starts) const {
        if (configs.empty()) return false;

        // start
        if (!sameConfig(starts, get(0))) {
            warn("validation, invalid starts");
            return false;
        }

        // check conflicts and continuity
        int num_agents = get(0).size();
        for (int t = 1; t <= getMakespan(); ++t) {
            if ((int) configs[t].size() != num_agents) {
                warn("validation, invalid size");
                return false;
            }
            for (int i = 0; i < num_agents; ++i) {
                Node *v_i_t = get(t, i);
                Node *v_i_t_1 = get(t - 1, i);
                Nodes cands = v_i_t_1->neighbor;
                cands.push_back(v_i_t_1);
                if (!inArray(v_i_t, cands)) {
                    warn("validation, invalid move at t=" + std::to_string(t));
                    return false;
                }
                // see conflicts
                for (int j = i + 1; j < num_agents; ++j) {
                    Node *v_j_t = get(t, j);
                    Node *v_j_t_1 = get(t - 1, j);
                    if (v_i_t == v_j_t) {
                        warn("validation, vertex conflict at v=" + std::to_string(v_i_t->id) +
                             ", t=" + std::to_string(t));
                        return false;
                    }
                    if (v_i_t == v_j_t_1 && v_i_t_1 == v_j_t) {
                        warn("validation, swap conflict");
                        return false;
                    }
                }
            }
        }
        return true;
    }

    int Plan::getMaxConstraintTime(const int id, Node *s, Node *g, path_pathfinding::Graph *G) const {
        const int makespan = getMakespan();
        const int dist = G->pathDist(s, g);
        const int num = configs[0].size();
        for (int t = makespan - 1; t >= dist; --t) {
            for (int i = 0; i < num; ++i) {
                if (i != id && get(t, i) == g) return t;
            }
        }
        return 0;
    }

    int Plan::getMaxConstraintTime(const int id, MAPF_Instance *P) const {
        return getMaxConstraintTime(id, P->getStart(id), P->getGoal(id), P->getG());
    }

    void Plan::halt(const std::string &msg) const {
        std::cout << "error@Plan: " << msg << std::endl;
        this->~Plan();
        std::exit(1);
    }

    void Plan::warn(const std::string &msg) const {
        std::cout << "warn@Plan: " << msg << std::endl;
    }
}