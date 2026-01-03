#pragma once

#include"common.h"
#include "../../../../freeNav-base/basic_elements/point.h"

#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

namespace freeNav::LayeredMAPF::LA_MAPF::CBSH2_RTC {

    int RANDOM_WALK_STEPS = 100000;

    // Currently only works for undirected unweighted 4-neighbor grids
    class Instance {
    public:
        int num_of_cols = 0;
        int num_of_rows = 0;
        int map_size = 0;

        Instance() = default;

        Instance(const string &map_fname, const string &agent_fname,
                 int num_of_agents = 0, const string &agent_indices = "",
                 int num_of_rows = 0, int num_of_cols = 0, int num_of_obstacles = 0, int warehouse_width = 0)  :
                map_fname(map_fname), agent_fname(agent_fname), num_of_agents(num_of_agents), agent_indices(agent_indices) {
            bool succ = loadMap();
            if (!succ) {
                if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 &&
                    num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
                {
                    generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
                    saveMap();
                } else {
                    cerr << "Map file " << map_fname << " not found." << endl;
                    exit(-1);
                }
            }

            succ = loadAgents();
            if (!succ) {
                if (num_of_agents > 0) {
                    generateRandomAgents(warehouse_width);
                    saveAgents();
                } else {
                    cerr << "Agent file " << agent_fname << " not found." << endl;
                    exit(-1);
                }
            }

        }

        // initialize instance in an freeNav way
        Instance(freeNav::DimensionLength *dim, const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                 const freeNav::Instances<2> &instance_sat) {
            num_of_cols = dim[0];
            num_of_rows = dim[1];
            map_size = dim[0] * dim[1];
            num_of_agents = instance_sat.size();
            for (const auto &instance : instance_sat) {
                int start_id = linearizeCoordinate(instance.first[1], instance.first[0]);
                int target_id = linearizeCoordinate(instance.second[1], instance.second[0]);
                start_locations.push_back(start_id);
                goal_locations.push_back(target_id);
            }
            freeNav::Id total_index = freeNav::getTotalIndexOfSpace<2>(dim);
            my_map.resize(total_index, false);
            for (int i = 0; i < dim[0]; i++) {
                for (int j = 0; j < dim[1]; j++) {
                    int pt_id = linearizeCoordinate(j, i);
                    freeNav::Pointi<2> pt({i, j});
                    if (isoc(pt)) {
                        my_map[pt_id] = true;
                    }
                }
            }
        }


        void printAgents() const {
            for (int i = 0; i < num_of_agents; i++) {
                cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << ","
                     << getColCoordinate(start_locations[i])
                     << ") ; G=(" << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i])
                     << ")" << endl;
            }
        }


        inline bool isObstacle(int loc) const { return my_map[loc]; }

        inline bool validMove(int curr, int next) const {
            if (next < 0 || next >= map_size)
                return false;
            if (my_map[next])
                return false;
            return getManhattanDistance(curr, next) < 2;
        }

        list<int> getNeighbors(int curr) const {
            list<int> neighbors;
            int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
            for (int next : candidates) {
                if (validMove(curr, next))
                    neighbors.emplace_back(next);
            }
            return neighbors;
        }

        inline int linearizeCoordinate(int row, int col) const { return (this->num_of_cols * row + col); }

        inline int linearizeCoordinate(const pair<int, int> &cell) const {
            return linearizeCoordinate(cell.first, cell.second);
        }

        inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }

        inline int getColCoordinate(int id) const { return id % this->num_of_cols; }

        inline pair<int, int> getCoordinate(int id) const {
            return make_pair(id / this->num_of_cols, id % this->num_of_cols);
        }

        inline int getCols() const { return num_of_cols; }

        inline int getManhattanDistance(int loc1, int loc2) const {
            int loc1_x = getRowCoordinate(loc1);
            int loc1_y = getColCoordinate(loc1);
            int loc2_x = getRowCoordinate(loc2);
            int loc2_y = getColCoordinate(loc2);
            return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
        }

        static inline int getManhattanDistance(const pair<int, int> &loc1, const pair<int, int> &loc2) {
            return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
        }

        int walkCounterClockwise(int from, int to) const {
            assert(validMove(from, to));
            int dir = turnLeft(to - from);
            while (!validMove(from, from + dir))
                dir = turnLeft(dir);
            return from + dir;
        }

        inline int turnLeft(int dir) const {
            if (dir == 1)
                return -num_of_cols;
            else if (dir == -num_of_cols)
                return -1;
            else if (dir == -1)
                return num_of_cols;
            else if (dir == num_of_cols)
                return 1;
            else
                return 0;
        }

        inline int turnRight(int dir) const {
            if (dir == 1)
                return num_of_cols;
            else if (dir == num_of_cols)
                return -1;
            else if (dir == -1)
                return -num_of_cols;
            else if (dir == -num_of_cols)
                return 1;
            else
                return 0;
        }

        int getDegree(int loc) const // return the number of neighbors
        {
            assert(loc >= 0 && loc < map_size && !my_map[loc]);
            int degree = 0;
            if (0 <= loc - num_of_cols && !my_map[loc - num_of_cols])
                degree++;
            if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
                degree++;
            if (loc % num_of_cols > 0 && !my_map[loc - 1])
                degree++;
            if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
                degree++;
            return degree;
        }

        int getDefaultNumberOfAgents() const { return num_of_agents; }

    private:
        // int moves_offset[MOVE_COUNT];
        vector<bool> my_map;
        string map_fname;
        string agent_fname;
        string agent_indices;
        int num_of_agents = 0;
        vector<int> start_locations;
        vector<int> goal_locations;

        bool loadMap() {
            using namespace boost;
            using namespace std;
            ifstream myfile(map_fname.c_str());
            if (!myfile.is_open())
                return false;
            string line;
            tokenizer<char_separator<char>>::iterator beg;
            getline(myfile, line);
            if (line[0] == 't') // Nathan's benchmark
            {
                char_separator<char> sep(" ");
                getline(myfile, line);
                tokenizer<char_separator<char>> tok(line, sep);
                beg = tok.begin();
                beg++;
                num_of_rows = atoi((*beg).c_str()); // read number of rows
                getline(myfile, line);
                tokenizer<char_separator<char>> tok2(line, sep);
                beg = tok2.begin();
                beg++;
                num_of_cols = atoi((*beg).c_str()); // read number of cols
                getline(myfile, line); // skip "map"
            } else // my benchmark
            {
                char_separator<char> sep(",");
                tokenizer<char_separator<char>> tok(line, sep);
                beg = tok.begin();
                num_of_rows = atoi((*beg).c_str()); // read number of rows
                beg++;
                num_of_cols = atoi((*beg).c_str()); // read number of cols
            }
            map_size = num_of_cols * num_of_rows;
            my_map.resize(map_size, false);
            // read map (and start/goal locations)
            for (int i = 0; i < num_of_rows; i++) {
                getline(myfile, line);
                for (int j = 0; j < num_of_cols; j++) {
                    my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
                }
            }
            myfile.close();

            // initialize moves_offset array
            /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
            moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
            moves_offset[Instance::valid_moves_t::EAST] = 1;
            moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
            moves_offset[Instance::valid_moves_t::WEST] = -1;*/
            return true;
        }

        void printMap() const {
            for (int i = 0; i < num_of_rows; i++) {
                for (int j = 0; j < num_of_cols; j++) {
                    if (this->my_map[linearizeCoordinate(i, j)])
                        cout << '@';
                    else
                        cout << '.';
                }
                cout << endl;
            }
        }

        void saveMap() const {
            ofstream myfile;
            myfile.open(map_fname);
            if (!myfile.is_open()) {
                cout << "Fail to save the map to " << map_fname << endl;
                return;
            }
            myfile << num_of_rows << "," << num_of_cols << endl;
            for (int i = 0; i < num_of_rows; i++) {
                for (int j = 0; j < num_of_cols; j++) {
                    if (my_map[linearizeCoordinate(i, j)])
                        myfile << "@";
                    else
                        myfile << ".";
                }
                myfile << endl;
            }
            myfile.close();
        }

        bool loadAgents() {
            using namespace std;
            using namespace boost;

            string line;
            ifstream myfile(agent_fname.c_str());
            if (!myfile.is_open())
                return false;

            getline(myfile, line);
            if (line[0] == 'v') // Nathan's benchmark
            {
                if (num_of_agents == 0) {
                    cerr << "The number of agents should be larger than 0" << endl;
                    exit(-1);
                }
                start_locations.resize(num_of_agents);
                goal_locations.resize(num_of_agents);

                vector<int> ids(num_of_agents);
                if (agent_indices != "") {
                    char_separator<char> sep(",");
                    tokenizer<char_separator<char> > chars(agent_indices, sep);
                    int i = 0;
                    for (auto c : chars) {
                        ids[i] = atoi(c.c_str());
                        if (i > 0 && ids[i] <= ids[i - 1])
                            exit(-1); // the indices of the agents should be strictly increasing!
                        i++;
                    }
                } else {
                    for (int i = 0; i < num_of_agents; i++)
                        ids[i] = i;
                }
                char_separator<char> sep("\t");
                int count = 0;
                int i = 0;
                while (i < num_of_agents) {
                    getline(myfile, line);
                    if (count == ids[i]) {
                        tokenizer<char_separator<char> > tok(line, sep);
                        tokenizer<char_separator<char> >::iterator beg = tok.begin();
                        beg++; // skip the first number
                        beg++; // skip the map name
                        beg++; // skip the columns
                        beg++; // skip the rows
                        // read start [row,col] for agent i
                        int col = atoi((*beg).c_str());
                        beg++;
                        int row = atoi((*beg).c_str());
                        start_locations[i] = linearizeCoordinate(row, col);
                        // read goal [row,col] for agent i
                        beg++;
                        col = atoi((*beg).c_str());
                        beg++;
                        row = atoi((*beg).c_str());
                        goal_locations[i] = linearizeCoordinate(row, col);
                        i++;
                    }
                    count++;
                }
            } else // My benchmark
            {
                char_separator<char> sep(",");
                tokenizer<char_separator<char>> tok(line, sep);
                tokenizer<char_separator<char>>::iterator beg = tok.begin();
                num_of_agents = atoi((*beg).c_str());
                start_locations.resize(num_of_agents);
                goal_locations.resize(num_of_agents);
                for (int i = 0; i < num_of_agents; i++) {
                    getline(myfile, line);
                    tokenizer<char_separator<char>> col_tok(line, sep);
                    tokenizer<char_separator<char>>::iterator c_beg = col_tok.begin();
                    pair<int, int> curr_pair;
                    // read start [row,col] for agent i
                    int row = atoi((*c_beg).c_str());
                    c_beg++;
                    int col = atoi((*c_beg).c_str());
                    start_locations[i] = linearizeCoordinate(row, col);
                    // read goal [row,col] for agent i
                    c_beg++;
                    row = atoi((*c_beg).c_str());
                    c_beg++;
                    col = atoi((*c_beg).c_str());
                    goal_locations[i] = linearizeCoordinate(row, col);
                }
            }
            myfile.close();
            return true;
        }

        void saveAgents() const {
            ofstream myfile;
            myfile.open(agent_fname);
            if (!myfile.is_open()) {
                cout << "Fail to save the agents to " << agent_fname << endl;
                return;
            }
            myfile << num_of_agents << endl;
            for (int i = 0; i < num_of_agents; i++)
                myfile << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) << ","
                       << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << "," << endl;
            myfile.close();
        }

        void generateConnectedRandomGrid(int rows, int cols,
                                         int obstacles) { // initialize new [rows x cols] map with random obstacles
            cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
            int i, j;
            num_of_rows = rows + 2;
            num_of_cols = cols + 2;
            map_size = num_of_rows * num_of_cols;
            my_map.resize(map_size, false);
            // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
            /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
            moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
            moves_offset[Instance::valid_moves_t::EAST] = 1;
            moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
            moves_offset[Instance::valid_moves_t::WEST] = -1;*/

            // add padding
            i = 0;
            for (j = 0; j < num_of_cols; j++)
                my_map[linearizeCoordinate(i, j)] = true;
            i = num_of_rows - 1;
            for (j = 0; j < num_of_cols; j++)
                my_map[linearizeCoordinate(i, j)] = true;
            j = 0;
            for (i = 0; i < num_of_rows; i++)
                my_map[linearizeCoordinate(i, j)] = true;
            j = num_of_cols - 1;
            for (i = 0; i < num_of_rows; i++)
                my_map[linearizeCoordinate(i, j)] = true;

            // add obstacles uniformly at random
            i = 0;
            while (i < obstacles) {
                int loc = rand() % map_size;
                if (addObstacle(loc)) {
                    printMap();
                    i++;
                }
            }
        }

        void generateRandomAgents(int warehouse_width) {
            cout << "Generate " << num_of_agents << " random start and goal locations " << endl;
            vector<bool> starts(map_size, false);
            vector<bool> goals(map_size, false);
            start_locations.resize(num_of_agents);
            goal_locations.resize(num_of_agents);

            if (warehouse_width == 0)//Generate agents randomly
            {
                // Choose random start locations
                int k = 0;
                while (k < num_of_agents) {
                    int x = rand() % num_of_rows, y = rand() % num_of_cols;
                    int start = linearizeCoordinate(x, y);
                    if (my_map[start] || starts[start])
                        continue;

                    // update start
                    start_locations[k] = start;
                    starts[start] = true;

                    // find goal
                    bool flag = false;
                    int goal = randomWalk(start, RANDOM_WALK_STEPS);
                    while (goals[goal])
                        goal = randomWalk(goal, 1);

                    //update goal
                    goal_locations[k] = goal;
                    goals[goal] = true;

                    k++;
                }
            } else //Generate agents for warehouse scenario
            {
                // Choose random start locations
                int k = 0;
                while (k < num_of_agents) {
                    int x = rand() % num_of_rows, y = rand() % warehouse_width;
                    if (k % 2 == 0)
                        y = num_of_cols - y - 1;
                    int start = linearizeCoordinate(x, y);
                    if (starts[start])
                        continue;
                    // update start
                    start_locations[k] = start;
                    starts[start] = true;

                    k++;
                }
                // Choose random goal locations
                k = 0;
                while (k < num_of_agents) {
                    int x = rand() % num_of_rows, y = rand() % warehouse_width;
                    if (k % 2 == 1)
                        y = num_of_cols - y - 1;
                    int goal = linearizeCoordinate(x, y);
                    if (goals[goal])
                        continue;
                    // update goal
                    goal_locations[k] = goal;
                    goals[goal] = true;
                    k++;
                }
            }
        }

        bool addObstacle(int obstacle) { // add this obsatcle only if the map is still connected
            if (my_map[obstacle])
                return false;
            my_map[obstacle] = true;
            int obstacle_x = getRowCoordinate(obstacle);
            int obstacle_y = getColCoordinate(obstacle);
            int x[4] = {obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1};
            int y[4] = {obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y};
            int start = 0;
            int goal = 1;
            while (start < 3 && goal < 4) {
                if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols
                    || my_map[linearizeCoordinate(x[start], y[start])])
                    start++;
                else if (goal <= start)
                    goal = start + 1;
                else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols
                         || my_map[linearizeCoordinate(x[goal], y[goal])])
                    goal++;
                else if (isConnected(linearizeCoordinate(x[start], y[start]),
                                     linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal
                {
                    start = goal;
                    goal++;
                } else {
                    my_map[obstacle] = false;
                    return false;
                }
            }
            return true;
        }

        bool isConnected(int start,
                         int goal) const { // run BFS to find a path between start and goal, return true if a path exists.
            std::queue<int> open;
            vector<bool> closed(map_size, false);
            open.push(start);
            closed[start] = true;
            while (!open.empty()) {
                int curr = open.front();
                open.pop();
                if (curr == goal)
                    return true;
                for (int next : getNeighbors(curr)) {
                    if (closed[next])
                        continue;
                    open.push(next);
                    closed[next] = true;
                }
            }
            return false;
        }

        int randomWalk(int curr, int steps) const {
            for (int walk = 0; walk < steps; walk++) {
                list<int> l = getNeighbors(curr);
                vector<int> next_locations(l.cbegin(), l.cend());
                auto rng = std::default_random_engine{};
                std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
                for (int next : next_locations) {
                    if (validMove(curr, next)) {
                        curr = next;
                        break;
                    }
                }
            }
            return curr;
        }

        // Class  SingleAgentSolver can access private members of Node
        friend class SingleAgentSolver;
    };

}