//
// Created by yaozhuo on 2024/6/27.
//

#ifndef LAYEREDMAPF_INSTANCE_SERIALIZE_AND_DESERIALIZE_H
#define LAYEREDMAPF_INSTANCE_SERIALIZE_AND_DESERIALIZE_H

#include "common.h"
#include "circle_shaped_agent.h"
#include "block_shaped_agent.h"
#include <ostream>
#include <istream>
#include "../basic.h"
#include <iostream>
#include <fstream>

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N>
    struct InstanceDeserializer {
        InstanceDeserializer() {}

        bool loadInstanceFromFile(const std::string& file_path, DimensionLength* dim) {
            std::ifstream is(file_path, std::ios::in);
            if(!is.is_open()) { return false; }
            agents_.clear();
            instances_.clear();
            strs_.clear();
            std::string line;
            while(getline(is, line)) {
                std::string copy_of_line = line;

                std::vector<std::string> strs;
                boost::split(strs, copy_of_line, boost::is_any_of(" "), boost::token_compress_on);

                std::pair<AgentPtr<N>, InstanceOrient<N> > temp_pair = {nullptr, {Pose<int, N>(), Pose<int, N>()}};
                if(strs[0] == "Circle") {
                    temp_pair = CircleAgent<N>::deserialize(line, dim);
                } else if(strs[0] == "Block_2D") {
                    temp_pair = BlockAgent_2D::deserialize(line, dim);
                } else {
                    std::cout << "undefined agent type" << std::endl;
                    assert(0);
                    break;
                }

                agents_.push_back(temp_pair.first);
                instances_.push_back(temp_pair.second);
                strs_.push_back(line);
            }
            return true;
        }

        std::vector<AgentPtr<N> > getAgents() const {
            return agents_;
        }

        InstanceOrients<N> getInstances() const {
            return instances_;
        }

        std::vector<std::pair<AgentPtrs<N>, InstanceOrients<N> > > getTestInstance(const std::vector<int>& required_counts,
                                                                                   int instance_count) {
            std::vector<std::set<int> > case_id_set = pickCasesFromScene<N>(agents_.size(), required_counts, instance_count);
            std::vector<std::pair<std::vector<AgentPtr<N> >, InstanceOrients<N> > > retv;
            for(int i=0; i<case_id_set.size(); i++) {
                std::pair<std::vector<AgentPtr<N> >, InstanceOrients<N> > instance;
                int local_id = 0;
                for(const auto& agent_id : case_id_set[i]) {
                    auto agent_copy = agents_[agent_id]->copy();
                    agent_copy->id_ = local_id;
                    instance.first.push_back(agent_copy);
                    local_id ++;
                    instance.second.push_back(instances_[agent_id]);
                }
                retv.push_back(instance);
            }
            return retv;
        }

        std::vector<std::string> getTextString() const {
            return strs_;
        }

        std::vector<AgentPtr<N> > agents_;
        InstanceOrients<N> instances_;

        std::vector<std::string> strs_;
    };

    template<Dimension N>
    struct InstanceSerializer {
        InstanceSerializer(const std::vector<AgentPtr<N> >& agents, const InstanceOrients<N>& instance)
            : agents_(agents), instance_(instance) {
            //
        }

        bool saveToFile(const std::string& file_path) {
            if(agents_.empty() || instance_.empty()) {
                std::cout << "serializer have nothing to save" << std::endl;
                return true;
            }
            strs_.clear();
            std::ofstream os(file_path, std::ios::trunc);
            if(!os.is_open()) { return false; }
            for(int i=0; i<agents_.size(); i++) {
                std::string str = agents_[i]->serialize(instance_[i].first, instance_[i].second);
                os << str << "\n";
                strs_.push_back(str);
            }
            os.close();
            return true;
        };

        bool saveStrsToFile(const std::vector<std::string>& strs, const std::string& file_path) {
            strs_.clear();
            std::ofstream os(file_path, std::ios::trunc);
            if(!os.is_open()) { return false; }
            for(const auto& str : strs) {
                os << str << "\n";
            }
            os.close();
            return true;
        };

        std::vector<std::string> getTextString() const {
            return strs_;
        }

        std::vector<AgentPtr<N> > agents_;
        InstanceOrients<N> instance_;

        std::vector<std::string> strs_;

    };
}
#endif //LAYEREDMAPF_INSTANCE_SERIALIZE_AND_DESERIALIZE_H
