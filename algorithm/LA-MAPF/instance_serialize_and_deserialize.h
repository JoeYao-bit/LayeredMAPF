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

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N, typename AgentType>
    struct InstanceDeserializer {
        InstanceDeserializer() {}

        bool loadInstanceFromFile(const std::string& file_path, DimensionLength* dim) {
            std::ifstream is(file_path, std::ios::in);
            if(!is.is_open()) { return false; }
            agents_.clear();
            instances_.clear();

            std::string line;
            while(getline(is, line)) {
                std::pair<AgentType, InstanceOrient<N> > temp_pair = AgentType::deserialize(line, dim);
                agents_.push_back(temp_pair.first);
                instances_.push_back(temp_pair.second);
            }
            return true;
        }

        std::vector<AgentType> getAgents() const {
            return agents_;
        }

        InstanceOrients<N> getInstances() const {
            return instances_;
        }

        std::vector<AgentType> agents_;
        InstanceOrients<N> instances_;
    };

    template<Dimension N, typename AgentType>
    struct InstanceSerializer {
        InstanceSerializer(const std::vector<AgentType>& agents, const InstanceOrients<N>& instance)
            : agents_(agents), instance_(instance) {
            //
        }

        bool saveToFile(const std::string& file_path) const {
            std::ofstream os(file_path, std::ios::trunc);
            if(!os.is_open()) { return false; }
            for(int i=0; i<agents_.size(); i++) {
                std::string str = agents_[i].serialize(instance_[i].first, instance_[i].second);
                os << str << "\n";
            }
            os.close();
            return true;
        };

        std::vector<AgentType> agents_;
        InstanceOrients<N> instance_;
    };
}
#endif //LAYEREDMAPF_INSTANCE_SERIALIZE_AND_DESERIALIZE_H
