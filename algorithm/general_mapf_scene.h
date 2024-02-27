//
// Created by yaozhuo on 2023/6/8.
//

#ifndef FREENAV_LAYERED_MAPF_GENERAL_MAPF_SCENE_H
#define FREENAV_LAYERED_MAPF_GENERAL_MAPF_SCENE_H

#include <memory>

#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF {

    template <Dimension N>
    struct GeneralMAPFScene {
    public:

        explicit GeneralMAPFScene(DimensionLength* dimension_info,
                                  IS_OCCUPIED_FUNC<N> is_occupied,
                                  const Paths<N>& agent_paths) {
            dimension_info_ = dimension_info;
            is_occupied_ = is_occupied;
            agent_paths_ = agent_paths;
            makespan_ = 1;
            // update make span
            for(auto & agent_path : agent_paths_) {
                if(makespan_ < agent_path.size()) {
                    makespan_ = agent_path.size();
                }
            }
        }

        DimensionLength* dimension_info_;
        IS_OCCUPIED_FUNC<N> is_occupied_;
        Paths<N> agent_paths_; // path of each agent

        int makespan_;

    };

    template <Dimension N>
    using GeneralMAPFScenePtr = std::shared_ptr<GeneralMAPFScene<N> >;

}

#endif //FREENAV_GENERAL_MAPF_SCENE_H
