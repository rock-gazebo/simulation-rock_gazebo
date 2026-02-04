#ifndef GZ_ROCK_DETAILS_HPP
#define GZ_ROCK_DETAILS_HPP

#include <sdf/Element.hh>
#include <gz_rock/details/rttfwd.hpp>

namespace gz_rock {
    namespace details {
        RTT::TaskContext* instanciateTask(sdf::ElementConstPtr task_sdf);
        RTT::base::ActivityInterface* setupTaskActivity(RTT::TaskContext* task);
    }
}

#endif
