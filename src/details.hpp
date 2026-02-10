#ifndef rock_gazebo_DETAILS_HPP
#define rock_gazebo_DETAILS_HPP

#include <sdf/Element.hh>
#include <rock_gazebo/details/rttfwd.hpp>

namespace rock_gazebo {
    namespace details {
        RTT::TaskContext* instanciateTask(sdf::ElementConstPtr task_sdf);
        RTT::base::ActivityInterface* setupTaskActivity(RTT::TaskContext* task);
    }
}

#endif
