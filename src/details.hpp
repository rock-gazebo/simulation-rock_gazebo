#ifndef rock_gazebo_DETAILS_HPP
#define rock_gazebo_DETAILS_HPP

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <rock_gazebo/details/rttfwd.hpp>
#include <sdf/Element.hh>

namespace rock_gazebo {
    namespace details {
        RTT::TaskContext* instanciateTask(std::string const& prefix,
            sdf::ElementConstPtr task_sdf);
        RTT::base::ActivityInterface* setupTaskActivity(RTT::TaskContext* task);

        gz::sim::Entity resolveSubmodelRecursive(gz::sim::Entity const& root,
            std::list<std::string> const& names,
            gz::sim::EntityComponentManager& ecm);

        gz::sim::Entity resolveSubmodelRecursive(gz::sim::Entity const& root,
            std::string const& scoped_name,
            gz::sim::EntityComponentManager& ecm);

        std::list<std::string> splitScopedName(std::string const& scopedName);
    }
}

#endif
