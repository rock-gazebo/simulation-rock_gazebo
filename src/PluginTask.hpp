#ifndef GZ_ROCK_PLUGIN_TASK_HPP
#define GZ_ROCK_PLUGIN_TASK_HPP

#include <gz_rock/details/rttfwd.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

namespace gz_rock {
    class PluginTask : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemUpdate,
                       public gz::sim::ISystemReset {
        typedef std::vector<RTT::TaskContext*> Tasks;
        Tasks m_tasks;
        typedef std::vector<RTT::base::ActivityInterface*> Activities;
        Activities m_activities;

        void processLoads(sdf::ElementConstPtr plugin_sdf);
        void processTasks(sdf::ElementConstPtr plugin_sdf);
        void configurePluginTasks(gz::sim::Entity entity,
            sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager);
        void deleteAllTasks();

    public:
        ~PluginTask() override;

        void Configure(gz::sim::Entity const& entity,
            std::shared_ptr<const sdf::Element> const& plugin_sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager) override;

        void Update(gz::sim::UpdateInfo const& info,
            gz::sim::EntityComponentManager& ecm) override;

        void Reset(const gz::sim::UpdateInfo& info,
            gz::sim::EntityComponentManager& ecm) override;
    };
}

#endif