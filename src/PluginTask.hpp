#ifndef rock_gazebo_PLUGIN_TASK_HPP
#define rock_gazebo_PLUGIN_TASK_HPP

#include <rock_gazebo/details/rttfwd.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

namespace rock_gazebo {
    class PluginTask : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemReset {

        struct Task {
            RTT::TaskContext* task = nullptr;
            RTT::base::ActivityInterface* activity = nullptr;
            sdf::ElementConstPtr sdf;
        };
        std::vector<Task> m_tasks;

        void processLoads(sdf::ElementConstPtr plugin_sdf);
        void processTasks(std::string const& prefix, sdf::ElementConstPtr plugin_sdf);
        void configurePluginTasks(gz::sim::Entity entity,
            sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager);
        void deleteAllTasks();

    public:
        ~PluginTask() override;

        /** Method called during the Configure step of the gazebo lifecycle
         *
         * @param entity the entity the <plugin> tag is attached to
         * @param sdf the SDF element representing the <plugin ...> tag for
         *    this plugin. It has no parent (so, can't discover the SDF definition
         *    of the entity)
         */
        void Configure(gz::sim::Entity const& entity,
            std::shared_ptr<const sdf::Element> const& plugin_sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager) override;

        /** Method called during the PreUpdate step of the gazebo lifecycle
         */
        void PreUpdate(gz::sim::UpdateInfo const& info,
            gz::sim::EntityComponentManager& ecm) override;

        /** Method called during the Reset step of the gazebo lifecycle
         */
        void Reset(const gz::sim::UpdateInfo& info,
            gz::sim::EntityComponentManager& ecm) override;
    };
}

#endif
