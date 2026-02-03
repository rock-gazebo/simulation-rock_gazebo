#ifndef _ROCK_SYSTEM_HPP_
#define _ROCK_SYSTEM_HPP_

// Gazebo headers
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>

namespace RTT
{
    class TaskContext;
    namespace base
    {
        class ActivityInterface;
    }
}

namespace gz_rock
{
    class RockSystem: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
    {
        public:
            // Pure virtual function implementation
            virtual void Load(int _argc = 0, char** _argv = NULL);
            RockSystem();
            ~RockSystem();

            void initCORBA();
            void loadStandardTypekits();
            void createInProcessLogger(std::string const& worldName);

        private:
            virtual void Configure(
                gz::sim::Entity const& entity,
                std::shared_ptr<const sdf::Element> const& sdf,
                gz::sim::EntityComponentManager& ecm,
                gz::sim::EventManager& eventMgr
            ) override;

            // void modelAdded(std::string const&);
            void PreUpdate(gz::sim::UpdateInfo const& info, gz::sim::EntityComponentManager& ecm) override;
            void setupTaskActivity(RTT::TaskContext* task);

            void processRockComponentsPlugin(sdf::ElementConstPtr pluginElement);
            RTT::TaskContext* instanciateTask(sdf::ElementConstPtr taskElement);
            void instantiatePluginComponents(
                sdf::ElementConstPtr model_sdf, gz::sim::Entity model_entity,
                gz::sim::EntityComponentManager& ecm,
                gz::sim::EventManager& event_manager
            );
            void instantiateSensorComponents(
                sdf::ElementConstPtr model_sdf, gz::sim::Entity model_entity,
                gz::sim::EntityComponentManager& ecm,
                gz::sim::EventManager& event_manager
            );

            template<typename RockTask>
            void setupSensorTask(
                gz::sim::Entity model_entity,
                sdf::ElementConstPtr sensor_sdf,
                gz::sim::EntityComponentManager& ecm,
                gz::sim::EventManager& event_manager
            );

            typedef std::vector<RTT::TaskContext*> Tasks;
            Tasks tasks;
            typedef std::vector<RTT::base::ActivityInterface*> Activities;
            Activities activities;
    };
}

#endif
