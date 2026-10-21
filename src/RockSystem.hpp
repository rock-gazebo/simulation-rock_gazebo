#ifndef _ROCK_SYSTEM_HPP_
#define _ROCK_SYSTEM_HPP_

// Gazebo headers
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

namespace RTT {
    class TaskContext;
    namespace base {
        class ActivityInterface;
    }
}

namespace rock_gazebo {
    class RockSystem : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPostUpdate {
    public:
        // Pure virtual function implementation
        RockSystem();
        ~RockSystem();

        void initCORBA();
        void loadStandardTypekits();
        void createInProcessLogger(std::string const& worldName);

    private:
        virtual void Configure(gz::sim::Entity const& entity,
            std::shared_ptr<const sdf::Element> const& sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& eventMgr) override;

        // void modelAdded(std::string const&);
        void PostUpdate(gz::sim::UpdateInfo const& info,
            gz::sim::EntityComponentManager const& ecm) override;

        RTT::TaskContext* m_logger_task = nullptr;
        RTT::base::ActivityInterface* m_logger_activity = nullptr;
    };
}

#endif
