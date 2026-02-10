//======================================================================================
#include "RockSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <sdf/Element.hh>
#include <std/transports/corba/TransportPlugin.hpp>
#include <std/transports/mqueue/TransportPlugin.hpp>
#include <std/transports/typelib/TransportPlugin.hpp>
#include <std/typekit/Plugin.hpp>

#include <base/transports/corba/TransportPlugin.hpp>
#include <base/transports/mqueue/TransportPlugin.hpp>
#include <base/transports/typelib/TransportPlugin.hpp>
#include <base/typekit/Plugin.hpp>

#include <rock_gazebo/transports/corba/TransportPlugin.hpp>
#include <rock_gazebo/transports/mqueue/TransportPlugin.hpp>
#include <rock_gazebo/transports/typelib/TransportPlugin.hpp>
#include <rock_gazebo/typekit/Plugin.hpp>

#include <logger/Logger.hpp>
#include <logger/transports/corba/TransportPlugin.hpp>
#include <logger/transports/mqueue/TransportPlugin.hpp>
#include <logger/transports/typelib/TransportPlugin.hpp>
#include <logger/typekit/Plugin.hpp>

#include <gps_base/transports/corba/TransportPlugin.hpp>
#include <gps_base/transports/mqueue/TransportPlugin.hpp>
#include <gps_base/transports/typelib/TransportPlugin.hpp>
#include <gps_base/typekit/Plugin.hpp>

#include <rtt/Activity.hpp>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <stdexcept>

using namespace std;
using namespace gz::sim;
using namespace rock_gazebo;
using sdf::ElementConstPtr;

RockSystem::RockSystem()
{
}

RockSystem::~RockSystem()
{
    if (m_logger_task) {
        RTT::corba::CorbaDispatcher::Release(m_logger_task->ports());
        RTT::corba::TaskContextServer::CleanupServer(m_logger_task);
        delete m_logger_activity;
        delete m_logger_task;
    }

    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
}

void RockSystem::Configure(Entity const& world_entity,
    std::shared_ptr<const sdf::Element> const& plugin_sdf,
    EntityComponentManager& ecm,
    EventManager& event_manager)
{
    initCORBA();
    loadStandardTypekits();

    auto worldName = World(world_entity).Name(ecm).value_or("world");
    RTT::Logger::In in("rock-gazebo");
    createInProcessLogger(worldName);
}

void RockSystem::initCORBA()
{
    const char* argv[] = {""};
    RTT::corba::ApplicationServer::InitOrb(1, const_cast<char**>(argv));
    RTT::corba::TaskContextServer::ThreadOrb(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0);
}

// Callback method triggered every update begin
// It triggers all rock components (world, model and plugins)
void RockSystem::PostUpdate(UpdateInfo const& info, EntityComponentManager const& ecm)
{
    m_logger_activity->execute();
}

void RockSystem::loadStandardTypekits()
{
    // Import typekits to allow RTT convert the types used by the components
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::baseTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::baseTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseTypekitPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::gps_baseCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::gps_baseMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::gps_baseTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::rock_gazeboTypekitPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::rock_gazeboCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::rock_gazeboMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::rock_gazeboTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypekitPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::loggerCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::loggerMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(
        new orogen_typekits::loggerTypelibTransportPlugin);
}

void RockSystem::createInProcessLogger(std::string const& worldName)
{
    // Create the logger component and start the activity
    logger::Logger* logger_task = new logger::Logger();
    logger_task->provides()->setName("gazebo::" + worldName + "_Logger");

    // RTT::Activity runs the task in separate thread
    RTT::Activity* logger_activity = new RTT::Activity(logger_task->engine());
    RTT::corba::TaskContextServer::Create(logger_task);

#if RTT_VERSION_GTE(2, 8, 99)
    logger_task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    logger_task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);
#else
    RTT::corba::CorbaDispatcher::Instance(logger_task->ports(),
        ORO_SCHED_OTHER,
        RTT::os::LowestPriority);
#endif
    logger_activity->start();

    m_logger_activity = logger_activity;
    m_logger_task = logger_task;
}

// Register plugin
GZ_ADD_PLUGIN(RockSystem,
    gz::sim::System,
    RockSystem::ISystemConfigure,
    RockSystem::ISystemPostUpdate);
