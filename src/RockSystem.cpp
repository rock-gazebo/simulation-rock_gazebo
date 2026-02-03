//======================================================================================
#include "RockSystem.hpp"
#include <boost/bind/bind.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>

#include <std/typekit/Plugin.hpp>
#include <std/transports/corba/TransportPlugin.hpp>
#include <std/transports/typelib/TransportPlugin.hpp>
#include <std/transports/mqueue/TransportPlugin.hpp>

#include <base/typekit/Plugin.hpp>
#include <base/transports/corba/TransportPlugin.hpp>
#include <base/transports/typelib/TransportPlugin.hpp>
#include <base/transports/mqueue/TransportPlugin.hpp>

#include <gz_rock/ModelPluginTaskI.hpp>
#include <gz_rock/ModelTask.hpp>
#include <gz_rock/WorldTask.hpp>
#include <gz_rock/LaserScanTask.hpp>
#include <gz_rock/CameraTask.hpp>
#include <gz_rock/ImuTask.hpp>
#include <gz_rock/GPSTask.hpp>
#include <gz_rock/typekit/Plugin.hpp>
#include <gz_rock/transports/corba/TransportPlugin.hpp>
#include <gz_rock/transports/typelib/TransportPlugin.hpp>
#include <gz_rock/transports/mqueue/TransportPlugin.hpp>

#include <logger/Logger.hpp>
#include <logger/typekit/Plugin.hpp>
#include <logger/transports/corba/TransportPlugin.hpp>
#include <logger/transports/typelib/TransportPlugin.hpp>
#include <logger/transports/mqueue/TransportPlugin.hpp>

#include <gps_base/typekit/Plugin.hpp>
#include <gps_base/transports/corba/TransportPlugin.hpp>
#include <gps_base/transports/typelib/TransportPlugin.hpp>
#include <gps_base/transports/mqueue/TransportPlugin.hpp>

#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/base/ActivityInterface.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <stdexcept>

using namespace std;
using namespace gz::sim;
using namespace gz_rock;

RockSystem::RockSystem()
{
}

RockSystem::~RockSystem()
{
    // Deregister the CORBA stuff
    RTT::corba::TaskContextServer::CleanupServers();
    RTT::corba::CorbaDispatcher::ReleaseAll();

    // Delete pointers to activity
    for(Activities::iterator activity_it = activities.begin();
            activity_it != activities.end(); ++activity_it)
    {
        delete *activity_it;
    }

    // Delete pointers to tasks
    for(Tasks::iterator task_it = tasks.begin();
            task_it != tasks.end(); ++task_it)
    {
        delete *task_it;
    }

    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
}

void RockSystem::Load(int _argc , char** _argv)
{
}

void RockSystem::Configure(
    Entity const& world_entity,
    std::shared_ptr<const sdf::Element> const& world_sdf,
    EntityComponentManager& ecm,
    EventManager& event_manager
) {
    initCORBA();
    loadStandardTypekits();

    auto worldName = World(world_entity).Name(ecm).value_or("world");
    RTT::Logger::In in("rock-gazebo");
    createInProcessLogger(worldName);

    gzmsg << "RockSystem: initializing world: " << worldName << endl;
    WorldTask* world_task = new WorldTask();
    world_task->setGazebo(world_entity, world_sdf, ecm, event_manager);
    setupTaskActivity(world_task);

    auto model_list = World(world_entity).Models(ecm);
    for(auto const& model_entity: model_list)
    {
        auto model_name = Model(model_entity).Name(ecm);
        gzmsg << "RockSystem: initializing model: "<< model_name << endl;

        auto model_sdf = world_sdf->FindElement(model_name);

        // Create and initialize a component for each gazebo model
        ModelTask* model_task = new ModelTask();
        model_task->setGazebo(ecm, world_entity, model_entity);
        setupTaskActivity(model_task);

        // Create and initialize a component for each model plugin
        instantiatePluginComponents(model_sdf, model_entity, ecm, event_manager);

        // Create and initialize a component for each sensor
        instantiateSensorComponents(model_sdf, model_entity, ecm, event_manager);
    }

    RTT::ComponentLoader::shared_ptr loader = RTT::ComponentLoader::Instance();

    sdf::ElementConstPtr pluginElement = world_sdf->FindElement("plugin");
    while (pluginElement)
    {
        if (pluginElement->Get<string>("name") == "rock_components") {
            processRockComponentsPlugin(pluginElement);
        }
        pluginElement = pluginElement->GetNextElement("plugin");
    }
}

void RockSystem::initCORBA() {
    const char* argv[] = { "" };
    RTT::corba::ApplicationServer::InitOrb(1, const_cast<char**>(argv));
    RTT::corba::TaskContextServer::ThreadOrb(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0);
}

void RockSystem::processRockComponentsPlugin(sdf::ElementConstPtr plugin_sdf)
{
    auto plugin_loader = RTT::plugin::PluginLoader::Instance();
    sdf::ElementConstPtr loadElement = plugin_sdf->FindElement("load");
    while (loadElement)
    {
        string path = loadElement->Get<string>("path");
        if (!plugin_loader->loadLibrary(path)) {
            throw std::invalid_argument(
                "rock-gazebo: failed to load requested library " + path + "\n"
            );
        }

        gzmsg << "rock-gazebo: loaded library " << path << endl;
        loadElement = loadElement->GetNextElement("load");
    }

    sdf::ElementConstPtr taskElement = plugin_sdf->FindElement("task");
    while (taskElement)
    {
        auto task_context = instanciateTask(taskElement);
        setupTaskActivity(task_context);
        taskElement = taskElement->GetNextElement("task");
    }
}

RTT::TaskContext* RockSystem::instanciateTask(sdf::ElementConstPtr taskElement) {
    string name  = taskElement->Get<string>("name");
    string model = taskElement->Get<string>("model");
    string file  = taskElement->Get<string>("filename");

    auto component_loader = RTT::ComponentLoader::Instance();

    if (!file.empty())
        component_loader->loadLibrary(file);

    RTT::TaskContext* task_context =
        component_loader->loadComponent(name, model);
    if (!task_context) {
        throw std::logic_error(
            "rock-gazebo: failed to load task context " + name +
            " of model " + model + "\n"
        );
    }

    gzmsg << "rock-gazebo: created task " << name << " of model " << model << endl;
    return task_context;
}

void RockSystem::setupTaskActivity(RTT::TaskContext* task)
{
    // Create and start sequential task activities
    RTT::extras::SlaveActivity* activity =
        new RTT::extras::SlaveActivity(task->engine());
    activity->start();
    activities.push_back(activity);
    tasks.push_back(task);

    // Export the component interface on CORBA to Ruby access the component
    RTT::corba::TaskContextServer::Create( task );
#if RTT_VERSION_GTE(2,8,99)
    task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);
#else
    RTT::corba::CorbaDispatcher::Instance(task->ports(), ORO_SCHED_OTHER, RTT::os::LowestPriority);
#endif
}

// Callback method triggered every update begin
// It triggers all rock components (world, model and plugins)
void RockSystem::PreUpdate(UpdateInfo const& info, EntityComponentManager& ecm)
{
    for (Activities::iterator it = activities.begin();
         it != activities.end();
         ++it) {
        (*it)->execute();
    }
}

void RockSystem::instantiatePluginComponents(
    sdf::ElementConstPtr model_sdf, gz::sim::Entity model_entity,
    EntityComponentManager& ecm,
    EventManager& event_manager
) {
    sdf::ElementConstPtr plugin_sdf = model_sdf->FindElement("plugin");
    while(plugin_sdf) {
        string filename  = plugin_sdf->Get<string>("filename");
        string name      = plugin_sdf->Get<string>("name");
        gzmsg << "RockSystem: found plugin " << "name='" << name << std::endl;

        auto rock_task = plugin_sdf->FindElement("task");
        if (rock_task) {
            string rock_task_name = rock_task->Get<string>("name");
            auto taskContext = instanciateTask(rock_task);
            auto plugin_task = dynamic_cast<ModelPluginTaskI*>(taskContext);
            plugin_task->setGazeboPluginTaskName(rock_task_name);
            plugin_task->setGazebo(name, model_entity, plugin_sdf, ecm, event_manager);
            setupTaskActivity(taskContext);
        }

        plugin_sdf = plugin_sdf->GetNextElement("plugin");
    }
}

template<typename RockTask>
void RockSystem::setupSensorTask(
    Entity model_entity,
    sdf::ElementConstPtr sensor_sdf,
    EntityComponentManager& ecm,
    EventManager& event_manager
) {
    string name = sensor_sdf->Get<string>("name");
    string type = sensor_sdf->Get<string>("type");
    gzmsg << "RockSystem: creating " << type << " component: " + name << endl;
    gz_rock::SensorTask* task = new RockTask();
    task->setGazebo(model_entity, sensor_sdf, ecm, event_manager);
    setupTaskActivity(task);
}

void RockSystem::instantiateSensorComponents(
    sdf::ElementConstPtr model_sdf, Entity model_entity,
    EntityComponentManager& ecm,
    EventManager& event_manager
) {
    auto link_sdf = model_sdf->FindElement("link");
    while( link_sdf ){
        auto sensor_sdf = link_sdf->FindElement("sensor");
        while( sensor_sdf ){
            string sensorName = sensor_sdf->Get<string>("name");
            string sensorType = sensor_sdf->Get<string>("type");
            gzmsg << "RockGazebo: looking for a Rock interface for sensor"
                  << sensorName << " of type " << sensorType << endl;

            if (sensorType == "ray")
                setupSensorTask<LaserScanTask>(model_entity, sensor_sdf, ecm, event_manager);
            else if(sensorType == "camera")
                setupSensorTask<CameraTask>(model_entity, sensor_sdf, ecm, event_manager);
            else if(sensorType == "imu")
                setupSensorTask<ImuTask>(model_entity, sensor_sdf, ecm, event_manager);
            else if (sensorType == "gps")
                setupSensorTask<GPSTask>(model_entity, sensor_sdf, ecm, event_manager);
            else
                gzmsg << "RockGazebo: cannot handle sensor " << sensorName << " of type " << sensorType << endl;

            sensor_sdf = sensor_sdf->GetNextElement("sensor");
        }
        link_sdf = link_sdf->GetNextElement("link");
    }
}

void RockSystem::loadStandardTypekits() {
    // Import typekits to allow RTT convert the types used by the components
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::baseTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::gz_rockTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gz_rockCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gz_rockMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gz_rockTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypelibTransportPlugin);
}

void RockSystem::createInProcessLogger(std::string const& worldName) {
    // Create the logger component and start the activity
    logger::Logger* logger_task = new logger::Logger();
    logger_task->provides()->setName("gazebo::" + worldName + "_Logger");

    // RTT::Activity runs the task in separate thread
    RTT::Activity* logger_activity = new RTT::Activity( logger_task->engine() );
    RTT::corba::TaskContextServer::Create( logger_task );

#if RTT_VERSION_GTE(2,8,99)
    logger_task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    logger_task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);
#else
    RTT::corba::CorbaDispatcher::Instance(logger_task->ports(), ORO_SCHED_OTHER, RTT::os::LowestPriority);
#endif
    logger_activity->start();
    activities.push_back( logger_activity );
    tasks.push_back( logger_task );
}

// Register plugin
GZ_ADD_PLUGIN(
    RockSystem,
    gz::sim::System,
    RockSystem::ISystemConfigure,
    RockSystem::ISystemPreUpdate
);

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(
    RockSystem, "rock::gz"
);