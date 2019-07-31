//======================================================================================
#include "RockBridge.hpp"
#include "Gazebo7Shims.hpp"

#include <std/typekit/Plugin.hpp>
#include <std/transports/corba/TransportPlugin.hpp>
#include <std/transports/typelib/TransportPlugin.hpp>
#include <std/transports/mqueue/TransportPlugin.hpp>

#include <base/typekit/Plugin.hpp>
#include <base/transports/corba/TransportPlugin.hpp>
#include <base/transports/typelib/TransportPlugin.hpp>
#include <base/transports/mqueue/TransportPlugin.hpp>

#include <rock_gazebo/ModelTask.hpp>
#include <rock_gazebo/WorldTask.hpp>
#include <rock_gazebo/ThrusterTask.hpp>
#include <rock_gazebo/LaserScanTask.hpp>
#include <rock_gazebo/CameraTask.hpp>
#include <rock_gazebo/ImuTask.hpp>
#include <rock_gazebo/GPSTask.hpp>
#include <rock_gazebo/typekit/Plugin.hpp>
#include <rock_gazebo/transports/corba/TransportPlugin.hpp>
#include <rock_gazebo/transports/typelib/TransportPlugin.hpp>
#include <rock_gazebo/transports/mqueue/TransportPlugin.hpp>

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
#include <rtt/extras/SequentialActivity.hpp>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>


using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

struct GazeboActivity : RTT::extras::SequentialActivity
{
    using SequentialActivity::SequentialActivity;

    bool trigger()
    {
        return false;
    }
    bool execute()
    {
        return RTT::extras::SequentialActivity::trigger();
    }
};

RockBridge::RockBridge()
{
}

RockBridge::~RockBridge()
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

void RockBridge::Load(int _argc , char** _argv)
{
    RTT::corba::ApplicationServer::InitOrb(_argc, _argv);
    RTT::corba::TaskContextServer::ThreadOrb(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0);

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

    RTT::types::TypekitRepository::Import(new orogen_typekits::rock_gazeboTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::rock_gazeboCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::rock_gazeboMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::rock_gazeboTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypelibTransportPlugin);

    // Each simulation step the Update method is called to update the simulated sensors and actuators
    eventHandler.push_back(
            event::Events::ConnectWorldUpdateBegin(
                boost::bind(&RockBridge::updateBegin,this, _1)));
    eventHandler.push_back(
            event::Events::ConnectWorldCreated(
                boost::bind(&RockBridge::worldCreated,this, _1)));
}

// worldCreated() is called every time a world is added
void RockBridge::worldCreated(string const& worldName)
{
    RTT::Logger::In in("rock-gazebo");

    // Create the logger component and start the activity
    logger::Logger* logger_task = new logger::Logger();
    logger_task->provides()->setName("gazebo:" + worldName +"_Logger");
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

    physics::WorldPtr world = physics::get_world(worldName);
    if (!world)
    {
        gzerr << "RockBridge: cannot find world " << worldName << endl;
        return;
    }

    gzmsg << "RockBridge: initializing world: " << worldName << endl;
    WorldTask* world_task = new WorldTask();
    world_task->setGazeboWorld(world);
    setupTaskActivity(world_task);

    typedef physics::Model_V Model_V;
    Model_V model_list = GzGet((*world), Models, ());
    sdf::ElementPtr worldSDF;
    for(Model_V::iterator model_it = model_list.begin(); model_it != model_list.end(); ++model_it)
    {
        gzmsg << "RockBridge: initializing model: "<< (*model_it)->GetName() << endl;
        sdf::ElementPtr modelElement = (*model_it)->GetSDF();

        if (!worldSDF)
            worldSDF = modelElement->GetParent();

        // Create and initialize a component for each gazebo model
        ModelTask* model_task = new ModelTask();
        model_task->setGazeboModel(world, (*model_it) );
        setupTaskActivity(model_task);

        // Create and initialize a component for each model plugin
        instantiatePluginComponents( modelElement, (*model_it) );

        // Create and initialize a component for each sensor
        instantiateSensorComponents( modelElement, (*model_it) );
    }

    if (worldSDF)
    {
        RTT::ComponentLoader::shared_ptr loader = RTT::ComponentLoader::Instance();

        sdf::ElementPtr pluginElement = worldSDF->GetElement("plugin");
        while (pluginElement)
        {
            if (pluginElement->Get<string>("name") == "rock_components")
                processRockComponentsPlugin(pluginElement);
            pluginElement = pluginElement->GetNextElement("plugin");
        }
    }
}

void RockBridge::processRockComponentsPlugin(sdf::ElementPtr pluginElement)
{
    auto plugin_loader = RTT::plugin::PluginLoader::Instance();
    sdf::ElementPtr loadElement = pluginElement->GetElement("load");
    while (loadElement)
    {
        string path = loadElement->Get<string>("path");
        if (!plugin_loader->loadLibrary(path))
            gzthrow("rock-gazebo: failed to load requested library " + path + "\n");

        gzmsg << "rock-gazebo: loaded library " << path << endl;
        loadElement = loadElement->GetNextElement("load");
    }

    auto component_loader = RTT::ComponentLoader::Instance();
    sdf::ElementPtr taskElement = pluginElement->GetElement("task");
    while (taskElement)
    {
        string name  = taskElement->Get<string>("name");
        string model = taskElement->Get<string>("model");
        string file  = taskElement->Get<string>("filename");

        if (!file.empty())
            component_loader->loadLibrary(file);

        RTT::TaskContext* task_context =
            component_loader->loadComponent(name, model);
        if (!task_context)
            gzthrow("rock-gazebo: failed to load task context " + name + " of model " + model + "\n");

        gzmsg << "rock-gazebo: created task " << name << " of model " << model << endl;
        setupTaskActivity(task_context);
        taskElement = taskElement->GetNextElement("task");
    }
}

void RockBridge::setupTaskActivity(RTT::TaskContext* task)
{
    // Export the component interface on CORBA to Ruby access the component
    RTT::corba::TaskContextServer::Create( task );
#if RTT_VERSION_GTE(2,8,99)
    task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);
#else
    RTT::corba::CorbaDispatcher::Instance(task->ports(), ORO_SCHED_OTHER, RTT::os::LowestPriority);
#endif

    // Create and start sequential task activities
    GazeboActivity* activity =
        new GazeboActivity(task->engine());
    activity->start();
    activities.push_back(activity);
    tasks.push_back(task);
}

// Callback method triggered every update begin
// It triggers all rock components (world, model and plugins)
void RockBridge::updateBegin(common::UpdateInfo const& info)
{
    for(Activities::iterator it = activities.begin(); it != activities.end(); ++it)
    {
        (*it)->execute();
    }
}


void RockBridge::instantiatePluginComponents(sdf::ElementPtr modelElement, ModelPtr model)
{
    sdf::ElementPtr pluginElement = modelElement->GetElement("plugin");
    while(pluginElement) {
        string filename = pluginElement->Get<string>("filename");
        string name = pluginElement->Get<string>("name");
        gzmsg << "RockBridge: found plugin name='" << name << "' "
              << "filename='" << filename << "'" << endl;

        // Add more model plugins testing them here
        if(pluginFilename == "libgazebo_thruster.so")
        {
            auto* task = new ThrusterTask();
            task->setGazeboModel(name, model);
            setupTaskActivity(task);
        }

        pluginElement = pluginElement->GetNextElement("plugin");
    }
}

template<typename RockTask>
void RockBridge::setupSensorTask(ModelPtr model, sdf::ElementPtr sensorElement)
{
    string sensorName = sensorElement->Get<string>("name");
    string sensorType = sensorElement->Get<string>("type");
    gzmsg << "RockBridge: creating " << sensorType << " component: " + sensorName << endl;
    rock_gazebo::SensorTask* task = new RockTask();
    task->setGazeboModel(model, sensorElement);
    setupTaskActivity(task);
}

void RockBridge::instantiateSensorComponents(sdf::ElementPtr modelElement, ModelPtr model)
{
    sdf::ElementPtr linkElement = modelElement->GetElement("link");
    while( linkElement ){
        sdf::ElementPtr sensorElement = linkElement->GetElement("sensor");
        while( sensorElement ){
            string sensorName = sensorElement->Get<string>("name");
            string sensorType = sensorElement->Get<string>("type");

            if (sensorType == "ray")
                setupSensorTask<LaserScanTask>(model, sensorElement);
            else if(sensorType == "camera")
                setupSensorTask<CameraTask>(model, sensorElement);
            else if(sensorType == "imu")
                setupSensorTask<ImuTask>(model, sensorElement);
            else if (sensorType == "gps")
                setupSensorTask<GPSTask>(model, sensorElement);
            else
                gzmsg << "RockGazebo: cannot handle sensor " << sensorName << " of type " << sensorType << endl;

            sensorElement = sensorElement->GetNextElement("sensor");
        }
        linkElement = linkElement->GetNextElement("link");
    }
}

