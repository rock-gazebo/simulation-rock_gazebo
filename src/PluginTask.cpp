#include "PluginTask.hpp"
#include "gz_rock/PluginTaskI.hpp"
#include "gz_rock/details.hpp"

#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/base/ActivityInterface.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>

using namespace std;
using namespace gz_rock;
using namespace gz::sim;

PluginTask::~PluginTask() {
    deleteAllTasks();
}

void PluginTask::Configure(
    Entity const& entity,
    sdf::ElementConstPtr const& plugin_sdf,
    EntityComponentManager& ecm,
    EventManager& event_manager
) {
    processLoads(plugin_sdf);
    processTasks(plugin_sdf);
    configurePluginTasks(entity, plugin_sdf, ecm, event_manager);
}

void PluginTask::processLoads(sdf::ElementConstPtr plugin_sdf) {
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
}

void PluginTask::processTasks(sdf::ElementConstPtr plugin_sdf) {
    sdf::ElementConstPtr taskElement = plugin_sdf->FindElement("task");
    while (taskElement)
    {
        auto task_context = details::instanciateTask(taskElement);
        auto activity = details::setupTaskActivity(task_context);

        m_tasks.push_back(task_context);
        m_activities.push_back(activity);

        taskElement = taskElement->GetNextElement("task");
    }
}

void PluginTask::configurePluginTasks(
    gz::sim::Entity entity,
    sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
) {
    for (auto task: m_tasks) {
        auto plugin_task = dynamic_cast<PluginTaskI*>(task);
        if (!plugin_task) {
            continue;
        }

        plugin_task->setGazebo("", entity, plugin_sdf, ecm, event_manager);
    }
}

void PluginTask::Reset(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm
) {
    deleteAllTasks();
}

void PluginTask::deleteAllTasks() {
    for (auto task: m_tasks) {
        RTT::corba::CorbaDispatcher::Release(task->ports());
        RTT::corba::TaskContextServer::CleanupServer(task);
    }

    while (!m_activities.empty()) {
        delete m_activities.back();
        m_activities.pop_back();
    }

    while (!m_tasks.empty()) {
        delete m_tasks.back();
        m_tasks.pop_back();
    }
}

void PluginTask::PreUpdate(UpdateInfo const& info, EntityComponentManager& ecm)
{
    for (auto activity: m_activities) {
        activity->execute();
    }
}


// Register plugin
GZ_ADD_PLUGIN(
    PluginTask,
    gz::sim::System,
    PluginTask::ISystemConfigure,
    PluginTask::ISystemPreUpdate,
    PluginTask::ISystemReset
);