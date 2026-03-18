#include "PluginTask.hpp"
#include "rock_gazebo/PluginTaskI.hpp"
#include "rock_gazebo/details.hpp"

#include <chrono>
#include <gz/plugin/RegisterMore.hh>
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
using namespace rock_gazebo;
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
        m_tasks.push_back({ task_context, activity, taskElement });

        taskElement = taskElement->GetNextElement("task");
    }
}

void PluginTask::configurePluginTasks(
    gz::sim::Entity entity,
    sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager
) {
    for (auto t: m_tasks) {
        auto plugin_task = dynamic_cast<PluginTaskI*>(t.task);
        if (!plugin_task) {
            continue;
        }

        plugin_task->setGazebo("", entity, t.sdf, ecm, event_manager);
    }
}

void PluginTask::Reset(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm
) {
    deleteAllTasks();
}

void PluginTask::deleteAllTasks() {
    for (auto t: m_tasks) {
        RTT::corba::CorbaDispatcher::Release(t.task->ports());
        RTT::corba::TaskContextServer::CleanupServer(t.task);
    }

    for (auto t: m_tasks) {
        delete t.activity;
        delete t.task;
        m_tasks.pop_back();
    }
}

void PluginTask::PreUpdate(UpdateInfo const& info, EntityComponentManager& ecm)
{
    auto sim_time_us = base::Time::fromMicroseconds(
        std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count()
    );
    for (auto t: m_tasks) {
        auto* plugin_task = dynamic_cast<PluginTaskI*>(t.task);
        if (plugin_task) {
            plugin_task->gazeboCriticalZone();
            plugin_task->setSimTime(sim_time_us);
        }
    }
    for (auto t: m_tasks) {
        t.activity->execute();
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
