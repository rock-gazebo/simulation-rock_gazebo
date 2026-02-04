#include <gz_rock/details.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <gz/common/Console.hh>

using namespace std;
using namespace gz_rock;

RTT::TaskContext* details::instanciateTask(sdf::ElementConstPtr task_sdf) {
    string name  = task_sdf->Get<string>("name");
    string model = task_sdf->Get<string>("model");
    string file  = task_sdf->Get<string>("filename");

    auto component_loader = RTT::ComponentLoader::Instance();

    if (!file.empty()) {
        component_loader->loadLibrary(file);
    }

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
RTT::base::ActivityInterface* details::setupTaskActivity(RTT::TaskContext* task) {
    // Create and start sequential task activities
    RTT::extras::SlaveActivity* activity =
        new RTT::extras::SlaveActivity(task->engine());
    activity->start();

    // Export the component interface on CORBA to Ruby access the component
    RTT::corba::TaskContextServer::Create( task );
#if RTT_VERSION_GTE(2,8,99)
    task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);
#else
    RTT::corba::CorbaDispatcher::Instance(task->ports(), ORO_SCHED_OTHER, RTT::os::LowestPriority);
#endif

    return activity;
}