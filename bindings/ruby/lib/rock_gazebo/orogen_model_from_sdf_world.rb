# frozen_string_literal: true

module RockGazebo
    # Creation of an oroGen deployment model representing what the
    # rock-gazebo plugin would do from a SDF world
    #
    # @param [String] name the name that should be used as deployment name
    # @param [SDF::World] world the SDF world that we have to represent
    # @param [OroGen::Loaders::Base] loader the oroGen loader that we should use
    #   to create the tasks
    # @return [OroGen::Spec::Deployment]
    def self.orogen_model_from_sdf_world(
        name, world, loader: Orocos.default_loader, period: 0.1
    )
        project = OroGen::Spec::Project.new(loader)
        project.using_task_library "logger"
        project.using_task_library "rock_gazebo"
        deployment = project.deployment(name)
        setup_orogen_model_from_sdf_world(deployment, world, period: period)
    end

    # Add tasks to a deployment, matching the tasks that would be deployed by
    # the rock-gazebo plugin
    #
    # This method configures an orogen deployment model to match what the
    # rock-gazebo plugin would do when given the provided SDF world
    #
    # @param [OroGen::Spec::Deployment] deployment the model to modify
    # @param [SDF::World] world the SDF world
    # @param [Float] period the update period, which should match the SDF's physics
    #   update rate.
    #
    # @see {.orogen_model_from_sdf_world}
    def self.setup_orogen_model_from_sdf_world(deployment, world, period: 0.1)
        deployment.task("gazebo::#{world.name}_Logger", "logger::Logger")
                  .periodic(period)

        setup_orogen_model_from_plugin_tasks(deployment, world, "gazebo::#{world.name}", period)

        deployment
    end

    # @api private
    #
    # Define tasks in an orogen deployment that match the name and models of tasks
    # declared with the rock_gazebo::PluginTask plugin
    def self.setup_orogen_model_from_plugin_tasks(deployment, context, prefix, period)
        context.each_direct_plugin do |plugin|
            if plugin.name == "rock_gazebo::PluginTask"
                plugin.xml.elements.each("task") do |el|
                    name = [prefix, el.attributes["name"]].compact.join("::")
                    deployment.task(name, el.attributes["model"])
                              .periodic(period)
                end
            end
        end

        context.each_direct_model do |model|
            setup_orogen_model_from_plugin_tasks(
                deployment, model, "#{prefix}::#{model.name}", period
            )
        end
    end
end
