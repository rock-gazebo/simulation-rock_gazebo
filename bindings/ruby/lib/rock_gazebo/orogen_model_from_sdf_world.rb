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

    SENSORS_TASK_MODELS = {
        "ray" => "rock_gazebo::LaserScanTask",
        "camera" => "rock_gazebo::CameraTask",
        "imu" => "rock_gazebo::ImuTask",
        "gps" => "rock_gazebo::GPSTask"
    }.freeze

    PLUGIN_TASK_MODELS = {
        /gazebo_thruster/ => "rock_gazebo::ThrusterTask",
        /gazebo_underwater/ => "rock_gazebo::UnderwaterTask"
    }.freeze

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
        deployment.task("gazebo:#{world.name}", "rock_gazebo::WorldTask")
                  .periodic(period)
        deployment.task("gazebo:#{world.name}_Logger", "logger::Logger")
                  .periodic(period)

        world.each_model do |model|
            setup_orogen_model_from_sdf_model(
                deployment, model, prefix: "gazebo:#{world.name}:", period: period
            )
        end

        world.each_plugin do |plugin|
            if plugin.name == "rock_components"
                setup_orogen_model_from_rock_components_plugin(plugin)
            end
        end

        deployment
    end

    # @api private
    #
    # Add the tasks deployed through the rock_components "plugin" to an orogen model
    #
    # @param [OroGen::Spec::Deployment] deployment the deployment model to modify
    # @param [SDF::Plugin] the plugin
    # @param [Float] period the task's period in seconds
    def self.setup_orogen_model_from_components_plugin(deployment, plugin, period: 0.1)
        plugin.xml.elements.each("task") do |el|
            deployment.task(el.attributes["name"], el.attributes["model"])
                      .periodic(period)
        end
    end

    # @api private
    #
    # Describe a model's rock task to a deployment, describing the rock-gazebo
    # plugin's behavior
    #
    # The model's own task name is always gazebo:${world}:${model}. The method
    # also adds tasks for the models sensors and plugins. See
    # {#setup_orogen_model_from_sdf_model_sensor} and {#setup_orogen_model_from_sdf_model_plugin}
    #
    # @param [OroGen::Spec::Deployment] deployment the deployment model to modify
    # @param [SDF::Model] the sensor description
    # @param [String] prefix the string to be put before the plugin name in
    #   the generated task name
    # @param [Float] period the task's period in seconds
    def self.setup_orogen_model_from_sdf_model(deployment, model, prefix: "", period: 0.1)
        deployment.task("#{prefix}#{model.name}", "rock_gazebo::ModelTask")
                  .periodic(period)

        model.each_sensor do |sensor|
            setup_orogen_model_from_sdf_model_sensor(
                deployment, sensor, prefix: "#{prefix}#{model.name}:", period: period
            )
        end

        model.each_plugin do |plugin|
            setup_orogen_model_from_sdf_model_plugin(
                deployment, plugin, prefix: "#{prefix}#{model.name}:", period: period
            )
        end
    end

    # @api private
    #
    # Describe a sensor's rock task to a deployment, describing the rock-gazebo
    # plugin's behavior
    #
    # The task name is always gazebo:${world}:${model}:${sensor}. The task model
    # is based on the sensor type, as listed in {SENSORS_TASK_MODELS}
    #
    # @param [OroGen::Spec::Deployment] deployment the deployment model to modify
    # @param [SDF::Sensor] the sensor description
    # @param [String] prefix the string to be put before the plugin name in
    #   the generated task name
    # @param [Float] period the task's period in seconds
    def self.setup_orogen_model_from_sdf_model_sensor(
        deployment, sensor, prefix: "", period: 0.1
    )
        return unless (task_model = SENSORS_TASK_MODELS[sensor.type])

        deployment.task("#{prefix}#{sensor.name}", task_model)
                  .periodic(period)
    end

    # @api private
    #
    # Describe a plugin's rock task to a deployment, describing the rock-gazebo
    # plugin's behavior
    #
    # Tasks in plugins are explicitely specified with a <task .../> element.
    # Only the plugins listed in PLUGIN_TASK_MODELS have hardcoded mapping from
    # the plugin filename to the task model, for backward compatibility reasons
    #
    # The task name is always gazebo:${world}:${model}:${plugin}
    #
    # @param [OroGen::Spec::Deployment] deployment the deployment model to modify
    # @param [SDF::Plugin] the plugin description
    # @param [String] prefix the string to be put before the plugin name in
    #   the generated task name
    # @param [Float] period the task's period in seconds
    def self.setup_orogen_model_from_sdf_model_plugin(
        deployment, plugin, prefix: "", period: 0.1
    )
        task_model = PLUGIN_TASK_MODELS.find { |k, _| k === plugin.filename }&.last

        return unless task_model

        deployment.task("#{prefix}#{plugin.name}", task_model).periodic(period)
    end
end
