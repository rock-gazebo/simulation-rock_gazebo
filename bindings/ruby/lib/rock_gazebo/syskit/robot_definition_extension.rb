# frozen_string_literal: true

module RockGazebo
    module Syskit
        # Gazebo-specific extensions to {Syskit::Robot::RobotDefinition}
        module RobotDefinitionExtension
            # Given a sensor, returns the device and device driver model that
            # should be used to handle it
            #
            # @return [nil,(Model<Syskit::Device>,Model<Syskit::Component>)]
            #   either nil if this type of sensor is not handled either by the
            #   rock-gazebo plugin or by the syskit integration (yet), or the
            #   device model and device driver that should be used for this
            #   sensor
            def sensors_to_device(sensor, device_name, frame_name)
                case sensor.type
                when 'ray'
                    require 'common_models/models/devices/gazebo/ray'
                    device(CommonModels::Devices::Gazebo::Ray,
                           as: device_name,
                           using: OroGen.rock_gazebo.LaserScanTask)
                        .frame(frame_name)
                when 'imu'
                    require 'common_models/models/devices/gazebo/imu'
                    device(CommonModels::Devices::Gazebo::Imu,
                           as: device_name,
                           using: OroGen.rock_gazebo.ImuTask)
                        .frame_transform(frame_name => 'world')
                when 'camera'
                    require 'common_models/models/devices/gazebo/camera'
                    device(CommonModels::Devices::Gazebo::Camera,
                           as: device_name,
                           using: OroGen.rock_gazebo.CameraTask)
                        .frame(frame_name)
                when 'gps'
                    require 'common_models/models/devices/gazebo/gps'
                    device(CommonModels::Devices::Gazebo::GPS,
                           as: device_name,
                           using: OroGen.rock_gazebo.GPSTask)
                        .frame_transform(frame_name => 'world')
                end
            end

            @plugin_device_mappings = {}
            def self.plugin_device_mappings
                @plugin_device_mappings ||= {}
            end

            # Registers a device model by a plugin task model
            def self.register_device_by_plugin_task_model(task_model, device_model)
                @plugin_device_mappings[task_model] = device_model
            end

            # Given a gazebo plugin, returns the device and device driver model that
            # should be used to handle it
            #
            # @return [nil,(Model<Syskit::Device>,Model<Syskit::Component>)]
            #   either nil if this type of sensor is not handled either by the
            #   rock-gazebo plugin or by the syskit integration (yet), or the
            #   device model and device driver that should be used for this
            #   sensor
            def plugins_to_device(plugin, device_name)
                has_task = false
                plugin.xml.elements.to_a("task").each do |task_element|
                    task_model_name = task_element.attributes["model"]
                    task_model = ::Syskit::TaskContext.find_model_from_orogen_name(task_model_name)
                    if (device = RobotDefinitionExtension.plugin_device_mappings[task_model])
                        return device(device,
                                      as: device_name,
                                      using: task_model)
                    end
                    # Maintain the old version for compatibility
                    has_task = true
                    case task_element.attributes["model"]
                    when "rock_gazebo::ThrusterTask"
                        require "common_models/models/devices/gazebo/thruster"
                        return device(
                            CommonModels::Devices::Gazebo::Thruster,
                            as: device_name,
                            using: OroGen.rock_gazebo.ThrusterTask
                        )
                    when "rock_gazebo::UnderwaterTask"
                        require "common_models/models/devices/gazebo/underwater"
                        return device(
                            CommonModels::Devices::Gazebo::Underwater,
                            as: device_name,
                            using: OroGen.rock_gazebo.UnderwaterTask
                        )
                    end
                end
                return if has_task

                case plugin.filename
                when /gazebo_thruster/
                    require "common_models/models/devices/gazebo/thruster"
                    device(CommonModels::Devices::Gazebo::Thruster,
                           as: device_name,
                           using: OroGen.rock_gazebo.ThrusterTask)
                when /gazebo_underwater/
                    require "common_models/models/devices/gazebo/underwater"
                    device(CommonModels::Devices::Gazebo::Underwater,
                           as: device_name,
                           using: OroGen.rock_gazebo.UnderwaterTask)
                end
            end

            # Create a device that provides access to a subset of the joints
            #
            # Use it for instance, to get a simulated actuator
            def sdf_export_joint(
                model_dev,
                as: nil, joint_names: [],
                ignore_joint_names: false,
                position_offsets: []
            )
                driver_def = model_dev.to_instance_requirements.to_component_model.dup
                driver_m = OroGen.rock_gazebo.ModelTask.specialize
                driver_srv = driver_m.require_dynamic_service(
                    'joint_export',
                    as: as, joint_names: joint_names,
                    position_offsets: position_offsets,
                    ignore_joint_names: ignore_joint_names
                )

                driver_def.add_models([driver_m])
                driver_def.select_service(driver_srv)

                device(CommonModels::Devices::Gazebo::Joint,
                       as: as, using: driver_def)
            end

            # Setup a link export feature of rock_gazebo::ModelTask
            #
            # @param [Syskit::Robot::MasterDeviceInstance] model_dev the model
            #   device that has the requested links
            # @param [String] as the name of the newly created device. It is
            #   also the name of the created port on the model task
            # @param [String] from_frame the 'from' frame of the exported link,
            #   which must match a link, model or frame name on the SDF model
            # @param [String] to_frame the 'to' frame of the exported link,
            #   which must match a link, model or frame name on the SDF model
            # @return [Syskit::Robot::MasterDeviceInstance] the exported link as
            #   a device instance of type CommonModels::Devices::Gazebo::Link
            def sdf_export_link(
                model_dev,
                as: nil, from_frame: nil, to_frame: nil,
                cov_position: nil, cov_orientation: nil,
                cov_velocity: nil
            )
                if !as
                    raise ArgumentError, 'provide a name for the device and port '\
                                         "through the 'as' option"
                elsif !from_frame
                    raise ArgumentError, "provide a name for the 'from' frame "\
                                         "through the 'from_frame' option"
                elsif !to_frame
                    raise ArgumentError, "provide a name for the 'to' frame "\
                                         "through the 'to_frame' option"
                end

                link_driver = model_dev.to_instance_requirements.to_component_model.dup
                link_driver_m = OroGen.rock_gazebo.ModelTask.specialize
                link_driver_srv = link_driver_m.require_dynamic_service(
                    'link_export',
                    as: as, frame_basename: as,
                    cov_position: cov_position,
                    cov_orientation: cov_orientation,
                    cov_velocity: cov_velocity
                )

                link_driver.add_models([link_driver_m])
                link_driver
                    .use_frames("#{as}_source" => from_frame,
                                "#{as}_target" => to_frame)
                    .select_service(link_driver_srv)

                dev = device(CommonModels::Devices::Gazebo::Link,
                             as: as, using: link_driver)
                dev.frame_transform(from_frame => to_frame) if from_frame != to_frame
                dev
            end

            def find_actual_model(name, candidates)
                candidates = candidates.dup
                until candidates.empty?
                    m, root = candidates.shift
                    return m, root if m.name == name

                    children = m.each_model.map { |child_m| [child_m, root || m] }
                    candidates.concat(children)
                end
                nil
            end

            def create_frame_mappings_for_used_model(model)
                @link_frame_names ||= {}
                @link_frame_names[model] = model.name
                model.each_link_with_name do |l, _|
                    @link_frame_names[l] = "#{model.name}::#{l.full_name(root: model)}"
                end
            end

            def link_frame_name(sdf)
                @link_frame_names ||= {}
                @link_frame_names[sdf] ||= sdf.full_name(
                    root: resolve_enclosing_world(sdf)
                )
            end

            def define_submodel_device(name, enclosing_device, actual_sdf_model)
                normalized_name = normalize_name(name)
                submodel_driver_m = OroGen.rock_gazebo.ModelTask.specialize
                driver_srv = submodel_driver_m.require_dynamic_service(
                    'submodel_export',
                    as: normalized_name, frame_basename: normalized_name
                )

                unless (root_link = actual_sdf_model.each_link.first)
                    raise ArgumentError, 'cannot refer to a submodel that has no links'
                end

                link_frame = link_frame_name(root_link)

                submodel_driver_m =
                    submodel_driver_m
                    .to_instance_requirements
                    .prefer_deployed_tasks(*enclosing_device
                                           .to_instance_requirements
                                           .deployment_hints)
                    .with_arguments(model_dev: enclosing_device)
                    .use_frames("#{normalized_name}_source" => link_frame,
                                "#{normalized_name}_target" => 'world')
                    .select_service(driver_srv)
                device(CommonModels::Devices::Gazebo::Model,
                       as: normalized_name, using: submodel_driver_m)
                    .doc("Gazebo: model #{name} inside #{enclosing_device.sdf.full_name}")
                    .frame_transform(link_frame => 'world')
                    .sdf(actual_sdf_model)
                    .advanced
            end

            # @api private
            #
            # Find the toplevel model that contains the given one
            #
            # @return [Model] the enclosing model. Might be the original model
            #   if it is toplevel.
            def resolve_enclosing_model(model)
                model = model.parent while model.parent.kind_of?(::SDF::Model)
                model
            end

            # @api private
            #
            # Find the world that contains the given model
            #
            # @return [World,nil] the enclosing world, or nil if the model is
            #   not included in one
            def resolve_enclosing_world(node)
                node = node.parent while node && !node.kind_of?(::SDF::World)
                node
            end

            # Create device information that models how the rock-gazebo plugin
            # will handle this SDF model
            #
            # I.e. it creates devices that match the tasks the rock-gazebo
            # plugin will create when given this SDF information
            #
            # It does it for all the models in 'world', and for links and
            # sensors only for 'model'
            #
            # @param [SDF::Model] robot_model the SDF model for this robot
            # @param [String] name the name of the model in the world, if it
            #   differs from the robot_model name (e.g. if you have multiple
            #   instances of the same robot)
            # @param [Array<SDF::Model>] models a set of models that should be
            #   exposed as devices on this robot model. Note that sensors and
            #   links are only exposed for the robot_model. It must contain
            #   robot_model
            # @param [Boolean] prefix_device_with_name if true, the name of
            #   the created devices are prefixed with the 'name' argument and an
            #   underscore. This will become the new default, and using false
            #   generates a deprecation warning. Setting it to true ensures that
            #   the device names stay the same regardless of the SDF world it's
            #   built in.
            # @return [Syskit::Robot::Device] the device that represents the model
            # @raise [ArgumentError] if models does not contain robot_model
            def load_gazebo(
                model, deployment_prefix,
                name: model.name, reuse: nil, prefix_device_with_name: false
            )
                # Allow passing a profile instead of a robot definition
                reuse = reuse.robot if reuse.respond_to?(:robot)
                enclosing_model = resolve_enclosing_model(model)

                unless prefix_device_with_name
                    Roby.warn_deprecated <<~EOMSG
                        The link naming scheme in #use_sdf_model and #use_gazebo_model will change from using the link
                        name as device name to prefixing it with the name given to #use_sdf_model (which defaults to the
                        model name itself) set the prefix_device_with_name: option to true to enable the new behavior
                        and remove this warning This warning will become an error before the functionality completely
                        disappears
                    EOMSG
                end

                if enclosing_model != model
                    create_frame_mappings_for_used_model(model)
                    enclosing_device = expose_gazebo_model(
                        enclosing_model, deployment_prefix,
                        reuse: reuse, device_name: enclosing_model.name
                    )
                    model_device = define_submodel_device(name, enclosing_device, model)
                    prefix_device_with_name = true
                else
                    enclosing_device = expose_gazebo_model(
                        enclosing_model, deployment_prefix,
                        reuse: reuse, device_name: name
                    )
                    model_device = enclosing_device
                    enclosing_device.advanced = false
                end
                load_gazebo_robot_model(model, enclosing_device,
                                        name: name, reuse: reuse,
                                        prefix_device_with_name: prefix_device_with_name)
                model_device
            end

            # @api private
            #
            # Define devices for each model in the world
            #
            # @param [Array<SDF::Model>] models the SDF representation of the models
            def expose_gazebo_model(
                sdf, deployment_prefix,
                reuse: nil, device_name: normalize_name(sdf.name)
            )
                if reuse && (existing = reuse.find_device(device_name))
                    register_device(device_name, existing)
                    return existing
                end

                device(CommonModels::Devices::Gazebo::RootModel,
                       as: device_name,
                       using: OroGen.rock_gazebo.ModelTask)
                    .prefer_deployed_tasks(
                        "#{deployment_prefix}:#{normalize_name(sdf.name)}"
                    )
                    .frame_transform(link_frame_name(sdf) => 'world')
                    .advanced
                    .sdf(sdf)
                    .doc("Gazebo: the #{sdf.name} model")
            end

            # @api private
            #
            # Normalize a Gazebo name to use in Syskit models
            def normalize_name(name)
                name.gsub(/:+/, '_')
            end

            # @api private
            #
            # Describes recursively all sensors and plugins in the model
            def load_gazebo_robot_submodel(
                sdf_model, root_device,
                name: sdf_model.name, prefix_device_with_name: true
            )

                sdf_model.each_sensor do |sensor|
                    gazebo_define_sensor_device(
                        root_device, sdf_model, sensor,
                        model_name: name,
                        prefix_device_with_name: prefix_device_with_name
                    )
                end

                sdf_model.each_plugin do |plugin|
                    gazebo_define_plugin_device(
                        root_device, sdf_model, plugin,
                        model_name: name,
                        prefix_device_with_name: prefix_device_with_name
                    )
                end

                sdf_model.each_model do |sdf_submodel|
                    load_gazebo_robot_submodel(
                        sdf_submodel, root_device,
                        name: name + "_" + sdf_submodel.name, prefix_device_with_name: prefix_device_with_name
                    )
                end
            end

            # @api private
            #
            # Define devices for all links and sensors in the model
            def load_gazebo_robot_model(
                sdf_model, root_device,
                reuse: nil, name: sdf_model.name, prefix_device_with_name: true
            )
                if (prefix = sdf_model.full_name(root: root_device.sdf))
                    frame_prefix = "#{normalize_name(prefix)}_"
                end

                sdf_model.each_link_with_name do |l, l_name|
                    gazebo_define_link_device(
                        root_device, sdf_model, l, l_name,
                        model_name: name, reuse: reuse, frame_prefix: frame_prefix,
                        prefix_device_with_name: prefix_device_with_name
                    )
                end

                load_gazebo_robot_submodel(
                    sdf_model, root_device,
                    name: sdf_model.name, prefix_device_with_name: prefix_device_with_name
                )
            end

            def gazebo_define_link_device(
                root_device, sdf_model, link, link_name,
                model_name: sdf_model.name, frame_prefix: '',
                prefix_device_with_name: true, reuse: false
            )
                device_name = "#{normalize_name(link_name)}_link"
                if prefix_device_with_name
                    device_name = "#{normalize_name(model_name)}_#{device_name}"
                end
                if reuse && (existing = reuse.find_device(device_name))
                    register_device(device_name, existing)
                    return
                end
                frame_basename = "#{frame_prefix}#{normalize_name(link.name)}"

                link_frame = link_frame_name(link)

                link_driver_m = OroGen.rock_gazebo.ModelTask.specialize
                driver_srv = link_driver_m.require_dynamic_service(
                    'link_export', as: device_name, frame_basename: frame_basename
                )
                link_driver_m =
                    link_driver_m
                    .to_instance_requirements
                    .prefer_deployed_tasks(
                        *root_device.to_instance_requirements.deployment_hints
                    )
                    .with_arguments(model_dev: root_device)
                    .use_frames("#{frame_basename}_source" => link_frame,
                                "#{frame_basename}_target" => 'world')
                    .select_service(driver_srv)
                device(CommonModels::Devices::Gazebo::Link,
                       as: device_name, using: link_driver_m)
                    .doc("Gazebo: state of the #{link.name} link of #{sdf_model.name}")
                    .frame_transform(link_frame => 'world')
                    .advanced
            end

            def gazebo_define_sensor_device(
                root_device, sdf_model, sensor,
                model_name: sdf_model.name, prefix_device_with_name: true
            )
                device_name = "#{normalize_name(sensor.name)}_sensor"
                if prefix_device_with_name
                    device_name = "#{normalize_name(model_name)}_#{device_name}"
                end

                device = sensors_to_device(
                    sensor, device_name, link_frame_name(sensor.parent)
                )
                unless device
                    RockGazebo.warn(
                        "Robot#load_gazebo: don't know how to handle" \
                        "sensor #{sensor.full_name} of type #{sensor.type}"
                    )
                    return
                end

                if (period = sensor.update_period)
                    device.period(period)
                end
                device.doc "Gazebo: #{sensor.name} sensor of #{sdf_model.full_name}"

                deployment_name =
                    root_device.to_instance_requirements.deployment_hints.first
                device
                    .sdf(sensor)
                    .prefer_deployed_tasks(
                        "#{deployment_name}:#{normalize_name(sensor.name)}"
                    )
            end

            def gazebo_define_plugin_device(
                root_device, sdf_model, plugin,
                model_name: sdf_model.name, prefix_device_with_name: true
            )
                device_name = "#{normalize_name(plugin.name)}_plugin"
                if prefix_device_with_name
                    device_name = "#{normalize_name(model_name)}_#{device_name}"
                end

                device = plugins_to_device(
                    plugin, device_name
                )

                unless device
                    RockGazebo.warn(
                        "Robot#load_gazebo: don't know how to handle " \
                        "plugin #{plugin.full_name}"
                    )
                    return
                end

                device.doc "Gazebo: #{plugin.name} plugin of #{sdf_model.full_name}"
                deployment_name =
                    root_device.to_instance_requirements.deployment_hints.first
                device.sdf(plugin)
                      .prefer_deployed_tasks(
                          "#{deployment_name}:#{normalize_name(plugin.name)}"
                      )
            end
        end
        ::Syskit::Robot::RobotDefinition.include RobotDefinitionExtension
    end
end
