module RockGazebo
    module Syskit
        module ConfigurationExtension
            Roby.app.on_setup do
                if Conf.sdf.world_path?
                    world_path = Conf.sdf.world_path
                end
                Conf.sdf = SDF.new
                if world_path
                    Conf.sdf.world_path = world_path
                end
                Conf.gazebo.use_sim_time = false
                Conf.gazebo.localhost = true
            end

            Roby.app.on_clear_models do
                Conf.sdf.world_file_path = nil
                Conf.sdf.has_profile_loaded = false
            end

            # Load a SDF world into the Syskit instance
            def use_sdf_world(*path, world_name: nil)
                if Conf.sdf.world_file_path
                    raise LoadError, "use_sdf_world already called"
                elsif Conf.sdf.has_profile_loaded?
                    raise LoadError, "you need to call #use_sdf_world before require'ing any profile that uses #use_sdf_model"
                end

                if Conf.sdf.world_path
                    override_path = Conf.sdf.world_path
                    Robot.info "world_file_path set on Conf.sdf.world_path with value #{override_path}, overriding the parameter #{File.join(*path)} given to #use_sdf_world"
                    path = override_path
                end

                Conf.sdf.load_sdf(*path, world_name: world_name)
            end

            # Sets up Syskit to use gazebo configured to use the given world
            #
            # @return [Syskit::Models::ConfiguredDeployment]
            def use_gazebo_world(*path, world_name: nil, localhost: Conf.gazebo.localhost?)
                world = use_sdf_world(*path, world_name: world_name)
                use_deployments_from_gazebo(world, localhost: localhost)
            end

            # Register the deployments from gazebo to be used in profiles
            #
            # The deployments are registerd in Conf.sdf.deployment_groups,
            # to be picked up by the use_gazebo_model stanza in the profiles
            #
            # @return [Syskit::Models::ConfiguredDeployment]
            def use_deployments_from_gazebo(world,
                    prefix: 'gazebo', localhost: Conf.gazebo.localhost?)
                deployment_model = ConfigurationExtension.world_to_orogen(world,
                    prefix: prefix)

                if !has_process_server?('gazebo')
                    if localhost
                        options = Hash[host_id: 'localhost']
                    else
                        options = Hash.new
                    end
                    ::Syskit.conf.register_process_server(
                        'gazebo', ::Syskit::RobyApp::UnmanagedTasksManager.new, app.log_dir, **options)
                end

                process_server_config =
                    if app.simulation?
                        sim_process_server('gazebo')
                    else
                        process_server_config_for('gazebo')
                    end

                configured_deployment = ::Syskit::Models::ConfiguredDeployment.
                    new(process_server_config.name, deployment_model, Hash[],
                        "#{prefix}:#{world.name}", Hash.new)
                group = ::Syskit::Models::DeploymentGroup.new
                group.register_configured_deployment(configured_deployment)
                Conf.sdf.deployment_group = group
                configured_deployment
            end

            # @api private
            #
            # Resolves a world from a SDF file, which may have more than one
            #
            # @param [String] path path to the world file
            # @param [String] world_name if not nil, the expected world name
            # @return [World]
            # @raise ArgumentError if the SDF file does not define a world
            # @raise ArgumentError if the SDF file has more than one world and
            #   world_name is not set
            # @raise ArgumentError if the SDF file has more than one world and
            #   none match the name provided as world_name
            def self.world_from_path(path, world_name: nil)
                worlds = ::SDF::Root.load(path, flatten: false).each_world.to_a
                if world_name
                    world = worlds.find { |w| w.name == world_name }
                    if !world
                        raise ArgumentError, "cannot find a world named #{world_name} in #{path}"
                    end
                    return world
                elsif worlds.size == 1
                    return worlds.first
                elsif worlds.empty?
                    raise ArgumentError, "no worlds declared in #{path}"
                else
                    raise ArgumentError, "more than one world declared in #{path}, select one explicitely by providing the world_name argument"
                end
            end

            def self.world_to_orogen(world, prefix: prefix)
                ::Syskit::Deployment.new_submodel(name: "Deployment::Gazebo::#{world.name}") do
                    RockGazebo.setup_orogen_model_from_sdf_world(self, world, prefix: prefix)
                end
            end
        end

        ::Syskit::RobyApp::Configuration.include ConfigurationExtension
    end
end
