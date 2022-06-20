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
                    raise LoadError,
                          "you need to call #use_sdf_world before require'ing any "\
                          "profile that uses #use_sdf_model"
                end

                if Conf.sdf.world_path
                    override_path = Conf.sdf.world_path
                    Robot.info <<~MSG
                        world_file_path set on Conf.sdf.world_path with value
                        #{override_path}, overriding the parameter #{File.join(*path)}
                        given to #use_sdf_world
                    MSG
                    path = override_path
                end

                setup_gazebo_model_path
                full_path = resolve_world_path(*path)
                Robot.info "loading world from #{full_path}"
                Conf.sdf.load_sdf(full_path, world_name: world_name)
            end

            # Add all models/sdf folders in our dependent bundles
            def setup_gazebo_model_path
                roby_paths = Roby.app.find_dirs(
                    "models", "sdf", all: true, order: :specific_first
                )
                Rock::Gazebo.model_path = (roby_paths + Rock::Gazebo.model_path).uniq
            end

            # Find the path to the world file, using the Roby search path if needed
            def resolve_world_path(*path)
                full_path = File.join(*path)
                return full_path if File.exist?(full_path)

                full_path = File.join(*path, "#{path.last}.world")
                return full_path if File.exist?(full_path)

                if path.size == 1
                    name = path.first
                    full_path = Roby.app.find_file(
                        "scenes", name, "#{name}.world",
                        all: false, order: :specific_first
                    )
                    return full_path if full_path
                end

                raise ArgumentError,
                      "cannot find world #{File.join(*path)}. It is not the path of an "\
                      "existing file, and cannot be found in Roby search path "\
                      "#{Roby.app.search_path.join(' ')}"
            end

            # Sets up Syskit to use gazebo configured to use the given world. One can set
            # some of entities in the world as read only, meaning they won't be configured
            # at startup, instead they will wait for another instance of syskit to do it.
            #
            # @param [Boolean|#===|Array<#===>] read_only expects a string that is a part
            # of the entity's name.
            #
            # @return [Syskit::Deployment] a deployment object that represents
            #   gazebo itself
            def use_gazebo_world(*path,
                                 world_name: nil,
                                 localhost: Conf.gazebo.localhost?,
                                 read_only: false,
                                 logger_name: nil)
                world = use_sdf_world(*path, world_name: world_name)
                deployment_model = ConfigurationExtension.world_to_orogen(world)

                unless has_process_server?("gazebo")
                    options =
                        if localhost
                            Hash[host_id: "localhost"]
                        else
                            {}
                        end

                    ::Syskit.conf.register_process_server(
                        "gazebo", ::Syskit::RobyApp::UnmanagedTasksManager.new,
                        app.log_dir, **options
                    )
                end

                process_server_config =
                    if app.simulation?
                        sim_process_server("gazebo")
                    else
                        process_server_config_for("gazebo")
                    end

                configured_deployment =
                    ::Syskit::Models::ConfiguredDeployment
                    .new(process_server_config.name, deployment_model,
                         {}, "gazebo:#{world.name}", {}, read_only: read_only,
                         logger_name: logger_name)
                register_configured_deployment(configured_deployment)
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

            def self.world_to_orogen(world)
                ::Syskit::Deployment.new_submodel(
                    name: "Deployment::Gazebo::#{world.name}"
                ) do
                    RockGazebo.setup_orogen_model_from_sdf_world(self, world)
                end
            end
        end

        ::Syskit::RobyApp::Configuration.include ConfigurationExtension
    end
end
