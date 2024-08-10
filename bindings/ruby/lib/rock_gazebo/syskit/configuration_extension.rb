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
                Rock::Gazebo.model_path =
                    Rock::Gazebo.default_model_path(path_resolver: Roby.app)
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
            def use_gazebo_world(
                *path, world_name: nil, localhost: Conf.gazebo.localhost?,
                read_only: false, logger_name: nil, period: 0.1
            )
                world = use_sdf_world(*path, world_name: world_name)
                Rock::Gazebo.process_gazebo_world(world)
                deployment_model =
                    ConfigurationExtension.world_to_orogen(world, period: period)

                unless has_process_server?("gazebo")
                    options =
                        if localhost
                            Hash[host_id: "localhost"]
                        else
                            {}
                        end

                    if ::Syskit.conf.respond_to?(:register_unmanaged_manager)
                        ::Syskit.conf.register_unmanaged_manager(
                            "gazebo", log_dir: app.log_dir, **options
                        )
                    else
                        ::Syskit.conf.register_process_server(
                            "gazebo", ::Syskit::RobyApp::UnmanagedTasksManager.new,
                            app.log_dir, **options
                        )
                    end
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

            def self.world_to_orogen(world, period: 0.1)
                ::Syskit::Deployment.new_submodel(
                    name: "Deployment::Gazebo::#{world.name}"
                ) do
                    RockGazebo.setup_orogen_model_from_sdf_world(
                        self, world, period: period
                    )
                end
            end
        end

        ::Syskit::RobyApp::Configuration.include ConfigurationExtension
    end
end
