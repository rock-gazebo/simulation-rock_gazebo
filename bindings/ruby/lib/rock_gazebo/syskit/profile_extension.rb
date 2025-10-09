module RockGazebo
    module Syskit
        module ProfileExtension
            # The globally-loaded SDF model that describes our world
            def sdf_world
                @sdf_world ||= Conf.sdf.world
            end

            # The model loaded with #use_sdf_model
            attr_accessor :sdf_model

            # @api private
            #
            # Resolve the 'path' argument of use_sdf_model into a model object
            #
            # @return [SDF::Root] the root. The method already validates that
            #   this root has exactly one Model child
            def resolve_sdf_model(*path)
                if path.size == 1 && !path.first.respond_to?(:to_str)
                    # Assume this is a SDF::Model object
                    return path.first
                end

                _, resolved_paths = Rock::Gazebo.resolve_worldfiles_and_models_arguments(
                    [File.join(*path)], path_resolver: Roby.app
                )
                full_path = resolved_paths.first
                unless File.file?(full_path)
                    if File.file?(File.join(full_path, "model.config"))
                        full_path = ::SDF::XML.model_path_of(full_path)
                    else
                        raise ArgumentError,
                              "#{full_path} cannot be resolved to a valid SDF file"
                    end
                end

                sdf = ::SDF::Root.load(full_path, flatten: false)
                models = sdf.each_model.to_a
                if models.size > 1
                    raise ArgumentError,
                          "#{full_path} has more than one top level model, " \
                          "cannot use in use_sdf_model"
                elsif models.empty?
                    raise ArgumentError,
                          "#{full_path} has no top level model, " \
                          "cannot use in use_sdf_model"
                end
                sdf
            end

            class AlreadyLoaded < ArgumentError; end

            # Setup the transformer based on the given model
            #
            # @return [Model]
            def use_sdf_model(*path, filter: nil, as: nil)
                if @sdf
                    raise AlreadyLoaded,
                          "SDF model already loaded, " \
                          "loading more than one is not possible"
                end

                # This is a guard used by
                # ConfigurationExtension#use_sdf_world to verify that
                # it was was called first. Otherwise, devices for the
                # world models won't be defined and stuff like vizkit3d_world
                # won't be configured properly
                Conf.sdf.has_profile_loaded = true

                @sdf = resolve_sdf_model(*path)
                @sdf_model = @sdf.each_model.first
                @sdf_model.name = as if as
                transformer.parse_sdf_model(@sdf_model, filter: filter)

                @sdf_model
            end

            # @api private
            #
            # Find the model given to {#use_sdf_model} into the loaded SDF world
            def resolve_model_in_world
                model_path = @sdf.metadata["path"]
                includes = sdf_world.root.find_all_included_models(model_path)
                model_in_world = includes.find { |m| m.name == @sdf_model.name }
                return model_in_world if model_in_world

                if includes.empty?
                    raise ArgumentError,
                          "cannot find model #{model_path}, passed to " \
                          "#use_sdf_model, in the current world #{sdf_world.name}. " \
                          "The expected model does not seem to be included anywhere."
                end

                found = includes.map(&:name).sort.join(", ")
                raise ArgumentError,
                      "cannot find model #{model_path}, passed to " \
                      "#use_sdf_model, in the current world #{sdf_world.name}. " \
                      "The expected model is included, but no includes have " \
                      "the expected name '#{@sdf_model.name}', found: #{found}"
            end

            # Configure the transformer to reflect the SDF environment loaded
            # using Conf.syskit.use_sdf_world
            def use_sdf_world
                world = sdf_world

                if @sdf_model
                    model_in_world = resolve_model_in_world

                    # Parse the whole world and massage it before we merge it in the
                    # profile's transformer
                    world_tr = ::Transformer::Configuration.new
                    world_tr.parse_sdf_world(world)

                    model_in_world_full_name = model_in_world.full_name(root: world)
                    model_in_world_frame_prefix = "#{model_in_world_full_name}::"

                    frame_mapping = Hash[model_in_world_full_name => @sdf_model.name]
                    world_tr.frames.each do |frame_name|
                        if frame_name.start_with?(model_in_world_frame_prefix)
                            frame_mapping[frame_name] = "#{@sdf_model.name}::#{frame_name[model_in_world_frame_prefix.size..-1]}"
                        end
                    end
                    world_tr.rename_frames(frame_mapping)
                    transformer.merge(world_tr)
                else
                    transformer.parse_sdf_world(world)
                end

                # There can be only one world ... name it 'world'
                if !transformer.has_frame?('world')
                    transformer.static_transform Eigen::Vector3.Zero, world.full_name => 'world'
                end
            end

            # Sets up this profile, robot and transformer according to the
            # information in the model at "path"
            #
            # In addition to {#use_sdf_model}, it adds the Gazebo devices that
            # expose a simulation
            #
            # @param [Boolean] use_world whether {#use_gazebo_world} should be
            #   called at the end. You usually want this
            def use_gazebo_model(*path, filter: nil, as: nil, use_world: true, reuse: nil, prefix_device_with_name: nil)
                use_sdf_model(*path, as: as, filter: filter)
                model_in_world = resolve_model_in_world

                # Load the model in the syskit subsystems
                robot.load_gazebo(model_in_world, "gazebo::#{sdf_world.name}",
                                  reuse: reuse,
                                  prefix_device_with_name: prefix_device_with_name)

                use_gazebo_world(reuse: reuse) if use_world
            end

            # Exports the world into the transformer and expose the models
            # within the gazebo world as devices on the robot interface
            def use_gazebo_world(reuse: nil)
                use_sdf_world
                if @sdf
                    model_in_world = resolve_model_in_world
                    enclosing_model = robot.resolve_enclosing_model(model_in_world)
                end

                reuse = reuse.robot if reuse.respond_to?(:robot)
                sdf_world.each_model do |model|
                    if model != enclosing_model
                        robot.expose_gazebo_model(model, "gazebo::#{sdf_world.name}::",
                            reuse: reuse)
                    end
                end
            end
        end
        ::Syskit::Actions::Profile.include ProfileExtension
    end
end
