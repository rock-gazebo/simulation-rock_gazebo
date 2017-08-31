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
                    path.first
                else
                    _, resolved_paths = Rock::Gazebo.resolve_worldfiles_and_models_arguments([File.join(*path)])
                    full_path = resolved_paths.first
                    if !File.file?(full_path)
                        if File.file?(model_sdf = File.join(full_path, 'model.sdf'))
                            full_path = model_sdf
                        else
                            raise ArgumentError, "#{full_path} cannot be resolved to a valid SDF file"
                        end
                    end

                    sdf = ::SDF::Root.load(full_path, flatten: false)
                    models = sdf.each_model.to_a
                    if models.size > 1
                        raise ArgumentError, "#{full_path} has more than one top level model, cannot use in use_sdf_model"
                    elsif models.empty?
                        raise ArgumentError, "#{full_path} has no top level model, cannot use in use_sdf_model"
                    end
                    return sdf
                end
            end

            class AlreadyLoaded < ArgumentError; end

            # Setup the transformer based on the given model
            #
            # @return [Model]
            def use_sdf_model(*path, as: nil)
                if @sdf
                    raise AlreadyLoaded, "SDF model already loaded, loading more than one is not possible"
                end

                # This is a guard used by
                # ConfigurationExtension#use_sdf_world to verify that
                # it was was called first. Otherwise, devices for the
                # world models won't be defined and stuff like vizkit3d_world
                # won't be configured properly
                Conf.sdf.has_profile_loaded = true

                @sdf = resolve_sdf_model(*path)
                @sdf_model = @sdf.each_model.first
                if as
                    @sdf_model.name = as
                end
                transformer.parse_sdf_model(@sdf_model)
                @sdf_model
            end

            # Configure the transformer to reflect the SDF environment loaded
            # using Conf.syskit.use_sdf_world
            def use_sdf_world
                world = sdf_world

                if @sdf_model
                    includes = Conf.sdf.find_all_included_models(@sdf.metadata['path'])
                    model_in_world = includes.find { |m| m.name == @sdf_model.name }
                    if !model_in_world
                        if includes.empty?
                            raise ArgumentError, "cannot find model passed to #use_sdf_model in the current world. The expected model is not included anywhere."
                        else
                            raise ArgumentError, "cannot find model passed to #use_sdf_model in the current world. The expected model is included, but no includes have the expected name '#{@sdf_model.name}', found: #{includes.map(&:name).sort.join(", ")}."
                        end
                    end

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
            def use_gazebo_model(*path, name: nil)
                model = use_sdf_model(*path)
                name ||= model.name

                # Load the model in the syskit subsystems
                robot.load_gazebo(model, "gazebo:#{sdf_world.name}", models: Conf.sdf.world.each_model.to_a)

                if !(device = robot.find_device(model.name))
                    raise RuntimeError, "cannot resolve device #{model.name}, it should have been created by RobotDefinitionExtension#load_gazebo, got #{robot.each_master_device.map(&:name).sort.join(", ")}"
                end
                device.frame_transform model.name => 'world'

                # Declare the transformations that can be generated by the
                # ModelTask on the transformer
                model.each_link do |link|
                    link_name = robot.normalize_name(link.name)
                    if dev = robot.find_device("#{link_name}_link")
                        dev.frame_transform link.full_name => 'world'
                    else
                        raise ArgumentError, "expected to have a device called #{link.name}_link providing CommonModels::Devices::Gazebo::Link, but it does not exist. Got #{robot.each_master_device.map(&:name).sort.join(", ")}"
                    end
                end
            end

        end
        ::Syskit::Actions::Profile.include ProfileExtension
    end
end
