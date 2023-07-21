require 'rock/bundles' unless ENV["SYSKIT_USE_ROCK_BUNDLES"] == "0"
require 'rock_gazebo/path_to_plugin'
require 'sdf'
require 'minitar'

require 'utilrb/logger'

module Rock
    module Gazebo
        extend Logger::Root('rock-gazebo', Logger::WARN)
        extend Logger::Forward

        def self.default_model_path(path_resolver: default_path_resolver)
            path_resolver.find_dirs('models', 'sdf', all: true, order: :specific_first) +
                (ENV['GAZEBO_MODEL_PATH']||"").split(':') +
                [File.join(Dir.home, '.gazebo', 'models')]
        end

        def self.model_path(path_resolver: default_path_resolver)
            @model_path || default_model_path(path_resolver: path_resolver)
        end

        def self.default_path_resolver
            Bundles if defined? Bundles
        end

        def self.model_path=(model_path)
            @model_path = model_path
            SDF::XML.model_path = model_path
            ENV['GAZEBO_MODEL_PATH'] = model_path.join(":")
        end

        def self.resolve_worldfiles_and_models_arguments(argv)
            model_path = self.model_path
            filtered_argv = Array.new
            argv = argv.dup
            while !argv.empty?
                arg = argv.shift

                if arg == '--model-dir'
                    dir = argv.shift
                    if !dir
                        STDERR.puts "no argument given to --model-dir"
                        exit 1
                    elsif !File.dirname(dir)
                        STDERR.puts "#{dir} is not a directory"
                        exit 1
                    end

                    model_path.unshift dir
                elsif File.file?(arg)
                    filtered_argv << arg
                elsif File.directory?(arg)
                    candidates = Dir.glob(File.join(arg, '*.world')) +
                        Dir.glob(File.join(arg, '*.sdf'))
                    if candidates.size == 1
                        filtered_argv << candidates.first
                    else
                        filtered_argv << arg
                    end
                elsif arg =~ /^model:\/\/(\w+)(.*)/
                    model_name, filename = $1, $2
                    require 'sdf'
                    model_dir = File.dirname(SDF::XML.model_path_from_name(model_name, model_path: model_path))
                    filtered_argv << File.join(model_dir, filename)
                elsif File.extname(arg) == '.world'
                    worldname = File.basename(arg, '.world')
                    if resolved_path = Bundles.find_file('scenes', worldname, arg, order: :specific_first)
                        filtered_argv << resolved_path
                    else
                        filtered_argv << arg
                    end
                elsif resolved_path = Bundles.find_file('scenes', arg, "#{arg}.world", order: :specific_first)
                    filtered_argv << resolved_path
                elsif resolved_path = Bundles.find_file(arg, "#{arg}.world", order: :specific_first)
                    filtered_argv << resolved_path
                else
                    filtered_argv << arg
                end
            end
            return model_path, filtered_argv
        end

        def self.initialize
            Bundles.setup_search_paths
            self.model_path = self.default_model_path
        end

        def self.prepare_spawn(cmd, *cmdline)
            env = Hash.new
            if cmdline.first.kind_of?(Hash)
                env = cmdline.shift
            end

            model_path, args = resolve_worldfiles_and_models_arguments(cmdline)
            SDF::XML.model_path.concat(model_path)
            @tempfiles ||= Array.new
            args = args.map do |arg|
                if arg =~ /\.sdf$|\.world$/
                    world = process_gazebo_file(arg)
                    processed_world = Tempfile.new
                    processed_world.write(world.xml.to_s)
                    processed_world.flush
                    @tempfiles << processed_world
                    processed_world.path
                else arg
                end
            end
            yield(Array[env, cmd, '-s', RockGazebo::PATH_TO_PLUGIN, *args])
        end

        def self.compute_spawn_arguments(cmd, *cmdline)
            env = Hash.new
            if cmdline.first.kind_of?(Hash)
                env = cmdline.shift
            end

            model_path, args = resolve_worldfiles_and_models_arguments(cmdline)
            env['GAZEBO_MODEL_PATH'] ||= model_path.join(":")
            Array[env, cmd, '-s', RockGazebo::PATH_TO_PLUGIN, *args]
        end

        # Perform Rock-specific post-processing of the SDF world
        #
        # This method handles use-cases that do not involve gazebo. Use
        # {.process_gazebo_world} when using with gazebo
        #
        # @param [SDF::Root,String] sdf the preloaded SDF world, or a string that
        #   describes a SDF file (either a path, or a model:// URI)
        # @return [SDF::Root]
        def self.process_sdf_file(sdf)
            sdf = SDF::Root.load(sdf) if sdf.respond_to?(:to_str)
            sdf
        end

        # @api private
        #
        # Create a fresh oroGen RTT loader
        #
        # @return [OroGen::Loaders::RTT]
        def self.create_rtt_loader
            OroGen::Loaders::RTT.new(ENV["OROCOS_TARGET"] || "gnulinux")
        end

        # Load a SDF world file for use in a Rock and Gazebo context
        #
        # This method handles use-cases that involves gazebo. It mainly normalizes
        # component definitions inside the SDF
        #
        # @param [SDF::Root,String] sdf the preloaded SDF world, or a string that
        #   describes a SDF file (either a path, or a model:// URI)
        # @return [SDF::Root]
        # @see process_sdf_file
        def self.process_gazebo_file(sdf, loader: create_rtt_loader)
            sdf = process_sdf_file(sdf)

            # post-process the rock_components info
            sdf.each_world { |w| process_gazebo_world(w, loader: loader) }
            sdf
        end

        # Perform Rock-specific post-processing of a single SDF world
        #
        # @param [SDF::World] sdf
        # @param [OroGen::Loaders::Base] loader
        # @return [void]
        def self.process_gazebo_world(world, loader: create_rtt_loader)
            needed_typekits = Set.new
            REXML::XPath.each(world.xml, "//plugin") do |plugin_xml|
                needed_typekits.merge(
                    process_gazebo_plugin(world, plugin_xml, loader: loader)
                )
            end
            ensure_typekits_loaded(world.xml, needed_typekits, loader)
        end

        # @api private
        #
        # Perform Rock-specific post-processing of a plugin tag within a world
        #
        # @param [SDF::World] world
        # @param [REXML::Element] plugin_xml
        # @param [OroGen::Loaders::Base] loader
        # @return [Set<String>] the set of typekits that are needed by the tasks
        #   within this plugin need
        def self.process_gazebo_plugin(world, plugin_xml, loader:)
            scope = resolve_plugin_full_name(world.xml, plugin_xml)
            typekits = normalize_rock_components(scope, plugin_xml, loader)
            plugin_xml.attributes["name"] = scope.gsub(/[^\w]/, "_")
            typekits
        rescue OroGen::NotFound => e
            plugin_name = plugin_xml.attributes["name"]
            raise e, "while processing the <task ..> elements of the #{plugin_name} "\
                     "Rock plugin: #{e.message}", e.backtrace
        end

        # @deprecated use either {#process_sdf_file} or {#process_gazebo_file}
        def self.process_sdf_world(sdf, loader: create_rtt_loader)
            process_gazebo_file(sdf, loader: loader).xml
        end

        def self.resolve_plugin_full_name(world_xml, plugin_xml)
            parts = []
            element = plugin_xml
            while world_xml != element
                parts << element.attributes["name"]
                element = element.parent
            end
            (["gazebo", world_xml.attributes["name"]] + parts.reverse).join("::")
        end

        def self.normalize_rock_components(scope, plugin_xml, loader)
            needed_typekits = Set.new
            plugin_xml.elements.each("task") do |task_xml|
                full_name = task_xml.attributes["name"] || scope

                task_xml.attributes["name"] = full_name
                model_name = task_xml.attributes["model"]
                task_model = loader.task_model_from_name(model_name)
                task_xml.attributes["filename"] =
                    loader.task_library_path_from_name(task_model.project.name)
                task_model.each_interface_type do |type|
                    needed_typekits << loader.typekit_for(type)
                end
            end
            needed_typekits
        end

        def self.create_or_update_rock_components(world_xml)
            existing = REXML::XPath.first(world_xml, "plugin[@name='rock_components']")
            return existing if existing

            plugin = REXML::Element.new("plugin")
            plugin.attributes["name"] = "rock_components"
            plugin.attributes["filename"] = "__default__"
            world_xml.elements << plugin
            plugin
        end

        def self.libraries_needed_by_typekit(typekit, loader)
            result = Set.new
            result << loader.typekit_library_path_from_name(typekit.name)
            %w[typelib mqueue corba].each do |transport|
                result << loader.transport_library_path_from_name(
                    typekit.name, transport
                )
            end
            result
        end

        def self.ensure_typekits_loaded(world_xml, needed_typekits, loader)
            needed_libraries = Set.new
            needed_typekits.each do |tk|
                next if tk.virtual?

                needed_libraries.merge(libraries_needed_by_typekit(tk, loader))
            end

            load_libraries(world_xml, needed_libraries)
        end

        def self.load_libraries(world_xml, libraries)
            rock_components = create_or_update_rock_components(world_xml)

            libraries.each do |path|
                el = REXML::Element.new("load")
                el.attributes["path"] = path
                rock_components.elements << el
            end
        end

        def self.download_path
            @download_path ||= File.join(Dir.home, ".gazebo", "models")
        end

        def self.download_path=(path)
            @download_path = File.expand_path(path)
        end

        def self.download_missing_models(world_path)
            SDF::Root.load(world_path)
        rescue SDF::XML::NoSuchModel => missing_model
            model_name = missing_model.model_name
            if !SDF::XML.model_path.include?(download_path)
                raise NoSuchModel, "#{world_path} refers to model #{model_name} which was not available in #{model_path.join(":")}. I can't download it as the configured download path #{download_path} is not part of SDF::XML.model_path."
            end

            info "#{world_path} refers to model #{model_name} that is not available, trying to download from models.gazebosim.org"
            begin
                download_model(model_name)
            rescue Exception => e
                raise SDF::XML::NoSuchModel, "#{world_path} refers to model #{model_name} which was not available in #{model_path.join(":")}, #{e.message}. You may want to update the GAZEBO_MODEL_PATH environment variable, or set SDF::XML.model_path explicitely."
            end
            retry
        end

        def self.download_model_uri(name)
            "http://models.gazebosim.org/#{name}/model.tar.gz"
        end

        def self.download_model(name)
            require 'open-uri'
            uri = download_model_uri(name)
            targz = URI(uri).read
            unzipped = Zlib::GzipReader.new(StringIO.new(targz))
            Minitar.unpack(unzipped, download_path)
        rescue Exception => e
            raise e, "attempted to download model #{name} from #{uri}, but this failed with #{e.message}"
        end

        def self.update_existing_models
            Dir.glob(File.join(download_path, '*')) do |path|
                name = File.basename(path)
                if name != '.' && name != '..' && File.directory?(path)
                    begin
                        download_model(name)
                        info "downloaded #{name}"
                    rescue Exception => e
                        warn "could not update #{name}: #{e.message}"
                    end
                end
            end
        end

        def self.spawn(cmd, *cmdline, **options)
            prepare_spawn(cmd, *cmdline) do |args|
                Process.spawn(*args, **options)
            end
        end

        def self.exec(cmd, *cmdline, download_missing_models: true, **options)
            prepare_spawn(cmd, *cmdline, download_missing_models: download_missing_models) do |args|
                Process.exec(*args, **options)
            end
        end
    end
end
