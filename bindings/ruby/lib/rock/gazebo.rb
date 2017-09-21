require 'rock/bundles'
require 'rock_gazebo/path_to_plugin'
require 'sdf'

module Rock
    module Gazebo
        def self.default_model_path
            Bundles.find_dirs('models', 'sdf', all: true, order: :specific_first) +
                (ENV['GAZEBO_MODEL_PATH']||"").split(':') +
                [File.join(Dir.home, '.gazebo', 'models')]
        end

        def self.model_path
            @model_path || default_model_path
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
            Bundles.load
            self.model_path = self.default_model_path
        end

        def self.prepare_spawn(cmd, *cmdline, download_missing_models: true)
            env = Hash.new
            if cmdline.first.kind_of?(Hash)
                env = cmdline.shift
            end

            model_path, args = resolve_worldfiles_and_models_arguments(cmdline)
            SDF::XML.model_path.concat(model_path)
            tempfiles = Array.new
            args = args.map do |arg|
                if arg =~ /\.sdf$|\.world$/
                    world_xml = process_sdf_world(arg)
                    processed_world = Tempfile.new
                    processed_world.write(world_xml.to_s)
                    processed_world.flush
                    tempfiles << processed_world
                    processed_world.path
                else arg
                end
            end
            yield(Array[env, cmd, '-s', RockGazebo::PATH_TO_PLUGIN, *args])
        ensure
            tempfiles.each(&:close)
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

        def self.process_sdf_world(world_path)
            sdf = SDF::Root.load(world_path)

            # post-process the rock_components info
            sdf.each_world do |w|
                w.each_plugin do |p|
                    if p.name == 'rock_components'
                        normalize_rock_components(p)
                    end
                end
            end

            sdf.xml
        end

        def self.normalize_rock_components(plugin)
            loader = OroGen::Loaders::RTT.new(ENV['OROCOS_TARGET'] || 'gnulinux')

            needed_typekits = Set.new
            needed_libraries = Set.new
            plugin.xml.elements.each('task') do |task_xml|
                model_name = task_xml.attributes['model']
                task_model = loader.task_model_from_name(model_name)
                task_xml.attributes['filename'] = loader.task_library_path_from_model_name(model_name)
                task_model.each_interface_type do |type|
                    needed_typekits << loader.typekit_for(type)
                end
            end

            needed_typekits.each do |tk|
                next if tk.virtual?
                plugins = Orocos.find_typekit_plugin_paths(tk.name)
                plugins.each do |path, required|
                    needed_libraries << [path, required]
                end
            end

            needed_libraries.each do |path, required|
                library_element = REXML::Element.new('load')
                library_element.attributes['path'] = path
                library_element.attributes['required'] = required ? '1' : '0'
                plugin.xml.elements << library_element
            end

        rescue OroGen::NotFound => e
            raise e, "while processing the rock_components plugin, #{e.message}", e.backtrace
        end

        def self.spawn(cmd, *cmdline, download_missing_models: true, **options)
            prepare_spawn(cmd, *cmdline, download_missing_models: download_missing_models) do |args|
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

