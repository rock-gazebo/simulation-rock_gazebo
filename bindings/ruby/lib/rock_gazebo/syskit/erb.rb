# frozen_string_literal: true

require 'erb'
require 'fileutils'
require 'tmpdir'

module RockGazebo
    module Syskit
        module ERB
            module_function

            STABLE_TMP_DIR = File.join(Dir.tmpdir, "gazebo_erb_models").freeze

            # Open an .erb file and returns its content as a string
            #
            # @param [String] file_path path to the .erb template file
            # @return [String] erb file content as a string
            #
            # @raise [ArgumentError] if the file path is not a .erb file, is invalid, or is unreadable
            def read_erb_file(file_path)
                unless file_path.end_with?('.erb')
                    raise ArgumentError, "Provided file path must have a '.erb' extension: #{file_path}"
                end

                begin
                    erb_content = File.read(file_path)
                rescue Errno::ENOENT
                    raise ArgumentError, "ERB template file not found at: #{file_path}"
                rescue Errno::EACCES
                    raise ArgumentError, "Permission denied reading ERB template at: #{file_path}"
                end
                erb_content
            end

            # Saves a rendered string directly to a physical file
            #
            # @param [String] sdf_string the rendered XML string to write
            # @param [String] sdf_folder_destination directory path where the file should be saved
            # @param [String] file_name destination filename (defaults to "model.sdf")
            # @return [Integer] the number of bytes written
            #
            # @raise [ArgumentError] if the file_name is not a .sdf file, if directories cannot be created, or file write fails
            def save_as_sdf_model(sdf_string, sdf_folder_destination, file_name = "model.sdf")
                unless file_name.end_with?('.sdf')
                    raise ArgumentError, "Output file must have a '.sdf' extension: #{file_name}"
                end

                begin
                    ::FileUtils.mkdir_p(sdf_folder_destination)
                rescue Errno::EACCES
                    raise ArgumentError, "Permission denied creating directory: #{sdf_folder_destination}"
                end

                full_path = File.join(sdf_folder_destination, file_name)
                begin
                    File.write(full_path, sdf_string)
                rescue Errno::EACCES
                    raise ArgumentError, "Permission denied writing to: #{full_path}"
                rescue Errno::ENOSPC
                    raise ArgumentError, "No disk space left on device to write: #{full_path}"
                rescue Errno::EISDIR
                    raise ArgumentError, "Cannot write file; a directory exists at: #{full_path}"
                end
            end

            # Parses an .erb string, evaluating the erb_args, and returns the raw rendered string
            #
            # @param [String] erb_content ERB template file content as string
            # @param [Hash] erb_args the configuration arguments to evaluate
            # @return [String] the raw rendered XML string representing the model
            def parse_erb_as_str(erb_content, **erb_args)
                erb_engine = ::ERB.new(erb_content, trim_mode: '-')

                # Render the ERB template with the passed hash arguments
                erb_engine.result_with_hash(erb_args)
            end

            # Resolves the path arguments of an ERB model into a full file path to the template.
            #
            # @return [String] the resolved full path to the .sdf.erb file
            def resolve_erb_template_path(*path)
                path_string = File.join(*path)
                if path_string.start_with?("model://") && !path_string.end_with?(".erb")
                    path_string = File.join(path_string, "model.sdf.erb")
                end

                # Filter out output/temp directories from model_path to resolve the template source
                original_paths = Rock::Gazebo.model_path
                begin
                    # 1. Reject standard stable tmp dir
                    filtered_paths = original_paths.reject do |p|
                        p == STABLE_TMP_DIR || p.start_with?(STABLE_TMP_DIR + File::SEPARATOR)
                    end

                    # 2. Reject dynamic log-based sdf output directory if active
                    if defined?(Roby) && Roby.app.respond_to?(:created_log_dir?) && Roby.app.created_log_dir?
                        log_sdf = File.join(Roby.app.log_dir, "sdf")
                        filtered_paths = filtered_paths.reject do |p|
                            p == log_sdf || p.start_with?(log_sdf + File::SEPARATOR)
                        end
                    end

                    Rock::Gazebo.model_path = filtered_paths

                    _, resolved_paths = Rock::Gazebo.resolve_worldfiles_and_models_arguments(
                        [path_string]
                    )
                    full_path = resolved_paths.first
                ensure
                    # Restore the original search path so Gazebo can still find rendered models
                    Rock::Gazebo.model_path = original_paths
                end

                if File.directory?(full_path)
                    File.join(full_path, "model.sdf.erb")
                elsif File.file?(full_path) && full_path.end_with?(".sdf")
                    "#{full_path}.erb"
                else
                    full_path
                end
            end

            # Returns a unique, non-existent folder path under the temporary SDF folder
            # to prevent model collision.
            #
            # @param [String] base_model_name the desired or default base name
            # @param [Bool] override whether to override an existing model in case
            # there is already a model folder with the same name
            # @return [String] the unique destination folder path
            def erb_unique_destination_dir(base_model_name, override = true)
                base_destination =
                    if defined?(Roby) &&
                        Roby.app.respond_to?(:created_log_dir?) &&
                        Roby.app.created_log_dir?
                        File.join(Roby.app.log_dir, "sdf")
                    else
                        STABLE_TMP_DIR
                    end

                unique_name = base_model_name
                candidate_dir = File.join(base_destination, unique_name)

                unless override
                    counter = 1
                    while File.exist?(candidate_dir)
                        unique_name = "#{base_model_name}#{counter}"
                        candidate_dir = File.join(base_destination, unique_name)
                        counter += 1
                    end
                end

                candidate_dir
            end

            # Copies meshes, subdirectories, configurations, and other assets from the template directory
            # to the target destination directory, skipping .erb files.
            def erb_copy_companion_assets(template_dir, destination_dir)
                ::FileUtils.mkdir_p(destination_dir)
                Dir.glob(File.join(template_dir, '*')).each do |item|
                    next if item.end_with?('.erb')
                    ::FileUtils.cp_r(item, destination_dir)
                end
            end

            # Checks if model.config is present in the destination, writing a fallback if missing.
            def erb_ensure_model_config(destination_dir, model_name, sdf_file_name)
                unless File.file?(File.join(destination_dir, "model.config"))
                    fallback_config = <<~XML
                        <?xml version="1.0"?>
                        <model>
                          <name>#{model_name}</name>
                          <version>1.0</version>
                          <sdf version="1.6">#{sdf_file_name}</sdf>
                        </model>
                    XML
                    File.write(File.join(destination_dir, "model.config"), fallback_config)
                end
            end

            # Pre-renders an ERB template into a physical model directory and registers its search path
            #
            # @return [String] the folder path of the generated model
            def pre_render_erb_sdf_model(
                *path,
                erb_args: {},
                output_file_name: "model.sdf",
                output_folder_name: nil
            )
                # 1. Resolve full path to template
                full_path = resolve_erb_template_path(*path)
                template_dir = File.dirname(full_path)

                # 2. Render template string
                erb_content = read_erb_file(full_path)
                solved_erb_as_sdf_str = parse_erb_as_str(erb_content, **erb_args)

                # 3. Determine unique output path & names
                base_name = output_folder_name || File.basename(template_dir)
                sdf_file_destination = erb_unique_destination_dir(base_name)
                unique_model_name = File.basename(sdf_file_destination)

                # 4. Copy meshes, subdirectories, configurations, and other assets
                erb_copy_companion_assets(template_dir, sdf_file_destination)

                # 5. Write rendered SDF
                save_as_sdf_model(
                    solved_erb_as_sdf_str, sdf_file_destination, output_file_name
                )

                # 6. Ensure model.config exists
                erb_ensure_model_config(sdf_file_destination, unique_model_name, output_file_name)

                # 7. Prepend base directory to model search paths so both Syskit & Gazebo find it
                base_destination = File.dirname(sdf_file_destination)
                current_paths = Rock::Gazebo.model_path
                unless current_paths.include?(base_destination)
                    Rock::Gazebo.model_path = [base_destination] + current_paths
                    ::SDF::XML.clear_cache
                end

                # 8. Create stable temp folder symlink pointing to active log dir
                if defined?(Roby) && Roby.app.respond_to?(:created_log_dir?) && Roby.app.created_log_dir?
                    stable_temp_dir = STABLE_TMP_DIR
                    ::FileUtils.mkdir_p(stable_temp_dir)

                    target_link = File.join(stable_temp_dir, File.basename(sdf_file_destination))
                    ::FileUtils.rm_rf(target_link)
                    ::FileUtils.ln_s(sdf_file_destination, target_link)
                end

                sdf_file_destination
            end

            def unlink_gazebo_model(sdf_file_destination)
                stable_temp_dir = STABLE_TMP_DIR
                link_name = File.basename(sdf_file_destination)
                full_link_path = File.join(stable_temp_dir, link_name)
                ::FileUtils.rm_f(full_link_path) if File.symlink?(full_link_path)
            end
        end
    end
end
