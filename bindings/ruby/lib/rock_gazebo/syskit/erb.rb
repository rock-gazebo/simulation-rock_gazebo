# frozen_string_literal: true

require 'erb'
require 'fileutils'

module RockGazebo
    module Syskit
        module ERB
            module_function

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

                _, resolved_paths = Rock::Gazebo.resolve_worldfiles_and_models_arguments(
                    [path_string]
                )
                full_path = resolved_paths.first

                if File.directory?(full_path)
                    File.join(full_path, "model.sdf.erb")
                elsif File.file?(full_path) && full_path.end_with?(".sdf")
                    "#{full_path}.erb"
                else
                    full_path
                end
            end

            # Pre-renders an ERB template and returns it as a REXML::Document
            #
            # @return [REXML::Document] the rendered sdf model
            def pre_render_erb_sdf_model(*path, erb_args: {})
                full_path = resolve_erb_template_path(*path)
                template_dir = File.dirname(full_path)

                erb_content = read_erb_file(full_path)
                solved_erb_as_sdf_str = parse_erb_as_str(erb_content, **erb_args)

                REXML::Document.new(solved_erb_as_sdf_str)
            end
        end
    end
end
