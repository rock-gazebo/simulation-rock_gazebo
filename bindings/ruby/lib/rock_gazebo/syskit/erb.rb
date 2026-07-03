# frozen_string_literal: true

require 'erb'

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

                require 'fileutils'
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
            def parse_erb_as_str(erb_content, erb_args)
                erb_engine = ::ERB.new(erb_content, trim_mode: '-')

                # Render the ERB template with the passed hash arguments
                erb_engine.result_with_hash(erb_args)
            end
        end
    end
end
