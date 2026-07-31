# frozen_string_literal: true

module RockGazebo
    module Syskit
        class ERBLoader
            # Represents the configuration for pre-rendering a single ERB template.
            class ModelTemplate
                attr_reader :model, :erb_args, :output_file_name, :output_folder_name

                def initialize(
                    model,
                    erb_args: {},
                    output_file_name: "model.sdf",
                    output_folder_name: nil
                )
                    @model = model
                    @erb_args = erb_args
                    @output_file_name = output_file_name
                    @output_folder_name = output_folder_name
                end

                def self.from_hash(hash)
                    model = hash.fetch(:model) do
                        raise ArgumentError,
                            "The :model key is required when constructing from a Hash"
                    end
                    options = hash.reject { |k| k == :model }
                    new(model, **options)
                end
            end

            # Initializes the loader with one or more templates.
            #
            # Supports:
            # - ERBLoader.new(model: "...", erb_args: ...)
            # - ERBLoader.new({ model: "a" }, { model: "b" })
            # - ERBLoader.new([{ model: "a" }, { model: "b" }])
            # - ERBLoader.new(ModelTemplate.new("a"), ModelTemplate.new("b"))
            def initialize(*templates)
                # Unwrap the outer array if multiple templates were passed inside an explicit Array
                templates = templates.first if templates.size == 1 && templates.first.is_a?(Array)

                @templates = templates.map do |template|
                    case template
                    when ModelTemplate
                        template
                    when Hash
                        ModelTemplate.from_hash(template)
                    when String
                        ModelTemplate.new(template)
                    else
                        raise ArgumentError, "Expected ERBLoader::ModelTemplate, Hash, or String, got #{template.class}"
                    end
                end
            end

            # Triggers pre-rendering of all registered templates.
            # Delegated through Syskit.conf to ensure generated folders are tracked and unlinked on shutdown.
            def load(syskit_conf)
                @templates.each do |t|
                    syskit_conf.pre_render_erb_sdf_model(
                        t.model,
                        erb_args: t.erb_args,
                        output_file_name: t.output_file_name,
                        output_folder_name: t.output_folder_name
                    )
                end
            end
        end
    end
end
