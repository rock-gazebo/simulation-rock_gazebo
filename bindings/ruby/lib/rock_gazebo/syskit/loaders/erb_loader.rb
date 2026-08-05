# frozen_string_literal: true

require_relative "abstract_loader.rb"
require_relative "../erb"

module RockGazebo
    module Syskit
        class ERBLoader < AbstractLoader
            # Represents the configuration for pre-rendering a single ERB template.
            class ModelTemplate
                attr_reader :model, :erb_args, :virtual_model_name

                def initialize(
                    model,
                    erb_args: {},
                    virtual_model_name: nil
                )
                    @model = model
                    @erb_args = erb_args
                    @virtual_model_name = virtual_model_name
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

            # Renders all registered templates into in-memory REXML Documents, registers them
            # in the model cache, and completes the loading procedure.
            def load(syskit_conf, path, world_name: nil)
                @templates.each do |t|
                    target_name = t.virtual_model_name || File.basename(t.model)

                    if ::SDF::XML.cached_model(target_name)
                        raise ArgumentError,
                              "Model cache collision: a model named '#{target_name}' is already registered. " \
                              "To load the same template multiple times with different arguments, " \
                              "you must provide an explicit 'virtual_model_name' for each."
                    end

                    # Pre-render directly to a REXML::Document
                    parsed_xml = ::RockGazebo::Syskit::ERB.pre_render_erb_sdf_model(
                        t.model,
                        erb_args: t.erb_args
                    )

                    # Resolve inclusions and relative URIs
                    virtual_path = "virtual://#{target_name}"
                    resolved_xml, metadata = ::SDF::XML.resolve_sdf_xml(
                        parsed_xml,
                        flatten: false,
                        metadata: true,
                        path: virtual_path
                    )

                    ::SDF::XML.register_in_memory_model(
                        target_name,
                        resolved_xml,
                        metadata: metadata
                    )
                end

                syskit_conf.sdf.load_sdf(path, world_name: world_name)
            end
        end
    end
end
