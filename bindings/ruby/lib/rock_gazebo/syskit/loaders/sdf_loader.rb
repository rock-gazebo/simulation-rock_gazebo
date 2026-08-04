# frozen_string_literal: true

require_relative "abstract_loader.rb"

module RockGazebo
    module Syskit
        class SDFLoader < AbstractLoader
            # @abstract
            def load(syskit_conf, path, world_name: nil)
                syskit_conf.sdf.load_sdf(path, world_name: world_name)
            end
        end
    end
end
