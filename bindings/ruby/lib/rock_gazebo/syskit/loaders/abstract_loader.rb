# frozen_string_literal: true

module RockGazebo
    module Syskit
        class AbstractLoader
            # @abstract
            def load(syskit_conf, path, world_name: nil)
                raise NotImplementedError, "#{self.class} must implement #load"
            end
        end
    end
end
