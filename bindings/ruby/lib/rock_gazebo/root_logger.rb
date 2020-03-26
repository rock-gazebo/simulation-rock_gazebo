# frozen_string_literal: true

require 'logger'
require 'utilrb/logger/root'
require 'utilrb/logger/hierarchy'

module RockGazebo
    extend Logger::Root('RockGazebo', Logger::WARN)
end
