# frozen_string_literal: true

require 'rock_gazebo/root_logger'

# simplecov must be loaded FIRST. Only the files required after it gets loaded
# will be profiled !!!
if ENV['TEST_ENABLE_COVERAGE'] == '1'
    begin
        require 'simplecov'
        SimpleCov.start
    rescue LoadError
        require 'rock_gazebo'
        RockGazebo.warn(
            "coverage is disabled because the 'simplecov' gem cannot be loaded"
        )
    rescue Exception => e # rubocop:disable Lint/RescueException
        require 'rock_gazebo'
        RockGazebo.warn "coverage is disabled: #{e.message}"
    end
end

require 'flexmock/minitest'
require 'minitest/spec'

if ENV['TEST_ENABLE_PRY'] != '0'
    begin
        require 'pry'
    rescue Exception # rubocop:disable Lint/RescueException
        RockGazebo.warn "debugging is disabled because the 'pry' gem cannot be loaded"
    end
end

module RockGazebo
    # This module is the common setup for all tests
    #
    # It should be included in the toplevel describe blocks
    #
    # @example
    #   require 'rock_gazebo/test'
    #   describe RockGazebo do
    #     include RockGazebo::SelfTest
    #   end
    #
    module SelfTest
        def setup
            # Setup code for all the tests
        end

        def teardown
            super
            # Teardown code for all the tests
        end
    end
end

Minitest::Test.include RockGazebo::SelfTest
