require 'rock/bundles'
rock_bundle = Rock::Bundles.each_bundle.find { |b| b.name == 'common_models' }
if !rock_bundle
    raise LoadError, "cannot find the common_models bundle"
end
$LOAD_PATH.unshift File.dirname(rock_bundle.path)
$LOAD_PATH.unshift rock_bundle.path
Roby.app.search_path << rock_bundle.path

require 'rock_gazebo/root_logger'

# simplecov must be loaded FIRST. Only the files required after it gets loaded
# will be profiled !!!
if ENV['TEST_ENABLE_COVERAGE'] == '1'
    begin
        require 'simplecov'
        SimpleCov.start
    rescue LoadError
        RockGazebo.warn(
            "coverage is disabled because the 'simplecov' gem cannot be loaded"
        )
    rescue Exception => e # rubocop:disable Lint/RescueException
        RockGazebo.warn "coverage is disabled: #{e.message}"
    end
end

if ENV['TEST_ENABLE_PRY'] != '0'
    begin
        require 'pry'
    rescue Exception # rubocop:disable Lint/RescueException
        RockGazebo.warn "debugging is disabled because the 'pry' gem cannot be loaded"
    end
end

require 'flexmock/minitest'
require 'minitest/spec'
require 'minitest/autorun'

require 'syskit'
require 'syskit/roby_app/plugin'
Syskit::RobyApp::Plugin.enable
require 'rock_gazebo/syskit'

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
            super
        end

        def teardown
            super
            # Teardown code for all the tests
            super
        end
    end
end

Minitest::Test.include RockGazebo::SelfTest
