# frozen_string_literal: true

require "minitest/autorun"
require "rock_gazebo/test"

module Rock
    describe "CLI Option Parsing" do
        before do
            # Safely load the bin script without executing its main body
            @bin_path = File.expand_path("../bin/rock-gazebo", __dir__)
            load @bin_path
        end

        it "parses --wait-for-sdf-generation boolean flag correctly" do
            argv = ["--wait-for-sdf-generation", "scene.world"]
            config = parse_options(argv)
            
            assert_equal true, config[:wait_for_sdf_generation]
            assert_equal 15, config[:wait_timeout] # Default timeout
            assert_equal ["scene.world"], config[:args]
        end

        it "parses custom --wait-timeout value correctly" do
            argv = ["--wait-for-sdf-generation", "--wait-timeout=25", "scene.world"]
            config = parse_options(argv)
            
            assert_equal true, config[:wait_for_sdf_generation]
            assert_equal 25, config[:wait_timeout]
            assert_equal ["scene.world"], config[:args]
        end

        it "defaults wait-for-sdf-generation to false" do
            argv = ["scene.world"]
            config = parse_options(argv)
            
            assert_equal false, config[:wait_for_sdf_generation]
            assert_equal 15, config[:wait_timeout]
        end

        it "supports other standard arguments like --start and --verbose" do
            argv = ["--start", "--verbose", "scene.world"]
            config = parse_options(argv)
            
            assert_equal true, config[:start]
            assert_includes config[:gzserver_args], "--verbose"
            assert_equal ["scene.world"], config[:args]
        end
    end
end
