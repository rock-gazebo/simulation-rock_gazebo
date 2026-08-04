# frozen_string_literal: true

require "rock_gazebo/syskit/loaders/erb_loader"
require "rock_gazebo/syskit/test"
require "rexml/document"
require_relative "helpers"

module RockGazebo
    module Syskit
        # include Helpers

        describe ERBLoader do
            describe "normalization and initialization" do
                it "normalizes a plain String model name" do
                    loader = ERBLoader.new("model://simple_model")
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 1, templates.size
                    assert_equal "model://simple_model", templates.first.model
                end

                it "normalizes a single Hash representation passed as keywords directly" do
                    loader = ERBLoader.new(
                        model: "model://direct_hash_model",
                        erb_args: { val: 42 }
                    )
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 1, templates.size
                    assert_equal "model://direct_hash_model", templates.first.model
                    assert_equal({ val: 42 }, templates.first.erb_args)
                end

                it "normalizes multiple hashes passed as separate arguments" do
                    loader = ERBLoader.new(
                        { model: "model://model_a" },
                        { model: "model://model_b", virtual_model_name: "b" }
                    )
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 2, templates.size
                    assert_equal "model://model_a", templates[0].model
                    assert_equal "model://model_b", templates[1].model
                    assert_equal "b", templates[1].virtual_model_name
                end

                it "normalizes a single ModelTemplate object directly" do
                    template_obj = ERBLoader::ModelTemplate.new(
                        "model://template_obj"
                    )
                    loader = ERBLoader.new(template_obj)
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 1, templates.size
                    assert_equal "model://template_obj", templates.first.model
                end

                it "normalizes an array of mixed types" do
                    template_obj = ERBLoader::ModelTemplate.new("model://object_model")
                    loader = ERBLoader.new([
                                               "model://string_model",
                                               { model: "model://hash_model", virtual_model_name: "custom" },
                                               template_obj
                                           ])
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 3, templates.size

                    assert_equal "model://string_model", templates[0].model
                    assert_equal "model://hash_model", templates[1].model
                    assert_equal "custom", templates[1].virtual_model_name
                    assert_equal "model://object_model", templates[2].model
                end

                it "fails fast if a Hash is missing the required :model key" do
                    assert_raises(ArgumentError) do
                        ERBLoader.new(erb_args: { key: "val" })
                    end
                end

                it "fails fast if passed an unsupported parameter type" do
                    assert_raises(ArgumentError) do
                        ERBLoader.new(12_345)
                    end
                end
            end

            describe "#load" do
                before do
                    # Clear the SDF::XML cache to avoid leaking across tests
                    ::SDF::XML.instance_variable_get(:@gazebo_models).clear
                end

                it "resolves target name to File.basename by default, parses ERB, and registers the model in SDF::XML" do
                    loader = ERBLoader.new(
                        model: "model://test_model",
                        erb_args: { noise: 0.1 }
                    )

                    mock_xml = REXML::Document.new("<model><name>test_model</name></model>")

                    flexmock(::RockGazebo::Syskit::ERB)
                        .should_receive(:pre_render_erb_sdf_model)
                        .with("model://test_model", erb_args: { noise: 0.1 })
                        .once
                        .and_return(mock_xml)

                    mock_sdf = flexmock
                    mock_sdf.should_receive(:load_sdf)
                            .with("some_world.world", world_name: nil)
                            .once

                    mock_conf = flexmock(sdf: mock_sdf)

                    loader.load(mock_conf, "some_world.world")

                    assert ::SDF::XML.cached_model?("test_model")

                    cached_xml = ::SDF::XML.instance_variable_get(:@gazebo_models)[nil]["test_model"].xml
                    assert_equal mock_xml.to_s, cached_xml.to_s
                end

                it "resolves target name to virtual_model_name when provided explicitly" do
                    loader = ERBLoader.new(
                        model: "model://test_model",
                        erb_args: { noise: 0.1 },
                        virtual_model_name: "test_model_low_noise"
                    )

                    mock_xml = REXML::Document.new("<model><name>test_model</name></model>")

                    flexmock(::RockGazebo::Syskit::ERB)
                        .should_receive(:pre_render_erb_sdf_model)
                        .with("model://test_model", erb_args: { noise: 0.1 })
                        .once
                        .and_return(mock_xml)

                    mock_sdf = flexmock
                    mock_sdf.should_receive(:load_sdf)
                            .with("some_world.world", world_name: nil)
                            .once

                    mock_conf = flexmock(sdf: mock_sdf)

                    loader.load(mock_conf, "some_world.world")

                    assert ::SDF::XML.cached_model?("test_model_low_noise")
                    refute ::SDF::XML.cached_model?("test_model")
                end

                it "raises an ArgumentError if a model cache collision occurs" do
                    loader = ERBLoader.new(
                        { model: "model://test_model", erb_args: { noise: 0.1 } },
                        { model: "model://test_model", erb_args: { noise: 0.5 } }
                    )

                    mock_xml = REXML::Document.new("<model><name>test_model</name></model>")

                    flexmock(::RockGazebo::Syskit::ERB)
                        .should_receive(:pre_render_erb_sdf_model)
                        .and_return(mock_xml)

                    mock_conf = flexmock(sdf: flexmock)

                    assert_raises(ArgumentError) do
                        loader.load(mock_conf, "some_world.world")
                    end
                end
            end

            describe "#loads real file" do
                before do
                    Conf.sdf = SDF.new
                    @old_model_path = Rock::Gazebo.model_path
                    Rock::Gazebo.model_path = [
                        File.expand_path("../models", __dir__)
                    ]
                end

                after do
                    Rock::Gazebo.model_path = @old_model_path
                end

                it "loads a real .erb file" do
                    erb_args = {
                        links: [
                            {
                                name: "gps",
                                pose: [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
                            },
                            {
                                name: "gps2",
                                pose: [6.0, 7.0, 8.0, 9.0, 0.0, 1.0]
                            }
                        ]
                    }
                    loader = ERBLoader.new(
                        {
                            model: "model://simple_model_erb",
                            erb_args: erb_args
                        }
                    )

                    world_path = File.expand_path("../worlds/simple_model_erb.world", __dir__)
                    loader.load(Conf, world_path)

                    cached_xml = ::SDF::XML.instance_variable_get(:@gazebo_models)[nil]["simple_model_erb"].xml
                    assert_equal "simple_model_erb", REXML::XPath.first(cached_xml, "//model").attributes["name"]
                    poses = REXML::XPath.match(cached_xml, "//pose").map(&:text)
                    assert_includes poses, "0.0 1.0 2.0 3.0 4.0 5.0"
                    assert_includes poses, "6.0 7.0 8.0 9.0 0.0 1.0"
                end
            end
        end
    end
end
