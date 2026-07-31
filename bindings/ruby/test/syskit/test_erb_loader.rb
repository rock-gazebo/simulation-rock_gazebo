# frozen_string_literal: true

require 'rock_gazebo/syskit/erb_loader'
require 'rock_gazebo/syskit/test'

module RockGazebo
    module Syskit
        describe ERBLoader do
            describe "normalization and initialization" do
                it "normalizes a plain String model name" do
                    loader = ERBLoader.new("model://simple_model")
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 1, templates.size
                    assert_equal "model://simple_model", templates.first.model
                    assert_equal "model.sdf", templates.first.output_file_name
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
                        { model: "model://model_b", output_folder_name: "b" }
                    )
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 2, templates.size
                    assert_equal "model://model_a", templates[0].model
                    assert_equal "model://model_b", templates[1].model
                    assert_equal "b", templates[1].output_folder_name
                end

                it "normalizes a single ModelTemplate object directly" do
                    template_obj = ERBLoader::ModelTemplate.new(
                        "model://template_obj", output_file_name: "custom.sdf")
                    loader = ERBLoader.new(template_obj)
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 1, templates.size
                    assert_equal "model://template_obj", templates.first.model
                    assert_equal "custom.sdf", templates.first.output_file_name
                end

                it "normalizes a Hash representation inside an array" do
                    loader = ERBLoader.new(
                        model: "model://hash_model",
                        erb_args: { arg1: "val1" },
                        output_folder_name: "custom_folder"
                    )
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 1, templates.size
                    assert_equal "model://hash_model", templates.first.model
                    assert_equal({ arg1: "val1" }, templates.first.erb_args)
                    assert_equal "custom_folder", templates.first.output_folder_name
                end

                it "normalizes an array of mixed types" do
                    template_obj = ERBLoader::ModelTemplate.new("model://object_model")
                    loader = ERBLoader.new([
                        "model://string_model",
                        { model: "model://hash_model", output_file_name: "custom.sdf" },
                        template_obj
                    ])
                    templates = loader.instance_variable_get(:@templates)
                    assert_equal 3, templates.size

                    assert_equal "model://string_model", templates[0].model
                    assert_equal "model://hash_model", templates[1].model
                    assert_equal "custom.sdf", templates[1].output_file_name
                    assert_equal "model://object_model", templates[2].model
                end

                it "fails fast if a Hash is missing the required :model key" do
                    assert_raises(ArgumentError) do
                        ERBLoader.new(erb_args: { key: "val" })
                    end
                end

                it "fails fast if passed an unsupported parameter type" do
                    assert_raises(ArgumentError) do
                        ERBLoader.new(12345)
                    end
                end
            end

            describe "#load" do
                it "delegates rendering details to syskit_conf" do
                    loader = ERBLoader.new(
                        model: "model://test_model",
                        erb_args: { param: "value" },
                        output_folder_name: "test_output"
                    )

                    mock_conf = flexmock
                    mock_conf.should_receive(:pre_render_erb_sdf_model)
                             .with(
                                 "model://test_model",
                                 erb_args: { param: "value" },
                                 output_file_name: "model.sdf",
                                 output_folder_name: "test_output"
                             ).once

                    loader.load(mock_conf)
                end
            end
        end
    end
end
