# frozen_string_literal: true

require 'rock/gazebo'
require 'rock_gazebo/syskit/test'
require 'rock_gazebo/syskit/erb_loader'
require_relative 'helpers'
require 'tmpdir'

module RockGazebo
    module Syskit
        describe ConfigurationExtension do

            before do
                Rock::Gazebo.model_path = [File.expand_path(File.join("..", "models"), __dir__)]

                @conf = ::Syskit.conf
                Conf.sdf = SDF.new

                # Clear any previously registered deployments to prevent "TaskNameAlreadyInUse" conflicts
                @conf.deployment_group.deployed_tasks.clear
                @conf.deployment_group.deployments.clear
            end

            describe "Pruning of Stale Symlinks" do
                it "prunes broken symlinks on startup while leaving valid files untouched" do
                    # Create a temporary directory to represent STABLE_TMP_DIR
                    sandbox_dir = Dir.mktmpdir
                    flexmock(::RockGazebo::Syskit::ERB).should_receive(:const_get).with(:STABLE_TMP_DIR).and_return(sandbox_dir)
                    # Force the helper to use our tmp dir
                    flexmock(::RockGazebo::Syskit::ERB).should_receive(:const_defined?).with(:STABLE_TMP_DIR).and_return(true)

                    original_stable_dir = ::RockGazebo::Syskit::ERB::STABLE_TMP_DIR
                    begin
                        ::RockGazebo::Syskit::ERB.send(:remove_const, :STABLE_TMP_DIR)
                        ::RockGazebo::Syskit::ERB.const_set(:STABLE_TMP_DIR, sandbox_dir)

                        # Create a valid file/link
                        valid_target = File.join(sandbox_dir, "valid_file.txt")
                        File.write(valid_target, "valid content")
                        valid_link = File.join(sandbox_dir, "valid_link")
                        FileUtils.ln_s(valid_target, valid_link)

                        # Create a broken/stale symlink pointing to a non-existent directory
                        broken_link = File.join(sandbox_dir, "broken_link")
                        FileUtils.ln_s("/tmp/non_existent_folder_path", broken_link)

                        assert File.symlink?(valid_link)
                        assert File.exist?(valid_link)
                        assert File.symlink?(broken_link)
                        refute File.exist?(broken_link)

                        # Run stale symlink pruning helper
                        ConfigurationExtension.prune_stale_symlinks

                        # Assert broken symlink has been deleted, but valid links/files are preserved
                        assert File.symlink?(valid_link)
                        assert File.exist?(valid_link)
                        refute File.symlink?(broken_link)
                        refute File.exist?(broken_link)
                    ensure
                        # Restore the original constant
                        ::RockGazebo::Syskit::ERB.send(:remove_const, :STABLE_TMP_DIR)
                        ::RockGazebo::Syskit::ERB.const_set(:STABLE_TMP_DIR, original_stable_dir)
                        FileUtils.rm_rf(sandbox_dir)
                    end
                end
            end

            describe "#use_sdf_world" do
                it "successfully loads a world and updates world_file_path" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    @conf.use_sdf_world(world_path)

                    assert_equal world_path, Conf.sdf.world_file_path
                end

                it "runs the ERBLoader before loading the world" do
                    template_path = File.expand_path('../models/simple_model_erb/model.sdf.erb', __dir__)
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)

                    loader = ERBLoader.new(
                        model: template_path,
                        output_folder_name: "test_erb_integration_model"
                    )

                    @conf.use_sdf_world(world_path, loader: loader)
                    assert Rock::Gazebo.model_path.any? { |p| p.include?("gazebo_erb_models") }
                end

                it "handles loader: nil safety cleanly" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    @conf.use_sdf_world(world_path, loader: nil)

                    assert_equal world_path, Conf.sdf.world_file_path
                end

                it "raises a LoadError if use_sdf_world is called twice" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    Conf.sdf.world_file_path = "/some/path"

                    assert_raises(LoadError) do
                        @conf.use_sdf_world(world_path)
                    end
                end

                it "raises a LoadError if a profile uses use_sdf_model before world load" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    Conf.sdf.has_profile_loaded = true

                    assert_raises(LoadError) do
                        @conf.use_sdf_world(world_path)
                    end
                end

                it "respects Conf.sdf.world_path override" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    override_world = File.expand_path('../worlds/simple_model.world', __dir__)
                    Conf.sdf.world_path = override_world

                    @conf.use_sdf_world(world_path)
                    assert_equal override_world, Conf.sdf.world_file_path
                end

                it "successfully unlinks symbolic links when Roby logging is enabled" do
                    template_path = File.expand_path('../models/simple_model_erb/model.sdf.erb', __dir__)

                    # Simulating a real scenario where logging is enabled and active
                    log_dir = Dir.mktmpdir
                    flexmock(Roby.app).should_receive(:created_log_dir?).and_return(true)
                    flexmock(Roby.app).should_receive(:log_dir).and_return(log_dir)

                    # Create a folder in log_dir and a symlink in STABLE_TMP_DIR
                    rendered_dir = @conf.pre_render_erb_sdf_model(
                        template_path,
                        output_folder_name: "test_unlink_symlink_model"
                    )

                    symlink_path = File.join(ERB::STABLE_TMP_DIR, "test_unlink_symlink_model")
                    assert File.symlink?(symlink_path)

                    # Trigger unlinking
                    @conf.unlink_gazebo_models

                    # Verify symlink is successfully removed
                    refute File.exist?(symlink_path)
                    refute File.symlink?(symlink_path)
                ensure
                    FileUtils.rm_rf(log_dir) if log_dir
                end

                it "does not crash when unlink_gazebo_models is called and there is no symlink to remove" do
                    template_path = File.expand_path('../models/simple_model_erb/model.sdf.erb', __dir__)

                    # Simulating an unlogged scenario where no symlink is created
                    rendered_dir = @conf.pre_render_erb_sdf_model(
                        template_path,
                        output_folder_name: "test_unlink_no_symlink_model"
                    )

                    symlink_path = File.join(ERB::STABLE_TMP_DIR, "test_unlink_no_symlink_model")
                    refute File.symlink?(symlink_path)

                    @conf.unlink_gazebo_models

                    assert File.directory?(rendered_dir)
                ensure
                    FileUtils.rm_rf(rendered_dir) if rendered_dir
                end

                it "raises a warning of pre-rendered model collisions" do
                    template_path = File.expand_path('../models/simple_model_erb/model.sdf.erb', __dir__)

                    rendered_dir_1 = @conf.pre_render_erb_sdf_model(
                        template_path,
                        output_folder_name: "test_collision_model"
                    )

                    # We expect a warning logger message to be dispatched upon second pre-render of the same name
                    flexmock(Robot).should_receive(:warn)
                                   .with(/SDF model collision detected: another model with the same basename 'test_collision_model'/)
                                   .once

                    rendered_dir_2 = @conf.pre_render_erb_sdf_model(
                        template_path,
                        output_folder_name: "test_collision_model"
                    )
                ensure
                    FileUtils.rm_rf(rendered_dir_1) if rendered_dir_1
                    FileUtils.rm_rf(rendered_dir_2) if rendered_dir_2
                end
            end

            describe "#use_gazebo_world" do
                before do
                    @log_dir = Dir.mktmpdir
                    flexmock(Roby.app).should_receive(:log_dir).and_return(@log_dir)
                end

                after do
                    FileUtils.rm_rf(@log_dir) if @log_dir
                end

                it "handles loader: nil safety cleanly" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    @conf.use_gazebo_world(world_path, loader: nil)

                    assert_equal world_path, Conf.sdf.world_file_path
                end

                it "delegates the loader argument to use_sdf_world" do
                    world_path = File.expand_path('../worlds/simple_model.world', __dir__)
                    mock_loader = flexmock
                    mock_loader.should_receive(:load).with(@conf).once

                    @conf.use_gazebo_world(world_path, loader: mock_loader)
                end
            end
        end
    end
end
