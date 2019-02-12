require_relative 'helpers'

module RockGazebo
    module Syskit
        describe ProfileExtension do
            before do
                Roby.app.using_task_library 'rock_gazebo'
                require 'models/orogen/rock_gazebo'
                @profile = ::Syskit::Actions::Profile.new
                Conf.sdf = SDF.new
            end

            describe "#use_sdf_model" do
                it "raises if the path does not resolve to a SDF file" do
                    e = assert_raises(ArgumentError) do
                        @profile.use_sdf_model 'does', 'not', 'exist'
                    end
                    assert_equal 'does/not/exist cannot be resolved to a valid SDF file',
                        e.message
                end

                it "raises if the path resolves to a SDF file that does not have a model" do
                    e = assert_raises(ArgumentError) do
                        @profile.use_sdf_model 'model://empty'
                    end
                    assert_equal "#{expand_fixture_model('empty')} has no top level model, cannot use in use_sdf_model",
                        e.message
                end

                it "raises if the path resolves to a SDF file that has more than one model" do
                    e = assert_raises(ArgumentError) do
                        @profile.use_sdf_model 'model://two_toplevel_models'
                    end
                    assert_equal "#{expand_fixture_model('two_toplevel_models')} has more than one top level model, cannot use in use_sdf_model",
                        e.message
                end

                it "returns the loaded model" do
                    model = @profile.use_sdf_model 'model://simple_model'
                    assert_equal 'simple test model', model.name
                end

                it "gives access to the loaded model in #sdf_model" do
                    model = @profile.use_sdf_model 'model://simple_model'
                    assert_same model, @profile.sdf_model
                end

                it "overrides the model name if the name: option is given" do
                    model = @profile.use_sdf_model 'model://simple_model', as: 'test'
                    assert_equal 'test', model.name
                end

                it "loads the model in the transformer instance" do
                    flexmock(@profile.transformer).should_receive(:parse_sdf_model).
                        with(->(m) { m.name == 'simple test model' }).
                        once
                    @profile.use_sdf_model 'model://simple_model'
                end
            end

            describe "#use_sdf_world" do
                before do
                    Conf.sdf.load_sdf expand_fixture_world('attached_simple_model.world')
                end

                it "parses the world stored in Conf.sdf.world into the transformer" do
                    @profile.use_sdf_world
                    assert @profile.transformer.has_frame?('attachment::included_model::link'), @profile.transformer.frames.to_a
                end
                it "does not add the model loaded by #use_sdf_model to the transformer instance" do
                    @profile.use_sdf_model 'model://simple_model', as: 'included_model'
                    assert @profile.transformer.has_frame?('included_model::link'), @profile.transformer.frames.to_a
                    @profile.use_sdf_world
                    refute @profile.transformer.has_frame?('test::included_model::link')
                end
                it "creates transforms for joints between an included model and its parent" do
                    @profile.use_sdf_model 'model://simple_model', as: 'included_model'
                    @profile.use_sdf_world
                    transform = @profile.transformer.transformation_for 'included_model::link', 'attachment::in_attachment'
                    assert transform.translation.approx?(Eigen::Vector3.Zero)
                    assert transform.rotation.approx?(Eigen::Quaternion.Identity)
                end
                it "creates transforms for joints between the world and an included model" do
                    @profile.use_sdf_model 'model://simple_model', as: 'included_model'
                    @profile.use_sdf_world
                    transform = @profile.transformer.transformation_for 'included_model::link', 'test'
                    assert transform.translation.approx?(Eigen::Vector3.Zero)
                    assert transform.rotation.approx?(Eigen::Quaternion.Identity)
                end
                it "raises if the model loaded by #use_sdf_model cannot be found in the world" do
                    @profile.use_sdf_model 'model://model'
                    e = assert_raises(ArgumentError) do
                        @profile.use_sdf_world
                    end
                    assert_equal "cannot find model passed to #use_sdf_model in the current world. The expected model is not included anywhere.", e.message
                end
                it "raises if the model loaded by #use_sdf_model cannot be found in the world" do
                    @profile.use_sdf_model 'model://simple_model'
                    e = assert_raises(ArgumentError) do
                        @profile.use_sdf_world
                    end
                    assert_equal "cannot find model passed to #use_sdf_model in the current world. The expected model is included, but no includes have the expected name 'simple test model', found: included_model.", e.message
                end
                it "aliases the world's frame to 'world'" do
                    @profile.use_sdf_world
                    transform = @profile.transformer.transformation_for 'test', 'world'
                    assert transform.translation.approx?(Eigen::Vector3.Zero)
                    assert transform.rotation.approx?(Eigen::Quaternion.Identity)
                end
            end

            describe "#use_gazebo_world" do
                before do
                    Conf.sdf.load_sdf expand_fixture_world('attached_simple_model.world')
                end

                it "calls #use_sdf_world" do
                    flexmock(@profile).should_receive(:use_sdf_world).once.pass_thru
                    @profile.use_gazebo_world
                end
                it "exports the other models from the world but not their links" do
                    @profile.use_gazebo_world
                    assert @profile.robot.has_device?('other_model')
                end
                it "does not export the other model's links" do
                    @profile.use_gazebo_world
                    refute @profile.robot.has_device?('l')
                    refute @profile.robot.has_device?('other_model_l')
                end
                it "does not re-exports the model passed to #use_gazebo_model" do
                    @profile.use_gazebo_model 'model://simple_model', as: 'included_model',
                        prefix_device_with_name: true, use_world: false
                    @profile.use_gazebo_world
                end
            end
        end
    end
end
