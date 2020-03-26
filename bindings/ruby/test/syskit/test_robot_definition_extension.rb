# frozen_string_literal: true

require 'rock_gazebo/syskit/test'
require_relative '../helpers'

module RockGazebo
    module Syskit
        describe RobotDefinitionExtension do
            before do
                @robot_model = ::Syskit::Robot::RobotDefinition.new
                Roby.app.using_task_library 'rock_gazebo'
                require 'models/orogen/rock_gazebo'
            end

            describe '#find_actual_model' do
                before do
                    @robot_sdf =
                        ::SDF::Root.load('model://simple_model', flatten: false)
                                   .each_model.first
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                end

                it 'resolves a toplevel model' do
                    actual_model, enclosing_model = @robot_model.find_actual_model(
                        'attachment', @world.each_model.to_a
                    )
                    assert_equal @world.each_model.first, actual_model
                    assert_nil enclosing_model
                end
                it 'resolves a model-in-model as well as its root' do
                    actual_model, enclosing_model = @robot_model.find_actual_model(
                        'included_model', @world.each_model.to_a
                    )
                    assert_equal @world.each_model.first, enclosing_model
                    assert_equal @world.each_model.first.each_model.first, actual_model
                end
            end

            describe '#define_submodel_device' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                end

                it 'creates a device that exports the submodel\'s joints' do
                    attachment = @world.each_model.to_a.first
                    root_device = @robot_model.expose_gazebo_model(
                        attachment, 'gazebo_prefix'
                    )
                    device = @robot_model.define_submodel_device(
                        'included_model', root_device, attachment.each_model.first
                    )

                    assert_equal CommonModels::Devices::Gazebo::Model, device.model

                    submodel_driver_m = device.to_instance_requirements

                    driver_m = submodel_driver_m.to_component_model
                    assert_equal driver_m.included_model_joints_cmd_port,
                                 submodel_driver_m.joints_cmd_port.to_component_port
                end
            end

            describe 'root model' do
                before do
                    root = ::SDF::Root.load expand_fixture_world('simple_model.world')
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                end

                describe 'the model export' do
                    before do
                        flexmock(Roby).should_receive(:warn_deprecated).at_least.once
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo', name: 'renamed_model'
                        )
                        @device = @robot_model.find_device('renamed_model')
                        @model_driver_m = @device.to_instance_requirements
                        @driver_m = @model_driver_m.to_component_model
                    end

                    it 'exposes the model device' do
                        assert_equal CommonModels::Devices::Gazebo::RootModel,
                                     @device.model
                    end

                    it 'sets up the deployment name' do
                        assert_equal ['gazebo:included_model'],
                                     @model_driver_m.deployment_hints.to_a
                    end

                    it 'sets up the transforms' do
                        assert_equal 'included_model', @device.frame_transform.from
                        assert_equal 'world', @device.frame_transform.to
                    end
                end

                describe 'deprecated link and sensor export behavior' do
                    before do
                        flexmock(Roby).should_receive(:warn_deprecated).at_least.once
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo', name: 'renamed_model'
                        )
                    end

                    it 'sets up the device transform on the link device' do
                        device = @robot_model.find_device('child_link')
                        assert_equal 'included_model::child', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end

                    it 'exposes the links from the model but does not prefix '\
                       'them with the model name' do
                        device = @robot_model.find_device('child_link')
                        link_driver_m = device.to_instance_requirements
                        driver_m = link_driver_m.to_component_model
                        assert_equal ['gazebo:included_model'],
                                     link_driver_m.deployment_hints.to_a
                        assert_equal(
                            driver_m.child_link_port,
                            link_driver_m.link_state_samples_port.to_component_port
                        )
                        transform = driver_m.find_transform_of_port(
                            driver_m.child_link_port
                        )
                        assert_equal 'child_source', transform.from
                        assert_equal 'child_target', transform.to

                        assert_equal 'included_model::child',
                                     link_driver_m.frame_mappings['child_source']
                    end

                    it 'exposes the sensors from the model but does not prefix '\
                       'them with the model name' do
                        device = @robot_model.find_device('g_sensor')
                        sensor_driver_m = device.to_instance_requirements
                        driver_m = sensor_driver_m.to_component_model
                        assert_equal ['gazebo:included_model:g'],
                                     sensor_driver_m.deployment_hints.to_a
                        assert_equal OroGen::RockGazebo::GPSTask, driver_m.model
                        driver_m.find_transform_of_port(driver_m.position_samples_port)

                        assert_equal 'included_model::root', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end
                end

                describe 'link export behavior' do
                    before do
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo',
                            name: 'renamed_model', prefix_device_with_name: true
                        )
                    end

                    it 'sets up the device transform on the link device' do
                        device = @robot_model.find_device('renamed_model_root_link')
                        assert_equal 'included_model::root', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                        device = @robot_model.find_device('renamed_model_child_link')
                        assert_equal 'included_model::child', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end

                    it 'exposes the links from the model' do
                        device, _, _, transform =
                            common_link_export_behavior

                        link_driver_m = device.to_instance_requirements
                        assert_equal ['gazebo:included_model'],
                                     link_driver_m.deployment_hints.to_a
                        assert_equal 'included_model::child',
                                     link_driver_m.frame_mappings['child_source']
                        assert_equal 'child_source', transform.from
                        assert_equal 'child_target', transform.to
                    end

                    it 'exposes the sensors from the model' do
                        device, sensor_driver_m, =
                            common_sensor_export_behavior

                        assert_equal ['gazebo:included_model:g'],
                                     sensor_driver_m.deployment_hints.to_a
                        assert_equal 'included_model::root', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end
                end
            end

            def common_link_export_behavior(link_name = 'child')
                device = @robot_model.find_device("renamed_model_#{link_name}_link")
                link_driver_m = device.to_instance_requirements
                driver_m = link_driver_m.to_component_model

                port = driver_m.find_port("renamed_model_#{link_name}_link")
                assert_equal port, link_driver_m.link_state_samples_port.to_component_port
                transform = driver_m.find_transform_of_port(port)
                [device, link_driver_m, driver_m, transform]
            end

            def common_sensor_export_behavior
                device = @robot_model.find_device('renamed_model_g_sensor')
                sensor_driver_m = device.to_instance_requirements
                driver_m = sensor_driver_m.to_component_model
                assert_equal OroGen::RockGazebo::GPSTask, driver_m.model
                transform = driver_m.find_transform_of_port(
                    driver_m.position_samples_port
                )
                [device, sensor_driver_m, driver_m, transform]
            end

            describe 'model-in-model' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo',
                        name: 'renamed_model',
                        prefix_device_with_name: true
                    )
                end

                it 'defines the enclosing device' do
                    assert @robot_model.find_device('attachment')
                end

                it 'sets up the device transform on the submodel device, '\
                   'using the submodel\'s root link as root frame' do
                    device = @robot_model.find_device('renamed_model')
                    assert_equal 'included_model::root', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'defines a device that exposes the submodel' do
                    device = @robot_model.find_device('renamed_model')
                    submodel_driver_m = device.to_instance_requirements
                    assert_equal 'included_model::root',
                                 submodel_driver_m.frame_mappings['renamed_model_source']
                end

                it 'ignores the links from the enclosing model' do
                    refute @robot_model.find_device('attachment_in_attachment_link')
                end

                it 'sets up the transforms on the submodel\'s links' do
                    device, = common_link_export_behavior
                    assert_equal 'included_model::child', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'exposes the links from the submodel' do
                    _, link_driver_m, _, transform = common_link_export_behavior

                    assert_equal ['gazebo:attachment'],
                                 link_driver_m.deployment_hints.to_a
                    assert_equal(
                        'included_model::child',
                        link_driver_m.frame_mappings['included_model_child_source']
                    )
                    assert_equal 'included_model_child_source', transform.from
                    assert_equal 'included_model_child_target', transform.to
                end

                it "sets up the transforms on the submodel's sensors" do
                    device, = common_sensor_export_behavior
                    assert_equal 'included_model::root', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'exposes the sensors from the submodel' do
                    _, sensor_driver_m, = common_sensor_export_behavior

                    assert_equal ['gazebo:attachment:g'],
                                 sensor_driver_m.deployment_hints.to_a
                end
            end

            describe 'model containing another model' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo',
                        name: 'renamed_model', prefix_device_with_name: true
                    )
                end

                it 'sets up the transforms on the submodel\'s links' do
                    device, = common_link_export_behavior 'included_model_child'
                    assert_equal 'attachment::included_model::child',
                                 device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'exposes the links from the submodel' do
                    _, link_driver_m, _, transform =
                        common_link_export_behavior 'included_model_child'

                    assert_equal ['gazebo:attachment'],
                                 link_driver_m.deployment_hints.to_a
                    assert_equal 'attachment::included_model::child',
                                 link_driver_m.frame_mappings['child_source']
                    assert_equal 'child_source', transform.from
                    assert_equal 'child_target', transform.to
                end

                it 'exposes the sensors from the submodel' do
                    device, sensor_driver_m, = common_sensor_export_behavior

                    assert_equal ['gazebo:attachment:g'],
                                 sensor_driver_m.deployment_hints.to_a
                    assert_equal 'attachment::included_model::root',
                                 device.frame_transform.from
                    assert_equal 'world',
                                 device.frame_transform.to
                end
            end
        end
    end
end
