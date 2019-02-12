require_relative 'helpers'

module RockGazebo
    module Syskit
        describe RobotDefinitionExtension do
            before do
                @robot_model = ::Syskit::Robot::RobotDefinition.new
                Roby.app.using_task_library 'rock_gazebo'
                Conf.sdf = SDF.new
                require 'models/orogen/rock_gazebo'
            end

            describe "#find_actual_model" do
                before do
                    @robot_sdf = ::SDF::Root.load('model://simple_model', flatten: false).each_model.first
                    root = ::SDF::Root.load expand_fixture_world('attached_simple_model.world'), flatten: false
                    @world = root.each_world.first
                end

                it "resolves a toplevel model" do
                    actual_model, enclosing_model =
                        @robot_model.find_actual_model('attachment', @world.each_model.to_a)
                    assert_equal @world.each_model.first, actual_model
                    assert_nil enclosing_model
                end
                it "resolves a model-in-model as well as its root" do
                    actual_model, enclosing_model =
                        @robot_model.find_actual_model('included_model', @world.each_model.to_a)
                    assert_equal @world.each_model.first, enclosing_model
                    assert_equal @world.each_model.first.each_model.first, actual_model
                end
            end

            describe "#define_submodel_device" do
                before do
                    root = ::SDF::Root.load expand_fixture_world('attached_simple_model.world'), flatten: false
                    @world = root.each_world.first
                    ::Syskit.conf.use_deployments_from_gazebo(@world, prefix: 'gazebo_prefix')
                end

                it "creates a device that exports the submodel" do
                    attachment = @world.each_model.to_a.first
                    root_device, _ = @robot_model.expose_gazebo_model(
                        attachment, "gazebo_prefix:test")
                    device = @robot_model.define_submodel_device(
                        'included_model', root_device, attachment.each_model.first)

                    assert_equal CommonModels::Devices::Gazebo::Model, device.model

                    submodel_driver_m = device.to_instance_requirements

                    assert_equal "attachment::included_model::root",
                        submodel_driver_m.frame_mappings['included_model_source']
                    assert_equal "world",
                        submodel_driver_m.frame_mappings['included_model_target']
                end
            end

            describe "root model" do
                before do
                    root = ::SDF::Root.load expand_fixture_world(
                        'simple_model.world'), flatten: false
                    ::Syskit.conf.use_deployments_from_gazebo(root.each_world.first)

                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                end

                describe "the model export" do
                    before do
                        flexmock(Roby).should_receive(:warn_deprecated).at_least.once
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo:test', name: 'renamed_model')
                        @device = @robot_model.find_device('renamed_model')
                        @model_driver_m = @device.to_instance_requirements
                        @driver_m = @model_driver_m.to_component_model
                    end

                    it "exposes the model device" do
                        assert_equal CommonModels::Devices::Gazebo::RootModel,
                            @device.model
                    end

                    it "selects the deployment" do
                        assert_has_single_deployed_task 'gazebo:test:included_model',
                            @model_driver_m
                    end

                    it "sets up the transforms" do
                        assert_equal "included_model",
                            @device.frame_transform.from
                        assert_equal "world",
                            @device.frame_transform.to
                    end
                end

                describe "deprecated link and sensor export behavior" do
                    before do
                        flexmock(Roby).should_receive(:warn_deprecated).at_least.once
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo:test', name: 'renamed_model')
                    end

                    it "sets up the device transform on the link device" do
                        device = @robot_model.find_device('child_link')
                        assert_equal 'included_model::child', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end

                    it "exposes the links from the model but does not prefix them with the model name" do
                        device = @robot_model.find_device('child_link')
                        link_driver_m = device.to_instance_requirements
                        driver_m = link_driver_m.to_component_model
                        assert_has_single_deployed_task 'gazebo:test:included_model',
                            link_driver_m
                        assert_equal driver_m.child_link_port,
                            link_driver_m.link_state_samples_port.to_component_port
                        transform = driver_m.find_transform_of_port(
                            driver_m.child_link_port)
                        assert_equal "child_source", transform.from
                        assert_equal "child_target", transform.to

                        assert_equal "included_model::child",
                            link_driver_m.frame_mappings['child_source']
                    end

                    it "exposes the sensors from the model but does not prefix them "\
                        "with the model name" do

                        device = @robot_model.find_device('g_sensor')
                        sensor_driver_m = device.to_instance_requirements
                        driver_m = sensor_driver_m.to_component_model
                        assert_has_single_deployed_task 'gazebo:test:included_model:g',
                            sensor_driver_m
                        assert_equal OroGen::RockGazebo::GPSTask,
                            driver_m.model
                        transform = driver_m.find_transform_of_port(
                            driver_m.position_samples_port)

                        assert_equal "included_model::root",
                            device.frame_transform.from
                        assert_equal "world",
                            device.frame_transform.to
                    end
                end

                describe "link export behavior" do
                    before do
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo:test', name: 'renamed_model', prefix_device_with_name: true)
                    end

                    it "sets up the device transform on the link device" do
                        device = @robot_model.find_device('renamed_model_root_link')
                        assert_equal 'included_model::root', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                        device = @robot_model.find_device('renamed_model_child_link')
                        assert_equal 'included_model::child', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end

                    it "exposes the links from the model" do
                        device, link_driver_m, driver_m, transform =
                            common_link_export_behavior

                        link_driver_m = device.to_instance_requirements
                        assert_has_single_deployed_task 'gazebo:test:included_model',
                            link_driver_m
                        assert_equal "included_model::child",
                            link_driver_m.frame_mappings['child_source']
                        assert_equal "child_source", transform.from
                        assert_equal "child_target", transform.to
                    end

                    it "exposes the sensors from the model" do
                        device, sensor_driver_m, driver_m, transform =
                            common_sensor_export_behavior

                        assert_has_single_deployed_task 'gazebo:test:included_model:g',
                            sensor_driver_m
                        assert_equal "included_model::root",
                            device.frame_transform.from
                        assert_equal "world",
                            device.frame_transform.to
                    end
                end
            end

            def common_link_export_behavior(link_name = "child")
                device = @robot_model.find_device("renamed_model_#{link_name}_link")
                link_driver_m = device.to_instance_requirements
                driver_m = link_driver_m.to_component_model

                port = driver_m.find_port("renamed_model_#{link_name}_link")
                assert_equal port, link_driver_m.link_state_samples_port.to_component_port
                transform = driver_m.find_transform_of_port(port)
                return device, link_driver_m, driver_m, transform
            end

            def common_sensor_export_behavior
                device = @robot_model.find_device('renamed_model_g_sensor')
                sensor_driver_m = device.to_instance_requirements
                driver_m = sensor_driver_m.to_component_model
                assert_equal OroGen::RockGazebo::GPSTask,
                    driver_m.model
                transform = driver_m.find_transform_of_port(
                    driver_m.position_samples_port)
                return device, sensor_driver_m, driver_m, transform
            end

            describe "model-in-model" do
                before do
                    root = ::SDF::Root.load expand_fixture_world(
                        'attached_simple_model.world'), flatten: false
                    @world = root.each_world.first
                    ::Syskit.conf.use_deployments_from_gazebo(@world)
                    @robot_sdf = @world.each_model.first.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo:test', name: 'renamed_model',
                        prefix_device_with_name: true)
                end

                it "defines the enclosing device" do
                    assert @robot_model.find_device('attachment')
                end

                it "sets up the device transform on the submodel device, "\
                   "using the submodel's root link as root frame" do
                    device = @robot_model.find_device('renamed_model')
                    assert_equal 'included_model::root', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it "defines a device that exposes the submodel" do
                    device = @robot_model.find_device('renamed_model')
                    submodel_driver_m = device.to_instance_requirements
                    assert_equal "included_model::root",
                        submodel_driver_m.frame_mappings['renamed_model_source']
                end

                it "ignores the links from the enclosing model" do
                    refute @robot_model.find_device('attachment_in_attachment_link')
                end

                it "sets up the transforms on the submodel's links" do
                    device, _  = common_link_export_behavior
                    assert_equal 'included_model::child', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it "exposes the links from the submodel" do
                    device, link_driver_m, driver_m, transform =
                        common_link_export_behavior

                    assert_has_single_deployed_task 'gazebo:test:attachment',
                        link_driver_m
                    assert_equal "included_model::child",
                        link_driver_m.frame_mappings['included_model_child_source']
                    assert_equal "included_model_child_source", transform.from
                    assert_equal "included_model_child_target", transform.to
                end

                it "sets up the transforms on the submodel's sensors" do
                    device, _  = common_sensor_export_behavior
                    assert_equal "included_model::root",
                        device.frame_transform.from
                    assert_equal "world",
                        device.frame_transform.to
                end

                it "exposes the sensors from the submodel" do
                    device, sensor_driver_m, driver_m, transform =
                        common_sensor_export_behavior

                    assert_has_single_deployed_task 'gazebo:test:attachment:g',
                        sensor_driver_m
                end
            end

            describe "model containing another model" do
                before do
                    root = ::SDF::Root.load expand_fixture_world(
                        'attached_simple_model.world'), flatten: false
                    @world = root.each_world.first
                    ::Syskit.conf.use_deployments_from_gazebo(@world)
                    @robot_sdf = @world.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo:test', name: 'renamed_model',
                        prefix_device_with_name: true)
                end

                it "sets up the transforms on the submodel's links" do
                    device, _  = common_link_export_behavior 'included_model_child'
                    assert_equal 'attachment::included_model::child', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it "exposes the links from the submodel" do
                    device, link_driver_m, driver_m, transform =
                        common_link_export_behavior 'included_model_child'

                    assert_has_single_deployed_task 'gazebo:test:attachment',
                        link_driver_m
                    assert_equal "attachment::included_model::child",
                        link_driver_m.frame_mappings['child_source']
                    assert_equal "child_source", transform.from
                    assert_equal "child_target", transform.to
                end

                it "exposes the sensors from the submodel" do
                    device, sensor_driver_m, driver_m, transform =
                        common_sensor_export_behavior

                    assert_has_single_deployed_task 'gazebo:test:attachment:g',
                        sensor_driver_m
                    assert_equal "attachment::included_model::root",
                        device.frame_transform.from
                    assert_equal "world",
                        device.frame_transform.to
                end
            end

            def assert_has_single_deployed_task(expected_name, object)
                group = object.deployment_group
                tasks = group.each_deployed_task.to_a
                assert_equal 1, tasks.size
                assert_equal expected_name, tasks.first.first
            end
        end
    end
end

