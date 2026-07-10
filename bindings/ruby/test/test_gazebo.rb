# frozen_string_literal: true

require 'rb-inotify'
require "rock_gazebo/test"
require_relative 'helpers'

module Rock
    describe Gazebo do
        include Helpers

        describe ".download_missing_models" do
            before do
                setup_gazebo_erb_sandbox
                @original_model_path = Rock::Gazebo.model_path.dup
                @original_download_path = Rock::Gazebo.download_path

                # Point model search path and download path into sandbox
                Rock::Gazebo.model_path = [@sandbox_dir]
                Rock::Gazebo.download_path = @sandbox_dir
                @stable_temp_dir = File.join(@sandbox_dir, "gazebo_erb_models")
                ::FileUtils.mkdir_p(@stable_temp_dir)

            end

            after do
                Rock::Gazebo.model_path = @original_model_path
                Rock::Gazebo.download_path = @original_download_path
                teardown_gazebo_erb_sandbox
            end

            it "immediately returns if the model already exists in the stable temp directory" do
                model_name = "test_model"
                model_dir = File.join(@stable_temp_dir, model_name)
                ::FileUtils.mkdir_p(model_dir)

                File.write(File.join(model_dir, "model.config"), "<model></model>")
                File.write(File.join(model_dir, "model.sdf"), "<sdf></sdf>")

                flexmock(SDF::Root).should_receive(:load).with("scene.world").once.and_return(true)

                flexmock(::INotify::Notifier).should_receive(:new).never
                flexmock(Rock::Gazebo).should_receive(:download_model).never

                Rock::Gazebo.download_missing_models("scene.world", wait: true)
            end

            it "blocks using inotify and wakes up instantly when the model folder is created" do
                model_name = "delayed_model"
                model_dir = File.join(@stable_temp_dir, model_name)

                mock_err = SDF::XML::NoSuchModel.new(model_name)
                flexmock(SDF::Root).should_receive(:load).with("scene.world").and_raise(mock_err).once

                Thread.new do
                    sleep 0.1
                    ::FileUtils.mkdir_p(model_dir)
                    File.write(File.join(model_dir, "model.sdf"), "<sdf></sdf>")
                end

                flexmock(SDF::Root).should_receive(:load).with("scene.world").and_return(true).once
                flexmock(SDF::XML).should_receive(:clear_cache).once

                Rock::Gazebo.download_missing_models("scene.world", wait: true, timeout: 2)
            end

            it " blocks when there is an existing model path with the correct name but \
            without .sdf (common for .erb model path)" do
                model_dir = File.join(@stable_temp_dir, 'simple_model_erb')

                flexmock(SDF::Root)
                    .should_receive(:load)
                    .with("scene.world")
                    .and_raise(Errno::ENOENT, File.expand_path(File.join("models", "simple_model_erb", "model.sdf"),__dir__) )
                    .once

                Thread.new do
                    sleep 0.1
                    ::FileUtils.mkdir_p(model_dir)
                    File.write(File.join(model_dir, "model.sdf"), "<sdf></sdf>")
                end

                flexmock(SDF::Root).should_receive(:load).with("scene.world").and_return(true).once
                flexmock(SDF::XML).should_receive(:clear_cache).once

                Rock::Gazebo.download_missing_models("scene.world", wait: true, timeout: 2)
            end

            it "raises a NoSuchModel error if the wait timeout expires before the model is created" do
                model_name = "non_existent_model"
                mock_err = SDF::XML::NoSuchModel.new(model_name)

                flexmock(SDF::Root).should_receive(:load).with("scene.world").and_raise(mock_err).once

                assert_raises(SDF::XML::NoSuchModel) do
                    Rock::Gazebo.download_missing_models("scene.world", wait: true, timeout: 0.1)
                end
            end

            it "does not wait and falls back immediately to standard downloader if wait is false" do
                model_name = "missing_no_wait"
                mock_err = SDF::XML::NoSuchModel.new(model_name)

                flexmock(SDF::Root).should_receive(:load).with("scene.world").and_raise(mock_err).once
                flexmock(Rock::Gazebo).should_receive(:download_model).with(model_name).once.and_return(true)
                flexmock(SDF::Root).should_receive(:load).with("scene.world").and_return(true).once

                Rock::Gazebo.download_missing_models("scene.world", wait: false)
            end
        end

        describe ".process_sdf_world" do
            it "resolve the 'filename' argument of a <task ...> element "\
               "in <plugin ...>" do
                loader = flexmock
                task_model = create_task_model_mock(loader, "some::Task")
                sdf = ::SDF::Root.from_string(
                    <<~SDF
                    <sdf><world><plugin>
                        <task model="some::Task" />
                    </plugin></world></sdf>
                    SDF
                )

                xml = Gazebo.process_sdf_world(sdf, loader: loader)
                task = REXML::XPath.first(xml, "//task")
                assert_equal "some::Task_project_path", task.attributes["filename"]
            end

            it "resolve the 'name' argument of a <plugin> element of a model inside a model "\
                "in <plugin ...>" do
                sdf = ::SDF::Root.from_string(
                    <<~SDF
                    <sdf><world name="world"><model name= "model_A"><model name= "model_B"><plugin name="plugin_A">
                    </plugin></model></model></world></sdf>
                    SDF
                )
                xml = Gazebo.process_sdf_world(sdf)
                plugin = REXML::XPath.first(xml, "//plugin")
                assert_equal "gazebo__world__model_A__model_B__plugin_A",
                                plugin.attributes["name"]
            end

            it "auto-loads a task's dependent typekits" do
                loader = flexmock
                tk_a = create_typekit_mock(loader, "tk_a")
                tk_b = create_typekit_mock(loader, "tk_b")
                task_model = create_task_model_mock(
                    loader, "some::Task",
                    interface_typekits: { "tk_a" => tk_a, "tk_b" => tk_b }
                )

                sdf = ::SDF::Root.from_string(
                    <<~SDF
                    <sdf><world><plugin>
                        <task model="some::Task" />
                    </plugin></world></sdf>
                    SDF
                )

                xml = Gazebo.process_sdf_world(sdf, loader: loader)
                expected = %w[
                    tk_a_library_path tk_a_transport_corba_path
                    tk_a_transport_mqueue_path tk_a_transport_typelib_path
                    tk_b_library_path tk_b_transport_corba_path
                    tk_b_transport_mqueue_path tk_b_transport_typelib_path
                ]
                assert_world_loads_libraries(xml, expected)
            end

            def assert_world_loads_libraries(xml, expected)
                loads =
                    REXML::XPath
                    .each(xml, "/sdf/world/plugin[@name='rock_components']/load")
                    .map { |xml| xml.attributes["path"] }
                assert_equal expected.to_set, loads.to_set
            end

            def create_task_model_mock(
                loader, name,
                interface_typekits: {}, project_name: "#{name}_project"
            )
                mock = flexmock
                mock.should_receive(project: Struct.new(:name).new(project_name))
                loader.should_receive(:task_model_from_name)
                      .with(name).and_return(mock)
                loader.should_receive(:task_library_path_from_name)
                      .with(project_name).and_return("#{project_name}_path")

                interface_types =
                    interface_typekits.each_key.map { |tk_name| "#{tk_name}_type" }
                mock.should_receive(:each_interface_type).and_iterates(*interface_types)
                interface_typekits.each do |typekit_name, typekit|
                    loader.should_receive(:typekit_for).with("#{typekit_name}_type")
                          .and_return(typekit)
                end
            end

            def create_typekit_mock(loader, name, virtual: false)
                mock = flexmock
                mock.should_receive(name: name, virtual?: virtual)
                loader.should_receive(:typekit_library_path_from_name)
                      .with(name).and_return("#{name}_library_path")
                %w[typelib mqueue corba].each do |transport|
                    loader.should_receive(:transport_library_path_from_name)
                          .with(name, transport)
                          .and_return("#{name}_transport_#{transport}_path")
                end
                mock
            end
        end
    end
end
