# frozen_string_literal: true

require 'rock_gazebo/syskit/erb'
require 'rock_gazebo/syskit/test'
require_relative 'helpers'

module RockGazebo
    module Syskit
        describe ERB do
            include Helpers

            before do
                Roby.app.using_task_library 'rock_gazebo'
                require 'models/orogen/rock_gazebo'
                @profile = ::Syskit::Actions::Profile.new
                Conf.sdf = SDF.new
            end

            it "read_erb_file" do
                template_path = File.expand_path('../models/simple_model_erb/model.sdf.erb',__dir__)

                sdf_str = ERB.read_erb_file(template_path)

                assert sdf_str

                expected_sdf_str = <<~XML
                    <?xml version="1.0" ?>
                    <sdf version="1.6">
                        <model name="<%= model_name %>">
                            <link name="root">
                                <sensor name="g" type="gps" />
                            </link>
                            <link name="child" />
                            <joint name="roo2child" type="revolute">
                                <parent>root</parent>
                                <child>child</child>
                                <axis>
                                </axis>
                            </joint>

                            <% gps_sensors.each do |gps| %>
                                <link name="<%= gps[:name] %>">
                                    <pose><%= gps[:pose].join(' ') %></pose>
                                </link>
                                <joint name="<%= gps[:name] %>_attachment" type="fixed">
                                    <parent>root</parent>
                                    <child><%= gps[:name] %></child>
                                </joint>
                            <% end %>

                            <plugin name="gps_test">
                                <task model="rock_gazebo::GPSTask"/>
                            </plugin>
                        </model>
                    </sdf>
                    XML
                assert_equal(sdf_str, expected_sdf_str)
            end

            it "read_erb_file_wrong_extension" do
                assert_raises(ArgumentError) do
                    ERB.read_erb_file("../models/simple_model_erb/model.config")
                end
            end

            it "read_erb_file_do_not_exist" do
                assert_raises(ArgumentError) do
                    ERB.read_erb_file("/tmp/i_do_not_exist_i_hope.erb")
                end
            end

            it "parse_erb_as_str" do
                erb_content = <<~XML
                    <?xml version="1.0" ?>
                    <sdf version="1.6">
                        <model name="<%= model_name %>">
                            <link name="root">
                                <sensor name="g" type="gps" />
                            </link>
                            <link name="child" />
                            <joint name="roo2child" type="revolute">
                                <parent>root</parent>
                                <child>child</child>
                                <axis>
                                </axis>
                            </joint>

                            <% gps_sensors.each do |gps| %>
                                <link name="<%= gps[:name] %>">
                                    <pose><%= gps[:pose].join(' ') %></pose>
                                </link>
                                <joint name="<%= gps[:name] %>_attachment" type="fixed">
                                    <parent>root</parent>
                                    <child><%= gps[:name] %></child>
                                </joint>
                            <% end %>

                            <plugin name="gps_test">
                                <task model="rock_gazebo::GPSTask"/>
                            </plugin>
                        </model>
                    </sdf>
                    XML

                erb_args = {
                    model_name: "my_model_name",
                    gps_sensors: [
                        {
                            name: "gps",
                            pose: [-0.679, 0.0, 1.920, 0.0, 0.0, 0.0]
                        },
                        {
                            name: "gps2",
                            pose: [2.571, 0.044, 0.808, 0, 0, 0]
                        }
                    ]
                }
                resulting_sdf = ERB.parse_erb_as_str(erb_content, erb_args)

                expected_content = <<~XML
                    <?xml version="1.0" ?>
                    <sdf version="1.6">
                        <model name="my_model_name">
                            <link name="root">
                                <sensor name="g" type="gps" />
                            </link>
                            <link name="child" />
                            <joint name="roo2child" type="revolute">
                                <parent>root</parent>
                                <child>child</child>
                                <axis>
                                </axis>
                            </joint>

                                <link name="gps">
                                    <pose>-0.679 0.0 1.92 0.0 0.0 0.0</pose>
                                </link>
                                <joint name="gps_attachment" type="fixed">
                                    <parent>root</parent>
                                    <child>gps</child>
                                </joint>

                                <link name="gps2">
                                    <pose>2.571 0.044 0.808 0 0 0</pose>
                                </link>
                                <joint name="gps2_attachment" type="fixed">
                                    <parent>root</parent>
                                    <child>gps2</child>
                                </joint>

                            <plugin name="gps_test">
                                <task model="rock_gazebo::GPSTask"/>
                            </plugin>
                        </model>
                    </sdf>
                    XML

                formatted_erb = resulting_sdf.gsub(/\s+/, ' ').strip
                formatted_expected = expected_content.gsub(/\s+/, ' ').strip

                assert_equal(formatted_expected, formatted_erb)
            end

            it "parse_erb_as_str_raises_on_missing_args" do
                erb_content = "<model name='<%= model_name %>'></model>"
                # Missing :model_name in erb_args
                assert_raises(NameError) do
                    ERB.parse_erb_as_str(erb_content, {})
                end
            end

            it "save_as_sdf_model_wrong_extension" do
                file_to_save = <<~XML
                        <?xml version="1.0"?>
                        <model>
                            <name>"a_model"</name>
                            <version>1.0</version>
                            <sdf version="1.6">"a_sdf_file"</sdf>
                        </model>
                    XML
                assert_raises(ArgumentError) do
                    ERB.save_as_sdf_model(
                        file_to_save, "/tmp/sdf", "wrong_extendsion.wrong")
                end

                assert !File.file?("/tmp/sdf/wrong_extendsion.wrong")
            end

            describe "tests_that_save_to_test_folder" do
                before(:all) do
                    @test_folder = "/tmp/test_erb/"
                end

                after(:all) do
                    ::FileUtils.rm_rf(@test_folder)
                end

                it "save_as_sdf_model" do
                    file_to_save = <<~XML
                            <?xml version="1.0"?>
                            <model>
                                <name>"a_model"</name>
                                <version>1.0</version>
                                <sdf version="1.6">"a_sdf_file"</sdf>
                            </model>
                        XML
                    ERB.save_as_sdf_model(file_to_save, @test_folder)
                    saved_model_path = File.join(@test_folder, "model.sdf")

                    assert File.file?(saved_model_path)

                    file = File.read(saved_model_path)
                    assert_equal(file_to_save, file)

                    ERB.save_as_sdf_model(file_to_save, @test_folder, "a_model.sdf")
                    saved_model_path = File.join(@test_folder, "a_model.sdf")
                    assert File.file?(saved_model_path)

                    file = File.read(saved_model_path)
                    assert_equal(file_to_save, file)
                end

                it "save_as_sdf_model_overwrites_existing_file" do
                    file_name = "test_overwrite.sdf"
                    full_path = File.join(@test_folder, file_name)

                    # Ensure clean start
                    File.delete(full_path) if File.file?(full_path)

                    # Write initial content
                    ERB.save_as_sdf_model("<initial/>", @test_folder, file_name)
                    assert_equal "<initial/>", File.read(full_path)

                    # Write new content and verify it overwrites completely
                    ERB.save_as_sdf_model("<new_content/>", @test_folder, file_name)
                    assert_equal "<new_content/>", File.read(full_path)
                end

                it "imports the root model in the transformer" do
                    pose_gps2 = [2.571, 0.044, 0.808, 0, 0, 0]
                    erb_args = {
                        model_name: "simple test model",
                        gps_sensors: [
                            {
                                name: "gps",
                                pose: [-0.679, 0.0, 1.920, 0.0, 0.0, 0.0]
                            },
                            {
                                name: "gps2",
                                pose: pose_gps2
                            }
                        ]
                    }

                    flexmock(Roby.app).should_receive(:created_log_dir?).and_return(true)
                    flexmock(Roby.app).should_receive(:log_dir).and_return(@test_folder)
                    ::RockGazebo::Syskit::ERB.pre_render_gazebo_erb_model(
                        "model://simple_model_erb",
                        erb_args: erb_args,
                        output_file_name: "model.sdf",
                        output_folder_name: "simple_model_erb"
                    )

                    Conf.sdf.load_sdf(
                        expand_fixture_world("attached_simple_model_erb.world")
                    )

                    # Delegate to standard use_gazebo_model
                    @profile.use_gazebo_model(
                        "model://simple_model_erb",
                        as: "included_model",
                        use_world: false
                    )


                    tr = @profile.transformer
                    assert tr.frame?("attachment")
                    assert tr.has_frame?("included_model::gps")
                    assert tr.frame?("included_model::gps2")

                    gps2_root_tf = Eigen::Vector3.new(
                        pose_gps2[0], pose_gps2[1], pose_gps2[2])
                    assert_equal(
                        gps2_root_tf,
                        tr.transform_for("included_model::gps2", "included_model::root").translation
                    )
                    # Resolves root to gps2
                    transform = tr.resolve_static_chain(
                        "included_model::root", "included_model::gps2")
                    assert_equal(
                        -gps2_root_tf,
                        transform.translation
                    )

                    # Resolves attachment to gps2 (composes attachment -> included_model -> root -> gps2)
                    transform = tr.resolve_static_chain("attachment", "included_model::gps2")
                    assert_equal(
                        -(gps2_root_tf + Eigen::Vector3.UnitX),
                        transform.translation
                    )
                ensure
                    ::FileUtils.rm_f("/tmp/gazebo_erb_models/simple_model_erb")
                end

                it "creates a stable temp folder symlink pointing to active log dir when created_log_dir is true" do
                    output_folder_name = "symlink_test_model"

                    flexmock(Roby.app).should_receive(:created_log_dir?).and_return(true)
                    flexmock(Roby.app).should_receive(:log_dir).and_return(@test_folder)

                    expected_src = File.join(@test_folder, "sdf", output_folder_name)
                    expected_dest = File.join(::RockGazebo::Syskit::ERB::STABLE_TMP_DIR, output_folder_name)
                    flexmock(::FileUtils).should_receive(:ln_s).with(expected_src, expected_dest).once

                    erb_args = { model_name: "symlink_test_model", gps_sensors: [] }
                    ::RockGazebo::Syskit::ERB.pre_render_gazebo_erb_model(
                        "model://simple_model_erb",
                        erb_args: erb_args,
                        output_file_name: "model.sdf",
                        output_folder_name: "symlink_test_model"
                    )
                end

                it "pre_render_gazebo_erb_model twice in a row" do
                    pose_gps2 = [2.571, 0.044, 0.808, 0, 0, 0]
                    erb_args = {
                        model_name: "parse_erb_twice",
                        gps_sensors: [
                            {
                                name: "gps",
                                pose: [-0.679, 0.0, 1.920, 0.0, 0.0, 0.0]
                            },
                            {
                                name: "gps2",
                                pose: pose_gps2
                            }
                        ]
                    }

                    flexmock(Roby.app).should_receive(:created_log_dir?).and_return(true)
                    flexmock(Roby.app).should_receive(:log_dir).and_return(@test_folder)
                    ::RockGazebo::Syskit::ERB.pre_render_gazebo_erb_model(
                        "model://simple_model_erb",
                        erb_args: erb_args,
                        output_file_name: "model.sdf",
                        output_folder_name: "simple_model_erb"
                    )
                    saved_model_path = File.join(@test_folder, "sdf", "simple_model_erb", "model.sdf")
                    assert File.file?(saved_model_path)

                    ::RockGazebo::Syskit::ERB.pre_render_gazebo_erb_model(
                        "model://simple_model_erb",
                        erb_args: erb_args,
                        output_file_name: "model.sdf",
                        output_folder_name: "simple_model_erb"
                    )
                    assert File.file?(saved_model_path)
                end
            end
        end
    end
end