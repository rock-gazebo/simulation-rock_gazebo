# frozen_string_literal: true

require 'rock_gazebo/syskit/erb'
require 'rock_gazebo/syskit/test'

module RockGazebo
    module Syskit
        describe ERB do
            # before do
            #     Roby.app.using_task_library 'rock_gazebo'
            #     require 'models/orogen/rock_gazebo'
            #     @profile = ::Syskit::Actions::Profile.new
            #     # Conf.sdf = SDF.new
            # end

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
                                    <parent>body</parent>
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

            it "save_as_sdf_model" do
                file_to_save = <<~XML
                        <?xml version="1.0"?>
                        <model>
                            <name>"a_model"</name>
                            <version>1.0</version>
                            <sdf version="1.6">"a_sdf_file"</sdf>
                        </model>
                    XML
                ERB.save_as_sdf_model(file_to_save, "/tmp/sdf")

                assert File.file?("/tmp/sdf/model.sdf")

                file = File.read("/tmp/sdf/model.sdf")
                assert_equal(file_to_save, file)

                ERB.save_as_sdf_model(file_to_save, "/tmp/sdf", "a_model.sdf")

                assert File.file?("/tmp/sdf/a_model.sdf")

                file = File.read("/tmp/sdf/a_model.sdf")
                assert_equal(file_to_save, file)
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
                                    <parent>body</parent>
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
                                    <parent>body</parent>
                                    <child>gps</child>
                                </joint>

                                <link name="gps2">
                                    <pose>2.571 0.044 0.808 0 0 0</pose>
                                </link>
                                <joint name="gps2_attachment" type="fixed">
                                    <parent>body</parent>
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

            it "save_as_sdf_model_overwrites_existing_file" do
                destination_dir = "/tmp/sdf_idempotent"
                file_name = "test_overwrite.sdf"
                full_path = File.join(destination_dir, file_name)

                # Ensure clean start
                File.delete(full_path) if File.file?(full_path)

                # Write initial content
                ERB.save_as_sdf_model("<initial/>", destination_dir, file_name)
                assert_equal "<initial/>", File.read(full_path)

                # Write new content and verify it overwrites completely
                ERB.save_as_sdf_model("<new_content/>", destination_dir, file_name)
                assert_equal "<new_content/>", File.read(full_path)
            end
        end
    end
end