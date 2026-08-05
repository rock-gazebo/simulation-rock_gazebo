# frozen_string_literal: true

require "rock_gazebo/syskit/erb"
require "rock_gazebo/syskit/test"
require_relative "helpers"

module RockGazebo
    module Syskit
        describe ERB do
            include Helpers

            before do
                Roby.app.using_task_library "rock_gazebo"
                require "models/orogen/rock_gazebo"
                @profile = ::Syskit::Actions::Profile.new
                Conf.sdf = SDF.new
            end

            it "read_erb_file" do
                template_path = File.expand_path("../models/simple_model_erb/model.sdf.erb", __dir__)

                sdf_str = ERB.read_erb_file(template_path)

                assert sdf_str

                expected_sdf_str = <<~XML
                    <?xml version="1.0" ?>
                    <%
                       	default_gps_pose = [-0.679, 0.0, 1.920, 0.0, 0.0, 0.0]
                       	default_gps2_pose = [2.571, 0.044, 0.808, 0.0, 0.0, 0.0]

                        gps1_pose = (defined?(links) && links.find { |link| link[:name] == "gps" }&.dig(:pose)) || default_gps_pose
                        gps2_pose = (defined?(links) && links.find { |link| link[:name] == "gps2" }&.dig(:pose)) || default_gps2_pose
                    %>
                    <sdf version="1.6">
                        <model name="simple_model_erb">
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
                                <pose><%= gps1_pose.join(' ') %></pose>
                            </link>
                            <joint name="gps_attachment" type="fixed">
                                <parent>root</parent>
                                <child>gps</child>
                            </joint>

                            <link name="gps2">
                                <pose><%= gps2_pose.join(' ') %></pose>
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

                formatted_str = sdf_str.gsub(/\s+/, " ").strip
                formatted_expected = expected_sdf_str.gsub(/\s+/, " ").strip

                assert_equal(formatted_expected, formatted_str)
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
                resulting_sdf = ERB.parse_erb_as_str(erb_content, **erb_args)

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

                formatted_erb = resulting_sdf.gsub(/\s+/, " ").strip
                formatted_expected = expected_content.gsub(/\s+/, " ").strip

                assert_equal(formatted_expected, formatted_erb)
            end

            it "parse_erb_as_str_with_extra_unused_args" do
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
                    ],
                    random_key: "random_value",
                    random_array: [1, 2, 3],
                    random_hash: { key1: "value1", key2: "value2" }
                }
                resulting_sdf = ERB.parse_erb_as_str(erb_content, **erb_args)

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

                formatted_erb = resulting_sdf.gsub(/\s+/, " ").strip
                formatted_expected = expected_content.gsub(/\s+/, " ").strip

                assert_equal(formatted_expected, formatted_erb)
            end

            it "parse_erb_as_str_raises_on_missing_args" do
                erb_content = "<model name='<%= model_name %>'></model>"
                # Missing :model_name in erb_args
                assert_raises(NameError) do
                    ERB.parse_erb_as_str(erb_content)
                end
            end

            it "pre_render_erb_sdf_model with args different from default" do
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
                rendered_content = ::RockGazebo::Syskit::ERB.pre_render_erb_sdf_model(
                    "model://simple_model_erb",
                    erb_args: erb_args
                )

                assert_equal "simple_model_erb", REXML::XPath.first(rendered_content, "//model").attributes["name"]
                poses = REXML::XPath.match(rendered_content, "//pose").map(&:text)
                assert_includes poses, "0.0 1.0 2.0 3.0 4.0 5.0"
                assert_includes poses, "6.0 7.0 8.0 9.0 0.0 1.0"
            end

            it "pre_render_erb_sdf_model with no args" do
                erb_args = {}

                rendered_content = ::RockGazebo::Syskit::ERB.pre_render_erb_sdf_model(
                    "model://simple_model_erb",
                    erb_args: erb_args
                )

                assert_equal "simple_model_erb", REXML::XPath.first(rendered_content, "//model").attributes["name"]
                poses = REXML::XPath.match(rendered_content, "//pose").map(&:text)
                assert_includes poses, "-0.679 0.0 1.92 0.0 0.0 0.0"
                assert_includes poses, "2.571 0.044 0.808 0.0 0.0 0.0"
            end
        end
    end
end
