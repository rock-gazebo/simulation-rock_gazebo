module RockGazebo
    module Syskit
        class << self
            # Feature flag that corrects how the library generates sensor device names
            #
            # When recursive models are used, the library's old behaviour is to
            # define sensor devices with the root model name and the sensor name,
            # that is
            #
            #     use_gazebo_model model://model_in_model
            #
            # will define 'root_gps_sensor' if the XML hierarchy is
            # root/included_model/link/gps. It will also define
            # root_included_model_gps_sensor along the way
            #
            # This rather obviously starts failing when one wants to include
            # the same sub-model more than once, or have two sensors of the
            # same type in the model (the latter could be worked around by
            # changing the sensor name though)
            #
            # This historical behaviour is retained when the flag is false (
            # this is the default). The new behaviour is to properly scope
            # sensors by the full path in the XML. In the example above,
            # the sensor device is root_included_model_link_gps_sensor_dev
            attr_accessor :scope_device_name_with_links_and_submodels

            # Feature flag that toggles the automatic definition of links when
            # loading a gazebo model.
            #
            # This starts failing when one uses more than two levels of submodels as it
            # quickly gets too complex to manage the automatically generated links names
            attr_accessor :use_gazebo_automatic_links

            # Feature flag that control use_gazebo_world and use_gazebo_model's
            # prefix_device_with_name flag.
            #
            # The historical default was false, but you really should switch it to true.
            # First migrate little by little each call and then turn it on globally
            # by setting this flag.
            attr_accessor :prefix_device_with_name

            # Feature flag that controls whether use_gazebo_model calls use_gazebo_world
            #
            # The historical default was true, but will become false. If switching the
            # flag to false globally does not work, a more gradual migration can be
            # done by passing `use_world: false` to the use_gazebo_model call
            attr_accessor :use_gazebo_model_calls_use_gazebo_world
        end

        @scope_device_name_with_links_and_submodels = false
        @use_gazebo_automatic_links = true
        @prefix_device_with_name = false
        @use_gazebo_model_calls_use_gazebo_world = true
    end
end
