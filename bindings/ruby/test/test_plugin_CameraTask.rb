# frozen_string_literal: true

require 'test/helpers'

describe 'rock_gazebo::CameraTask' do
    include Orocos::Test::Component
    include Helpers

    Helpers.common_sensor_behavior(
        self, world_basename: 'camera',
              task_name: '/gazebo::w::m::l::c',
              port_name: 'frame',
              model_name: 'rock_gazebo::CameraTask'
    )

    def camera_configure_start_and_read_one_new_sample(
        world, port_name = 'frame', nested_models_prefix: ""
    )
        @task = gzserver world, "/gazebo::w::m::#{nested_models_prefix}l::c"
        yield(@task) if block_given?
        configure_start_and_read_one_new_sample(port_name)
    end

    it 'exports the camera samples' do
        camera_configure_start_and_read_one_new_sample 'camera.world', 'frame'
    end
end
