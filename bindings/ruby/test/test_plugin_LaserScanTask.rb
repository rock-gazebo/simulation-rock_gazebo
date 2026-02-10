# frozen_string_literal: true

require 'test/helpers'

describe 'rock_gazebo::LaserScanTask' do
    include Orocos::Test::Component
    include Helpers

    # Common behaviour for depth maps
    Helpers.common_sensor_behavior(
        self, world_basename: 'depth_map',
              task_name: '/gazebo::w::m::l::laser',
              port_name: 'depth_map_samples',
              model_name: 'rock_gazebo::LaserScanTask'
    )

    # Common behaviour for depth maps
    Helpers.common_sensor_behavior(
        self, world_basename: 'laser_scan',
              task_name: '/gazebo::w::m::l::laser',
              port_name: 'laser_scan_samples',
              model_name: 'rock_gazebo::LaserScanTask'
    )
end
