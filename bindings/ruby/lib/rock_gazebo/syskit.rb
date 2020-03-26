# frozen_string_literal: true

require 'rock_gazebo/syskit/root_logger'

require 'rock/gazebo'
require 'rock_gazebo/orogen_model_from_sdf_world'
require 'sdf'
require 'orocos'
require 'syskit'
require 'models/devices/gazebo/model'
require 'models/devices/gazebo/root_model'
require 'models/devices/gazebo/link'
require 'models/devices/gazebo/joint'
require 'transformer/syskit'
Transformer::SyskitPlugin.enable
require 'transformer/sdf'
require 'rock_gazebo/syskit/sdf'
require 'rock_gazebo/syskit/configuration_extension'
require 'rock_gazebo/syskit/profile_extension'
require 'rock_gazebo/syskit/robot_definition_extension'
require 'rock_gazebo/syskit/instance_requirements_extension'
require 'rock_gazebo/syskit/master_device_instance_extension'
