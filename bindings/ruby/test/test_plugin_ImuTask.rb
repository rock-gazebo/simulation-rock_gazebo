# frozen_string_literal: true

require 'test/helpers'

describe 'rock_gazebo::ImuTask' do
    include Orocos::Test::Component
    include Helpers

    Helpers.common_sensor_behavior(
        self, world_basename: 'imu',
              task_name: '/gazebo:w:m:i',
              port_name: 'orientation_samples',
              model_name: 'rock_gazebo::ImuTask'
    )

    def imu_configure_start_and_read_one_new_sample(world, port_name = 'orientation_samples')
        @task = gzserver world, '/gazebo:w:m:i'
        yield(@task) if block_given?
        configure_start_and_read_one_new_sample(port_name)
    end

    it 'exports the raw samples' do
        sample = imu_configure_start_and_read_one_new_sample 'imu.world', 'imu_samples'
        assert(sample.time.to_f > 0 && sample.time.to_f < 10)
        assert(sample.acc.norm < 1e-6)
        assert(sample.gyro.norm < 1e-6)
        assert(sample.mag.norm < 1e-6)
    end

    it 'properly resolves the topic in a SDF file that uses nested model' do
        sample = imu_configure_start_and_read_one_new_sample(
            'imu-nested.world', 'imu_samples'
        )
        assert(sample.time.to_f > 0 && sample.time.to_f < 10)
        assert(sample.acc.norm < 1e-6)
        assert(sample.gyro.norm < 1e-6)
        assert(sample.mag.norm < 1e-6)
    end

    it 'stamps raw samples using the realtime instead of the logical time '\
        'if use_sim_time is false' do
        sample = imu_configure_start_and_read_one_new_sample 'imu.world' do |task|
            task.use_sim_time = false
        end
        diff_t = (Time.now - sample.time)
        assert(diff_t > 0 && diff_t < 10)
    end

    it 'provides an orientation-to-starting pose by default' do
        sample = imu_configure_start_and_read_one_new_sample 'imu-not-aligned.world'
        assert sample.orientation.approx?(Eigen::Quaternion.Identity)
    end

    it 'provides an orientation-to-horizontal if '\
        'reference == REFERENCE_HORIZONTAL_PLANE' do
        sample = imu_configure_start_and_read_one_new_sample 'imu-not-aligned.world' do |task|
            task.reference = :REFERENCE_HORIZONTAL_PLANE
        end
        expected = Eigen::Quaternion.from_euler(
            Eigen::Vector3.new(0, 0.1, 0.1), 2, 1, 0
        )
        assert sample.orientation.approx?(expected)
    end

    it 'provides an orientation-to-absolute if reference == REFERENCE_ABSOLUTE' do
        sample = imu_configure_start_and_read_one_new_sample 'imu-not-aligned.world' do |task|
            task.reference = :REFERENCE_ABSOLUTE
        end
        expected = Eigen::Quaternion.from_euler(
            Eigen::Vector3.new(0.1, 0.1, 0.1), 2, 1, 0
        )
        assert sample.orientation.approx?(expected)
    end
end
