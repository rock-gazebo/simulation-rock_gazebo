# frozen_string_literal: true

require 'test/helpers'

describe 'rock_gazebo::GPSTask' do
    include Orocos::Test::Component
    include Helpers

    Helpers.common_sensor_behavior(
        self, world_basename: 'gps',
                task_name: '/gazebo:w:m:g',
                port_name: 'gps_solution',
                model_name: 'rock_gazebo::GPSTask'
    )

    def gps_configure_start_and_read_one_sample(world, port = 'gps_solution')
        @task = gzserver world, '/gazebo:w:m:g'
        yield(@task) if block_given?
        configure_start_and_read_one_new_sample(port)
    end

    it 'exports the solution' do
        sample = gps_configure_start_and_read_one_sample('gps.world')
        # Values in the SDF file are in degrees, convert to radians and
        # give a good precision (1e-9 is around 1mm)
        assert_in_epsilon(-22.9068, sample.latitude, 1e-9)
        assert_in_epsilon(-43.1729, sample.longitude, 1e-9)
        assert_equal :AUTONOMOUS, sample.positionType
        assert_in_delta 10, sample.altitude, 0.01
    end

    it 'properly resolves the topic in a SDF file that uses nested model' do
        sample = gps_configure_start_and_read_one_sample('gps-nested.world')
        # Values in the SDF file are in degrees, convert to radians and
        # give a good precision (1e-9 is around 1mm)
        assert_in_epsilon(-22.9068, sample.latitude, 1e-9)
        assert_in_epsilon(-43.1729, sample.longitude, 1e-9)
        assert_equal :AUTONOMOUS, sample.positionType
        assert_in_delta 10, sample.altitude, 0.01
    end

    it 'sets the time to the simulation\'s logical time' do
        sample = gps_configure_start_and_read_one_sample('gps.world')
        assert(sample.time.to_f > 0 && sample.time.to_f < 10)
    end

    it 'exports a 1m deviation in both vertical an horizontal by default' do
        sample = gps_configure_start_and_read_one_sample('gps.world')
        assert_equal 1, sample.deviationLatitude
        assert_equal 1, sample.deviationLongitude
        assert_equal 1, sample.deviationAltitude
    end

    it 'uses the GPS noise parameters to set the deviations if present' do
        sample = gps_configure_start_and_read_one_sample('gps-noise.world')
        assert_equal 3, sample.deviationLatitude
        assert_equal 3, sample.deviationLongitude
        assert_equal 2, sample.deviationAltitude
    end

    it 'exports the realtime instead of the logical time if use_sim_time is false' do
        sample = gps_configure_start_and_read_one_sample('gps.world') do |task|
            task.use_sim_time = false
        end
        diff_t = (Time.now - sample.time)
        assert(diff_t > 0 && diff_t < 10)
    end

    describe 'UTM conversion' do
        # For these tests, we move the model outside of the origin so that
        # we can check the projection parameters
        #
        # The resulting lat/long values are as follows:
        #    latitude   = -22.91582976364718
        #    longitude  = -43.18264805678904
        def gps_configure_start_and_read_one_sample(world_file, port_name)
            super do |task|
                task.use_proper_utm_conversion = true
                yield task if block_given?
            end
        end

        it 'converts the position to the configured UTM frame' do
            sample = gps_configure_start_and_read_one_sample(
                'gps-check-utm.world', 'utm_samples'
            ) do |task|
                task.gps_frame = 'gps_test'
                task.utm_frame = 'utm_test'
                task.nwu_origin = Eigen::Vector3.Zero
                task.utm_zone = 23
                task.utm_north = false
            end
            # The position samples are UTM N/E. Rock is in NWU, so we must convert the
            # UTM coordinates to NWU
            assert_equal 'gps_test', sample.sourceFrame
            assert_equal 'utm_test', sample.targetFrame
            assert_in_delta 687_394, sample.position.x, 1
            assert_in_delta 7_465_634, sample.position.y, 1
        end

        it 'converts the position to the configured NWU frame' do
            sample = gps_configure_start_and_read_one_sample(
                'gps-far-from-origin.world', 'position_samples'
            ) do |task|
                task.gps_frame = 'gps_test'
                task.nwu_frame = 'nwu_test'
                task.nwu_origin = Eigen::Vector3.new(7_465_634.13, 312_605.41)
                task.utm_zone = 23
                task.utm_north = false
            end
            # The position samples are UTM N/E. Rock is in NWU, so we must convert the
            # UTM coordinates to NWU
            assert_equal 'gps_test', sample.sourceFrame
            assert_equal 'nwu_test', sample.targetFrame
            assert_in_delta 1012, sample.position.x, 1
            assert_in_delta 988, sample.position.y, 1
        end

        it 'exports the realtime instead of the logical time if '\
            'use_sim_time is false' do
            sample = gps_configure_start_and_read_one_sample(
                'gps.world', 'position_samples'
            ) do |task|
                task.use_sim_time = false
            end
            diff_t = (Time.now - sample.time)
            assert(diff_t > 0 && diff_t < 10)
        end

        it 'exports the realtime instead of the logical time '\
            'if use_sim_time is false' do
            sample = gps_configure_start_and_read_one_sample(
                'gps.world', 'utm_samples'
            ) do |task|
                task.use_sim_time = false
            end
            diff_t = (Time.now - sample.time)
            assert(diff_t > 0 && diff_t < 10)
        end
    end

    describe 'Gazebo spherical coordinate conversion' do
        # For these tests, we move the model outside of the origin so that
        # we can check the projection parameters
        #
        # The resulting lat/long values are as follows:
        #    latitude   = -22.91582976364718
        #    longitude  = -43.18264805678904
        def gps_configure_start_and_read_one_sample(world_file, port_name)
            super do |task|
                task.gps_frame = 'gps_test'
                task.utm_frame = 'utm_test'
                task.nwu_frame = 'nwu_test'
                task.use_proper_utm_conversion = false
                task.latitude_origin =
                    Types.base.Angle.new(rad: -22.9068 * Math::PI / 180)
                task.longitude_origin =
                    Types.base.Angle.new(rad: -43.1729 * Math::PI / 180)
                yield task if block_given?
            end
        end

        it 'converts the position to the configured local frame' do
            sample = gps_configure_start_and_read_one_sample(
                'gps-far-from-origin.world', 'utm_samples'
            )
            # The position samples are UTM N/E. Rock is in NWU, so we must convert the
            # UTM coordinates to NWU
            assert_equal 'gps_test', sample.sourceFrame
            assert_equal 'utm_test', sample.targetFrame
            assert_in_delta(-1000, sample.position.x, 1)
            assert_in_delta 1000, sample.position.y, 1
        end

        it 'converts the position to the local frame' do
            sample = gps_configure_start_and_read_one_sample(
                'gps-far-from-origin.world', 'position_samples'
            )
            # The position samples are UTM N/E. Rock is in NWU, so we must convert the
            # UTM coordinates to NWU
            assert_equal 'gps_test', sample.sourceFrame
            assert_equal 'nwu_test', sample.targetFrame
            assert_in_delta 1000, sample.position.x, 1
            assert_in_delta 1000, sample.position.y, 1
        end

        it 'exports the realtime instead of the logical time if '\
            'use_sim_time is false' do
            sample = gps_configure_start_and_read_one_sample(
                'gps.world', 'position_samples'
            ) do |task|
                task.use_sim_time = false
            end
            diff_t = (Time.now - sample.time)
            assert(diff_t > 0 && diff_t < 10)
        end

        it 'exports the realtime instead of the logical time if '\
            'use_sim_time is false' do
            sample = gps_configure_start_and_read_one_sample(
                'gps.world', 'utm_samples'
            ) do |task|
                task.use_sim_time = false
            end
            diff_t = (Time.now - sample.time)
            assert(diff_t > 0 && diff_t < 10)
        end
    end
end
