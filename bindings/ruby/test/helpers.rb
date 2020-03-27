# frozen_string_literal: true

require 'minitest/autorun'
require 'orocos/test/component'
require 'minitest/spec'
require 'rock/gazebo'

module Helpers
    def setup
        Orocos.initialize unless Orocos.initialized?
        @gazebo_output = Tempfile.open 'rock_gazebo'
        Rock::Gazebo.model_path = [File.expand_path('models', __dir__)]
        super
    end

    def teardown
        unless passed?
            @gazebo_output.rewind
            puts @gazebo_output.read
        end

        if @gazebo_pid
            begin
                Process.kill 'INT', @gazebo_pid
                Process.waitpid @gazebo_pid
            rescue Errno::ESRCH, Errno::ECHILD # rubocop:disable Lint/SuppressedException
            end
        end
        @gazebo_output.close
    end

    def expand_fixture_world(path)
        fixture_world_dir = File.expand_path('worlds', __dir__)
        File.expand_path(path, fixture_world_dir)
    end

    def expand_fixture_model(name)
        File.expand_path(File.join(name, 'model.sdf'),
                         File.join(__dir__, 'models'))
    end

    def gzserver(world_file, expected_task_name, timeout: 10)
        raise '@gazebo_output is nil, have you overriden #setup ?' unless @gazebo_output

        @gazebo_pid = Rock::Gazebo.spawn(
            'gzserver', expand_fixture_world(world_file), '--verbose',
            '--model-dir', File.join(__dir__, 'models'),
            out: @gazebo_output,
            err: @gazebo_output
        )

        deadline = Time.now + timeout
        loop do
            sleep 0.01
            begin return Orocos.get(expected_task_name)
            rescue Orocos::NotFound # rubocop:disable Lint/SuppressedException
            end
            begin
                if Process.waitpid(@gazebo_pid, Process::WNOHANG)
                    gazebo_flunk(
                        "gzserver terminated before '#{expected_task_name}' "\
                        'could be reached'
                    )
                end
            rescue Errno::ECHILD
                gazebo_flunk('gzserver failed to start')
            end

            break if Time.now > deadline
        end
        gazebo_flunk("failed to gazebo_reach task '#{expected_task_name}'" \
                     "within #{timeout} seconds, available tasks: "\
                     "#{Orocos.name_service.names.join(', ')}")
    end

    def gazebo_flunk(message)
        @gazebo_output.rewind
        puts @gazebo_output.read
        flunk(message)
    end

    def matrix3_rand
        values = (0...9).map { rand.abs }
        Types.base.Matrix3d.new(data: values)
    end

    def assert_matrix3_in_delta(expected, actual, delta = 1e-9)
        9.times do |i|
            assert_in_delta(
                expected.data[i], actual.data[i], delta,
                "element #{i} differs by more than #{delta}"
            )
        end
    end

    def poll_until(timeout: 10, period: 0.01, message: 'could not reach condition')
        start = Time.now
        while (Time.now - start) < timeout
            return if yield

            sleep(period)
        end
        flunk(message)
    end

    def configure_start_and_read_one_sample(port_name, &block)
        task.configure
        task.start
        assert_port_has_one_new_sample(port_name, 10, &block)
    end

    def assert_port_has_one_new_sample(port_name, timeout = 10, &block)
        reader = @task.port(port_name).reader
        assert_has_one_new_sample_matching(reader, timeout, &block)
    end

    def assert_has_one_new_sample_matching(reader, timeout = 10)
        if block_given?
            poll_until do
                sample = assert_has_one_new_sample(reader, timeout)
                return sample if yield(sample)
            end
        else
            assert_has_one_new_sample(reader, timeout)
        end
    end
end

class Minitest::Test # rubocop:disable Style/ClassAndModuleChildren
    include Helpers
end
