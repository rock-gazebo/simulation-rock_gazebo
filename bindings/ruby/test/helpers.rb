# frozen_string_literal: true

require 'minitest/autorun'
require 'orocos/test/component'
require 'minitest/spec'
require 'rock/gazebo'
require 'sdf'

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

    def poll_until(
        timeout: 10, period: 0.01,
        flunk: true, message: 'could not reach condition'
    )
        deadline = Time.now + timeout
        while Time.now < deadline
            return if yield

            sleep(period)
        end
        flunk(message) if flunk
    end

    def create_active_reader(port_name, **policy)
        @task.configure
        reader = @task.port(port_name).reader(**policy)
        @task.start
        reader
    end

    def configure_start_and_read_one_new_sample(port_name)
        reader = create_active_reader(port_name)
        assert_has_one_new_sample(reader)
    ensure
        reader&.disconnect
    end

    def configure_start_read_samples_and_stop(port_name, duration, buffer_size: 100)
        reader = create_active_reader(port_name, type: :buffer, size: buffer_size)
        sleep duration
        @task.stop
        samples = []
        while (s = reader.read_new)
            samples << s
        end
        samples
    end

    def configure_start_count_samples_and_stop(port_name, duration, buffer_size: 100)
        reader = create_active_reader(port_name, type: :buffer, size: buffer_size)
        sleep 1
        reader.clear
        sleep duration
        @task.stop
        count = 0
        sample = reader.new_sample
        count += 1 while reader.raw_read_new(sample)
        count
    end

    def configure_start_and_read_one_new_sample_matching(port_name, timeout = 10, &block)
        reader = create_active_reader(port_name)
        assert_has_one_new_sample_matching(reader, timeout, &block)
    ensure
        reader&.disconnect
    end

    def assert_port_has_one_new_sample_matching(port_name, timeout = 10, &block)
        reader = @task.port(port_name).reader
        assert_has_one_new_sample_matching(reader, timeout, &block)
    ensure
        reader&.disconnect
    end

    def assert_has_one_new_sample_matching(reader, timeout = 10)
        return assert_has_one_new_sample(reader, timeout) unless block_given?

        rejected_samples = []
        poll_until(timeout: timeout, flunk: false) do
            if (sample = reader.read_new)
                return sample if yield(sample)

                rejected_samples << sample
                nil
            end
        end

        formatted_samples = rejected_samples.map do |s|
            PP.pp(s, ''.dup)
        end.join("\n")

        if rejected_samples.empty?
            flunk("no sample arrived in #{timeout}s")
        else
            flunk("got #{rejected_samples.size} samples but none matched "\
                    "what was expected: #{formatted_samples}")
        end
    end

    def self.common_sensor_behavior(
        ctx, world_basename:, task_name:, port_name:, model_name:,
             skip_one_update_per_cycle_test: false
    )
        ctx.send(:describe, "common sensor behavior for #{model_name}") do
            it "exports the sensor using #{model_name}" do
                task = gzserver "#{world_basename}.world", task_name
                assert_equal model_name, task.model.name
            end

            it 'does not output anything while the component is only configured' do
                @task = gzserver "#{world_basename}.world", task_name
                reader = @task.port(port_name).reader
                @task.configure
                sleep 0.5
                refute reader.read_new
            end

            it 'does not output anything after the component is stopped' do
                @task = gzserver "#{world_basename}.world", task_name
                @task.configure
                @task.start
                @task.stop
                reader = @task.port(port_name).reader
                sleep 0.5
                refute reader.read_new
            end

            unless skip_one_update_per_cycle_test
                it 'outputs one sample per cycle by default' do
                    @task = gzserver "#{world_basename}.world", task_name
                    count = configure_start_count_samples_and_stop(port_name, 1)
                    assert_includes 9..11, count
                end
            end

            it 'honors the rate set in the SDF file' do
                @task = gzserver "#{world_basename}-custom-rate.world", task_name
                count = configure_start_count_samples_and_stop(port_name, 1)
                assert_includes 4..6, count
            end
        end
    end
end
