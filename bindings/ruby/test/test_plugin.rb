# frozen_string_literal: true

require 'test/helpers'

describe 'The Rock/Gazebo plugin' do
    include Orocos::Test::Component
    include Helpers

    describe 'Models' do
        attr_reader :task

        describe 'creation by the plugin' do
            before do
                @task = gzserver 'model.world', '/gazebo:w:m'
            end

            it 'exports the model using a ModelTask' do
                assert_equal 'rock_gazebo::ModelTask', task.model.name
            end
        end

        describe 'the pose samples' do
            before do
                @task = gzserver 'model.world', '/gazebo:w:m'
            end

            it "exports the model's pose" do
                pose = configure_start_and_read_one_sample 'pose_samples'
                assert Eigen::Vector3.new(1, 2, 3).approx?(pose.position)
                assert Eigen::Quaternion.from_angle_axis(0.1, Eigen::Vector3.UnitZ)
                                        .approx?(pose.orientation)
            end

            it 'sets the model\'s cov_position from the component\'s '\
               'cov_position property' do
                cov = matrix3_rand
                task.cov_position = cov
                pose = configure_start_and_read_one_sample 'pose_samples'
                assert_matrix3_in_delta cov, pose.cov_position
            end

            it 'sets the model\'s cov_orientation from the component\'s '\
               'cov_orientation property' do
                cov = matrix3_rand
                task.cov_orientation = cov
                pose = configure_start_and_read_one_sample 'pose_samples'
                assert_matrix3_in_delta cov, pose.cov_orientation
            end

            it 'sets the model\'s cov_velocity from the component\'s '\
               'cov_velocity property' do
                cov = matrix3_rand
                task.cov_velocity = cov
                pose = configure_start_and_read_one_sample 'pose_samples'
                assert_matrix3_in_delta cov, pose.cov_velocity
            end
        end

        describe "the model's joints" do
            before do
                @task = gzserver 'joints_export.world', '/gazebo:w:m'
            end

            it 'exports the state of all the non-fixed joints' do
                joints = configure_start_and_read_one_sample 'joints_samples'
                assert_equal ['m::j_00', 'm::j_01', 'm::child::j_00', 'm::child::j_01'],
                             joints.names
                assert_includes (-0.1..0.11), joints.elements[0].position
                assert_includes (0.19..0.31), joints.elements[1].position
                assert_includes (0.39..0.51), joints.elements[2].position
                assert_includes (0.59..0.71), joints.elements[3].position
            end

            it 'allows commanding the joints' do
                cmd = Types.base.samples.Joints.new(
                    elements: [Types.base.JointState.Effort(-1),
                               Types.base.JointState.Effort(1),
                               Types.base.JointState.Effort(-1),
                               Types.base.JointState.Effort(1)]
                )

                task.configure
                task.start
                reader = task.port('joints_samples').reader
                writer = task.port('joints_cmd').writer
                poll_until do
                    writer.write(cmd)
                    if (joints = reader.read_new)
                        (-0.01..0.01).include?(joints.elements[0].position) &&
                            (0.29..0.31).include?(joints.elements[1].position) &&
                            (0.39..0.41).include?(joints.elements[2].position) &&
                            (0.69..0.71).include?(joints.elements[3].position)
                    end
                end
            end
        end

        describe 'the joints export' do
            before do
                @task = gzserver 'joints_export.world', '/gazebo:w:m'
            end

            it 'exports a set of joints in the specified order' do
                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: '',
                    joints: ['m::child::j_00', 'm::j_01']
                )]
                joints = configure_start_and_read_one_sample 'test_samples'
                assert_equal ['m::child::j_00', 'm::j_01'], joints.names
                assert_includes (0.39..0.51), joints.elements[0].position
                assert_includes (0.19..0.31), joints.elements[1].position
            end

            it 'fails if the joint is given relative to the model' do
                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: '', joints: ['child::j_00']
                )]
                assert_raises(Orocos::StateTransitionFailed) do
                    task.configure
                end
            end

            it 'gives command access to a subset of the joints' do
                cmd = Types.base.samples.Joints.new(
                    elements: [Types.base.JointState.Effort(-1),
                               Types.base.JointState.Effort(1)]
                )

                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: '',
                    joints: ['m::child::j_00', 'm::j_01']
                )]

                task.configure
                task.start
                reader = task.port('test_samples').reader
                writer = task.port('test_cmd').writer
                poll_until do
                    writer.write(cmd)
                    if (joints = reader.read_new)
                        (0.39..0.41).include?(joints.elements[0].position) &&
                            (0.29..0.31).include?(joints.elements[1].position)
                    end
                end
            end

            describe 'the prefix settting' do
                it 'raises if a joint name does not start with the requested prefix' do
                    task.exported_joints = [Types.rock_gazebo.JointExport.new(
                        port_name: 'test', prefix: 'm::child::',
                        joints: ['m::child::j_00', 'm::j_01']
                    )]
                    assert_raises(Orocos::StateTransitionFailed) do
                        task.configure
                    end
                end
                it 'allows to remove a scope prefix from the joint names' do
                    task.exported_joints = [Types.rock_gazebo.JointExport.new(
                        port_name: 'test', prefix: 'm::',
                        joints: ['m::child::j_00', 'm::j_01']
                    )]
                    joints = configure_start_and_read_one_sample 'test_samples'
                    assert_equal ['child::j_00', 'j_01'], joints.names
                    assert_includes (0.39..0.51), joints.elements[0].position
                    assert_includes (0.19..0.31), joints.elements[1].position
                end
            end

            it 'removes the ports on cleanup' do
                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: '',
                    joints: ['m::child::j_00', 'm::j_01']
                )]
                task.configure
                task.cleanup
                refute task.has_port?('test_cmd')
                refute task.has_port?('test_samples')
            end

            it 'fails to configure if a joint does not exist' do
                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: '',
                    joints: ['does_not_exist', 'm::j_01']
                )]
                assert_raises(Orocos::StateTransitionFailed) do
                    task.configure
                end
            end
        end

        describe 'the link export' do
            describe 'RBS export' do
                describe 'general properties' do
                    before do
                        @task = gzserver 'model.world', '/gazebo:w:m'
                    end

                    it 'uses the link names as frames by default' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            port_period: Time.at(0)
                        )]
                        pose = configure_start_and_read_one_sample 'test'
                        assert_equal 'l', pose.sourceFrame
                        assert_equal 'root', pose.targetFrame
                    end

                    it 'allows to override the frame names' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            source_frame: 'src', target_frame: 'target',
                            port_period: Time.at(0)
                        )]
                        pose = configure_start_and_read_one_sample 'test'
                        assert_equal 'src', pose.sourceFrame
                        assert_equal 'target', pose.targetFrame
                    end

                    it 'sets cov_position from the provided cov_position' do
                        cov = matrix3_rand
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            cov_position: cov, port_period: Time.at(0)
                        )]
                        pose = configure_start_and_read_one_sample 'test'
                        assert_matrix3_in_delta cov, pose.cov_position, 1e-6
                    end

                    it 'sets cov_orientation from the provided cov_orientation' do
                        cov = matrix3_rand
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            cov_orientation: cov, port_period: Time.at(0)
                        )]
                        pose = configure_start_and_read_one_sample 'test'
                        assert_matrix3_in_delta cov, pose.cov_orientation, 1e-6
                    end

                    it 'sets cov_velocity from the provided cov_velocity' do
                        cov = matrix3_rand
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            cov_velocity: cov, port_period: Time.at(0)
                        )]
                        pose = configure_start_and_read_one_sample 'test'
                        assert_matrix3_in_delta cov, pose.cov_velocity, 1e-6
                    end
                end

                describe 'pose' do
                    before do
                        @task = gzserver 'model.world', '/gazebo:w:m'
                    end

                    it 'exports a link\'s pose' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            port_period: Time.at(0)
                        )]
                        link_pose = configure_start_and_read_one_sample 'test'
                        assert Eigen::Vector3.new(2, 3, 4).approx?(link_pose.position)
                        assert(
                            Eigen::Quaternion
                            .from_angle_axis(0.2, Eigen::Vector3.UnitZ)
                            .approx?(link_pose.orientation)
                        )
                    end
                end

                describe 'velocity' do
                    before do
                        @task = gzserver 'freefall.world', '/gazebo:w:m'
                    end

                    it 'exports a link\'s linear velocity' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            port_period: Time.at(0)
                        )]
                        rbs = configure_start_and_read_one_sample 'test' do |sample|
                            sample.velocity.z < -1
                        end
                        assert_in_delta 0, rbs.velocity.x, 1e-3
                        assert_in_delta 0, rbs.velocity.y, 1e-3
                    end

                    it 'exports the velocity in the target link\'s frame' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test',
                            source_link: 'root', target_link: 'rotated_link',
                            port_period: Time.at(0)
                        )]
                        rbs = configure_start_and_read_one_sample 'test' do |sample|
                            sample.velocity.x > 1
                        end
                        assert_in_delta 0, rbs.velocity.y, 1e-3
                        assert_in_delta 0, rbs.velocity.z, 1e-3
                    end

                    it 'exports a link\'s angular velocity' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root',
                            port_period: Time.at(0)
                        )]
                        wrench = Types.base.samples.Wrench.new(
                            time: Time.now,
                            force: Eigen::Vector3.Zero,
                            torque: Eigen::Vector3.new(1, 0, 0)
                        )
                        @task.configure
                        @task.start
                        writer = @task.test_wrench.writer
                        rbs = assert_port_has_one_new_sample 'test' do |sample|
                            writer.write(wrench)
                            sample.angular_velocity.x > 1
                        end
                        assert_in_delta 0, rbs.angular_velocity.y, 1e-3
                        assert_in_delta 0, rbs.angular_velocity.z, 1e-3
                    end

                    it 'exports the angular velocity always in the source '\
                       'links\'s frame' do
                        task.exported_links = [Types.rock_gazebo.LinkExport.new(
                            port_name: 'test',
                            source_link: 'root', target_link: 'rotated_link',
                            port_period: Time.at(0)
                        )]
                        wrench = Types.base.samples.Wrench.new(
                            time: Time.now,
                            force: Eigen::Vector3.Zero,
                            torque: Eigen::Vector3.new(1, 0, 0)
                        )
                        @task.configure
                        @task.start
                        writer = @task.test_wrench.writer
                        rbs = assert_port_has_one_new_sample 'test' do |sample|
                            writer.write(wrench)
                            sample.angular_velocity.x > 1
                        end
                        assert_in_delta 0, rbs.angular_velocity.y, 1e-3
                        assert_in_delta 0, rbs.angular_velocity.z, 1e-3
                    end
                end
            end

            describe 'acceleration export' do
                before do
                    @task = gzserver 'freefall.world', '/gazebo:w:m'
                end

                it 'exports a link\'s linear acceleration' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    link_acceleration =
                        configure_start_and_read_one_sample 'test_acceleration'
                    assert_in_delta 9.8, link_acceleration.acceleration.norm, 1e-3
                end

                it 'exports the linear acceleration expressed in the source frame' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test',
                        source_link: 'root', target_link: 'rotated_link',
                        port_period: Time.at(0)
                    )]
                    link_acceleration =
                        configure_start_and_read_one_sample 'test_acceleration'
                    assert_in_delta(
                        -9.8, link_acceleration.acceleration.z, 1e-3
                    )
                end

                it 'exports a link\'s angular acceleration' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'root', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    wrench = Types.base.samples.Wrench.new(
                        time: Time.now,
                        force: Eigen::Vector3.Zero,
                        torque: Eigen::Vector3.new(1, 0, 0)
                    )

                    task.configure
                    task.start
                    writer = task.test_wrench.writer

                    acc = assert_port_has_one_new_sample 'test_acceleration' do |sample|
                        writer.write(wrench)
                        sample.angular_acceleration.norm > 0.1
                    end
                    assert_in_delta 0.5, acc.angular_acceleration.x, 1e-3
                    assert_in_delta 0, acc.angular_acceleration.y, 1e-3
                    assert_in_delta 0, acc.angular_acceleration.z, 1e-3
                end

                it 'exports the angular acceleration expressed in the source frame' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test',
                        source_link: 'root', target_link: 'rotated_link',
                        port_period: Time.at(0)
                    )]
                    wrench = Types.base.samples.Wrench.new(
                        time: Time.now,
                        force: Eigen::Vector3.Zero,
                        torque: Eigen::Vector3.new(1, 0, 0)
                    )

                    task.configure
                    task.start
                    writer = task.test_wrench.writer

                    acc = assert_port_has_one_new_sample 'test_acceleration' do |sample|
                        writer.write(wrench)
                        sample.angular_acceleration.norm > 0.1
                    end
                    assert_in_delta 0.5, acc.angular_acceleration.x, 1e-3
                    assert_in_delta 0, acc.angular_acceleration.y, 1e-3
                    assert_in_delta 0, acc.angular_acceleration.z, 1e-3
                end
            end

            describe 'wrench export' do
                before do
                    @task = gzserver 'wrench.world', '/gazebo:w:m'
                    Orocos.load_typekit 'base'
                end

                it 'allows to send a force command to its source link' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'leaf', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    task.configure
                    task.start
                    wrench_w = task.port('test_wrench').writer
                    joints_r = task.port('joints_samples').reader

                    wrench = Types.base.samples.Wrench.new(
                        time: Time.now,
                        force: Eigen::Vector3.new(0, 1, 0),
                        torque: Eigen::Vector3.new(0, 0, 0)
                    )

                    100.times do
                        wrench_w.write(wrench)
                        sleep 0.01
                    end

                    joints = assert_has_one_new_sample joints_r
                    root2middle = joints.elements[0].position
                    assert root2middle > 0.01,
                           'expected the root2middle joint position to be greater '\
                           "than 0.01 radians, but it is #{root2middle}"
                    middle2leaf = joints.elements[1].position
                    assert middle2leaf < -0.01,
                           'expected the middle2leaf joint position to be lower '\
                           "than -0.01 radians, but it is #{middle2leaf}"
                end

                it 'allows to send a torque command to its source link' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'middle', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    task.configure
                    task.start
                    wrench_w = task.port('test_wrench').writer
                    joints_r = task.port('joints_samples').reader

                    wrench = Types.base.samples.Wrench.new(
                        time: Time.now,
                        force: Eigen::Vector3.new(0, 0, 0),
                        torque: Eigen::Vector3.new(0, 0, 1)
                    )

                    100.times do
                        wrench_w.write(wrench)
                        sleep 0.01
                    end

                    joints = assert_has_one_new_sample joints_r
                    root2middle = joints.elements[0].position
                    assert root2middle > 0.01,
                           'expected the root2middle joint position to be greater '\
                           "than 0.01 radians, but it is #{root2middle}"
                end
            end

            describe 'common features' do
                before do
                    @task = gzserver 'model.world', '/gazebo:w:m'
                    Orocos.load_typekit 'base'
                end

                it 'knows how to export a link in a nested model' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'nested::l', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    task.configure
                    task.start

                    link_pose = assert_has_one_new_sample(task.test.reader)
                    assert Eigen::Vector3.new(3, 2, 1).approx?(link_pose.position)
                    assert Eigen::Quaternion.from_angle_axis(-0.2, Eigen::Vector3.UnitZ)
                                            .approx?(link_pose.orientation)
                end

                it 'the pose\'s update period is controlled by the '\
                   'port_period parameter' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0.1)
                    )]
                    task.configure
                    task.start
                    reader = task.test.reader
                    first_pose = assert_has_one_new_sample(reader)
                    second_pose = assert_has_one_new_sample(reader)
                    assert_in_delta(second_pose.time - first_pose.time, 0.1, 0.025)
                end

                it 'refuses to configure if the source link does not exist' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test',
                        source_link: 'does_not_exist', target_link: 'root'
                    )]
                    assert_raises(Orocos::StateTransitionFailed) do
                        task.configure
                    end
                end

                it 'refuses to configure if the target link does not exist' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test',
                        source_link: 'l', target_link: 'does_not_exist'
                    )]
                    assert_raises(Orocos::StateTransitionFailed) do
                        task.configure
                    end
                end

                it 'refuses to configure if the port is already in use' do
                    task.exported_links = [
                        Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root'
                        ),
                        Types.rock_gazebo.LinkExport.new(
                            port_name: 'test', source_link: 'l', target_link: 'root'
                        )
                    ]
                    assert_raises(Orocos::StateTransitionFailed) do
                        task.configure
                    end
                end
            end
        end
    end

    describe 'IMU sensor' do
        def read_orientation_sample(world_file)
            @task = gzserver world_file, '/gazebo:w:m:i'
            yield(@task) if block_given?
            reader = @task.orientation_samples.reader
            @task.configure
            @task.start
            assert_has_one_new_sample(reader)
        ensure
            reader&.disconnect
        end

        def read_imu_sample(world_file)
            @task = gzserver world_file, '/gazebo:w:m:i'
            yield(@task) if block_given?
            reader = @task.imu_samples.reader
            @task.configure
            @task.start
            assert_has_one_new_sample(reader)
        ensure
            reader&.disconnect
        end

        it 'exports the sensor using an ImuTask' do
            task = gzserver 'imu.world', '/gazebo:w:m:i'
            assert_equal 'rock_gazebo::ImuTask', task.model.name
        end

        it 'exports the raw samples' do
            sample = read_imu_sample 'imu.world'
            assert(sample.time.to_f > 0 && sample.time.to_f < 10)
            assert(sample.acc.norm < 1e-6)
            assert(sample.gyro.norm < 1e-6)
            assert(sample.mag.norm < 1e-6)
        end

        it 'properly resolves the topic in a SDF file that uses nested model' do
            sample = read_imu_sample 'imu-nested.world'
            assert(sample.time.to_f > 0 && sample.time.to_f < 10)
            assert(sample.acc.norm < 1e-6)
            assert(sample.gyro.norm < 1e-6)
            assert(sample.mag.norm < 1e-6)
        end

        it 'stamps raw samples using the realtime instead of the logical time '\
           'if use_sim_time is false' do
            sample = read_imu_sample 'imu.world' do |task|
                task.use_sim_time = false
            end
            diff_t = (Time.now - sample.time)
            assert(diff_t > 0 && diff_t < 10)
        end

        it 'provides an orientation-to-starting pose by default' do
            sample = read_orientation_sample 'imu-not-aligned.world'
            assert sample.orientation.approx?(Eigen::Quaternion.Identity)
        end

        it 'provides an orientation-to-horizontal if '\
           'reference == REFERENCE_HORIZONTAL_PLANE' do
            sample = read_orientation_sample 'imu-not-aligned.world' do |task|
                task.reference = :REFERENCE_HORIZONTAL_PLANE
            end
            expected = Eigen::Quaternion.from_euler(
                Eigen::Vector3.new(0, 0.1, 0.1), 2, 1, 0
            )
            assert sample.orientation.approx?(expected)
        end

        it 'provides an orientation-to-absolute if reference == REFERENCE_ABSOLUTE' do
            sample = read_orientation_sample 'imu-not-aligned.world' do |task|
                task.reference = :REFERENCE_ABSOLUTE
            end
            expected = Eigen::Quaternion.from_euler(
                Eigen::Vector3.new(0.1, 0.1, 0.1), 2, 1, 0
            )
            assert sample.orientation.approx?(expected)
        end
    end

    describe 'GPS sensor' do
        def read_sample(world_file, port_name)
            @task = gzserver world_file, '/gazebo:w:m:g'
            yield(@task) if block_given?
            reader = @task.port(port_name).reader
            @task.configure
            @task.start
            assert_has_one_new_sample(reader)
        ensure
            reader&.disconnect
        end

        it 'exports the sensor using a GPSTask' do
            task = gzserver 'gps.world', '/gazebo:w:m:g'
            assert_equal 'rock_gazebo::GPSTask', task.model.name
        end

        it 'exports the solution' do
            sample = read_sample 'gps.world', 'gps_solution'
            # Values in the SDF file are in degrees, convert to radians and
            # give a good precision (1e-9 is around 1mm)
            assert_in_epsilon(-22.9068, sample.latitude, 1e-9)
            assert_in_epsilon(-43.1729, sample.longitude, 1e-9)
            assert_equal :AUTONOMOUS, sample.positionType
            assert_in_delta 10, sample.altitude, 0.01
        end

        it 'properly resolves the topic in a SDF file that uses nested model' do
            sample = read_sample 'gps-nested.world', 'gps_solution'
            # Values in the SDF file are in degrees, convert to radians and
            # give a good precision (1e-9 is around 1mm)
            assert_in_epsilon(-22.9068, sample.latitude, 1e-9)
            assert_in_epsilon(-43.1729, sample.longitude, 1e-9)
            assert_equal :AUTONOMOUS, sample.positionType
            assert_in_delta 10, sample.altitude, 0.01
        end

        it 'sets the time to the simulation\'s logical time' do
            sample = read_sample 'gps.world', 'gps_solution'
            assert(sample.time.to_f > 0 && sample.time.to_f < 10)
        end

        it 'exports a 1m deviation in both vertical an horizontal by default' do
            sample = read_sample 'gps.world', 'gps_solution'
            assert_equal 1, sample.deviationLatitude
            assert_equal 1, sample.deviationLongitude
            assert_equal 1, sample.deviationAltitude
        end

        it 'uses the GPS noise parameters to set the deviations if present' do
            sample = read_sample 'gps-noise.world', 'gps_solution'
            assert_equal 3, sample.deviationLatitude
            assert_equal 3, sample.deviationLongitude
            assert_equal 2, sample.deviationAltitude
        end

        it 'exports the realtime instead of the logical time if use_sim_time is false' do
            sample = read_sample 'gps.world', 'gps_solution' do |task|
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
            def read_sample(world_file, port_name)
                super do |task|
                    task.use_proper_utm_conversion = true
                    yield task if block_given?
                end
            end

            it 'converts the position to the configured UTM frame' do
                sample = read_sample 'gps-check-utm.world', 'utm_samples' do |task|
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
                sample = read_sample(
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
                sample = read_sample 'gps.world', 'position_samples' do |task|
                    task.use_sim_time = false
                end
                diff_t = (Time.now - sample.time)
                assert(diff_t > 0 && diff_t < 10)
            end

            it 'exports the realtime instead of the logical time '\
               'if use_sim_time is false' do
                sample = read_sample 'gps.world', 'utm_samples' do |task|
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
            def read_sample(world_file, port_name)
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
                sample = read_sample 'gps-far-from-origin.world', 'utm_samples'
                # The position samples are UTM N/E. Rock is in NWU, so we must convert the
                # UTM coordinates to NWU
                assert_equal 'gps_test', sample.sourceFrame
                assert_equal 'utm_test', sample.targetFrame
                assert_in_delta(-1000, sample.position.x, 1)
                assert_in_delta 1000, sample.position.y, 1
            end

            it 'converts the position to the local frame' do
                sample = read_sample 'gps-far-from-origin.world', 'position_samples'
                # The position samples are UTM N/E. Rock is in NWU, so we must convert the
                # UTM coordinates to NWU
                assert_equal 'gps_test', sample.sourceFrame
                assert_equal 'nwu_test', sample.targetFrame
                assert_in_delta 1000, sample.position.x, 1
                assert_in_delta 1000, sample.position.y, 1
            end

            it 'exports the realtime instead of the logical time if '\
               'use_sim_time is false' do
                sample = read_sample 'gps.world', 'position_samples' do |task|
                    task.use_sim_time = false
                end
                diff_t = (Time.now - sample.time)
                assert(diff_t > 0 && diff_t < 10)
            end

            it 'exports the realtime instead of the logical time if '\
               'use_sim_time is false' do
                sample = read_sample 'gps.world', 'utm_samples' do |task|
                    task.use_sim_time = false
                end
                diff_t = (Time.now - sample.time)
                assert(diff_t > 0 && diff_t < 10)
            end
        end
    end
end
