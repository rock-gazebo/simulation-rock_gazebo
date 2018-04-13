require 'rock_gazebo/syskit/test'

module RockGazebo
    module Syskit
        describe SDF do
            describe "#global_origin" do
                def dataset
                    [[48.8580, 2.2946, 31, true, 448_258.92, 5_411_910.38],
                     [-22.9707, -43.1823, 23, false, 686_342.74, 7_458_569.92]]
                end

                it "sets the UTM zone to default and returns the UTM coordinates of the spherical_coordinates element" do
                    dataset.each do |lat, long, zone, zone_north, easting, northing|
                        sdf = SDF.new
                        sdf.world = ::SDF::World.from_string(
                            "<world>
                                <spherical_coordinates>
                                    <latitude_deg>#{format('%.4f', lat)}</latitude_deg>
                                    <longitude_deg>#{format('%.4f', long)}</longitude_deg>
                                    <elevation>42</elevation>
                                </spherical_coordinates>
                             </world>")

                        utm = sdf.utm_global_origin
                        assert_equal zone, sdf.utm_zone
                        refute(zone_north ^ sdf.utm_north?)
                        assert_in_delta easting, utm.x, 0.01
                        assert_in_delta northing, utm.y, 0.01
                        assert_in_delta 42, utm.z, 0.01

                        nwu = sdf.global_origin
                        assert_in_delta northing, nwu.x, 0.01
                        assert_in_delta (1_000_000 - easting), nwu.y, 0.01
                        assert_in_delta 42, nwu.z, 0.01
                    end
                end

                it "allows to force the UTM zone used" do
                    sdf = SDF.new
                    flexmock(sdf.world).
                        should_receive(spherical_coordinates: coordinates = flexmock)
                    coordinates.should_receive(:utm).
                        with(zone: 43, north: false).
                        once.and_return(flexmock(easting: 100, northing: 1000))
                    coordinates.should_receive(elevation: 42)

                    sdf.select_utm_zone(43, false)
                    utm = sdf.utm_global_origin
                    assert_equal 43, sdf.utm_zone
                    refute sdf.utm_north?
                    assert_in_delta 100, utm.x, 0.01
                    assert_in_delta 1000, utm.y, 0.01
                    assert_in_delta 42, utm.z, 0.01
                end

                it "resolves other coordinates w.r.t. the local origin" do
                    sdf = SDF.new
                    sdf.world = ::SDF::World.from_string(
                        "<world>
                            <spherical_coordinates>
                                <latitude_deg>48.8580</latitude_deg>
                                <longitude_deg>2.2946</longitude_deg>
                                <elevation>42</elevation>
                            </spherical_coordinates>
                            </world>")

                    assert Eigen::Vector3.Zero.approx?(
                        sdf.local_position(48.8580, 2.2946, 42))

                    d = (Eigen::Vector3.new(5.52, -3.72, 1) -
                        sdf.local_position(48.85805, 2.29465, 43)).norm
                    assert(d < 0.1)
                end
            end
        end
    end
end
