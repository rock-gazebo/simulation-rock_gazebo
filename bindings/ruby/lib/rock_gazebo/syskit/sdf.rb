# frozen_string_literal: true

module RockGazebo
    module Syskit
        # SDF-related configuration
        #
        # It is accessible as Conf.sdf
        class SDF
            # Guard value that allows the Gazebo/Syskit integration to check
            # that a profile's use_sdf_model has indeed been called after the
            # configuration's use_sdf_world
            attr_predicate :has_profile_loaded?, true

            # @return [SDF::World] The loaded world
            attr_accessor :world

            # The world name that should be or has been loaded
            attr_accessor :world_path

            # The full path to the worl file
            attr_accessor :world_file_path

            def initialize
                @world = ::SDF::World.empty
            end

            # Load a SDF file and sets it as the current file on self
            #
            # @param [Array<String>] path elements of the path to the file
            # @param [String,nil] world_name if given, the method will look for
            #   a world of the given name in the loaded file. Otherwise, the loaded
            #   file must have a single world
            # @return [SDF::World]
            def load_sdf(*path, world_name: nil)
                path = File.join(*path)
                _, resolved_paths =
                    Rock::Gazebo.resolve_worldfiles_and_models_arguments([path])
                full_path = resolved_paths.first
                full_path = autodetect_sdf_file_path(full_path)
                ::SDF::XML.model_path = Rock::Gazebo.model_path
                world = sdf_world_from_path(full_path, world_name: world_name)
                use_sdf_world(world, path: full_path)
            end

            # Directly pass a SDF world object to be used for Syskit
            #
            # @param [SDF::World] world the SDF world
            # @param [String] path path to the SDF file, for debugging reasons
            def use_sdf_world(world, path: nil)
                @world_file_path = path
                @sdf = world.parent
                @world = world
            end

            # @api private
            #
            # Try to turn a path into the path of a SDF file
            #
            # The method tries ${path} and ${path}/model.sdf to match SDF model
            # directories
            #
            # @param [String] path
            # @return [String]
            def autodetect_sdf_file_path(path)
                return path if File.file?(path)

                model_sdf = File.join(path, "model.sdf")
                return model_sdf if File.file?(model_sdf)

                raise ArgumentError, "#{path} cannot be resolved to a SDF file"
            end

            # @api private
            #
            # Resolves a world from a SDF file, which may have more than one
            #
            # @param [String] path path to the world file
            # @param [String] world_name if not nil, the expected world name
            # @return [World]
            # @raise ArgumentError if the SDF file does not define a world
            # @raise ArgumentError if the SDF file has more than one world and
            #   world_name is not set
            # @raise ArgumentError if the SDF file has more than one world and
            #   none match the name provided as world_name
            def sdf_world_from_path(path, world_name: nil)
                sdf = ::SDF::Root.load(path, flatten: false)
                Rock::Gazebo.process_sdf_file(sdf)
                worlds = sdf.each_world.to_a
                if world_name
                    world = worlds.find { |w| w.name == world_name }
                    unless world
                        raise ArgumentError,
                              "cannot find a world named #{world_name} in #{path}"
                    end
                    world
                elsif worlds.size == 1
                    worlds.first
                elsif worlds.empty?
                    raise ArgumentError, "no worlds declared in #{path}"
                else
                    raise ArgumentError,
                          "more than one world declared in #{path}, select one "\
                          "explicitely by providing the world_name argument"
                end
            end

            # Find all the models that have been included in the loaded world
            #
            # @param [String] model_name either a model:// URI or the full path
            #   to the model's SDF file.
            # @return [Array<Model>] the list of MOdel objects, included in
            #   {#world}, that are the expected model
            def find_all_included_models(model_name)
                @sdf.find_all_included_models(model_name)
            end

            # Force-select the UTM zone that should be used to compute
            # {#global_origin}
            def select_utm_zone(zone, north)
                @utm_zone  = zone
                @utm_north = !!north # rubocop:disable Style/DoubleNegation
                utm = spherical_coordinates.utm(zone: utm_zone, north: utm_north?)
                @utm_global_origin = Eigen::Vector3.new(
                    utm.easting, utm.northing,
                    world.spherical_coordinates.elevation
                )
                @global_origin = self.class.utm2nwu(utm_global_origin)
            end

            # Select the UTM zone number that contains the global origin
            def select_default_utm_zone
                select_utm_zone(*spherical_coordinates.default_utm_zone)
            end

            # Return the full UTM zone (number and letter) that contains the
            # global origin
            def default_utm_full_zone
                GeoUtm::UTMZones.calc_utm_default_zone(
                    spherical_coordinates.latitude_deg,
                    spherical_coordinates.longitude_deg
                )
            end

            # The currently selected UTM zone
            #
            # It is guessing the zone from the world's spherical coordinates if
            # it has not been set
            def utm_zone
                select_default_utm_zone unless @utm_zone
                @utm_zone
            end

            # Whether we're on the north or south part of the UTM zone
            #
            # It is guessing the zone from the world's spherical coordinates if
            # it has not been set
            def utm_north?
                select_default_utm_zone unless @utm_zone
                @utm_north
            end

            # Return the global origin in UTM coordinates
            #
            # @return [Eigen::Vector3] the coordinates in ENU convention, relative
            #   to the origin of the UTM grid
            def utm_global_origin
                select_default_utm_zone unless @utm_zone
                @utm_global_origin
            end

            # The world's GPS origin in NWU (Rock coordinates)
            #
            # @return [Eigen::Vector3] the coordinates in Rock's NWU convention,
            #   relative to the origin of the UTM grid
            def global_origin
                select_default_utm_zone unless @utm_zone
                @global_origin
            end

            # Converts UTM coordinates in Rock's NWU coordinates
            #
            # @param [Eigen::Vector3] utm
            # @return [Eigen::Vector3]
            def self.utm2nwu(utm)
                Eigen::Vector3.new(utm.y, 1_000_000 - utm.x, utm.z)
            end

            # Converts Rock's NWU coordinates into UTM coordinates
            #
            # @param [Eigen::Vector3] nwu
            # @return [Eigen::Vector3] east (x), north (y) and altitude (z)
            def self.nwu2utm(nwu)
                Eigen::Vector3.new(1_000_000 - nwu.y, nwu.x, nwu.z)
            end

            # The local position of the given lat/lon/alt coordinates
            def local_position(latitude, longitude, altitude)
                zone_letter = GeoUtm::UTMZones.calc_utm_default_letter(latitude)
                utm = GeoUtm::LatLon.new(latitude, longitude)
                                    .to_utm(zone: "#{utm_zone}#{zone_letter}")
                utm_nwu = self.class.utm2nwu(
                    Eigen::Vector3.new(utm.e, utm.n, altitude)
                )
                utm_nwu - global_origin
            end

            # The global (lat/long/alt) coordinates corresponding to the given NWU
            # coordinates
            #
            # @return [(Float,Float,Float)] lat, lon and altitude
            def global_position(nwu)
                zone_letter = GeoUtm::UTMZones.calc_utm_default_letter(
                    spherical_coordinates.latitude_deg
                )
                utm_nwu = nwu + global_origin
                utm_v = self.class.nwu2utm(utm_nwu)
                altitude = utm_v.z
                utm = GeoUtm::UTM.new("#{utm_zone}#{zone_letter}", utm_v.x, utm_v.y)
                latlon = utm.to_lat_lon

                [latlon.lat, latlon.lon, altitude]
            end

            def respond_to_missing?(method_name, include_all = false)
                world.respond_to?(method_name, include_all)
            end

            def method_missing(*args, &block) # rubocop:disable Style/MethodMissingSuper
                world.public_send(*args, &block)
            end
        end
    end
end
