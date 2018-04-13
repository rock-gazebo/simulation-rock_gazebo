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

            # The loaded world
            attr_accessor :world

            # The world name that should be or has been loaded
            attr_accessor :world_path

            # The full path to the worl file
            attr_accessor :world_file_path

            def initialize
                @world = ::SDF::World.empty
            end

            def load_sdf(*path, world_name: nil)
                path = File.join(*path)
                _, resolved_paths = Rock::Gazebo.
                    resolve_worldfiles_and_models_arguments([path])
                full_path = resolved_paths.first
                unless File.file?(full_path)
                    if File.file?(model_sdf = File.join(full_path, 'model.sdf'))
                        full_path = model_sdf
                    else
                        raise ArgumentError, "#{path} cannot be resolved to a SDF file"
                    end
                end
                ::SDF::XML.model_path = Rock::Gazebo.model_path
                world = ConfigurationExtension.
                    world_from_path(full_path, world_name: world_name)
                @world_file_path = full_path
                @sdf = world.parent
                @world = world
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
                @utm_north = !!north
                utm = spherical_coordinates.utm(zone: utm_zone, north: utm_north?)
                @utm_global_origin = Eigen::Vector3.new(utm.easting, utm.northing,
                    world.spherical_coordinates.elevation)
                @global_origin = self.class.utm2nwu(utm_global_origin)
            end

            # Select the UTM zone that contains the global origin
            def select_default_utm_zone
                select_utm_zone(*spherical_coordinates.default_utm_zone)
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

            # The local position of the given lat/lon/alt coordinates
            def local_position(latitude, longitude, altitude)
                zone_letter = GeoUtm::UTMZones.calc_utm_default_letter(latitude)
                utm = GeoUtm::LatLon.new(latitude, longitude).
                    to_utm(zone: "#{utm_zone}#{zone_letter}")
                utm_nwu = self.class.utm2nwu(
                    Eigen::Vector3.new(utm.e, utm.n, altitude))
                utm_nwu - global_origin
            end

            def respond_to_missing?(method_name, include_all = false)
                world.respond_to?(method_name, include_all)
            end

            def method_missing(*args, &block)
                world.public_send(*args, &block)
            end
        end
    end
end
