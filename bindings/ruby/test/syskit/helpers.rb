$LOAD_PATH.unshift(File.expand_path("../../lib", __dir__))

require 'rock_gazebo/syskit/test'
require_relative '../helpers'

module SyskitHelpers
    def setup
        super
        Roby.app.log_dir = Dir.mktmpdir
    end

    def teardown
        super
        FileUtils.rm_rf Roby.app.log_dir
    end
end

class Minitest::Test
    include SyskitHelpers
end
