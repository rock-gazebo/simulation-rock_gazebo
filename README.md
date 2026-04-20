# Interface between a Rock system and gazebo

## General SDF Usage

The library offers two plugins: RockSystem and PluginTask. The former is general setup
and must be included for all other features to work. The second allows to instanciate
any Rock components. In addition, if that component implements the
`rock_gazebo::PluginTaskI` interface, it will call the interface's methods at various
points of the gazebo lifecycle (read `PluginTaskI` documentation for more details).

The first plugin must be declared in the world block, and takes no extra elements or
configuration:

```xml
<world name="world">
  <plugin filename="rock_gazebo" name="rock_gazebo::RockSystem" />
</world>
```

The `PluginTask` plugin accepts `task` children elements to declare the tasks to be
instanciated. General (non-`PluginTaskI`) tasks require only a `model` attribute.
For instance, the next block will create two instances of the `Component` task context
from the `some` orogen project, one named `some_name` and the other `foobar`. These tasks
will be executed synchronously in the gazebo loop.

```xml
<world name="world">
  <plugin filename="rock_gazebo" name="rock_gazebo::PluginTask">
    <task name="some_name" model="some::Component" />
    <task name="foobar" model="some::Component" />
  </plugin>
</world>
```

Tasks that are meant to interface with gazebo itself (such as tasks that represent
functionality exported by plugins) subclass the `PluginTaskI` interface. These tasks
are "attached" the plugin's parent entity. For instance, in the following example, the
`vehicle` ModelTask is attached to the `world::m` model.

```xml
<world name="world">
  <model name="m">
    <plugin filename="rock_gazebo" name="rock_gazebo::PluginTask">
      <task name="vehicle" model="rock_gazebo::ModelTask" />
    </plugin>
  </model>
</world>
```

However, one may want to keep the task definitions outside of the model definition (to
be used differently in different scenes). In that case, the `gz` attribute to `task`
allows to explicitly give the entity the task should be attached to, relative to the
plugin's parent scope. For instance:

```xml
<world name="world">
  <plugin filename="rock_gazebo" name="rock_gazebo::PluginTask">
    <task name="vehicle" model="rock_gazebo::ModelTask" gz="m" />
  </plugin>

  <model name="m">
  </model>
</world>
```

In addition, each task may support extra arguments that would be given child elements
or attributes, and are documented below. Each plugin task should document which elements
it supports

# Syskit usage

## Directory Layout

In a bundle, scenes go in scenes/$scenename/$scenename.world. Bundle-specific
models go in models/sdf/$modelname/{model.config,model.sdf}.

## Using Gazebo

At the profile level, one can use SDF to declare the type of robot that is
being used. This is done in a profile by calling <tt>use\_gazebo\_model</tt>. The
method accepts any kind of model (i.e. model:// or a path that can be resolved
under models/sdf and scenes/)

```ruby
profile 'Base' do
    # Sets up the transformer based on the model's kinematic structure
    #
    # Also, sets up the robot devices that will allow Syskit to connect
    # to the Gazebo instance
    use_gazebo_model 'model://myrobot'
    # Also imports frame from the environment description defined in the
    # robot configuration
    use_sdf_world
end
```

Gazebo is *not* managed by Syskit. You have to start it manually. The design
rationale is that one does not reboot the world every time one has to reboot its
software system (which would essentially be what having gazebo managed by the
robot look like)

However, there is a limitation. While gazebo itself is not started by syskit,
the tasks that the rock-gazebo plugin spawns are. So, if you start a
visualization with e.g. rock-gazebo-viz, you *must* add the --no-start option.

Global setup is done by adding the following statement in your robot
configuation file's requires statement. It must be done before requiring any
profile that uses use\_sdf\_model.

~~~ruby
Robot.init do
  require 'rock_gazebo/syskit'
end

Robot.requires do
  Syskit.conf.use_gazebo_world('flat_fish')
end
~~~

This declares all the tasks that the rock-gazebo plugin creates to syskit's set
of deployments, thus allowing you to use them in your systems. Note that in most
cases you won't have to use them explicitely as the profile-level declaration
declares all the devices automatically.

The selected world passed to `use_gazebo_world` is a default. It can be
overriden on the command line with the `--set=sdf.world_path=world_name` option.

## Devices bound to the gazebo instance

## Using SDF without Gazebo

A number of SDF-related functionality (static environment description,
spherical coordinates, transformer setup) are desirable in a Syskit system even
if not using Gazebo.

This is done at the config level with

```ruby
Robot.init do
  require 'rock_gazebo/syskit'
end
Robot.requires do
  Syskit.conf.use_sdf_world 'flat_fish'
end
```

and at the profile level

```ruby
profile 'Base' do
    # Sets up the transformer based on the model's kinematic structure
    use_sdf_model 'model://myrobot'
    # Also imports frame from the environment description defined in the
    # robot configuration
    use_sdf_world
end
```
