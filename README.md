# Robotic Assistive Mobility and Manipulation Simulation (RAMMS) Core Plugin

This plugin provides foundational components and utilities for simulating
robotic assistive mobility devices and manipulation tasks within Unreal
Engine 5. It is designed to support the development of advanced robotic
assistive technologies, including powered wheelchairs and robotic arms, by
providing realistic physics, sensor simulation, and control interfaces.

This repository is designed as a `plugin` for Unreal Engine 5, allowing for easy
integration into UE5 projects. It can be added to any UE5 project by cloning it
or adding it as a `git submodule` within your project's `Plugins/` directory.

For an example project demonstrating the use of this plugin, please refer to the
[RAMMS Sim Project](https://github.com/rammp-org/ramms-sim).

## Quick Start

1. Clone or add this repository as a git submodule within your UE5 project's
   `Plugins/` directory.
2. Open your UE5 project. The plugin should be automatically detected, compiled
   and available for use.
3. Enable the plugin in your project's Plugins settings if it is not already
   enabled.
4. Explore the provided components, blueprints, and example scenes to get started
   with robotic assistive mobility and manipulation simulation.

## Provided Features

- Differential drive controller for powered wheelchair drive simulation.
- Mebot robotic wheelchair base controller component providing linear and
  angular actuator control for realistic wheelchair movement.
- Accessible Van Ramp component for easy integration and control of van ramp
  animations.
- Accessible Door component for simulating door interactions.
