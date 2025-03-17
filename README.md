# ROS2 Serial Reader

A ROS2 driver node used for reading serial data from a serial port and publishing it to a ROS2 topic


## Including as a Subrepository (Subrepo)

To include this package in a parent ROS2 workspace:

Add as a submodule:

```bash
cd ~/<your_workspace>/src
git submodule add <this-repo-url>  serial_reader
```

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the parent workspace:

```bash
cd ~/<your_workspace>
colcon build --symlink-install
source install/setup.bash
```

## Usage

Run the publisher node:

```bash
ros2 run serial_reader publisher
```

Subscribe to the serial data:

```bash
ros2 topic echo /serial_data
```

### Parameters

You can change the publiser topic name by passing the topic name as a parameter:

```bash
ros2 run serial_reader publisher --ros-args -r /serial_data:=/encoder
```

## License

This project is licensed under the Apache License 2.0.

## Maintainers

[Andrew Johnson](https://github.com/anjrew) – Maintainer – andrewmjohnson549@gmail.com

## Contributing

Contributions are welcome via PR!