ros2 launch your_package your_launch_file.py experiment_name:=walking_test_01

```

This will create a bag with a name like: `2025-11-13_14-30-45_walking_test_01`

**Key changes:**
1. Added `DeclareLaunchArgument` for the experiment name
2. Used `LaunchConfiguration` to get the value
3. Modified the bag path to be a list that concatenates: `timestamp_experiment_name`
4. The `LaunchConfiguration` substitution gets resolved when the command is executed

The bag will be saved to something like:
```

~/bag_data/2025-11-13_14-30-45_walking_test_01/
