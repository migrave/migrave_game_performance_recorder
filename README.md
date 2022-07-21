# migrave_game_performance_recorder
Repository for a game performance recorder used in the MigrAVE project

The package contains the following node:
- migrave_game_performance_recorder

## Depedencies

- `migrave_ros_msgs`

## Usage 

Launch the two recorder nodes:
```sh
roslaunch migrave_game_performance_recorder migrave_game_performance_recorder.launch
```

Start recording

```sh
rostopic pub /migrave_data_recorder/is_record std_msgs/Bool "True"
```
Stop recording

```sh
rostopic pub /migrave_data_recorder/is_record std_msgs/Bool "False"
```


## Project Structure

```
├── CMakeLists.txt
├── package.xml
├── README.md
├── ros
│   ├── launch
│   │   └── migrave_game_performance_recorder.launch
│   ├── scripts
│   │   └── migrave_game_performance_recorder
│   └── src
│       └── game_performance_recorder
│           └── game_performance_recorder.py
└── setup.py
```
