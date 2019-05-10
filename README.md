# wifi_heatmap

THE Wi-Fi heatmap for augmenting maps and predicting location.

## Installation

Clone this repo into your catkin workspace under the `src` folder.

Some pre-requisite python libraries are: 
* `numpy`
* `pandas`
* `sklearn`

Before running either file (`ros_collect_aps.py` or `publish_wifi_pose.py`), make sure to make them executable:

```bash
chmod +x {path_to/file_name}
```

## Usage

Once you have ROS running, your navigation (i.e. `amcl_navigation`) is running and your Turtlebot is on, you can use `rosrun` on a new terminal to execute either script.

### Collect Data 

To collect data, you can also initialize teleoperation (i.e. `turtlebot_teleop`) and execute the following:

```bash
rosrun wifi_heatmap ros_collect_aps.py file_name.json -n {number}
```

This will start the script and prompt you to press enter when you are ready to scan. The parameters are a `.json` file where the data will be stored and the number of scans at each location (after pressing enter). You can then teleop the Turtlebot (or set a goal) to the next location and repeat.

You should look at `file_name.json` after terminating the script for all your data.

### Predict Pose

Before executing the prediction script, you will need to load your map into the `WifiMap` class in `publish_wifi_pose.py`. To do so, you'll notice on line `30` the following:

```python
wmap = WifiMap('../data/dis.json')
```

For convention, we organize all output `.json` files of the `ros_collect_aps.py` script inside the `data` folder. All you need to do is change the parameter of `WifiMap()` to the path (relative or absolute) where your `.json` file lives.

To predict a pose based on your observed Wi-Fi strengths in that file, you just need to execute the following:

```bash
rosrun wifi_heatmap publish_wifi_pose.py
```
This will update the pose of the Turtlebot based on the prediction model of the observed Wi-Fi networks, and if you're using rviz it should update right away.

## Customization

In its current state, `publish_wifi_pose.py` publishes only once to the `/initialpose` rostopic. It is also possible to update the `publisher` to a custom topic and update line `52` to have a `while` loop like so:

```python
while not rospy.is_shutdown(): 
```

This would allow to constantly have a readily avaiable source of the predicted location from Wi-Fi signals and incorporate it into any localization system.

## Authors
* Alvaro Mendez (alvaro.mndz93@gmail.com)
* Andrew Sterner (andrew.john.sterner@gmail.com)

## License
[MIT](https://choosealicense.com/licenses/mit/)
