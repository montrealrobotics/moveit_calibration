# MoveIt Calibration

*Tools for robot arm hand-eye calibration.*

MoveIt Calibration supports ArUco boards and ChArUco boards as calibration targets. Experiments have demonstrated that a
ChArUco board gives more accurate results, so it is recommended.

This package was originally developed by Dr. Yu Yan at Intel, and was originally submitted as a PR to the core MoveIt
repository. For background, see this [Github discussion](https://github.com/ros-planning/moveit/issues/1070).

## Instructions

### Build from Source

```sh
mkdir -p ws_moveit/src
cd ws_moveit
git clone https://github.com/montrealrobotics/moveit_calibration.git src/moveit_calibration
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Example
Launch your robot arm with moveit. In rviz click 'Add' to add a new display, from the box that opens, select 'HandEyeCalibration'.
![Screenshot from 2025-06-11 13-30-50](https://github.com/user-attachments/assets/d91efd44-f7ef-457b-8ed2-24a2e3727f0e)

The display that opens will give you the option to create a new calibration board that can be printed or load an existing board. Make your choice and enter the parameters of the calibration board.
![Screenshot from 2025-06-11 11-45-15](https://github.com/user-attachments/assets/84b62854-d7dc-4ee1-b989-b73715408a3c)

Select either Load/Save depending on your previous choice. This will load the parameters of your board, ready for calibration.

Select the camera topic of the stream you are calibrating in dropdown 'Camera Image Topic'.

Go to the Context tab, here you can select sensor configuration, eye in hand or eye to hand.

Next, select the frames. The calibration board should be in view of the camera do that it is available to select in the dropdown for object frame.

- Sensor frame: The frame of the sensor capturing the image
- Object frame: The calibration board
- End-effector frame: The frame of the robot that the camera is attached to
- Robot base frame: The base frame of the robot arm
- Camera base link: This is the link in the camera that is attached to the robot, the final calibration will create aa tf from the end effector frame to this camera link, all other camera links should be relative to this link

![Screenshot from 2025-06-11 11-47-11](https://github.com/user-attachments/assets/b573e593-2ced-4b03-bfef-1963b0633b89)

If you wish, you can enter initial guesses for the calibration.

Next, go to the Caibrate tab.

For AX = XB solver, select 'OpenCV/Daniilidis1998'.

There are a number of options for calibrating, more details can be found [here](https://moveit.picknik.ai/humble/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html).
To have more control over the outcome, I would suggest the following:

- Put your robot arm into teach mode.
- Guide the arm into a pose where the calibration board is visible in the frame
- at the bottom left of the calibrate tab, there is a 'Detection error', if this is green, you have a good sample.
- Take your robot out of teach mode
- click, 'Take Sample'.
- If the detection error is too high, (highlighted red)
- move the arm into a new pose until the error is reduced.
- Then save the sample.

5 Samples are required for a calibration, after 5 samples have been collected, a reprojection error will be displayed bottom left. If you go to the Context tab, the 'Camera Pose Initial Guess' boxes will be populated with the calibrated values. 

![Screenshot from 2025-06-11 11-59-10](https://github.com/user-attachments/assets/a2193975-e2e9-4a15-848c-39b8a18847a9)

To save the calibration, select 'Save camera pose', this will save your calibration to a file which is a ros2 launch file and so can be launched directly with your robot.

The calibrated camera pose is published so you can check it in the TF section of rviz.
