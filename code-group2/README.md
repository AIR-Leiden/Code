In order to run the planning algorithm (used in the finals, everything before that was the reactive code, both of these have only been tested on the red robot in the new Gorlaeus robot lab, due to time constraints but could be easily fixed):
# Planning
0. Ensure that the needed files in planning scripts and launch are added to the air_challenge_real/launch and the air_challenge_real/Scripts folder and that the parameters are added in the turn_on_wheeltec_robot and navigation params folder.

1. `roslaunch turn_on_wheeltec_robot rrt_slam.launch`
(This step is needed to launch the needed nodes for slam, the unneeded ones get automatically killed later on)

2. `roslaunch turn_on_wheeltec_robot wheeltec_camera.launch camera_mode:=Astra_Pro+RgbCam`
(This step turns on the camera, please check that there aren't any errors in red)

3. `roslaunch air_challenge_real rviz.launch` (through realVNC viewer, on the terminal of the robot)
(This is for a visualization of the planning algorithm, not necessary, do check if the robot is in it normal position and not falling into the ground in the rviz application.
The last scenario indicates an issue with the robot and warrants a reboot of the system).

Follow path 'a' for seeker, 'b' for avoided:

4a. `roslaunch air_challenge_real detection.launch`
(If the camera works this should display a camera feed, through RealVNC viewer on the terminal of the robot, otherwise go back to step 2)

5a. `roslaunch air_challenge_real rrt.launch`
(this launches the program, which will start driving in approx 30 seconds from hitting enter).

4b. `roslaunch air_challenge_real detection_away.launch`
(If the camera works this should display a camera feed, otherwise go back to step 2)

5b. `roslaunch air_challenge_real rrt_away.launch`
(this launches the program, which will start driving in approx 30 seconds from hitting enter).

# Reactive algorithm
For the reactive algorithm:

0. Ensure that the needed files in the reactive folder are added to the air_challenge_real folder.
1. `roslaunch turn_on_wheeltec_robot wheeltec_lidar.launch`
2. `roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch`
3. `roslaunch turn_on_wheeltec_robot wheeltec_camera.launch camera_mode:=Astra_Pro+RgbCam`
4. `roslaunch air_challenge_real get_him.launch` (for the seeker, please run through RealVNC on the terminal of the robot)
5. `roslaunch air_challenge_real run_away.launch` (for the hider, please run through RealVNC on the terminal of the robot)

# General remarks
if the air_challenge_real package does not exist yet, please use copy the folder from the scripts folder into the correct folder in the robot.
Also, add the correct contents (only the files from the repository into the launch and script folder).
