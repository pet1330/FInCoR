FInCoR
======

### Camera_setup
This node can be used to align two points clouds of the same scene observed from difference angles. The cameras must be roughly oposite each other observing the same scene.

To run this node source the workspace and use `roslaunch camera_setup start_cameras.launch`

The launch file accepts two param `quick` and `frames`. quick allows the user to define whether or calibration should be run using the calibration grid, or whether it should broadcast the last successful calibration from the param file.

### circle_detection
This node can be used to detect and identify circles within a 3D scene.

To run this node source the workspace and use `roslaunch circle_detection multi_cam_circle_detection.launch`

### External Models Required

* [FInCoR] (https://github.com/pet1330/FInCoR) (this repository)
* [STRANDS Human Robot Interaction] (https://github.com/pet1330/strands_hri)
* [STRANDS QSR Library] (https://github.com/pet1330/strands_qsr_lib)
* [Bayes Probabilistic Tracking] (https://github.com/pet1330/bayestracking)
* [STRANDS People Perception] (https://github.com/pet1330/strands_perception_people)
