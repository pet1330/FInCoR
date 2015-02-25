FInCoR
======

### Camera_setup
This node can be used to align two points clouds of the same scene observed from difference angles. The cameras must be roughly oposite each other observing the same scene.

To run this node source the workspace and use `roslaunch camera_setup start_cameras.launch`

### circle_detection
THis node can be used to detect and identify circles within a 3D scene.

To run this node source the workspace and use `roslaunch circle_detection multi_cam_circle_detection.launch`

### object_detection (discontinued)
This node was used to attempt  to track objects within an image stream using CMT (similar to preditor tracking). However, this was not reliable enought and was therefore dropped in favour of using circle detection and applying labels to the objects.

To run this node source the workspace and use `roslaunch object_detection object_dection.launch`
