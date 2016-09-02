#groundObjectDetectionFilter

This project aims at exploiting probabilistic properties of the data collected by the robot to run estimation with bayes filter on several topologic parameters such as the occupancy and the height of the floor, and build a model of the surface. The experiment world is described in the report.

Index:
- demo.mp4: Height estimation video. From left to right: 
    - height estimation gradient (color is related to the height: from green to blue as height increases). Gradient is not very refined. Irregularities corresponds to the various objects in the world (sofas, trees ...)
    - confidence gradient drawn from the estimated height variance (pink color is related to low variance i.e. high confidence in the estimation)
    - Average square error of the estimation computed (dark red is related to low error).
- report.pdf: Experiment description and analysis
- floor_nav: ROS package for the robot autonomous driving.
- floor_mapping_b: ROS package implementing the ground traversability classification using a binary bayesian filter.
- floor_plane_filter: ROS package implementing the world elevation map using a Kalman filter.
