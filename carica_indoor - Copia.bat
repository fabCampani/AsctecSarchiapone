set ip=130.251.13.181
pscp -pw asctec kernel_obstacles.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/kernel.cpp
pscp -pw asctec module5_obstacles.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/module5.cpp
pscp -pw asctec module1.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/module1.cpp
pscp -pw asctec pelican_control_indoor_predictive_obs_old.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/module0.cpp
pause