set ip=130.251.13.178
pscp -pw asctec kernel_outdoor.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/kernel.cpp
pscp -pw asctec module5_outdoor.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/module5.cpp
pscp -pw asctec module1_outdoor.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/module1.cpp
pscp -pw asctec pelican_control_outdoor_predictive_obs.cpp asctec@%ip%:/home/asctec/workspace/aruco/utils/module0.cpp
pause