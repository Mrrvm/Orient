CFLAGS = -Wall -Wextra -Werror -g -Wpedantic -Wshadow -Wstrict-overflow -fno-strict-aliasing
OPENCV = -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_xfeatures2d -lopencv_highgui -lopencv_imgcodecs -lopencv_flann -lopencv_imgproc

get_intrinsics: get_intrinsics.cpp
	g++ get_intrinsics.cpp $(OPENCV) -o get_intrinsics

get_rotation: get_rotation.cpp
	g++ get_rotation.cpp $(OPENCV) -o get_rotation