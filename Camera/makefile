CFLAGS = -Wall -Wextra -Werror -g -Wpedantic -Wshadow -Wstrict-overflow -fno-strict-aliasing
OPENCV = -lopencv_video -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_xfeatures2d -lopencv_highgui -lopencv_imgcodecs -lopencv_flann -lopencv_imgproc

get_intrinsics: get_intrinsics.cpp
	g++ get_intrinsics.cpp $(OPENCV) -o get_intrinsics

get_image: get_image.cpp
	g++ get_image.cpp $(OPENCV) -o get_image

get_rotationH: get_rotationH.cpp
	g++ get_rotationH.cpp $(OPENCV) -o get_rotationH

get_rotationSVD: get_rotationSVD.cpp
	g++ get_rotationSVD.cpp $(OPENCV) -o get_rotationSVD

