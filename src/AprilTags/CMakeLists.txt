add_library(AprilTags_lib 
	Edge.cc
	FloatImage.cc
	Gaussian.cc
	GLine2D.cc
    GLineSegment2D.cc 
    GrayModel.cc 
    Homography33.cc 
    MathUtil.cc 
    Quad.cc 
    Segment.cc 
    TagDetection.cc 
    TagDetector.cc 
    TagFamily.cc 
    UnionFindSimple.cc 
)

target_link_libraries(AprilTags_lib ${OpenCV_LIBS})