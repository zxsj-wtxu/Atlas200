TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    car_color_inference/car_color_inference.cpp \
    car_plate_detection/car_plate_detection.cpp \
    car_plate_recognition/car_plate_recognition.cpp \
    car_type_inference/car_type_inference.cpp \
    object_detection/object_detection.cpp \
    object_detection_post/object_detection_post.cpp \
    video_analysis_post/video_analysis_message.pb.cpp \
    video_analysis_post/video_analysis_post.cpp \
    video_decode/video_decode.cpp

INCLUDEPATH+=$$PWD/common/include
INCLUDEPATH+=$$PWD/include/
INCLUDEPATH+=$$PWD/include/third_party/
INCLUDEPATH+=$$PWD/include/libc_sec/include
INCLUDEPATH+=$$PWD/include/inc
INCLUDEPATH+=$$PWD/include/inc/cce
INCLUDEPATH+=$$PWD/include/inc/custom/common/
INCLUDEPATH+=$$PWD/include/inc/custom/common/op
INCLUDEPATH+=$$PWD/include/inc/custom/common/op_def
INCLUDEPATH+=$$PWD/include/inc/custom/omg
INCLUDEPATH+=$$PWD/include/inc/custom/omg/graph
INCLUDEPATH+=$$PWD/include/inc/custom/omg/model
INCLUDEPATH+=$$PWD/include/inc/custom/omg/model/op_builder
INCLUDEPATH+=$$PWD/include/inc/custom/omg/parser
INCLUDEPATH+=$$PWD/include/inc/custom/omg/parser/caffe
INCLUDEPATH+=$$PWD/include/inc/custom/omg/parser/tensorflow
INCLUDEPATH+=$$PWD/include/inc/custom/proto
INCLUDEPATH+=$$PWD/include/inc/custom/proto/caffe
INCLUDEPATH+=$$PWD/include/inc/custom/proto/tensorflow
INCLUDEPATH+=$$PWD/include/inc/custom/toolchain
INCLUDEPATH+=$$PWD/include/inc/driver
INCLUDEPATH+=$$PWD/include/inc/dvpp
INCLUDEPATH+=$$PWD/include/inc/hiaiengine/proto
INCLUDEPATH+=$$PWD/include/inc/mmpa/sub_inc
INCLUDEPATH+=$$PWD/include/inc/proto
INCLUDEPATH+=$$PWD/include/inc/tensor_engine
INCLUDEPATH+=$$PWD/include/third_party/glog
INCLUDEPATH+=$$PWD/include/third_party/glog/include
INCLUDEPATH+=$$PWD/include/third_party/gflags
INCLUDEPATH+=$$PWD/include/third_party/gflags/include
INCLUDEPATH+=$$PWD/include/third_party/opencv
INCLUDEPATH+=$$PWD/include/third_party/opencv/include
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/stitching
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/core
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/core/opencl
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/core/opencl/runtime
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/core/cuda
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/features2d
INCLUDEPATH+=$$PWD/include/third_party/opencv/include/opencv2/imgproc
INCLUDEPATH+=$$PWD/include/third_party/cereal
INCLUDEPATH+=$$PWD/include/third_party/cereal/include
INCLUDEPATH+=$$PWD/include/third_party/cereal/include/cereal
INCLUDEPATH+=$$PWD/include/third_party/cereal/include/cereal/types
INCLUDEPATH+=$$PWD/include/third_party/cereal/include/cereal/external
INCLUDEPATH+=$$PWD/include/third_party/cereal/include/cereal/external/rapidjson
INCLUDEPATH+=$$PWD/include/third_party/protobuf
INCLUDEPATH+=$$PWD/include/third_party/protobuf/include
INCLUDEPATH+=$$PWD/include/third_party/protobuf/include/google
INCLUDEPATH+=$$PWD/include/third_party/protobuf/include/google/protobuf
INCLUDEPATH+=$$PWD/include/third_party/protobuf/include/google/protobuf/compiler

HEADERS += \
    car_color_inference/car_color_inference.h \
    car_plate_detection/car_plate_detection.h \
    car_plate_recognition/car_plate_recognition.h \
    car_type_inference/car_type_inference.h \
    object_detection/hiai_data_sp_son.h \
    object_detection/object_detection.h \
    object_detection/thread_safe_queue.h \
    object_detection_post/object_detection_post.h \
    video_analysis_post/video_analysis_message.pb.h \
    video_analysis_post/video_analysis_post.h \
    video_decode/video_decode.h

DISTFILES += \
    car_plate_detection/Makefile \
    car_plate_recognition/Makefile \
    video_analysis_post/video_analysis_message.proto \
    graph_template.config
