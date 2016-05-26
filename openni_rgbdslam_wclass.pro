HEADERS += \
    SlamBase.h

SOURCES += \
    SlamBase.cpp \
    main.cpp
######c++11######
QMAKE_CXXFLAGS += -std=c++11
INCLUDEPATH +=      /usr/local/include \
######opencv######
                    /usr/local/include/opencv \
                    /usr/local/include/opencv2 \
######openni######
                    /usr/include/openni2 \
                    /usr/include/nite \
###### pcl ########
                    /usr/include/pcl-1.7  \
######eigen3#####
                    /usr/include/eigen3 \
                    /usr/include/eigen3/Eigen \
###### boost ######
                    /usr/local/include/boost \
###### VTK ######
                    /usr/include/vtk-5.8  \
###### g2o ########
                   /usr/local/include/g2o \
######sparse#####
                    /usr/include/suitesparse  \

LIBS +=     /usr/local/lib/libopencv_highgui.so \
            /usr/local/lib/libopencv_core.so    \
            /usr/local/lib/libopencv_imgproc.so  \
            /usr/local/lib/libopencv_calib3d.so  \
            /usr/local/lib/libopencv_nonfree.so  \
            /usr/local/lib/libopencv_flann.so  \
            /usr/local/lib/libopencv_features2d.so \
            /usr/lib/libXnVNite_1_5_2.so  \
            /usr/lib/libOpenNI2.so   \
#######pcl#####
            /usr/lib/libpcl_common.so.1.7 \
            /usr/lib/libpcl_io.so.1.7 \
            /usr/lib/libpcl_io_ply.so.1.7 \
            /usr/lib/libpcl_visualization.so.1.7 \
            /usr/lib/libpcl_filters.so.1.7 \
#######boost######
#            /usr/local/lib/libboost_filesystem.so \
            /usr/local/lib/libboost_system.so \
######g2o###########
            /usr/local/lib/libg2o_cli.so  \
            /usr/local/lib/libg2o_core.so  \
            /usr/local/lib/libg2o_stuff.so  \
            /usr/local/lib/libg2o_types_slam2d.so  \
            /usr/local/lib/libg2o_types_slam3d.so  \
            /usr/local/lib/libg2o_solver_cholmod.so  \
            /usr/local/lib/libg2o_solver_csparse.so  \
            /usr/local/lib/libg2o_csparse_extension.so  \
#######csparce#######
            /usr/lib/libcxsparse.so \
