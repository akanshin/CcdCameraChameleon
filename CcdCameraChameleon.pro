TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    CcdCameraChameleonStateMachine.cpp \
    ClassFactory.cpp \
    CcdCameraChameleonClass.cpp \
    CcdCameraChameleon.cpp \
    camerachameleon.cpp \
    BeamAnalyzer.cpp

HEADERS += \
    CcdCameraChameleonClass.h \
    CcdCameraChameleon.h \
    camerachameleon.h \
    BeamAnalyzer.h


LIBS += /usr/local/lib/libtango.so
LIBS += /usr/lib/libomniORB4.so
LIBS += /usr/lib/libomnithread.so

LIBS += /usr/local/lib/libopencv_imgproc.so \
    /usr/local/lib/libopencv_imgcodecs.so \
    /usr/local/lib/libopencv_core.so \
    /usr/local/lib/libgsl.so \
    /usr/local/lib/libgslcblas.so \
    -lm \
    /usr/local/lib/libtango.so \
    /usr/local/lib/liblog4tango.so \
    /usr/lib/libomniDynamic4.so \
    -pthread \
    -ljpeg \
    -lflycapture

INCLUDEPATH += /usr/local/include/
INCLUDEPATH += /usr/include/gsl
INCLUDEPATH += /usr/local/include/tango
INCLUDEPATH += /usr/local/omniORB4
INCLUDEPATH += /usr/local/omnithread
