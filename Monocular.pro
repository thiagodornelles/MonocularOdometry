#-------------------------------------------------
#
# Project created by QtCreator 2017-05-27T18:37:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Monocular
TEMPLATE = app


SOURCES += main.cpp

HEADERS  +=

INCLUDEPATH += /usr/local/opt/opencv3/include/

LIBS += -L/usr/local/opt/opencv3/lib \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_features2d \
    -lopencv_highgui \
    -lopencv_objdetect \
    -lopencv_ximgproc \
    -lopencv_imgcodecs \
    -lopencv_calib3d \
    -lopencv_photo \
    -lopencv_video \
    -lopencv_videoio \
