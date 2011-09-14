#-------------------------------------------------
#
# Project created by QtCreator 2011-08-13T12:01:24
#
#-------------------------------------------------

QT       += core

QT       -= gui

DEFINES += "OSG_WITH_GLUT"

INCLUDEPATH += /opt/local/include
#INCLUDEPATH += /opt/local/include/AR


#INCLUDEPATH += /User/stefanreinke/Diplomarbeit/impl/artoolkit/ARToolKit/include/AR

#ADDING GLUT FRAMEWORK
QMAKE_LFLAGS += -F/System/Library/Frameworks/
LIBS += -framework GLUT
LIBS += -framework OpenGL
LIBS += -framework Cocoa

#AR TOOLKIT
LIBS += /usr/local/lib/AR/libAR.a
LIBS += /usr/local/lib/AR/libARvideo.a
LIBS += /usr/local/lib/AR/libARgsub.a
LIBS += /usr/local/lib/AR/libARmulti.a
LIBS += /usr/local/lib/AR/libARgsubUtil.a
LIBS += /usr/local/lib/AR/libARgsub_lite.a

#ADDING OSG LIBS
LIBS += /usr/local/lib/dbg/libOSGBase.dylib
LIBS += /usr/local/lib/dbg/libOSGSystem.dylib
LIBS += /usr/local/lib/dbg/libOSGWindowCarbon.dylib
LIBS += /usr/local/lib/dbg/libOSGWindowCocoa.dylib
#LIBS += /usr/local/lib/dbg/libOSGCoreGL.dylib
LIBS += /usr/local/lib/dbg/libOSGWindowGLUT.dylib
#
#LIBS += /usr/local/lib/libOSGEffectGroups.dylib
#LIBS += /usr/local/lib/libOSGBase.dylib
#LIBS += /usr/local/lib/libOSGFileIO.dylib
#LIBS += /usr/local/lib/libOSGCluster.dylib
#LIBS += /usr/local/lib/libOSGGroup.dylib
#LIBS += /usr/local/lib/libOSGContribBackgroundLoader.dylib
##LIBS += /usr/local/lib/libOSGImageFileIO.dylib
#LIBS += /usr/local/lib/libOSGContribCSM.dylib
#LIBS += /usr/local/lib/libOSGState.dylib
#LIBS += /usr/local/lib/libOSGContribCSMSimplePlugin.dylib
#LIBS += /usr/local/lib/old_lib/libOSGSystem.dylib
#LIBS += /usr/local/lib/libOSGContribCgFX.dylib
#LIBS += /usr/local/lib/libOSGText.dylib
#LIBS += /usr/local/lib/libOSGContribComputeBase.dylib
#LIBS += /usr/local/lib/libOSGUtil.dylib
#LIBS += /usr/local/lib/libOSGContribGUI.dylib
#LIBS += /usr/local/lib/libOSGWindow.dylib
#LIBS += /usr/local/lib/libOSGContribPLY.dylib
#LIBS += /usr/local/lib/libOSGWindowCarbon.dylib
#LIBS += /usr/local/lib/libOSGContribTrapezoidalShadowMaps.dylib
#LIBS += /usr/local/lib/libOSGWindowCocoa.dylib
#LIBS += /usr/local/lib/libOSGDrawable.dylib
#LIBS += /usr/local/lib/libOSGWindowCoreGL.dylib
#LIBS += /usr/local/lib/libOSGDynamics.dylib
#LIBS += /usr/local/lib/libOSGWindowGLUT.dylib#

#OPEN CV LIBS
LIBS += /opt/local/lib/libopencv_calib3d.dylib
LIBS += /opt/local/lib/libopencv_flann.dylib
LIBS += /opt/local/lib/libopencv_legacy.dylib
LIBS += /opt/local/lib/libopencv_contrib.dylib
LIBS += /opt/local/lib/libopencv_gpu.dylib
LIBS += /opt/local/lib/libopencv_ml.dylib
LIBS += /opt/local/lib/libopencv_core.dylib
LIBS += /opt/local/lib/libopencv_highgui.dylib
LIBS += /opt/local/lib/libopencv_objdetect.dylib
LIBS += /opt/local/lib/libopencv_features2d.dylib
LIBS += /opt/local/lib/libopencv_imgproc.dylib
LIBS += /opt/local/lib/libopencv_video.dylib


TARGET = track
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += main.cpp
