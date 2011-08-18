#-------------------------------------------------
#
# Project created by QtCreator 2011-08-13T12:01:24
#
#-------------------------------------------------

QT       += core

QT       -= gui

#ADDING GLUT FRAMEWORK
QMAKE_LFLAGS += -F/System/Library/Frameworks/
LIBS += -framework GLUT

#ADDING OSG LIBS
LIBS += /usr/local/lib/libOSGEffectGroups.dylib
LIBS += /usr/local/lib/libOSGBase.dylib
LIBS += /usr/local/lib/libOSGFileIO.dylib
LIBS += /usr/local/lib/libOSGCluster.dylib
LIBS += /usr/local/lib/libOSGGroup.dylib
LIBS += /usr/local/lib/libOSGContribBackgroundLoader.dylib
LIBS += /usr/local/lib/libOSGImageFileIO.dylib
LIBS += /usr/local/lib/libOSGContribCSM.dylib
LIBS += /usr/local/lib/libOSGState.dylib
LIBS += /usr/local/lib/libOSGContribCSMSimplePlugin.dylib
LIBS += /usr/local/lib/libOSGSystem.dylib
LIBS += /usr/local/lib/libOSGContribCgFX.dylib
LIBS += /usr/local/lib/libOSGText.dylib
LIBS += /usr/local/lib/libOSGContribComputeBase.dylib
LIBS += /usr/local/lib/libOSGUtil.dylib
LIBS += /usr/local/lib/libOSGContribGUI.dylib
LIBS += /usr/local/lib/libOSGWindow.dylib
LIBS += /usr/local/lib/libOSGContribPLY.dylib
LIBS += /usr/local/lib/libOSGWindowCarbon.dylib
LIBS += /usr/local/lib/libOSGContribTrapezoidalShadowMaps.dylib
LIBS += /usr/local/lib/libOSGWindowCocoa.dylib
LIBS += /usr/local/lib/libOSGDrawable.dylib
LIBS += /usr/local/lib/libOSGWindowCoreGL.dylib
LIBS += /usr/local/lib/libOSGDynamics.dylib
LIBS += /usr/local/lib/libOSGWindowGLUT.dylib

TARGET = track
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += main.cpp
