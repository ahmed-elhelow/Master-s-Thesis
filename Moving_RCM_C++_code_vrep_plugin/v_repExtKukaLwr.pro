QT -= core
QT -= gui

TARGET = v_repExtKukaLwr
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
DEFINES += QT_DEPRECATED_WARNINGS
CONFIG += shared
CONFIG += c++11
INCLUDEPATH += "../include"
INCLUDEPATH += "./include"
#INCLUDEPATH += ${EIGEN3_INCLUDE_DIR}
#INCLUDEPATH += "/usr/include/eigen3/Eigen"
INCLUDEPATH += ${Boost_INCLUDE_DIR}
INCLUDEPATH += /usr/include/python2.7
INCLUDEPATH += /usr/include/
INCLUDEPATH += /usr/include/python2.7/numpy
INCLUDEPATH += /usr/lib/python2.7/dist-packages/numpy/core/include/numpy
DEPENDPATH += /usr/include/python2.7

LIBS += ${Boost_LIBRARY_DIR}
#LIBS += -L /usr/local/lib/python2.7 -lpython2.7
LIBS += -L /usr/lib/python2.7 -lpython2.7

#find_package(Eigen3 3.3 REQUIRED NO_MODULE )
#find_package(PythonLibs 2.7)
#find_package(Boost COMPONENTS system filesystem REQUIRED)
#target_link_libraries( main Eigen3::Eigen ${Boost_LIBRARIES})
#target_link_libraries( main ${PYTHON_LIBRARIES})


*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

win32 {
    DEFINES += WIN_VREP
}

macx {
    DEFINES += MAC_VREP
}

unix:!macx {
    DEFINES += LIN_VREP
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

HEADERS += \
    ./include/v_repExtKukaLwr.h \
    ./include/common.h \
    ./include/forces.h \
    ./include/kukaDynRCM.h \
    ./include/matplotlibcpp.h \
    ../include/scriptFunctionData.h \
    ../include/scriptFunctionDataItem.h \
    ../include/v_repLib.h

SOURCES += \
    ./source/v_repExtKukaLwr.cpp \
    ./source/main.cpp \
    ./source/externalForcesEstimation.cpp \
    ./source/others.cpp \
    ../common/scriptFunctionData.cpp \
    ../common/scriptFunctionDataItem.cpp \
    ../common/v_repLib.cpp

