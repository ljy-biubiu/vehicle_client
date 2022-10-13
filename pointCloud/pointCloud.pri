SOURCES += \
    $$PWD/qvtkopenglwindow.cpp \

HEADERS += \
    $$PWD/qvtkopenglwindow.h \

INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /usr/local/include/vtk-8.2
LIBS += /usr/local/lib/libvtk*.so

INCLUDEPATH += /usr/local/include/pcl-1.11
LIBS += /usr/local/lib/libpcl_*.so

INCLUDEPATH += $$PWD/.
