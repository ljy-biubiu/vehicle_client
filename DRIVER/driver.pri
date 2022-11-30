SOURCES += \
    $$PWD/CAMERA/src/ptz.cpp \
    $$PWD/POINTCLOUD/src/lidar_drive_interface.cpp \
    $$PWD/POINTCLOUD/src/lidardrive128s1.cpp

HEADERS += \
    $$PWD/CAMERA/include/ptz.h \
    $$PWD/CAMERA/sdk/HCInclude/incEn/DataType.h \
    $$PWD/CAMERA/sdk/HCInclude/incEn/DecodeCardSdk.h \
    $$PWD/CAMERA/sdk/HCInclude/incEn/HCNetSDK.h \
    $$PWD/CAMERA/sdk/HCInclude/incEn/LinuxPlayM4.h \
    $$PWD/POINTCLOUD/include/lidar_drive_interface.h \
    $$PWD/POINTCLOUD/include/lidardrive128s1.h

INCLUDEPATH += $$PWD/CAMERA/sdk/HCInclude/incEn/.
INCLUDEPATH += $$PWD/CAMERA/include/.
INCLUDEPATH += $$PWD/CAMERA/src/.

INCLUDEPATH += $$PWD/POINTCLOUD/include/.
INCLUDEPATH += $$PWD/POINTCLOUD/src/.




DISTFILES +=
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lAudioRender
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lcrypto
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lHCCore
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lhcnetsdk
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lhpr
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lPlayCtrl
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lSuperRender
unix:!macx: LIBS += -L$$PWD/CAMERA/sdk/lib/ -lssl
