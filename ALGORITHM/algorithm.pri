SOURCES += \
    $$PWD/src/algorithm.cpp \
    $$PWD/src/algorithm_interface.cpp \
    $$PWD/src/GroundMaterialDetection.cpp \
    $$PWD/src/BaseDetection.cpp \
    $$PWD/src/ShipDetection.cpp \
    $$PWD/ROIFilter/hdmapRoiFilter.cpp \
    $$PWD/ROIFilter/polygonScanConverter.cpp \
    $$PWD/ROIFilter/ROIFilter.cpp \
    $$PWD/UtilMethod/dbscan.cpp \
    $$PWD/UtilMethod/LShapedFitting.cpp \
    $$PWD/UtilMethod/PlaneMethod.cpp \
    $$PWD/UtilMethod/Util.cpp \
    $$PWD/UtilMethod/VoxelGrid.cpp \
    $$PWD/UtilMethod/RTMethod.cpp \
    $$PWD/CSF/c2cdist.cpp \
    $$PWD/CSF/Constraint.cpp \
    $$PWD/CSF/Particle.cpp \
    $$PWD/CSF/Rasterization.cpp \
    $$PWD/CSF/Cloth.cpp \
    $$PWD/CSF/CSF.cpp \
    $$PWD/CSF/point_cloud.cpp \
    $$PWD/CSF/XYZReader.cpp

HEADERS += \
    $$PWD/include/algorithm.h \
    $$PWD/include/algorithm_interface.h \
    $$PWD/include/BaseDetection.h \
    $$PWD/include/GroundMaterialDetection.h \
    $$PWD/include/ShipDetection.h \
    $$PWD/ROIFilter/hdmapRoiFilter.h \
    $$PWD/ROIFilter/polygonScanConverter.h \
    $$PWD/ROIFilter/ROIFilter.h \
    $$PWD/UtilMethod/dbscan.h \
    $$PWD/UtilMethod/LShapedFitting.h \
    $$PWD/UtilMethod/PlaneMethod.h \
    $$PWD/UtilMethod/Util.h \
    $$PWD/UtilMethod/VoxelGrid.h \
    $$PWD/UtilMethod/RTMethod.h \
    $$PWD/CSF/c2cdist.h \
    $$PWD/CSF/Constraint.h \
    $$PWD/CSF/Particle.h \
    $$PWD/CSF/Rasterization.h \
    $$PWD/CSF/Cloth.h \
    $$PWD/CSF/CSF.h \
    $$PWD/CSF/point_cloud.h \
    $$PWD/CSF/XYZReader.h \
    $$PWD/CSF/Vec3.h


INCLUDEPATH += $$PWD/src/.
INCLUDEPATH += $$PWD/include/.
INCLUDEPATH += $$PWD/ROIFilter/.
INCLUDEPATH += $$PWD/UtilMethod/.
INCLUDEPATH += $$PWD/CSF/.
