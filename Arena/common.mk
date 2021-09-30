ARCH_TYPE = $(shell dpkg --print-architecture)

ifeq ($(ARCH_TYPE), amd64)

LDFLAGS = -L../../../lib64 \
          -L../../../GenICam/library/lib/Linux64_x64 \
          -L../../../ffmpeg
          
GENICAMLIBS = -lGCBase_gcc54_v3_3_LUCID \
              -lGenApi_gcc54_v3_3_LUCID \
              -lLog_gcc54_v3_3_LUCID \
              -llog4cpp_gcc54_v3_3_LUCID \
              -lMathParser_gcc54_v3_3_LUCID \
              -lNodeMapData_gcc54_v3_3_LUCID \
              -lXmlParser_gcc54_v3_3_LUCID

OUTDIR = ../../../OutputDirectory/Linux/x64Release/

else ifeq ($(ARCH_TYPE), armhf)

LDFLAGS = -L../../../lib \
          -L../../../GenICam/library/lib/Linux32_ARMhf \
          -L../../../ffmpeg

GENICAMLIBS = -lGCBase_gcc540_v3_3_LUCID \
              -lGenApi_gcc540_v3_3_LUCID \
              -lLog_gcc540_v3_3_LUCID \
              -llog4cpp_gcc540_v3_3_LUCID \
              -lMathParser_gcc540_v3_3_LUCID \
              -lNodeMapData_gcc540_v3_3_LUCID \
              -lXmlParser_gcc540_v3_3_LUCID


OUTDIR = ../../../OutputDirectory/armhf/x32Release/

else ifeq ($(ARCH_TYPE), arm64)

LDFLAGS = -L../../../lib \
          -L../../../GenICam/library/lib/Linux64_ARM \
          -L../../../ffmpeg

GENICAMLIBS = -lGCBase_gcc54_v3_3_LUCID \
              -lGenApi_gcc54_v3_3_LUCID \
              -lLog_gcc54_v3_3_LUCID \
              -llog4cpp_gcc54_v3_3_LUCID \
              -lMathParser_gcc54_v3_3_LUCID \
              -lNodeMapData_gcc54_v3_3_LUCID \
              -lXmlParser_gcc54_v3_3_LUCID


OUTDIR = ../../../OutputDirectory/arm64/x64Release/
endif

CC=g++

INCLUDE= -I../../../include/Arena \
         -I../../../include/Save \
         -I../../../include/GenTL \
         -I../../../GenICam/library/CPP/include \
         -I/usr/include/pcl-1.9 \
         -I/usr/include/eigen3 \
         -I/usr/include/vtk-6.3 \

CFLAGS=-Wall -g -O2 -std=c++11 -Wno-unknown-pragmas


FFMPEGLIBS = -lavcodec \
             -lavformat \
             -lavutil \
             -lswresample

LIBS = -larena -lsave -lgentl $(GENICAMLIBS) $(FFMPEGLIBS) -lpthread -llucidlog
LIBS += `pkg-config opencv --libs --cflags` \
        `pkg-config pcl_visualization-1.9 --libs --cflags` \
        `pkg-config pcl_io-1.9 --libs --cflags` \
        `pkg-config pcl_common-1.9 --libs --cflags` \
        `pkg-config pcl_filters-1.9 --libs --cflags` \
        -lvtkRenderingQt-6.3 \
        -lvtkCommonDataModel-6.3 \
        -lvtkCommonMath-6.3 \
        -lvtkCommonCore-6.3 \
        -lvtkViewsCore-6.3 \
        -lvtkGUISupportQt-6.3 \
        -lvtkRenderingCore-6.3 \
        -lboost_system
RM = rm -f

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:%.cpp=%.o)
DEPS = $(OBJS:%.o=%.d)

.PHONY: all
all: ${TARGET}

${TARGET}: ${OBJS}
	${CC} ${LDFLAGS} $^ -o $@ $(LIBS)
	-mkdir -p $(OUTDIR)
	-cp $(TARGET) $(OUTDIR)

%.o: %.cpp ${SRCS}
	${CC} ${INCLUDE}  ${LDFLAGS} -o $@ $< -c ${CFLAGS}

${DEPS}: %.cpp
	${CC} ${CLAGS} ${INCLUDE} -MM $< >$@

-include $(OBJS:.o=.d)

.PHONY: clean
clean:
	-${RM} ${TARGET} ${OBJS} ${DEPS}
