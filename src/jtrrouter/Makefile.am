if CHECK_MEMORY_LEAKS
MEM_LIBS  = ../foreign/nvwa/libnvwa.a
endif

bin_PROGRAMS = jtrrouter

jtrrouter_SOURCES = jtrrouter_main.cpp \
ROJTREdge.cpp ROJTREdge.h \
ROJTREdgeBuilder.cpp ROJTREdgeBuilder.h \
ROJTRRouter.cpp ROJTRRouter.h ROJTRTurnDefLoader.cpp ROJTRTurnDefLoader.h \
ROJTRFrame.cpp ROJTRFrame.h


jtrrouter_LDFLAGS = $(XERCES_LDFLAGS)

jtrrouter_LDADD   = ../router/librouter.a \
../utils/geom/libgeom.a \
../utils/options/liboptions.a \
../utils/common/libcommon.a \
../utils/xml/libxml.a \
../utils/importio/libimportio.a \
../utils/iodevices/libiodevices.a \
../foreign/tcpip/libtcpip.a \
../scenload/libscenload.a \
$(MEM_LIBS) \
-l$(LIB_XERCES)

