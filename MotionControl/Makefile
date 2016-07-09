# -----------------------------------------------------------------------------
#
# Motion control makefile
#
# Original Author: Stephen Fegan - 2007-10-19
# $Author: sfegan $
# $Date: 2008/04/14 06:13:07 $
# $Revision: 1.2 $
# $Tag$
#
# -----------------------------------------------------------------------------

include ../Makefile.common

TARGETS = libMotionControl.a

CXXFLAGS += -I.

all: $(TARGETS)

# ALL HEADERS
HEADERS = Exception.hpp Debug.hpp DataStream.hpp VSDataConverter.hpp \
	VSOptions.hpp VSAssert.hpp \
	ESPProtocol.hpp

# All SOURCES
SOURCES = $(HEADERS:.hpp=.cpp)
ALLSRCS = $(SOURCES)

# All OBJECTS
OBJECTS = $(SOURCES:.cpp=.o)
ALLOBJS = $(OBJECTS)

libMotionControl.a: $(ALLOBJS)
	$(AR) r $@ $^

$(HEADERS:.hpp=.o): %.o: %.hpp

$(HEADERS:.hpp=.o): %.o: %.cpp

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	$(RM) -f $(TARGETS) \
		core core.* *~ *.o 
