# -----------------------------------------------------------------------------
#
# Motion control makefile
#
# Original Author: Stephen Fegan - 2007-10-19
# $Author: sfegan $
# $Date: 2008/04/17 17:17:31 $
# $Revision: 1.8 $
# $Tag$
#
# -----------------------------------------------------------------------------

include Makefile.common

TARGETS = id dx ip dx_hit scan abs reset calib_point_test
SUBDIR = MotionControl

CXXFLAGS += -I. $(addprefix -I,$(SUBDIR))
LDFLAGS += $(addprefix -L,$(SUBDIR))
LIBS += -lMotionControl

all: $(TARGETS)

# ALL HEADERS
HEADERS = 

# All SOURCES
SOURCES = id.cpp
ALLSRCS = $(SOURCES)

# All OBJECTS
OBJECTS = $(SOURCES:.cpp=.o)
ALLOBJS = $(OBJECTS)

$(TARGETS): MotionControl/libMotionControl.a
MotionControl/libMotionControl.a: $(SUBDIR)

calib_point_test: calib_point_test.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

testcode: testcode.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

id: id.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

dx: dx.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

abs: abs.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

dx_hit: dx_hit.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

ip: ip.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

scan: scan.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

reset: reset.o
	$(CXX) $(LDFLAGS) -o $@ $< $(LIBS)

clean: $(addsuffix -clean,$(SUBDIR))

.PHONY: $(SUBDIR) $(addsuffix -clean,$(SUBDIR))

$(SUBDIR):
	$(MAKE) -C $@ $(STATICFLAG)

$(addsuffix -clean,$(SUBDIR)):
	$(MAKE) -C $(@:-clean=) clean

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	$(RM) -f $(TARGETS) testcode \
		core core.* *~ *.o 
