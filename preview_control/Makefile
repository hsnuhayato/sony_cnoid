# -*- Makefile -*-

include ../Make.rules

CXXFLAGS:=$(CXXFLAGS) -I../

OBJS    = PreviewControl.o
HEADERS = $(OBJS:.o=.h)
SRCS    = $(HEADERS:.h=.cpp)


.SUFFIXES : .cpp .o

all: $(OBJS)

.cpp.o: $(HEADERS)
	rm -f $@
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.o
	rm -f *~

install:
	-$(MKDIR) $(INSTALL_DIR)/bin
	-$(MKDIR) $(INSTALL_DIR)/include
	-$(INSTALLDATA) $(OBJS) $(INSTALL_DIR)/bin
	-$(INSTALLDATA) $(HEADERS) $(INSTALL_DIR)/include
