CXX = clang++
LD = clang++
ALL_CXXFLAGS = -Wall -flto -MMD -O3 $(CFLAGS)
ALL_LDFLAGS = -lboost_system-mt -lboost_thread-mt $(LDFLAGS)

TARGET = rdk-bldc
OBJECTS = main.cc rdk-bldc.cc

$(TARGET): $(OBJECTS)
	$(LD) $(ALL_CXXFLAGS) $(ALL_LDFLAGS) -o $@ $^

%.cc.o: %.cc
	$(CXX) $(ALL_CXXFLAGS) -c -o $@ $^

#-include $(wildcard *.d)
# vim: set noet:
