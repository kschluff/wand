# This is a quirk of the pkg-config install on my Mac.  You can
# probably just set this to 'pkg-config'
PKG_CONFIG = /usr/local/bin/pkg-config

CFLAGS = -O2 -g -Wall `$(PKG_CONFIG) opencv --cflags`
LIBS = `$(PKG_CONFIG) opencv --libs` 
SRCS = main.cpp kalman.cpp particle.cpp
HEADERS =  kalman.h particle.h
APP = wand

$(APP): $(SRCS) $(HEADERS)
	g++ $(CFLAGS) -o $(APP) $(LIBS) $(SRCS)