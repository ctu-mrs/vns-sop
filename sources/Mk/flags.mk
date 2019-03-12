CXX:=ccache $(CXX)

CXXFLAGS+=-std=c++0x

CPPFLAGS+=$(LOCAL_CFLAGS)
LDFLAGS+=$(LOCAL_LDFLAGS)

CPPFLAGS+=$(IMR-H_CFLAGS) $(BOOST_CFLAGS) $(CAIRO_CFLAGS) $(LOG4CXX_CPPFLAGS)
LDFLAGS+=$(CRL-ALGORITHM) $(CRL-GUI_LDFLAGS) $(CRL-LDFLAGS) $(CAIRO_LDFLAGS) $(BOOST_LDFLAGS) $(LOG4CXX_LDFLAGS) $(OPENDUBINS_LDFLAGS)

CXXFLAGS+=-O2 -Wno-ignored-attributes 
#-Wall
#CXXFLAGS+= -march=native
#CXXFLAGS+=-std=c++11
