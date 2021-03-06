#
# Date:      2011/07/11 17:55
# Author:    Jan Faigl
#

OPSYS=$(shell uname)

PLATFORM=$(shell uname -p)
ARCH=.$(PLATFORM)

ifeq ($(OPSYS),FreeBSD)
   BOOST_CFLAGS=-I/usr/local/include
   BOOST_LDFLAGS=-L/usr/local/lib

else
   LOG4CXX_CPPFLAGS=$(shell pkg-config --cflags liblog4cxx)
   LOG4CXX_LDFLAGS=$(shell pkg-config --libs liblog4cxx)

   CAIRO_LDFLAGS:=-L/usr/X11/lib
   CAIRO_CFLAGS:=-I/usr/X11/include
endif

CPLEX_LIBS=-L$(CPLEX_ROOT_DIR)/cplex/lib/x86-64_linux/static_pic -L$(CPLEX_ROOT_DIR)/opl/lib/x86-64_linux/static_pic -lilocplex -lconcert -lcplex -lm -lpthread 
CPLEX_INCLUDE=-DIL_STD -I$(CPLEX_ROOT_DIR)/cplex/include -I$(CPLEX_ROOT_DIR)/opl/include

BOOST_LDFLAGS+=-lboost_program_options -lboost_thread -lboost_filesystem -lboost_iostreams -lboost_system
LOG4CXX_LDFLAGS+=-llog4cxx
CAIRO_LDFLAGS+=-lcairo -pthread -lX11

LOCAL_CFLAGS=-Icomrob/include 
LOCAL_LDFLAGS=-Lcomrob/lib 

CRL-GUI_LDFLAGS=-lcrl-gui
CRL-LDFLAGS=-lcrl
CRL-ALGORITHM=-lcrl-algorithm

CAIRO_LDFLAGS+=-lcairo -pthread -lX11

