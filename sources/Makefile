
# Date:      2018/06/11 11:55
# Author:    Robert Pěnička
#

CPLEX_ROOT_DIR=/opt/cplex-12.9
CXXFLAGS+=-DIL_STD -I$(CPLEX_ROOT_DIR)/concert/include -I$(CPLEX_ROOT_DIR)/cplex/include -I$(CPLEX_ROOT_DIR)/opl/include/
LDFLAGS+=-DIL_STD -L$(CPLEX_ROOT_DIR)/cplex/lib/x86-64_linux/static_pic/ \
		  -L$(CPLEX_ROOT_DIR)/concert/lib/x86-64_linux/static_pic/ -lilocplex -lconcert -lcplex -lm -lpthread -lgmp -lz -ldl

include Mk/libs.mk
include Mk/flags.mk

OBJS=coords.o math_common.o heuristic_types.o dataset_loader_op.o dataset_loader_sop.o convexhull.o sop_loader.o
OBJS_VNS=vns_sop_path.o vns_sop_solver.o 
OBJS_ILP=ilp_sop_definition.o ilp_sop_solver.o

TARGETS=sop_vns sop_ilp
#TARGETS=sop_ilp
#TARGETS=sop_vns 

OBJ_DIR=obj

include Mk/comrob.mk
