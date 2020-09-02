#
# Date:      2013/06/10 15:21
# Author:    Jan Faigl
#

#CXX:=ccache $(CXX)

uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

CXXFLAGS:=$(call uniq,$(CXXFLAGS))

OBJS_W_DIRS:=$(addprefix $(OBJ_DIR)/,$(OBJS))
OBJS_VNS_W_DIRS:=$(addprefix $(OBJ_DIR)/,$(OBJS_VNS))
OBJS_ILP_W_DIRS:=$(addprefix $(OBJ_DIR)/,$(OBJS_ILP))
OBJS_SUBDIRS:=$(addprefix $(OBJ_DIR)/,$(SUBDIRS))
OBJS_SUBDIRS_ALL:=$(addsuffix /*.o,$(OBJS_SUBDIRS))

all: $(TARGETS)

$(OBJS): %.o: %.cpp
	$(info compiling OBJS $@)
	$(CXX) -c $< $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@
	
$(OBJS_VNS): %.o: %.cpp
	$(info compiling OBJS_VNS)
	$(CXX) -c $< -DSOP_VNS $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@
	
$(OBJS_ILP): %.o: %.cpp
	$(info compiling OBJS_ILP)
	$(CXX) -c $< -DSOP_ILP $(CPLEX_INCLUDE) $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@

sop_vns: create_directories $(OBJS) $(OBJS_VNS) 
	$(info compiling target sop_vns)
	$(CXX) -c sop.cpp -DSOP_VNS $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@.o
	$(CXX) -o $@ $(OBJ_DIR)/$@.o $(OBJS_W_DIRS) $(OBJS_VNS_W_DIRS) $(OBJS_SUBDIRS_ALL) $(LDFLAGS)
    
sop_ilp: create_directories $(OBJS) $(OBJS_ILP) 
	$(info compiling target sop_ilp)
	$(CXX) -c sop.cpp -DSOP_ILP $(CPLEX_INCLUDE) $(CXXFLAGS) $(CPPFLAGS) -o $(OBJ_DIR)/$@.o
	$(CXX) -o $@ $(OBJ_DIR)/$@.o $(OBJS_W_DIRS) $(OBJS_ILP_W_DIRS) $(OBJS_SUBDIRS_ALL) $(LDFLAGS) $(CPLEX_LIBS) 

create_directories:
	$(info create dircetory $(OBJ_DIR))
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/comrob_lite

clean:
	$(info cleaning)
	$(RM) $(TARGETS)
	$(RM) -r $(OBJ_DIR)/*


