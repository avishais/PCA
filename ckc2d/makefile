#compiler
OMPL_DIR = /usr
INC_CLASSES = ./proj_classes/
INC_PLANNERS = ../planners/
INC_VALIDITY = ./validity_checkers/
INC_STATEVALIDITY = ../StateValidCheckers/
INC_RUN = ./run/
INC_UTILS = ../utils/

EIGEN_DIR = /home/avishai/Documents/eigen
KDL_DIR = /usr/local

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/local/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${KDL_DIR}/include -I$(EIGEN_DIR) -I$(INC_CLASSES) -I$(INC_VALIDITY) -I$(INC_UTILS)
LDFLAGS=  -L${OMPL_DIR}/local/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -larmadillo -lorocos-kdl 
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPPCA = ${INC_UTILS}pca.cpp ${INC_UTILS}utils.cpp

CPPCOL = ${INC_VALIDITY}collisionDetection.cpp 
CPPGD = ${INC_CLASSES}kdl_class.cpp ${INC_STATEVALIDITY}StateValidityChecker.cpp
CPP_P = ${INC_RUN}plan.cpp ${INC_PLANNERS}RRTConnect.cpp ${INC_PLANNERS}RRT.cpp

all:
	$(CXX) ${CPP_P} ${CPPGD} ${CPPPCA} ${CPPCOL} -o pln $(CXXFLAGS) $(LDFLAGS) -DPGD -std=c++11


