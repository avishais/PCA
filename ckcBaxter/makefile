#compiler
OMPL_DIR = /usr/local
INC_CLASSES = ./proj_classes/
INC_PLANNERS = ../planners/
INC_VALIDITY = ./validity_checkers/
INC_STATEVALIDITY = ../StateValidCheckers/
INC_RUN = ./run/
INC_UTILS = ../utils/

EIGEN_DIR = /home/avishai/Documents/eigen

KDL_DIR = /usr/local

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${PQP_DIR}/include $(GL_INCPATH) -I${KDL_DIR}/include -I$(EIGEN_DIR) -I$(INC_CLASSES) -I$(INC_VALIDITY)
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -lPQP -lm $(GL_LIBS) -L${KDL_DIR}/lib -larmadillo -lorocos-kdl

CPPPCA = ${INC_UTILS}pca.cpp ${INC_UTILS}utils.cpp

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp

CPP = ${INC_CLASSES}kdl_class.cpp ${INC_STATEVALIDITY}StateValidityChecker.cpp

CPP_P = ${INC_RUN}plan.cpp ${INC_PLANNERS}CBiRRT.cpp ${INC_PLANNERS}RRT.cpp ${INC_PLANNERS}LazyRRT.cpp #${INC_PLANNERS}SBL.cpp 

all:
	$(CXX) ${CPP_P} ${CPP} ${CPPPQP} ${CPPPCA} -o pln $(CXXFLAGS) $(LDFLAGS) -std=c++11 