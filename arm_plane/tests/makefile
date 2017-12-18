

OMPL_DIR = /usr/local
INC_CLASSES = ../proj_classes/
INC_PLANNERS = ../planners/
INC_VALIDITY = ../validity_checkers/
INC_RUN = ../run/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

EIGEN_DIR = /home/avishai/Documents/eigen

DLIB = /home/avishai/Documents/dlib_197/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${PQP_DIR}/include $(GL_INCPATH) -I$(EIGEN_DIR) -I$(DLIB)
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -lPQP -lm $(GL_LIBS) -lpthread

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp

CPP = $(INC_CLASSES)projectC.cpp $(INC_VALIDITY)StateValidityChecker.cpp

all:
	$(CXX) test_sample_svc.cpp $(CPP) $(CPPPQP) -o tsmp $(CXXFLAGS) $(LDFLAGS) -std=c++11
	#$(CXX) test_sample.cpp $(CPP) -o tsmp $(CXXFLAGS) $(LDFLAGS) -std=c++11


