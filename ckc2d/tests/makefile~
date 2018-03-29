#compiler
OMPL_DIR = /usr/local
INC_CLASSES = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/proj_classes/
INC_PLANNERS = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/planners/
INC_VALIDITY = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/validity_checkers/
INC_RUN = /home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/run/

EIGEN_DIR = /home/avishai/Documents/eigen
KDL_DIR = /usr/local

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${KDL_DIR}/include -I$(EIGEN_DIR) 
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -larmadillo -lorocos-kdl 
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPVRF = ${INC_VALIDITY}verification_class.cpp

CPPGDKDL = ${INC_CLASSES}kdl_class.cpp 
CPPAPC = ${INC_CLASSES}apc_class.cpp ${INC_VALIDITY}StateValidityCheckerPCS.cpp
CPPGD = ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityCheckerGD.cpp

CPP_P_PCS = ${INC_RUN}plan_PCS.cpp ${INC_PLANNERS}CBiRRT_PCS.cpp

all:
	#$(CXX) test_rbs_pcs.cpp ${CPPAPC} ${CPPVRF} -o trbs $(CXXFLAGS) $(LDFLAGS) -std=c++11
	#$(CXX) test_rbs_gd.cpp ${CPPGD} ${CPPVRF} -o trbs $(CXXFLAGS) $(LDFLAGS) -std=c++11	
	#$(CXX) test_gd_kdl.cpp ${CPPGDKDL} -o tgd $(CXXFLAGS) $(LDFLAGS) -std=c++11
	#$(CXX) test_apc.cpp ${CPPAPC} -o tapc $(CXXFLAGS) $(LDFLAGS) -std=c++11

	$(CXX) test_pcs_vary_m.cpp ${CPPAPC} ${CPPVRF} -o tm $(CXXFLAGS) $(LDFLAGS) -std=c++11




