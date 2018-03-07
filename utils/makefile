#compiler

CXX= g++
LDFLAGS=  -larmadillo -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options
LIBS += -L/usr/lib/x86_64-linux-gnu 

EIGEN_DIR = /home/avishai/Documents/eigen

all:
	#Test kdl
	$(CXX) pca_example.cpp pca.cpp utils.cpp -o tpca $(LDFLAGS) -I$(EIGEN_DIR) -std=c++11 #-Wall 




