#compiler

CXX= g++
LDFLAGS=  -larmadillo -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options
LIBS += -L/usr/lib/x86_64-linux-gnu 

all:
	$(CXX) gen.cpp ../../utils/pca.cpp ../../utils/utils.cpp $(LDFLAGS) -o g -std=c++11 #-Wall 




