CXX = g++
CXXFLAGS = -std=c++17 -O2

all: sim

sim: main.o BalanceBotController.o
	$(CXX) $(CXXFLAGS) -o sim main.o BalanceBotController.o

main.o: main.cpp BalanceBotController.h
	$(CXX) $(CXXFLAGS) -c main.cpp

BalanceBotController.o: BalanceBotController.cpp BalanceBotController.h
	$(CXX) $(CXXFLAGS) -c BalanceBotController.cpp

clean:
	rm -f *.o sim sim_stabilize.csv
