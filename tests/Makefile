CXX = g++

CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I. -Ifake_arduino -I..

TARGET = run_ino_tests

all: $(TARGET)

$(TARGET): test_lolin32lite_ino.o FakeArduino.o
	$(CXX) $(CXXFLAGS) -o $(TARGET) test_lolin32lite_ino.o FakeArduino.o

test_lolin32lite_ino.o: test_lolin32lite_ino.cpp ../BalanceBotLolin32lite/BalanceBotLolin32lite.ino ../BalanceBot.h
	$(CXX) $(CXXFLAGS) -c test_lolin32lite_ino.cpp -o test_lolin32lite_ino.o

FakeArduino.o: fake_arduino/FakeArduino.cpp fake_arduino/Arduino.h fake_arduino/Wire.h
	$(CXX) $(CXXFLAGS) -c fake_arduino/FakeArduino.cpp -o FakeArduino.o

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f *.o $(TARGET)
