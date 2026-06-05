CXX = g++

CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I. -Ifake_arduino -I..

TARGET = run_ino_tests

all: $(TARGET)

$(TARGET): test_esp32_ino.o FakeArduino.o
	$(CXX) $(CXXFLAGS) -o $(TARGET) test_esp32_ino.o FakeArduino.o

test_esp32_ino.o: test_esp32_ino.cpp ../BalanceBotEsp32/BalanceBotEsp32.ino ../BalanceBot.h
	$(CXX) $(CXXFLAGS) -c test_esp32_ino.cpp -o test_esp32_ino.o

FakeArduino.o: fake_arduino/FakeArduino.cpp fake_arduino/Arduino.h fake_arduino/Wire.h
	$(CXX) $(CXXFLAGS) -c fake_arduino/FakeArduino.cpp -o FakeArduino.o

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f *.o $(TARGET)
