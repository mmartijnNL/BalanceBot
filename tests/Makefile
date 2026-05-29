CXX = g++
CC = gcc

CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -I. -Ifake_arduino -I..
CFLAGS = -std=c11 -O2 -Wall -Wextra -I..

TARGET = run_ino_tests

all: $(TARGET)

$(TARGET): test_pico_ino.o FakeArduino.o BalanceBot.o
	$(CXX) $(CXXFLAGS) -o $(TARGET) test_pico_ino.o FakeArduino.o BalanceBot.o

test_pico_ino.o: test_pico_ino.cpp ../BalanceBotPico/BalanceBotPico.ino ../BalanceBot.h
	$(CXX) $(CXXFLAGS) -c test_pico_ino.cpp -o test_pico_ino.o

FakeArduino.o: fake_arduino/FakeArduino.cpp fake_arduino/Arduino.h fake_arduino/Wire.h
	$(CXX) $(CXXFLAGS) -c fake_arduino/FakeArduino.cpp -o FakeArduino.o

BalanceBot.o: ../BalanceBot.c ../BalanceBot.h
	$(CC) $(CFLAGS) -c ../BalanceBot.c -o BalanceBot.o

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f *.o $(TARGET)
