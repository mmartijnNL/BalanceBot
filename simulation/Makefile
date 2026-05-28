CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -I..
LDFLAGS = -lm

all: sim

sim: main.o BalanceBot.o
	$(CC) $(CFLAGS) -o sim main.o BalanceBot.o $(LDFLAGS)

main.o: main.c ../BalanceBot.h
	$(CC) $(CFLAGS) -c main.c

BalanceBot.o: ../BalanceBot.c ../BalanceBot.h
	$(CC) $(CFLAGS) -c ../BalanceBot.c -o BalanceBot.o

clean:
	rm -f *.o sim sim_stabilize.csv
