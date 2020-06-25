NAME=cbgb

CC=gcc
CFLAGS=-lSDL2 -O2
LIBS=-I/usr/local/include -L/usr/local/lib

SDIR=./src/

$(NAME): $(SDIR)*.c
	$(CC) $(CFLAGS) $(SDIR)*.c $(LIBS) -o $(NAME)

clean:
	rm -f *.o
	rm -f $(NAME)