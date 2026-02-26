TARGET = Kali

LIBDIRS = lib/libDaisy lib/DaisySP

CC = gcc
CFLAGS = -I$(LIBDIRS)
LDFLAGS = 

SOURCES = main.c
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET)
