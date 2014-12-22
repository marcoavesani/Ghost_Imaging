# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-Wall  `pkg-config --cflags --libs opencv` `root-config --cflags`  `root-config --glibs` -O2 -march=native

all: 
	$(CC) $(CFLAGS) cvlib2_root.cpp -o cvlib2_root.out
