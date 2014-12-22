# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS= -Wall -std=c++11 -march=native -lboost_system -lboost_thread  -pthread `pkg-config --cflags --libs opencv` `root-config --cflags`  `root-config --glibs` -O2


all: 
	$(CC) $(CFLAGS) multithread_show_mutex.cpp -o final_multithread_show_mutex.out




