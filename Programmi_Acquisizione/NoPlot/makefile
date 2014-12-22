# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-lboost_system -lboost_thread -std=c++11 -march=native  `pkg-config --cflags --libs opencv` `root-config --cflags`  `root-config --glibs` -O2

all: 
	$(CC) $(CFLAGS) multithread_nopot_mutex.cpp -o final_multithread_no_plot_mutex.out
