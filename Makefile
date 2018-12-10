all: modelksi.exe
modelksi.exe: modelksi.cc binomialHeap.h skewHeap.h mark_interest.h \
  HeapStat.h TreeBackTrack.h \
  lexpattern.h\
  cgnode.h
	g++   -g  -std=c++11 modelksi.cc -I ./util -o $@ -Wall

test_mark_interest.exe: test_mark_interest.cc mark_interest.h 
	g++   -g  -std=c++11 test_mark_interest.cc -I ./util -o $@ -Wall 

