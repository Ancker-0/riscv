all:
	g++ main.cpp -o main -std=c++20 -O2 -Wall

sanitize:
	g++ main.cpp -o main -std=c++20 -fsanitize=address,undefined

run:
	./main ../asm/lui.data
	# ./main ../handout/testcases/array_test2.data

test: all run

.PHONY: all run sanitize test
