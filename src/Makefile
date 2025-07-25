all:
	g++ main.cpp -o main -std=c++20 -O2 -Wall

sanitize:
	g++ main.cpp -o main -std=c++20 -fsanitize=address,undefined

run:
	# ./main ../handout/sample/sample.data
	./main ../handout/testcases/array_test2.data

.PHONY: all run
