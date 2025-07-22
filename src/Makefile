all:
	g++ main.cpp -o main -std=c++20 -O2

sanitize:
	g++ main.cpp -o main -std=c++20 -fsanitize=address,undefined

run:
	./main ../handout/sample/sample.data

.PHONY: all run
