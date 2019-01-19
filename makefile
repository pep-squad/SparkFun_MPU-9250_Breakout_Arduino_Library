all:
	g++ src/main.cpp src/MPU9250.cpp -Iinclude/ -lwiringPi -Wall -Wextra -Werror -std=c++14

clean:
	rm -f bin/* a.out
