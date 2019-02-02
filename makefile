all: main

main: bin/9DOF.o bin/MPU9250.o bin/quaternionFilters.o
	g++ src/main.cpp bin/9DOF.o bin/MPU9250.o bin/quaternionFilters.o -Iinclude/ -lwiringPi -Wall -Wextra -Werror -std=c++14 -o bin/main

bin/9DOF.o: src/9DOF.cpp bin/MPU9250.o bin/quaternionFilters.o
	g++ src/9DOF.cpp bin/MPU9250.o bin/quaternionFilters.o -Iinclude/ -lwiringPi -Wall -Wextra -Werror -std=c++14 -c -o bin/9DOF.o

bin/MPU9250.o: src/MPU9250.cpp bin/quaternionFilters.o
	g++ src/MPU9250.cpp bin/quaternionFilters.o -Iinclude/ -lwiringPi -Wall -Wextra -Werror -std=c++14 -c -o bin/MPU9250.o

bin/quaternionFilters.o: src/quaternionFilters.cpp
	g++ src/quaternionFilters.cpp --Wall -Wextra -Werror -std=c++14 -c -o bin/quaternionFilters.o -Iinclude/ -lwiringPi

clean:
	rm -f bin/* a.out
