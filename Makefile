CC = g++
CXXFLAGS = -Ofast -I/usr/include/eigen3/
LFLAGS = -lsfml-system -lsfml-window -lsfml-graphics
BIN = ressorts.exe

$(BIN) : obj/main.o
	$(CC) $(LFLAGS) $^ -o $(BIN)

obj/main.o : main.cpp
	$(CC) $(CXXFLAGS) -c $^ -o $@

clean : 
	rm -rf $(BIN)
	rm -rf obj/*.o
