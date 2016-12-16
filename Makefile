main: Main.cpp Camera.cpp Mesh.cpp LaplacianEditing.cpp
	g++ Main.cpp Camera.cpp Mesh.cpp LaplacianEditing.cpp -o main -lGL -std=c++11 -lGLU -lglut -lGLEW -I .
