
main: Main.cpp Camera.cpp Mesh.cpp LaplacianEditing.cpp
# ubuntu
# 	g++ Main.cpp Camera.cpp Mesh.cpp LaplacianEditing.cpp -o main -lGL -std=c++11 -lGLU -lglut -lGLEW -I .
	g++ Main.cpp Camera.cpp Mesh.cpp LaplacianEditing.cpp -o main -framework GLUT -framework OpenGL -I .
