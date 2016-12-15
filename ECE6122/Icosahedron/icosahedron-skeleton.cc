// Draw an Icosahedron
// ECE4893/8893 Project 4
// Assia Benbihi

#include <iostream>
#include <math.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <GL/gl.h>
#include <GL/glu.h>

using namespace std;

#define NFACE 20
#define NVERTEX 12

#define X .525731112119133606 
#define Z .850650808352039932

// These are the 12 vertices for the icosahedron
static GLfloat vdata[NVERTEX][3] = {    
   {-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z},    
   {0.0, Z, X}, {0.0, Z, -X}, {0.0, -Z, X}, {0.0, -Z, -X},    
   {Z, X, 0.0}, {-Z, X, 0.0}, {Z, -X, 0.0}, {-Z, -X, 0.0} 
};

// These are the 20 faces.  Each of the three entries for each 
// vertex gives the 3 vertices that make the face.
static GLint tindices[NFACE][3] = { 
   {0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},    
   {8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},    
   {7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6}, 
   {6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11} };

int testNumber; // Global variable indicating which test number is desired


void init(){
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_FLAT);
}


void drawTriangle(){
   GLuint vertexbuffer; 
   glGenBuffers(1, &vertexbuffer); // Generate 1 buffer
   glBindBuffer(GL_ARRAY_BUFFER);
   glBufferData(GL_ARRAY_BUFFER, sizeof(v_data), v_data, GL_STATIC_DRAW);
}

void display(void){
    static int pass;
    std::cout << "Displaying pass" << ++pass << std::endl;
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity(); // clear the matrix
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    glTranslatef(250, 250, 0);
    glScalef(100.0, 100.0, 0);
    drawTriangle();
    glutSwapBuffers();

}

void timer(int) {
    glutPostRedisplay();
    glutTimerFunc(1000.0 / updateRate, timer, 0);
}


int main(int argc, char** argv)
{
    /*
       if (argc < 2)
       {
       std::cout << "Usage: icosahedron testnumber" << endl;
       exit(1);
       }
       */
    // Set the global test number
    testNumber = atol(argv[1]);
    
    // init
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(500,500);
    glutInitWindowPosition(100,100);
    glutCreateWindow("Triangle");
    init();

    glutDisplayFunc(display);
    glutTimerFunc(1000.0/updateRate, timer, 0);
    glutMainLoop();

    // Initialize glut  and create your window here
    // Set your glut callbacks here
    // Enter the glut main loop here
    return 0;
}




// Test cases.  Fill in your code for each test case
void Test1()
{
}

void Test2()
{
}

void Test3()
{
}

void Test4()
{
}

void Test5(int depth)
{
}

void Test6(int depth)
{
}

