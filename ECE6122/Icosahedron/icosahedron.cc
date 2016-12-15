// Draw an Icosahedron
// ECE4893/8893 Project 4
// Assia Benbihi

#include <iostream>
#include <math.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include <map>
#include <fstream>

using namespace std;

#define NFACE 20
#define NVERTEX 12

#define X .525731112119133606 
#define Z .850650808352039932

#define DEG2RAD M_PI/180
#define UPDATE_RATE 50
#define RAD2DEG 180/M_PI


typedef std::vector<float> Vertex;
typedef std::vector<int> Face;
typedef std::pair<int, int> MapIndex;
typedef std::map< MapIndex, float> Color;

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
float updateRate = UPDATE_RATE;
float xAngle = 0.0;
float yAngle = 0.0;
int nface = NFACE;
int depth = 0;
Color color;
std::vector<Vertex> vertices;
std::vector<Face> faces;
bool isInit = false;

void Test0();
void Test1();
void Test2();
void Test3();
void Test4();
void Test5(int depth);
void Test6(int depth);

void buildFaces3();
void buildFacesDepth();

float norm(float a, float b, float c){
    return sqrt(a*a + b*b + c*c);
}

void init(){
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_DEPTH_BUFFER_BIT);
}

void drawTriangle(){
    glLineWidth(2.0);
    glLoadIdentity();

    switch(testNumber) {
        case 1: 
            {
                Test1(); 
                break;
            }
        case 2:
            {
                Test2();
                break;
            }
        case 3:
            {
                Test3();
                break;
            }
        case 4:
            {
                Test4();
                break;
            }
        case 5:
            {
                Test5(depth);
                break;
            }
        case 6:
            {
                Test6(depth);
                break;
            }
        default: 
            {
                std::cout << "This test does not exist." << std::endl;
                exit(-1);
                break;
            }
    }
}


void display(void){
    static int pass;
    //std::cout << "Displaying pass " << ++pass << std::endl;
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity(); // clear the matrix
    gluLookAt(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    glEnable(GL_DEPTH_TEST);
    drawTriangle();
    glutSwapBuffers();
}

void reshape(int w, int h){
    glViewport(0,0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void timer(int) {
    glutPostRedisplay();
    glutTimerFunc(1000.0 / updateRate, timer, 0);
}

int main(int argc, char** argv){
    if (argc < 2){
        std::cout << "Usage: icosahedron testnumber" << endl;
        exit(1);
    }

    // Set the global test number
    testNumber = atol(argv[1]);
    nface = NFACE;
    isInit = false;
    if (testNumber==2){ 
        updateRate = 10;
    }
    if((testNumber==3) || (testNumber==4)){
        buildFaces3();
        nface = NFACE*4;
    }

    if((testNumber==5)||(testNumber==6)){
        //updateRate = 10;
        depth = atoi(argv[2]);
        if(depth<=0){
            std::cout << "Error: depth must be strictly positive. Call ./icosahedron 1." << std::endl;
            exit(-1);
        }
        nface = NFACE*pow(4,depth); 
        buildFacesDepth();
    }

    //Set colors
    for(int i=0;i<nface;i++){ 
        for(int j=0;j<3;j++){
            color[MapIndex(i,j)] = ( (float) (rand() % 10 +1 ) )/10;
        }
    }

    // init
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500,500);
    glutInitWindowPosition(100,100);
    glutCreateWindow("Triangle");
    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(1000.0/updateRate, timer, 0);
    glutMainLoop();
    return 0;
}


// Test cases.  Fill in your code for each test case

void Test1() {
    glBegin(GL_TRIANGLES);
    for(int i=0;i<NFACE;i++){ // for each face
        // Boring version 
        glColor3f(color[MapIndex(i,0)], color[MapIndex(i,1)], color[MapIndex(i,2)]);
        for(int j=0;j<3;j++){ // for each face, draw 3 vertex of the face
            glVertex3f(vdata[tindices[i][j]][0], vdata[tindices[i][j]][1], vdata[tindices[i][j]][2]);
        }
    }
    glEnd();

    glBegin(GL_LINE);
    glLineWidth(2);
    glColor3f(0.0, 0.0, 0.0);
    for(int i=0;i<NFACE;i++){ // for each face
        for(int j=0;j<3;j++){ // for each of the 3 vertex of the face
            for(int k=0;k<3;k++){ // for each of the 3 coordinate of the vertex
                glVertex3f(vdata[tindices[i][j]][0], vdata[tindices[i][j]][1], vdata[tindices[i][j]][2]);
            }
        }
    }
    glEnd();
}


void Test2(){
    xAngle +=1;
    yAngle +=1;
    if(xAngle > 360.0){
        xAngle = 0.0;
    }
    if(yAngle > 360.0){
        yAngle = 0.0;
    }
    glRotatef(xAngle, 1.0, 0.0, 0.0);
    glRotatef(yAngle, 0.0, 1.0, 0.0);
    Test1();
}

void Test3(){
    int i,j;
    glBegin(GL_TRIANGLES);
    for(i=0;i<faces.size();i++){ // for each face
        glColor3f(color[MapIndex(i,0)], color[MapIndex(i,1)], color[MapIndex(i,2)]);
        for(j=0;j<3;j++){ // for each face, draw 3 vertex of the face
            glVertex3f(vertices[faces[i][j]][0], vertices[faces[i][j]][1], vertices[faces[i][j]][2]);
        }
    }
    glEnd();

    glBegin(GL_LINE);
    glLineWidth(1.0);
    glColor3f(1.0, 1.0, 1.0);
    for(int i=0;i<faces.size();i++){ // for each face
        for(int j=0;j<3;j++){ // for each of the 3 vertex of the face
            glVertex3f(vertices[faces[i][j]][0], vertices[faces[i][j]][1], vertices[faces[i][j]][2]);
        }
    }
    glEnd();
}

void Test4(){
    xAngle +=1;
    yAngle +=1;
    if(xAngle > 360.0){
        xAngle = 0.0;
    }

    if(yAngle > 360.0){
        yAngle = 0.0;
    }
    glRotatef(xAngle, 1.0, 0.0, 0.0);
    glRotatef(yAngle, 0.0, 1.0, 0.0);
    Test3();
}

void Test5(int depth){
    Test3();
}

void Test6(int depth){
    xAngle +=1;
    yAngle +=1;
    if(xAngle > 360.0){
        xAngle = 0.0;
    }

    if(yAngle > 360.0){
        yAngle = 0.0;
    }
    glRotatef(xAngle, 1.0, 0.0, 0.0);
    glRotatef(yAngle, 0.0, 1.0, 0.0);
    Test5(depth);
}

void buildFacesDepth(){
    for(int i=0;i<depth;i++){
        buildFaces3();
    }
}

void buildFaces3(){
    int i,j;
    // Initialisation
    if(isInit !=true){
        for(i=0;i<NVERTEX;i++){
            Vertex vertex;
            for(j=0;j<3;j++){
                vertex.push_back(vdata[i][j]);
            }
            vertices.push_back(vertex);
        }
        for(i=0;i<NFACE;i++){
            Face face;
            for(j=0;j<3;j++){
                face.push_back(tindices[i][j]);
            }
            faces.push_back(face);
        }
        isInit=true;
    }
    
    std::vector<Face> tmpFaces;
    // Build new triangles
    for(i=0;i<faces.size();i++){
        int idxA = faces[i][0]; // Index of the vertex
        int idxB = faces[i][1];
        int idxC = faces[i][2];
        Vertex A = vertices[idxA]; // Corresponding vertex
        Vertex B = vertices[idxB];
        Vertex C = vertices[idxC];

        Vertex Ap, Bp, Cp;
        int idxAp, idxBp, idxCp;
        float tmpAp, tmpBp, tmpCp, dA, dB, dC;
        for(j=0;j<3;j++){
            tmpAp = (B[j] + C[j])/2;
            Ap.push_back(tmpAp);
            tmpBp = (A[j] + C[j])/2;
            Bp.push_back(tmpBp);
            tmpCp = (A[j] + B[j])/2;
            Cp.push_back(tmpCp);
        }
        dA = norm(Ap[0], Ap[1], Ap[2]);
        dB = norm(Bp[0], Bp[1], Bp[2]);
        dC = norm(Cp[0], Cp[1], Cp[2]);
        for(j=0;j<3;j++){
            Ap[j] /=dA;
            Bp[j] /=dB;
            Cp[j] /=dC;
        }
        idxAp = vertices.size();
        vertices.push_back(Ap);
        idxBp = vertices.size();
        vertices.push_back(Bp);
        idxCp = vertices.size();
        vertices.push_back(Cp);

        Face face(3,0);
        face[0]=idxA; face[1]=idxBp; face[2]=idxCp; tmpFaces.push_back(face);
        face[0]=idxC; face[1]=idxAp; face[2]=idxBp; tmpFaces.push_back(face);
        face[0]=idxB; face[1]=idxCp; face[2]=idxAp; tmpFaces.push_back(face);
        face[0]=idxAp; face[1]=idxBp; face[2]=idxCp; tmpFaces.push_back(face);
    }
    
    faces.clear();
    faces = tmpFaces;
}
