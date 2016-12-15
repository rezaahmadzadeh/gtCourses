// Calculate and display the Mandelbrot set
// ECE4893/8893 final project, Fall 2011
// BENBIHI Assia

#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <map>
#include <vector>

#include <GL/glut.h>
#include <GL/glext.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "complex.h"

#define DEG2RAD M_PI/180
#define updateRate 100

#define IMAGE_SIZE 512

using namespace std;

typedef std::pair<int, int> MatIndex;
typedef std::vector<float> Color;
typedef std::map< MatIndex, Color> ColorMap;
typedef std::vector<ColorMap> ColorHistory;

typedef std::pair<Complex, Complex> Range; //min max
typedef std::vector<Range> ComplexHistory;

ColorMap colorMap;
ColorHistory colorHistory;
ComplexHistory complexHistory;
int currentColorMap=0;

int xUp=0;
int yUp=0;
int xDown=0;
int yDown=0;
bool drawSquare=false;
#define ITER_MAX 2000 //debug purpos

// Min and max complex plane values
Complex  minC(-2.0, -1.2);
Complex  maxC( 1.0, 1.8);
int      maxIt = 2000;     // Max iterations for the set computations

// Mandelbrot functions
void indexToComplex(int i, int j, double* x, double* y);
void indexToPixel(int i, int j, double* x, double* y);
void mandelbrot(int i, int j, bool* isIn, int* it);
void setColor();
void draw();

//OpenGL functions
void init();   
void display(void);
void reshape(int w, int h);
void timer(int);

// Callback functions
void mouse(int button, int state, int x, int y);

// Other
int iabs(int i);


void indexToComplex(int i, int j, double* x, double* y){
    (*x) = minC.real + ((double)i)*(maxC.real - minC.real)/511.0;
    (*y) = maxC.imag + ((double)j)*(minC.imag - maxC.imag)/511.0;
}

void indexToPixel(int i, int j, double* x, double* y){
    (*x) = -1.0 + ((double)i)*2.0/511.0;
    (*y) = 1.0 - ((double) j)*2.0/511.0;
}

void mandelbrot(int i, int j, bool* isIn, int* it){
    double x,y;
    indexToComplex(i,j,&x,&y);
    //std::cout << "(x,y)=(" << x << "," << y << ")" << std::endl;
    Complex c0(x,y);
    Complex c=c0;
    *it=0;
    *isIn=true;

    int iter=0;
    while( (*isIn) && (*it<maxIt) && iter<ITER_MAX){
        c=c*c+c0;
        (*it)++;
        iter++;
        if(sqrt(c.Mag2())>2.0){
            *isIn =false;
        }
    }

}

void setColor(){
    int i,j,k, it;
    bool isIn;
    for(i=0;i<IMAGE_SIZE;i++){
        for(j=0;j<IMAGE_SIZE;j++){
            Color tmp(3,1.0);
            mandelbrot(i,j,&isIn, &it);
            //std::cout << it << std::endl;
            if(isIn){
                tmp[0]=tmp[1]=tmp[2]=0.0;
            }
            else{
                if(it<50){
                    tmp[0] = (float)it / 50.0;
                    tmp[1] = 0.0;
                    tmp[2] = 1-tmp[0];
                }
                else {
                    tmp[0] = 1.0;
                    tmp[2] = 1.0;
                    tmp[1] = 1.0;
                }
            }
            colorMap[MatIndex(i,j)]=tmp;
        }
    }
}


void draw(){
    glLineWidth(2.0);
    glLoadIdentity();
    
    glBegin(GL_POINTS);
    int i,j;
    for(i=0;i<IMAGE_SIZE;i++){
       for(j=0;j<IMAGE_SIZE;j++){
           glColor3f(colorMap[MatIndex(i,j)][0], colorMap[MatIndex(i,j)][1], colorMap[MatIndex(i,j)][2]);
           glVertex2f((float)(i-256)/((float)IMAGE_SIZE/2.0),(float)(256-j)/((float)IMAGE_SIZE/2.0));
       }
    }
    glEnd();

    if(drawSquare){
    glBegin(GL_LINE_LOOP);
        glColor3f(0.0, 1.0, 0.0);
        //std::cout << "Drawing the square (" << xDown << "," << yDown << ") (" << xDown << "," << yUp << ") (" << xUp << "," << yDown << ") (" << xUp << "," << yUp << ")" << std::endl;
        glVertex2f((float)(xDown-256)/((float)IMAGE_SIZE/2.0),(float)(256-yDown)/((float)IMAGE_SIZE/2.0));
        glVertex2f((float)(xDown-256)/((float)IMAGE_SIZE/2.0),(float)(256-yUp)/((float)IMAGE_SIZE/2.0));
        glVertex2f((float)(xUp-256)/((float)IMAGE_SIZE/2.0),(float)(256-yUp)/((float)IMAGE_SIZE/2.0));
        
        glVertex2f((float)(xUp-256)/((float)IMAGE_SIZE/2.0),(float)(256-yDown)/((float)IMAGE_SIZE/2.0));
       
        glEnd();
    }
    glutSwapBuffers();
}


double xMin, xMax, yMin, yMax;
void mouse(int button, int state, int x, int y){
    
    // Your mouse click processing here
    // state == 0 means pressed, state != 0 means released
    // Note that the x and y coordinates passed in are in
    // PIXELS, with y = 0 at the top.
    if(button==GLUT_LEFT_BUTTON){
        //std::cout << "Mouse (x,y)=(" << x << "," << y << ")" << std::endl;
        if(state==GLUT_UP){ //release
            // Save previous configuration 
            Range range;
            range.first=minC;
            range.second=maxC;
            complexHistory.push_back(range); 
            currentColorMap++;
            
            // Compute new minC and maxC
            drawSquare=false;
            minC = Complex(xMin, yMin);
            maxC = Complex(xMax, yMax);
            //std::cout << "(xD,yD) (xU,yU) (x,y) c cadran = (" << xDown << "," << yDown << ") (" << xUp << "," << yUp << ") (" << x << "," << y << ")" << squareLength << " " << cadran << std::endl; 
            //std::cout << "(xMin,yMin) (xMax,yMax) = (" << xMin << "," << yMin << ") (" << xMax << "," << yMax << ")" << std::endl;
            setColor();
            glutPostRedisplay();
        }
        else if(state == GLUT_DOWN){ //press
            //std::cout << "Press (x,y)=(" << x << "," << y << ")" << std::endl;
            xDown = x;
            yDown = y;
            drawSquare=true;
            //canMove = true;
        }
    }
    else{
        std::cout << "Wrong mouse button." << std::endl;
    }
}


void motion(int x, int y){
    //std::cout << "Motion (x,y)=(" << x << "," << y << ")" << std::endl;
    // Your mouse motion here, x and y coordinates are as above
    double xcD, ycD, xcU, ycU;
    int dx = x-xDown;
    int dy = y-yDown;
    int squareLength= std::min(iabs(dx), iabs(dy));//square length
    // Set the square limitsand new complex range based on the mouse movement
    if((dx>0) && (dy>0)){
        xUp = xDown+squareLength;
        yUp = yDown+squareLength;
    }
    else if((dx<0) && (dy>0)){
        xUp = xDown-squareLength; 
        yUp = yDown+squareLength;
    }
    else if((dx<0) && (dy<0)){
        xUp = xDown-squareLength;
        yUp = yDown-squareLength;
    }
    else if((dx>0) && (dy<0)){
        xUp = xDown+squareLength;
        yUp = yDown-squareLength;
    }
    else{
        xUp = xDown;
        yUp = yDown;
    }
    indexToComplex(xDown,yDown,&xcD,&ycD);
    indexToComplex(xUp, yUp, &xcU, &ycU);
    if(xcD<xcU){xMin=xcD; xMax=xcU; }
    else{xMin=xcU; xMax=xcD;}
    if(ycD<ycU){yMin=ycD; yMax=ycU;}
    else{yMin=ycU; yMax=ycD;}

    glutPostRedisplay();
}

void keyboard(unsigned char c, int x, int y){ 
    // Your keyboard processing here
    if(c=='b'){
        if(currentColorMap==0){
            std::cout << "It is already the oldest image." << std::endl;
        }
        else{
            currentColorMap--;
            Range range = complexHistory[currentColorMap];
            minC=range.first;
            maxC=range.second;
            if(currentColorMap<colorHistory.size()-1){
                complexHistory.erase(complexHistory.begin()+complexHistory.size()-1);
            }
            setColor();
            glutPostRedisplay();
        }
    }
}

int main(int argc, char** argv){
    // Initialize OpenGL, but only on the "master" thread or process.
    // See the assignment writeup to determine which is "master" 
    // and which is slave.
    
    // Set color
    setColor();
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(IMAGE_SIZE, IMAGE_SIZE);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Mandelbrot");
    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(1000.0/updateRate, timer, 0);
    glutMainLoop();
    
    return 0;
}


// OpenGL functions
void init(){ // Your OpenGL initialization code here
    glClearColor(0.0, 0.0, 0.0, 0.0);
    //glClear(GL_DEPTH_BUFFER_BIT);
}

void display(void){ // Your OpenGL display code here
    //static int pass;
    //std::cout << "Displaying pass " << ++pass << std::endl;
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity(); // clear the matrix
    gluLookAt(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    //glEnable(GL_DEPTH_TEST);
    draw();
    glutSwapBuffers();

}

void reshape(int w, int h){ // Your OpenGL window reshape code here
    glViewport(0,0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void timer(int){
    glutPostRedisplay();
    glutTimerFunc(1000.0/updateRate, timer, 0);
}


//Other
int iabs(int i){
    if(i>=0){
        return i;
    }
    else{
        return -i;
    }
}
