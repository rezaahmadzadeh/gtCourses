// Distributed two-dimensional Discrete FFT transform
// BENBIHI Assia
// ECE8893 Project 1


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <signal.h>
#include <math.h>
#include <mpi.h>

#include "Complex.h"
#include "InputImage.h"

using namespace std;


void Transform1D(Complex* h, int w, Complex* H)
{
  // Implement a simple 1-d DFT using the double summation equation
  // given in the assignment handout.  h is the time-domain input
  // data, w is the width (N), and H is the output array.
    int n,k;
    
    for(n=0;n<w;n++){
        H[n]=Complex(0,0);
        for(k=0;k<w;k++) {
            double arg = 2*M_PI*n*k/w;
            double r = cos(arg);
            double i = -sin(arg);
            Complex W2nk = Complex(r,i);
            H[n] = H[n] + W2nk*h[k];
        }
    }
}


void TransformI1D(Complex* h, int w, Complex* H)
{
  // Implement a simple 1-d DFT using the double summation equation
  // given in the assignment handout.  h is the time-domain input
  // data, w is the width (N), and H is the output array.
    int n,k;
    
    for(n=0;n<w;n++){
        H[n]=Complex(0,0);
        for(k=0;k<w;k++) {
            double arg = 2*M_PI*n*k/w;
            double r = cos(arg)/w;
            double i = sin(arg)/w;
            Complex W2nk = Complex(r,i);
            H[n] = H[n] + W2nk*h[k];
        }
    }
}





void Transform2D(const char* inputFN) 
{ // Do the 2D transform here.
  // 1) Use the InputImage object to read in the Tower.txt file and
  //    find the width/height of the input image.
  // 2) Use MPI to find how many CPUs in total, and which one
  //    this process is
  // 3) Allocate an array of Complex object of sufficient size to
  //    hold the 2d DFT results (size is width * height)
  // 4) Obtain a pointer to the Complex 1d array of input data
  // 5) Do the individual 1D transforms on the rows assigned to your CPU
  // 6) Send the resultant transformed values to the appropriate
  //    other processors for the next phase.
  // 6a) To send and receive columns, you might need a separate
  //     Complex array of the correct size.
  // 7) Receive messages from other processes to collect your columns
  // 8) When all columns received, do the 1D transforms on the columns
  // 9) Send final answers to CPU 0 (unless you are CPU 0)
  //   9a) If you are CPU 0, collect all values from other processors
  //       and print out with SaveImageData().
    InputImage image(inputFN);  // Create the helper object for reading the image
  // Step (1) in the comments is the line above.
  // Your code here, steps 2-9
   

    int i,l,c,numtasks, rank;
    int lineNumber = image.GetHeight();
    int columnNumber = image.GetWidth();
    Complex* h = image.GetImageData();
    MPI_Comm_size(MPI_COMM_WORLD, &numtasks);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Status status;
    int subLineNumber = lineNumber/numtasks;
    int startLineNumber = subLineNumber*rank;
    Complex* H = new Complex[columnNumber * lineNumber];
    
    // Row computation
    for(i=0;i<subLineNumber;i++){
        Transform1D(h+columnNumber*(startLineNumber+i), columnNumber, H+columnNumber*(startLineNumber+i));
    }

    if(rank!=0) {
        MPI_Send(H+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD);
        //MPI_Isend(H+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD,&req);
    }
    else {
        for(i=1;i<numtasks;i++){
            int startLineNumber = subLineNumber*i;
            MPI_Recv(H+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, i, startLineNumber, MPI_COMM_WORLD, &status);
        }
        
        image.SaveImageData("test2Dline.txt", H, columnNumber, lineNumber);
    
        //transpose H
        Complex tmp = Complex(0,0);
        for(l=0;l<lineNumber;l++){
            for(c=0;c<columnNumber;c++){
                if(l<c) {
                    tmp = H[l*columnNumber+c];
                    H[l*columnNumber+c]=H[c*columnNumber+l];
                    H[c*columnNumber+l]=tmp;
                }
            }
        }
        
        for(i=1;i<numtasks;i++){
            MPI_Send(H, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, i, 0, MPI_COMM_WORLD);
            //MPI_Isend(H, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, i, 0, MPI_COMM_WORLD, &req);
        }
        
    }
    
    if(rank!=0){
        MPI_Recv(H, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, 0, 0, MPI_COMM_WORLD, &status);
    }
    
    
     
    Complex* H2D = new Complex[lineNumber*columnNumber];

    //column computation
    for(i=0;i<subLineNumber;i++){
        Transform1D(H+columnNumber*(startLineNumber+i), columnNumber, H2D+columnNumber*(startLineNumber+i));
    }

    if(rank!=0) {
        MPI_Send(H2D+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD);
        //MPI_Isend(H2D+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD, &req);
    }
    else {
        for(i=1;i<numtasks;i++){
            int startLineNumber = subLineNumber*i;
            MPI_Recv(H2D+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, i, startLineNumber, MPI_COMM_WORLD, &status);
        }

        //transpose H
        Complex tmp = Complex(0,0);
        for(l=0;l<lineNumber;l++){
            for(c=0;c<columnNumber;c++){
                if(l<c) {
                    tmp = H2D[l*columnNumber+c];
                    H2D[l*columnNumber+c]=H2D[c*columnNumber+l];
                    H2D[c*columnNumber+l]=tmp;
                }
            }
        }


        image.SaveImageData("MyAfter2D.txt", H2D, columnNumber, lineNumber);
        
        
        // Inverse transform setup
        for(i=1;i<numtasks;i++){
            MPI_Send(H2D, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, i, 0, MPI_COMM_WORLD);
            //MPI_Isend(H2D, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, i, 0, MPI_COMM_WORLD,&req);
        }
        
    }
    
    if(rank!=0){
        MPI_Recv(H2D, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, 0, 0, MPI_COMM_WORLD, &status);
    }

    // Inverse transform computation
    // Row computation
    for(i=0;i<subLineNumber;i++){
        TransformI1D(H2D+columnNumber*(startLineNumber+i), columnNumber, H+columnNumber*(startLineNumber+i));
    }

    if(rank!=0) {
        MPI_Send(H+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD);
        //MPI_Isend(H+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD,&req);
    }
    else {
        for(i=1;i<numtasks;i++){
            int startLineNumber = subLineNumber*i;
            MPI_Recv(H+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, i, startLineNumber, MPI_COMM_WORLD, &status);
        }

        //transpose H
        Complex tmp = Complex(0,0);
        for(l=0;l<lineNumber;l++){
            for(c=0;c<columnNumber;c++){
                if(l<c) {
                    tmp = H[l*columnNumber+c];
                    H[l*columnNumber+c]=H[c*columnNumber+l];
                    H[c*columnNumber+l]=tmp;
                }
            }
        }
    
        for(i=1;i<numtasks;i++){
            MPI_Send(H, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, i, 0, MPI_COMM_WORLD);
            //MPI_Isend(H, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, i, 0, MPI_COMM_WORLD, &req);
        }
    }
    
    if(rank!=0){
        MPI_Recv(H, columnNumber*lineNumber*sizeof(Complex), MPI_CHAR, 0, 0, MPI_COMM_WORLD, &status);
    }
    
    //column computation
    for(i=0;i<subLineNumber;i++){
        TransformI1D(H+columnNumber*(startLineNumber+i), columnNumber, h+columnNumber*(startLineNumber+i));
    }

    if(rank!=0) {
        MPI_Send(h+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD);
        //MPI_Isend(h+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, 0, startLineNumber, MPI_COMM_WORLD,&req);
    }
    else {
        for(i=1;i<numtasks;i++){
            int startLineNumber = subLineNumber*i;
            MPI_Recv(h+startLineNumber*columnNumber, columnNumber*subLineNumber*sizeof(Complex), MPI_CHAR, i, startLineNumber, MPI_COMM_WORLD, &status);
        }

        //transpose h
        Complex tmp = Complex(0,0);
        for(l=0;l<lineNumber;l++){
            for(c=0;c<columnNumber;c++){
                if(l<c) {
                    tmp = h[l*columnNumber+c];
                    h[l*columnNumber+c]=h[c*columnNumber+l];
                    h[c*columnNumber+l]=tmp;
                }
            }
        }


        image.SaveImageDataReal("MyAfterInverse.txt", h, columnNumber, lineNumber);
    }
    
    delete[] h;
    delete[] H; 
    delete[] H2D;
}




int main(int argc, char** argv)
{

    string fn("Tower.txt"); // default file name
  if (argc > 1) fn = string(argv[1]);  // if name specified on cmd line
    int rc=MPI_Init(&argc, &argv);
    if(rc!=MPI_SUCCESS){
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }
    Transform2D(fn.c_str()); // Perform the transform.
    MPI_Finalize();

}  
  

  
