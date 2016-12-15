// Threaded two-dimensional Discrete FFT transform
// YOUR NAME HERE
// ECE8893 Project 2


#include <iostream>
#include <string>
#include <math.h>

#include "Complex.h"
#include "InputImage.h"

// You will likely need global variables indicating how
// many threads there are, and a Complex* that points to the
// 2d image being transformed.

using namespace std;

#define NUM_THREADS 16

// Data variables
int N = 0;
Complex* imageData = NULL;
Complex* Warray = NULL;
Complex* WarrayI = NULL;
int activeThreads = 0;

// Mutex variables
pthread_mutex_t activeMutex;
pthread_cond_t allDoneCondition;

// Barrier Variables
int P=0;
int count =0;
pthread_mutex_t countMutex;
bool* localSense;
bool globalSense;

// Function to reverse bits in an unsigned integer
// This assumes there is a global variable N that is the
// number of points in the 1D transform.
unsigned ReverseBits(unsigned v)
{ //  Provided to students
  unsigned n = N; // Size of array (which is even 2 power k value)
  unsigned r = 0; // Return value
   
  for (--n; n > 0; n >>= 1)
    {
      r <<= 1;        // Shift return value
      r |= (v & 0x1); // Merge in next bit
      v >>= 1;        // Shift reversal value
    }
  return r;
}

// GRAD Students implement the following 2 functions.
// Undergrads can use the built-in barriers in pthreads.

int FetchAndDecrementCount() {
    pthread_mutex_lock(&countMutex);
    int myCount = count;
    count --;
    pthread_mutex_unlock(&countMutex);
    return myCount;
}


// Call MyBarrier_Init once in main
void MyBarrier_Init(int numThreads)// you will likely need some parameters)
{
    P = numThreads;
    count = numThreads;
    pthread_mutex_init(&countMutex,0);
    localSense = new bool[P];
    for(int i=0;i<P;i++){
        localSense[i] = true;
    }
    globalSense = true;
}

// Each thread calls MyBarrier after completing the row-wise DFT
void MyBarrier(int myId) // Again likely need parameters
{
    localSense[myId] = !localSense[myId];
    if(FetchAndDecrementCount() == 1){
        count = P;
        globalSense = localSense[myId];
    }
    else{
        while(globalSense != localSense[myId]) {
            // Spin
        }
    }
}

                    
void Transform1D(Complex* h, int N0)
{
  // Implement the efficient Danielson-Lanczos DFT here.
  // "h" is an input/output parameter
  // "N" is the size of the array (assume even power of 2)
    
    int k, arrayStart;

    //Step 1: Reoder the array
    Complex tmp[N0];
    for(k=0;k<N;k++){    
        unsigned int r = ReverseBits(k);
        tmp[k] = h[r];
    }
    for(k=0;k<N;k++){
        h[k] = tmp[k];
    }

    // Step 3: Compute transformation
    int arrayLevel = 2; 
    for(arrayLevel=2;arrayLevel<=N0;arrayLevel*=2){
        //int arrayNumber = N/arrayLevel;
        for(arrayStart=0;arrayStart<N0;arrayStart+=arrayLevel) {
            for(k=0;k<(arrayLevel/2);k++){
                int index = k*N0/arrayLevel;
                if(index >= N0/2) {
                    index = index - N0/2;
                }
                Complex w = Warray[index];
                Complex He = h[arrayStart+k];
                Complex Ho = h[arrayStart+k+(arrayLevel/2)];
                h[arrayStart+k] = He + w*Ho;
                h[arrayStart+k+(arrayLevel/2)] = He - w*Ho;
            }
        }
    }

}


void Transform1DI(Complex* h, int N0)
{
  // Implement the efficient Danielson-Lanczos DFT here.
  // "h" is an input/output parameter
  // "N" is the size of the array (assume even power of 2)
    
    int k, arrayStart;

    //Step 1: Reoder the array
    Complex tmp[N0];
    for(k=0;k<N;k++){    
        unsigned int r = ReverseBits(k);
        tmp[k] = h[r];
    }
    for(k=0;k<N;k++){
        h[k] = tmp[k];
    }

    // Step 3: Compute transformation
    int arrayLevel = 2; 
    for(arrayLevel=2;arrayLevel<=N0;arrayLevel*=2){
        for(arrayStart=0;arrayStart<N0;arrayStart+=arrayLevel) {
            for(k=0;k<(arrayLevel/2);k++){
                int index = k*N0/arrayLevel;
                if(index >= N0/2) {
                    index = index - N0/2;
                }
                Complex w = WarrayI[index];
                Complex He = h[arrayStart+k];
                Complex Ho = h[arrayStart+k+(arrayLevel/2)];
                h[arrayStart+k] = He + w*Ho;
                h[arrayStart+k+(arrayLevel/2)] = He - w*Ho;
            }
        }
    }

    for(k=0;k<N0;k++){
        h[k]=h[k]/N;
    }

}



void* Transform2DThread(void* v)
{ // This is the thread startign point.  "v" is the thread number
  // Calculate 1d DFT for assigned rows
  // wait for all to complete
  // Calculate 1d DFT for assigned columns
  // Decrement active count and signal main if all complete
    
    int i;
    int lineNumber = N;
    int columnNumber = lineNumber;
    int rank = (long) v;
    int sublineNumber = lineNumber/NUM_THREADS;
    int startLineNumber = sublineNumber * rank;

    //Row computation
    for(i=0;i<sublineNumber;i++){
        Transform1D(imageData+columnNumber*(startLineNumber+i),N);
    }
    
    MyBarrier(rank);
    /**
    pthread_mutex_lock(&activeMutex);
    activeThreads--;
    if(activeThreads == 0) {
        pthread_cond_signal(&allDoneCondition);
    }
    pthread_mutex_unlock(&activeMutex);
    */
    return 0;
}




void* Transform2DThreadI(void* v)
{ // This is the thread startign point.  "v" is the thread number
  // Calculate 1d DFT for assigned rows
  // wait for all to complete
  // Calculate 1d DFT for assigned columns
  // Decrement active count and signal main if all complete
    
    int i;
    int lineNumber = N;
    int columnNumber = lineNumber;
    int rank = (long) v;
    int sublineNumber = lineNumber/NUM_THREADS;
    int startLineNumber = sublineNumber * rank;

    //Row computation
    for(i=0;i<sublineNumber;i++){
        Transform1DI(imageData+columnNumber*(startLineNumber+i),N);
    }
    
    MyBarrier(rank);
    /**
    pthread_mutex_lock(&activeMutex);
    activeThreads--;
    if(activeThreads == 0) {
        pthread_cond_signal(&allDoneCondition);
    }
    pthread_mutex_unlock(&activeMutex);
    */
    return 0;
}



void Transform2D(const char* inputFN) 
{ // Do the 2D transform here.
  InputImage image(inputFN);  // Create the helper object for reading the image
  // Create the global pointer to the image array data
  // Create 16 threads
  // Wait for all threads complete
  // Write the transformed data
    
    
    int k=0, l=0, c=0;
    N = image.GetWidth();
    imageData = image.GetImageData();
    int lineNumber = N;
    int columnNumber = N;

    // Step 2: Compute the W array
    Warray = new Complex[(N/2)];
    WarrayI = new Complex[(N/2)];
    for(k=0;k<(N/2);k++){
        double arg = 2*M_PI*k/N;
        double r = cos(arg);
        double i = -sin(arg);
        double iI = sin(arg);
        Warray[k] = Complex(r,i);
        WarrayI[k] = Complex(r,iI);
    }

    // Pthread init
    pthread_mutex_init(&activeMutex, 0);
    pthread_cond_init(&allDoneCondition,0);

    MyBarrier_Init(NUM_THREADS+1);
    
    pthread_mutex_lock(&activeMutex);
    activeThreads = NUM_THREADS;
    pthread_t threads[NUM_THREADS];
    int rc;
    long t;
    
    // Forward DFT
    // Row DFT computation
    for(t=0;t<NUM_THREADS;t++){
        rc = pthread_create(&threads[t], NULL, Transform2DThread, (void *) t);
    }
    MyBarrier(NUM_THREADS);
    image.SaveImageData("test1D.txt", imageData, N, N);
    
    // Transpose
    Complex tmp = Complex(0,0);
    for(l=0;l<lineNumber;l++){
        for(c=0;c<columnNumber;c++){
            if(l<c){
                tmp = imageData[l*columnNumber+c];
                imageData[l*columnNumber+c]=imageData[c*columnNumber+l];
                imageData[c*columnNumber+l]=tmp;
            }
        }
    }
    
    // Column DFT computation
    for(t=0;t<NUM_THREADS;t++){
        rc = pthread_create(&threads[t], NULL, Transform2DThread, (void *) t);
    }
    MyBarrier(NUM_THREADS); 
    // Transpose
    tmp = Complex(0,0);
    for(l=0;l<lineNumber;l++){
        for(c=0;c<columnNumber;c++){
            if(l<c){
                tmp = imageData[l*columnNumber+c];
                imageData[l*columnNumber+c]=imageData[c*columnNumber+l];
                imageData[c*columnNumber+l]=tmp;
            }
        }
    }
    image.SaveImageData("MyAfter2D.txt", imageData, N, N);
   
    
    // Inverse DFT
    // Row DFT computation
    for(t=0;t<NUM_THREADS;t++){
        rc = pthread_create(&threads[t], NULL, Transform2DThreadI, (void *) t);
    }
    MyBarrier(NUM_THREADS);
    //image.SaveImageData("test1DI.txt", imageData, N, N);
    
    // Transpose
    tmp = Complex(0,0);
    for(l=0;l<lineNumber;l++){
        for(c=0;c<columnNumber;c++){
            if(l<c){
                tmp = imageData[l*columnNumber+c];
                imageData[l*columnNumber+c]=imageData[c*columnNumber+l];
                imageData[c*columnNumber+l]=tmp;
            }
        }
    }
    
    // Column DFT computation
    for(t=0;t<NUM_THREADS;t++){
        rc = pthread_create(&threads[t], NULL, Transform2DThreadI, (void *) t);
    }
    MyBarrier(NUM_THREADS); 
    // Transpose
    tmp = Complex(0,0);
    for(l=0;l<lineNumber;l++){
        for(c=0;c<columnNumber;c++){
            if(l<c){
                tmp = imageData[l*columnNumber+c];
                imageData[l*columnNumber+c]=imageData[c*columnNumber+l];
                imageData[c*columnNumber+l]=tmp;
            }
        }
    }
    image.SaveImageData("MyAfterInverse.txt", imageData, N, N);


    delete[] imageData;
    delete[] Warray;
    delete[] WarrayI;
    delete[] localSense;
}



int main(int argc, char** argv)
{
  string fn("Tower.txt"); // default file name
  if (argc > 1) fn = string(argv[1]);  // if name specified on cmd line
  Transform2D(fn.c_str()); // Perform the transform.
    
    return 0;

}  
  

  
