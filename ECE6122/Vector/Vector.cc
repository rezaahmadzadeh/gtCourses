// Implementation of the templated Vector class
// ECE4893/8893 lab 3
// Assia BENBIHI

#include <iostream> 
#include "Vector.h"


// Your implementation here
// Fill in all the necessary functions below
using namespace std;

#define SAFE_SPACE 1

// Default constructor
template <typename T>
Vector<T>::Vector()
{
    //std::cout << "Vector<T>::Vector()\n" << std::endl;
    elements=NULL;
    count=0;
    reserved=0;
}

// Copy constructor
template <typename T>
Vector<T>::Vector(const Vector& rhs)
{
    //std::cout << "Copy constructor" << std::endl;
    count = rhs.Size();
    reserved = rhs.reserved; 
    elements = (T*) malloc(reserved * sizeof(T));

    if(elements==NULL){
        std::cerr << "Error: Copy constructor: memory allocation failed." << std::endl;
        free(elements);
        exit(EXIT_FAILURE); 
    }
    int i=0;
    for(i=0;i<count;i++) {
        new(&elements[i]) T(rhs[i]);
    }

}

// Assignment operator
template <typename T>
Vector<T>& Vector<T>::operator=(const Vector& rhs) {
   
    // If same reserved size, just copy 
    size_t newSize = rhs.reserved;
    int i=0;
    if(reserved==newSize){ 
        count = rhs.count;
        for(i=0;i<count;i++){
            new (&elements[i]) T(rhs.elements[i]);
        }
    }
    // Else, free and reallocate current memory, then copy.
    else {
        reserved = newSize;
        count = rhs.count;
        free(elements);
        elements = (T*) malloc (reserved*sizeof(T));      
        if(elements==NULL){
            std::cerr << "Error: operator= : memory allocation failed." << std::endl;
            free(elements);
            exit(EXIT_FAILURE); 
        }
        for(i=0;i++;i<count){
            new (&elements[i]) T(rhs.elements[i]);
            elements[i].~T();
        }
    }
}

#ifdef GRAD_STUDENT
// Other constructors
template <typename T>
Vector<T>::Vector(size_t nReserved)
{ // Initialize with reserved memory
    // Allocate memory
    elements = (T*) malloc(nReserved*sizeof(T));
}

template <typename T>
Vector<T>::Vector(size_t n, const T& t)
{ // Initialize with "n" copies of "t"
    reserved = n;
    count = n;
    elements = (T*) malloc(reserved *sizeof(T)); 
    int i=0;
    for(i=0;i<n;i++){
        new (&elements[i]) T(t);
    }
}

template <typename T>
void Vector<T>::Reserve(size_t n) {
    int i=0;
    // If not already the right size, reallocate and copy
    // Free previous memory
    if(reserved!=n){
        reserved = n;
        free(elements);
        T* newT = (T*) malloc(reserved*sizeof(T));
        if(newT==NULL){
            std::cerr << "Error: operator= : memory allocation failed." << std::endl;
            free(elements);
            exit(EXIT_FAILURE); 
        }
        for(i=0;i<count;i++){
            new (&newT[i]) T(elements[i]);
            elements[i].~T();
        }
        free(elements);
        elements = newT;
    }
}
#endif

// Destructor
template <typename T>
Vector<T>::~Vector()
{
    // Destroy objects and clear the reserved memory
    int i=0;
    for(i=0;i<count;i++){
        elements[i].~T();
    }
    free(elements);
}

// Add and access front and back
template <typename T>
void Vector<T>::Push_Back(const T& rhs)
{

    //std::cout << "Push_Back" << std::endl;
    int i=0;
    // If there is space left, add the element
    if(count<reserved){
        new (&elements[count]) T(rhs);
        count++;
    }
    // Else reallocate memory, copy and free previous memory
    else{
        reserved = reserved + SAFE_SPACE;
        T* newT = (T*) malloc (reserved*sizeof(T));      
        if(newT==NULL){
            std::cerr << "Error: Push_Back: memory allocation failed." << std::endl;
            free(elements);
            exit(EXIT_FAILURE); 
        }
        // Copy
        for(i=0;i<count;i++){
            new (&newT[i]) T(elements[i]);
            elements[i].~T();
        }
        free(elements);
        elements = newT;
        // Add new element
        new (&elements[count]) T(rhs);
        count++;
    }
}

template <typename T>
void Vector<T>::Push_Front(const T& rhs)
{
    //std::cout << "Push_Front" << std::endl;
    int i=0;
    // If enough reserved space, swap objects
    if(count<reserved) {
        for(i=count;i>0;i--){
            new(&elements[i]) T(elements[i-1]);
            elements[i-1].~T();
        }
        elements[0] = T(rhs);
        count ++;
    }
    // Reallocare memory, copy and free previous memory 
    else {
        reserved = reserved + SAFE_SPACE;
        T* newT = (T*) malloc(reserved*sizeof(T));
        if(newT==NULL){
            std::cerr << "Error:Push_Front: memory allocation failed." << std::endl;
            free(elements);
            exit(EXIT_FAILURE);
        }
        // Copy and decale
        for(i=0;i<count;i++){
            new (&newT[i+1]) T(elements[i]);
            elements[i].~T();
        }
        free(elements);
        elements = newT;
        new (elements) T(rhs);
        count ++;

    }
}

template <typename T>
void Vector<T>::Pop_Back()
{ // Remove last element
    //std::cout << "Pop_Back()" << std::endl;
    if(count==0){
        std::cerr << "Error: Pop_Back(): The vector is empty." << std::endl;
        exit(EXIT_FAILURE);
    }
    elements[count-1].~T();
    count--;
}

template <typename T>
void Vector<T>::Pop_Front()
{ // Remove first element
    //std::cout << "Pop_Front()" << std::endl;
    if(count==0){
        std::cerr << "Error: Pop_Front(): The vector is empty." << std::endl;
        exit(EXIT_FAILURE);
    }
    int i=0;
    count--;
    elements[0].~T();
    for(i=0;i<count;i++){
        new (&elements[i]) T(elements[i+1]);
        elements[i+1].~T();
    }
}

// Element Access
template <typename T>
T& Vector<T>::Front() const
{
    //std::cout << "Front()" << std::endl;
    if(count==0){
        std::cerr << "Error: Front(): The vector is empty." << std::endl;
        exit(EXIT_FAILURE);
    }
    return elements[0];
}

// Element Access
template <typename T>
T& Vector<T>::Back() const
{
    //std::cout << "Back()" << std::endl;
    if(count==0){
        std::cerr << "Error: Back(): The vector is empty." << std::endl;
        exit(EXIT_FAILURE);
    }
    return elements[count-1];
}


template <typename T>
const T& Vector<T>::operator[](size_t i) const
{
    return elements[i];
}


template <typename T>
T& Vector<T>::operator[](size_t i)
{
    return elements[i];
}


template <typename T>
size_t Vector<T>::Size() const
{
    //std::cout << "Size()" << std::endl;
    return count;
}

template <typename T>
bool Vector<T>::Empty() const{
    //std::cout << "Empty" << std::endl;
    return (count==0);
}

// Implement clear
template <typename T>
void Vector<T>::Clear()
{
    //std::cout << "Clear()" << std::endl;
    int i=0;
    for(i=0;i<count;i++){
        elements[i].~T();
    }
    count=0;
}

// Iterator access functions
template <typename T>
VectorIterator<T> Vector<T>::Begin() const
{
  return VectorIterator<T>(elements);
}

template <typename T>
VectorIterator<T> Vector<T>::End() const
{
    return VectorIterator<T>(elements+count);
}

#ifdef GRAD_STUDENT
// Erase and insert
template <typename T>
void Vector<T>::Erase(const VectorIterator<T>& it)
{
    //std::cout << "Erase" << std::endl;

    // Find the item to erase
    int place=0;
    bool found = false;
    while(!found){
        if( (&elements[place])== it.current){
            found = true;
        }
        else{
            place++;
        }
    }

    int i=0;
    elements[place].~T();
    for(i=place;i<count-1;i++){
        new (&elements[i]) T(elements[i+1]);
        elements[i+1].~T();
    }

    count--;
}

template <typename T>
void Vector<T>::Insert(const T& rhs, const VectorIterator<T>& it)
{
    //std::cout << "Insert" << std::endl;
    // Find the item to erase
    int place=0;
    bool found = false;
    while(!found){
        if( (&elements[place])== it.current){
            found = true;
        }
        else{
            place++;
        }
    }

    // If there is enough space, insert and shift to the right
    int i=0;
    if(count<reserved){
        for(i=count;i>place;i--){
           new (&elements[i]) T(elements[i-1]);
           elements[i-1].~T();
        }
        new (&elements[place]) T(rhs);
    }
    // else reallocate, copy, free previous memory
    else{
        reserved = reserved + SAFE_SPACE;
        T* newT = (T*) malloc(reserved*sizeof(T));
        if(newT==NULL){
            std::cerr << "Error:Insert: memory allocation failed." << std::endl;
            free(elements);
            exit(EXIT_FAILURE);
        }
        // Copy and decale
        for(i=0;i<place;i++){
            new (&newT[i]) T(elements[i]);
            elements[i].~T();
        }
        new (&newT[place]) T(rhs);
        for(i=place; i<count; i++){
            new (&newT[i+1]) T(elements[i]);
            elements[i].~T();
        }
        free(elements);
        elements = newT;

    }
    count++;
}


#endif

// Implement the iterators

// Constructors
template <typename T>
VectorIterator<T>::VectorIterator()
{
    current = NULL;
}

template <typename T>
VectorIterator<T>::VectorIterator(T* c)
{
    current = c;
}


// Copy constructor
template <typename T>
VectorIterator<T>::VectorIterator(const VectorIterator<T>& rhs)
{
   current = rhs.current;
}

// Iterator defeferencing operator
template <typename T>
T& VectorIterator<T>::operator*() const
{
    return *current;
}

// Prefix increment
template <typename T>
VectorIterator<T>  VectorIterator<T>::operator++()
{
    current++;
    return *this;
}

// Postfix increment
template <typename T>
VectorIterator<T> VectorIterator<T>::operator++(int)
{
    VectorIterator<T> it(current);
    current++;
    return it;
}

// Comparison operators
template <typename T>
bool VectorIterator<T>::operator !=(const VectorIterator<T>& rhs) const
{
    return ( current != rhs.current); 
}

template <typename T>
bool VectorIterator<T>::operator ==(const VectorIterator<T>& rhs) const
{
    return ( current == rhs.current);
}


