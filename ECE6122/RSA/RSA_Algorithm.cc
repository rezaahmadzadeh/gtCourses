// RSA Assignment for ECE4122/6122 Fall 2015

#include <iostream>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include "RSA_Algorithm.h"

using namespace std;

#define PRIME_TEST_ITERATION 100

// Implement the RSA_Algorithm methods here


// Constructor
RSA_Algorithm::RSA_Algorithm()
  : rng(gmp_randinit_default)
{
  // get a random seed for the random number generator
  int dr = open("/dev/random", O_RDONLY);
  if (dr < 0)
    {
      cout << "Can't open /dev/random, exiting" << endl;
      exit(0);
    }
  unsigned long drValue;
  read(dr, (char*)&drValue, sizeof(drValue));
  //cout << "drValue " << drValue << endl;
  rng.seed(drValue);
// No need to init n, d, or e.
}

// Fill in the remainder of the RSA_Algorithm methods
void RSA_Algorithm::GenerateRandomKeyPair(size_t sz){
    //std::cout << "Generate KEYS !!!!!" << std::endl;

    mpz_class p,q,phi_n,k;

    bool isPrime = false;
    while(!isPrime){
        p = rng.get_z_bits((unsigned long)sz);
        int primeRes = mpz_probab_prime_p(p.get_mpz_t(), PRIME_TEST_ITERATION);
        isPrime = (primeRes >0);
        //std::cout << "P GENERATED" << std::endl;
    }
    isPrime = false;
    while(!isPrime){
        q = rng.get_z_bits(sz);
        int primeRes = mpz_probab_prime_p(q.get_mpz_t(), PRIME_TEST_ITERATION);
        isPrime = (primeRes >0);
    }

    n=p*q;
    phi_n = (p-1)*(q-1);

    isPrime = false;
    while(!isPrime){
        d = rng.get_z_bits(2*sz);
        mpz_gcd(k.get_mpz_t(), d.get_mpz_t(), phi_n.get_mpz_t());
        if(k==1){
            int invertExist = mpz_invert(e.get_mpz_t(),d.get_mpz_t(),phi_n.get_mpz_t());
            if (invertExist!=0){
                isPrime = true;
            }
        }
    }

    //std::cout << "(p,q,n,phi_n, d,e)=" << p << "," << q << "," << n << "," << phi_n << "," << d << "," << e <<std::endl;

}


mpz_class RSA_Algorithm::Encrypt(mpz_class M){
    //std::cout << "Encrypt" << std::endl;
    mpz_class C;
    mpz_powm(C.get_mpz_t(), M.get_mpz_t(), e.get_mpz_t(), n.get_mpz_t());
    return C;

}

mpz_class RSA_Algorithm::Decrypt(mpz_class C){
    //std::cout << "Decrypt" << std::endl;
    mpz_class M;
    mpz_powm(M.get_mpz_t(), C.get_mpz_t(), d.get_mpz_t(), n.get_mpz_t());
    return M;
}


void RSA_Algorithm::PrintND()
{ // Do not change this, right format for the grading script
  cout << "n " << n << " d " << d << endl;
}

void RSA_Algorithm::PrintNE()
{ // Do not change this, right format for the grading script
  cout << "n " << n << " e " << e << endl;
}

void RSA_Algorithm::PrintNDE()
{ // Do not change this, right format for the grading script
  cout << "n " << n << " d " << d << " e " << e << endl;
}

void RSA_Algorithm::PrintM(mpz_class M)
{ // Do not change this, right format for the grading script
  cout << "M " << M << endl;
}

void RSA_Algorithm::PrintC(mpz_class C)
{ // Do not change this, right format for the grading script
  cout << "C " << C << endl;
}




