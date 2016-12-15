// ECE4122/6122 RSA Encryption/Decryption assignment
// Fall Semester 2015

#include <iostream>
#include "RSA_Algorithm.h"

using namespace std;

#define ITERATION 100
#define KEYSIZE_MAX 2000

int main()
{
    // Instantiate the one and only RSA_Algorithm object
    RSA_Algorithm RSA;

    // Loop from sz = 32 to 1024 inclusive
    // for each size choose 10 different key pairs
    // For each key pair choose 10 differnt plaintext 
    // messages making sure it is smaller than n.
    // If not smaller then n then choose another
    // For eacm message encrypt it using the public key (n,e).
    // After encryption, decrypt the ciphertext using the private
    // key (n,d) and verify it matches the original message.

    // your code here

    std::size_t keySize = 32;
    int i,j;
    mpz_class M,C,M_c;
    bool isMessage = false;

    while(keySize<KEYSIZE_MAX){
        for(i=0;i<ITERATION;i++){
            RSA.GenerateRandomKeyPair(keySize);
            RSA.PrintNDE();

            for(j=0;j<ITERATION;j++){
                isMessage = false;
                while(!isMessage){
                    M = RSA.rng.get_z_bits(keySize);
                    if(M<RSA.n){
                        isMessage = true;
                    }
                }

                RSA.PrintM(M);
                C = RSA.Encrypt(M);
                RSA.PrintC(C);
                M_c = RSA.Decrypt(C);
                //RSA.PrintM(M_c);

                if (M==M_c) {
                    //std::cout << "OK" << std::endl;
                }
                else{
                   // std::cout << "Fail" << std::endl;
                }
            }
        }
        keySize*=2;
        //std::cout << keySize << std::endl;
    }

}
  
