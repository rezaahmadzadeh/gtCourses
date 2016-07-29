#Text clustering with expectation maximization algorithm

The goal is to find how to group a set of different text documents based on their topics. 

The model is detailed in the following file : documentModel.pdf. 

Index : 
- documentModel.pdf : Exposes the model.
- mycluster.m : Implements the EM algorithm.
- AccMeasure.m : Computes an evaluation function.
- homework2.m : Main program.
- data.mat : dataset. The data contains a 400*101 matrix called X, in which the last column is the true label to help evaluate the algorithm.
