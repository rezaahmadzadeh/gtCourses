#kmeans

Clustering application for image compression. 
Homework1.m contains the image processing part : it reads an RGB bitmap image file, then clusters pixels with the given number of clusters. It shows converted image only using K colors, each of them with the representative color of the centroid. 

To see what it looks like : import the project in Matlab and run homework1('up2.bmp',3)

The clustering is implemented using two algorithms : K-means and K-medoids with the manhattan distance. It follows the following iterative procedure : 
- Initialize the cluster centers.
- Iterate until conveergence: 
	- Update the cluster assignements for every data point. 
	- Update the center of each cluster. 
