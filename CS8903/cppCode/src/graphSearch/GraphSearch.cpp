/**
 * \file GraphSearch.cpp
 * \brief Graph search implementation. Check this site for algo and structure definitions. (http://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/)
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <list>
#include <utility>
#include <set>
#include <limits>

#include  "GraphSearch.h"


Graph::Graph(){

}

Graph::~Graph(){
    if(adj!=NULL){
        delete[] adj;
        adj=NULL;
    }
}

Graph::Graph(int V){
    this->V = V;
    adj = new AdjList[V];
}

void Graph::addEdge(Vertex u, Vertex v, Weight w){
    adj[u].push_back(std::make_pair(v,w));
}

void Graph::shortestPath(Vertex src, Vertex dst, Vertex* parent){
    std::set< InvertedNode > setds; 
    float MAX_F = std::numeric_limits<float>::max();
    std::vector<Weight> dist(V,MAX_F);
    setds.insert(InvertedNode(0.0, src));
    dist[src]=0.0;
    parent[0]=-1;

    while(!setds.empty()){
        // Get the nearest vertex in the set
        InvertedNode tmp = *(setds.begin()); 
        setds.erase(setds.begin());
        Vertex u = tmp.second; // get vertex u

        // Update the weights of all neighbors of u and store the nearest one in set
        AdjList::iterator it;
        for(it = adj[u].begin(); it!=adj[u].end(); ++it){
            Vertex v = (*it).first;
           /* 
            if(v == dst) {
                parent[v]=u; // if u leads to dst, stop searching
                break;
            }
            */
            Weight w = (*it).second;
            if(dist[v] > (dist[u]+w)){
                if(dist[v]!=MAX_F){
                    setds.erase(setds.find(InvertedNode(dist[v],v)));
                }
                dist[v] = dist[u]+w;
                setds.insert(InvertedNode(dist[v],v));
                parent[v] = u;
            }
        }
    }
}

void Graph::print(std::vector<q_t> ikSolutions){
    // For each vertex, print the corresponding joint configuration and the one of its neighbors.
    for(int i=0;i<V;i++){
        std::cout << "\nNode: " << i << "\t" << 
            ikSolutions[i][0] << " " <<  
            ikSolutions[i][1] << " " <<  
            ikSolutions[i][2] << " " <<  
            ikSolutions[i][3] << " " <<  
            ikSolutions[i][4] <<  " " <<   
            ikSolutions[i][5] << std::endl;
        
        for(AdjList::iterator it=adj[i].begin();it!=adj[i].end();it++){
            int qIdx = (*it).first;
            std::cout << qIdx << "\t" << 
                ikSolutions[qIdx][0] << " " <<  
                ikSolutions[qIdx][1] << " " <<  
                ikSolutions[qIdx][2] << " " <<  
                ikSolutions[qIdx][3] << " " <<  
                ikSolutions[qIdx][4] << " " <<  
                ikSolutions[qIdx][5] << "\t" << 
                (*it).second << std::endl;  
        }
    }
}

