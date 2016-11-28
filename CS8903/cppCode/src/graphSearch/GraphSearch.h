#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "types.h"
#include <list>



typedef float Weight;
typedef int Vertex; //vertex index
typedef std::list< std::pair<Vertex,Weight> > AdjList;

class Graph {
    public:
    int V; //Number of vertices
    AdjList *adj; // Array of adjacence list

    public:
    /**
     * \fn
     * \brief 
     */
    Graph();
    
    /**
     * \fn
     * \brief 
     * TODO free memory
     */
    ~Graph();

    /**
     * \fn
     * \brief Create a graph of v vertices.
     */
    Graph(int V);


    /**
     * \fn
     * \brief Add an edge between u and v with wieght w.
     */
    void addEdge(Vertex u, Vertex v, Weight w);

    /**
     * \fn
     * \brief Find the shortest path to every node from s.
     */
    void shortestPath(Vertex src, Vertex dst, Vertex* parent);

    /**
     * \ fn
     * \ brief For debug purpose
     */
    void print(std::vector<q_t> ikSolutions);

};



#endif
