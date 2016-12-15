/**
 * \file GraphSearch.h
 * \brief Graph structure and Dijkstra search
 */

#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "types.h"
#include <list>

/**
 * Index of the joint in the array storing all solutions.
 */
typedef int Vertex;

/**
 * q_norm norm between two joints.
 */
typedef float Weight;

/**
 * Adjacence list reprensentation: adjList[i] = {(j,weight(i,j)} with j neighbor of i.
 */
typedef std::list< std::pair<Vertex,Weight> > AdjList;

/**
 *
 */
typedef std::pair<Weight, Vertex> InvertedNode;

class Graph {
    public:
        /**
         * Number of vertices
         */
        int V; 

        /**
         * Adjacence list array. One list per vertex
         */
        AdjList *adj; 

    public:
        /**
         * \fn Graph();
         * \brief Default constructor 
         */
        Graph();

        /**
         * \fn ~Graph()
         * \brief Destructor (free memory) 
         */
        ~Graph();

        /**
         * \fn Graph(int V)
         * \param V  number of vertices.
         * \brief Allocate memory for a graph of v vertices.
         */
        Graph(int V);

        /**
         * \fn void addEdge(Vertex u, Vertex v, Weight w);
         * \brief Add an edge between u and v with weight w.
         * \param[in] u vertex
         * \param[in] v: vertex
         * \param[in] w: weight
         */
        void addEdge(Vertex u, Vertex v, Weight w);

        /**
         * \fn void shortestPath(Vertex src, Vertex dst, Vertex* parent);
         * \brief Find the shortest path between src and dst, and stores the path in parent.
         * \param[in] src Start of the path
         * \param[in] dst: End of the path
         * \param[out] parent: parent[i]=j where j is the parent of i in the shortest path
         */
        void shortestPath(Vertex src, Vertex dst, Vertex* parent);

        /**
         * \fn void print(std::vector<q_t> ikSolutions);
         * \brief (Debug purpose) Prints the neighboring joints for each vertex.
         * \param[in] ikSolutions: Array of all constrained joint solutions (several joints/ee)
         */
        void print(std::vector<q_t> ikSolutions);

};

#endif
