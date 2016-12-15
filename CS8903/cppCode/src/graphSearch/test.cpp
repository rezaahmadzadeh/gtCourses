

#include <stdio.h>
#include <stdlib.h>
#include "GraphSearch.h"
#include "misc.h"
#include <stack>

int main(){
    
    Graph graph(11);
    graph.addEdge(0,1,0.0);
    graph.addEdge(0,2,0.0);
    graph.addEdge(0,3,0.0);
    graph.addEdge(1,4,2.0);
    graph.addEdge(1,5,2.0);
    graph.addEdge(2,4,2.0);
    graph.addEdge(2,5,1.0);
    graph.addEdge(3,4,2.0);
    graph.addEdge(3,5,2.0);
    graph.addEdge(4,6,2.0);
    graph.addEdge(4,7,2.0);
    graph.addEdge(4,8,2.0);
    graph.addEdge(4,9,2.0);
    graph.addEdge(5,6,1.0);
    graph.addEdge(5,7,2.0);
    graph.addEdge(5,8,2.0);
    graph.addEdge(5,9,2.0);
    graph.addEdge(6,10,0.0);
    graph.addEdge(7,10,0.0);
    graph.addEdge(8,10,0.0);
    graph.addEdge(9,10,0.0);

    Vertex parent[graph.V];
    Vertex src=0;
    Vertex dst=10;
    graph.shortestPath(src,dst,parent);

    Vertex trajectory = dst;
    int iter=0;
    trajectory = parent[trajectory];
    while( (trajectory!=src) && (iter<(graph.V+1))){
        std::cout << trajectory << std::endl;
        trajectory = parent[trajectory];
        iter++;
    }


}
