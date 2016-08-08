This project compares the performance of several caching policy in an proxy server application.

Index:
- report.pdf: Experiment description and analysis.
- Result_analysis: Experiment values
- gen-cpp: Cache proxy server implementation




The code folder contains a thrift file used to generate the server skeleton via : 

thrift --gen cpp cache_server.thrift

Once created, we complete the skeleton to implement the desired service : a proxy web server with the desired caching policy.

If the user wants to run both the server and the client on the same machine, it must compile the programs in the 'gen-cpp' folder with 'make' after setting the right path for the path and curl library.

If the user wants to run the server and the client on different machines, he must download the 'gen-cpp/server' folder in the server machine and the 'gen-cpp:client' folder in the client machine. The compilation is done via 'make' after the setting the appropriate path for thirft and curl. 
Before running the client, the user must complete the main with the server IP address. The user must run the server before running the client.

The experimental result are obtained through C++ writing file methods. When the user run the server and the client, the server measures the hit rate all along the excution and the client measure the time service. They produce a 'hr_test.csv' and 't_test.csv' file. 

If the user wants to run experiment for a particular setting, he can : 
-set the desired cache policy in 'cache_server_server.cpp' in the main part by uncommenting the line corresponding to the wanted cache.
-set the cache size in 'cache_web.cpp' through the macro definition at the beginning of the file CACHE_MAX_SIZE
-set the workload in the 'scqche_server_client.cpp' file in the fstream methodby choosing either 'url_sort.txt' or 'url_shuffle.txt' for respectively the
sorted workload and the random workload.

We gather all the experimental results both with a MATLAB file 'AOS3.m' to exploit and visualize them in the 'Result_analysis' folder. The user needs to uncomment the results it want to see to get the corresponding graph/values.
