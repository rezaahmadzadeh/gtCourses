#include <fstream>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>
#include "../cache_server.h"

using namespace std;
using namespace apache::thrift;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;

using namespace aos;

int main () {
    boost::shared_ptr<TTransport> socket(new TSocket("192.93.8.101", 9090));
    boost::shared_ptr<TTransport> transport(new TBufferedTransport(socket));
    boost::shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));
    cache_serverClient client(protocol);

    try {
        transport->open();

        string body;
        struct timeval tbegin,tend;
        int compt = 1;

        ifstream fp("./url_sort.txt");
        if(!fp) {
            cerr << "Cannot open " << endl;
        }
        string line;

        while(getline(fp,line)) {
            double texec=0.;
            gettimeofday(&tbegin,NULL);

            client.ping(body,line);

            gettimeofday(&tend,NULL);
            texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
            std::ofstream ft;
            ft.open("./t_test.csv", std::ios::app);
            ft << compt << ',' << texec << "\n";
            ft.close();
            compt++;

            //cout << body << endl;
        }

        transport->close();
    } catch (TException& tx) {
        cout << "ERROR: " << tx.what() << endl;
    }
}
