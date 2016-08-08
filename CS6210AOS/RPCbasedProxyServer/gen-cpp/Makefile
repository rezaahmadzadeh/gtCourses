GEN_SRC := cache_server.cpp cache_server_constants.cpp cache_server_types.cpp cache_web.cpp
GEN_OBJ := $(patsubst %.cpp,%.o, $(GEN_SRC))
CC := g++
FLAGS := -Wall -g 
THRIFT_DIR := /usr/local/include/thrift
BOOST_DIR := /usr/local/include
CURL_DIR := /usr/include
LDFLAGS := -L/usr/lib/x86_64-linux-gnu -L/usr/local/lib 
LDLIBS := -lcurl -lthrift 

INC := -I$(THRIFT_DIR) -I$(BOOST_DIR) -I$(CURL_DIR)

.PHONY: all clean

all: cache_server_server cache_server_client

%.o: %.cpp
	$(CC) -Wall -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H $(INC) -c $< -o $@

cache_server_server: cache_server_server.o $(GEN_OBJ)
	$(CC) $^ -o $@  $(LDFLAGS) $(LDLIBS)

cache_server_client: cache_server_client.o $(GEN_OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	$(RM) *.o cache_server_server cache_server_client


