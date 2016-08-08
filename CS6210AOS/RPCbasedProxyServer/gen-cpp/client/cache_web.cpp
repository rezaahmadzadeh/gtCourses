#include <iostream>
#include <string>
#include <map>
#include <deque>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <fstream>


#include "cache_web.h"

#define CACHE_MAX_SIZE 1024*100


/*
 * PARENT CACHE *
 */
void ParentCache::inc_hit() {
    hit_number++;
}

void ParentCache::inc_request() {
    request_number++;
}

void ParentCache::data_publish() {
    double hit_rate = ((double) hit_number)/((double) request_number);
    std::ofstream fp;
    fp.open("./hr_test.csv", std::ios::app);
    fp <<  request_number<< ',' << hit_rate << "\n";
    fp.close();
}

bool ParentCache::isURL (std::string url) {
    Cache::iterator it = cache.find(url);
    if(it!= cache.end()) {return true;}
    else {return false;}
}


/*
 * RANDOM CACHE *
 */
RandomCache::RandomCache() {
    sz = 0;
    hit_number = request_number = 0 ;
}

void RandomCache::replace(std::string url, std::string body) {

    if (body.size() > CACHE_MAX_SIZE) {
        //std::cout << "Web page too big for the cache random" << std::endl;
        }
    else if ( sz + body.size() < CACHE_MAX_SIZE) {
        //std::cout << "Add web page to cache without replacement" << std::endl;
        cache[url] = body;
        index.push_back(url);
        sz = sz + body.size();
    }
    else {
        //std::cout << "Run replacement algorithm" << std::endl;
        while(sz+body.size()>CACHE_MAX_SIZE) {
            int r = rand() % index.size();
            sz = sz - cache[index[r]].size();
            cache.erase(index[r]);
            index.erase(index.begin()+r);
        }
        cache[url] = body;
        index.push_back(url);
        sz = sz + body.size();
        //std::cout << cache[url] << std::endl;
    }
}

std::string RandomCache::retrieve(std::string url) { 
    return cache[url];
}


/*
 * LRU CACHE * 
 */
LRUCache::LRUCache() {
    sz = 0;
    hit_number = request_number = 0 ;
}


void LRUCache::replace(std::string url, std::string body) {
    if (body.size() > CACHE_MAX_SIZE) {
        //std::cout << "Web page too big for the cache lru" << std::endl;
    }
    else if ( sz + body.size() < CACHE_MAX_SIZE) {
        //std::cout << "Add web page to cache without replacement" << std::endl;
        cache[url] = body;
        index.push_back(url);
        sz = sz + body.size();
    }
    else {
        //std::cout << "Run replacement algorithm" << std::endl;
        while(sz+body.size()>CACHE_MAX_SIZE) {
            sz = sz - cache[index.front()].size();
            cache.erase(index.front()); 
            index.pop_front();
        }
        cache[url] = body;
        index.push_back(url);
        sz = sz + body.size();
    }
}

//assumes the web page is in the cache
std::string LRUCache::retrieve(std::string url) { 
    URL_Index::iterator it = find(index.begin(), index.end(), url);
    index.erase(it);
    index.push_back(url);
    return cache[url];
}


/*
 * LRU_MIN *
 */
LRU_MINCache::LRU_MINCache() {
    sz = 0;
    hit_number = request_number = 0 ;
}

void LRU_MINCache::replace(std::string url,std::string body) {

    if (body.size() > CACHE_MAX_SIZE) {
        //std::cout << "Web page too big for the cachelru-min" << std::endl;
    }
    else if ( sz + body.size() < CACHE_MAX_SIZE) {
        // std::cout << "Add web page to cache without replacement" << std::endl;
        cache[url] = body;
        index.push_back(url);
        sz = sz + body.size();
    }
    else {
        //std::cout << "Run replacement algorithm" << std::endl;
        size_t sz_tmp = (size_t) body.size()/2;

        while (sz+body.size()>(size_t)CACHE_MAX_SIZE) {
            unsigned i = 0;
            while ((sz+body.size()>(size_t)CACHE_MAX_SIZE) && (i<index.size())) {
                std::string url_tmp = index[i];
                size_t size_body = cache[url_tmp].size();

                if( size_body >= sz_tmp ) {
                    sz = sz - size_body;
                    cache.erase(url_tmp);
                    URL_Index::iterator it = index.begin() + i;
                    index.erase(it);
                }
                i++;
            }

            sz_tmp = (size_t) sz_tmp/2;

        }
        cache[url] = body;
        index.push_back(url);
        sz = sz + body.size();
    }
}



    std::string LRU_MINCache::retrieve(std::string url) { 
        URL_Index::iterator it = find(index.begin(), index.end(), url);
        index.erase(it);
        index.push_back(url);
        return cache[url];
    }



