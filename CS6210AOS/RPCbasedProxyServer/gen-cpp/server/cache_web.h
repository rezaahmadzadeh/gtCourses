#include <iostream>
#include <string>
#include <map>
#include <deque>
#include <vector>
//#define CACHE_SIZE_MAX 1024*100

typedef std::map<std::string,std::string> Cache;
typedef std::vector<std::string> Random_Index;
typedef std::deque<std::string> URL_Index;
typedef std::pair<std::string,size_t> LRU_MIN_Entry ;
//typedef std::map<LRU_MIN_Entry> LRU_MIN_Index;
typedef std::map<std::string,size_t> LRU_MIN_Index;

/*PARENT CACHE*/
class ParentCache {

    protected:
        Cache cache;
        int hit_number;
        int request_number;

    public:
        virtual void inc_hit();
        virtual void inc_request();
        virtual void data_publish();

        virtual bool isURL (std::string url) ;
        virtual void replace(std::string url, std::string body) = 0 ;
        virtual std::string retrieve(std::string url)=0;
        virtual ~ParentCache(){};
};


/*
 * RANDOM CACHE *
 */
class RandomCache : public ParentCache { 
    private:
        int sz;
        Random_Index index;

    public:
        RandomCache();
        void replace(std::string url, std::string body);
        std::string retrieve(std::string url);
        virtual ~RandomCache(){};
};


/*
 * LRU CACHE * 
 */
class LRUCache : public ParentCache {
    private:
        int sz;
        URL_Index index;

    public:
        LRUCache();
        void replace(std::string url, std::string body);
        std::string retrieve(std::string url);
        virtual ~LRUCache(){};
};


/*
 * LRU_MIN *
 */
class LRU_MINCache : public ParentCache {
    private:
        int sz;
        URL_Index index;
        LRU_MIN_Index Lindex;

    public:
        LRU_MINCache();
        void replace(std::string url, std::string body);
        std::string retrieve(std::string url);
        virtual ~LRU_MINCache(){};
};



