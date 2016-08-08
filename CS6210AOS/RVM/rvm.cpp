#include "rvm_internal.h"
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <errno.h>
#include <unistd.h>

Trans_list trans_list;
DIR_to_RVM dir_to_rvm; //<direcory, RVM>
VM_to_Segname VM_to_segname;
int gtid = 0;



//assumes the segment is unmapped, and then out of any transaction
void apply_redoLog (const char* redo_full_name, const char* segment_full_name) {

    std::ifstream is(redo_full_name, std::ifstream::binary);
    std::ofstream os(segment_full_name, std::ofstream::binary);

    if (is) {
        is.seekg(0,is.end);
        int length = is.tellg();
        char* buffer = new char[length];
        is.seekg(0,is.beg);
        is.read(buffer,length);

        if(os) {
            os.write(buffer,length);
            os.close();
        }

        delete[] buffer;
        is.close();
        remove(redo_full_name);  
    }
}



//return true if the region is in transaction
bool is_in_trans(void* segbase) {
    bool tmp = false;
    Trans_list::iterator it =  trans_list.begin();
    while( (!tmp) && (it!=trans_list.end()) ) {
        tmp = (it->second).is_segbase(segbase);
        it ++;
    }
    return tmp;
}



rvm_t rvm_init(const char *directory) {

    //if the directory does not exist
    struct stat info;
    if ( (stat(directory, &info) == -1) || !(S_ISDIR(info.st_mode)) ) {
        mkdir(directory, S_IRWXU | S_IRWXG | S_IRWXO);
    }

    //if the rvm runtime does not exist create it, else return the existing one
    DIR_to_RVM::iterator it = dir_to_rvm.find(directory);
    if(it != dir_to_rvm.end()) { return it->second; }
    else {
        rvm_t rvm(directory);
        dir_to_rvm[directory] = rvm;
        return rvm;
    }
}



void *rvm_map(rvm_t rvm, const char *segname, int size_to_create) {
    //check that rvm has been initialized
    if (!rvm.is_init()) {
        perror("ERROR : rvm_map : rvm not initialized");
        exit(2);
    }

    std::string str_segname(segname);
    std::string str_dir(rvm.dir);
    std::string segment_full_name = str_dir + '/' +  str_segname;
    std::string redo_full_name = str_dir + '/' + str_segname + "_redo";

    struct stat EDS_info;
    struct stat redo_info;

    //if segment does not exist create it at the right size
    if ( stat(segment_full_name.c_str(),&EDS_info) == -1) {
        std::ofstream os ;
        os.open(segment_full_name.c_str());
        if(os) {
            os.close();
            truncate(segment_full_name.c_str(),size_to_create);} 
        else {
            std::cout << "ERROR : rvm_map : Fail at opening the file" << std::endl;}
    }

    //if segment exists and is already mapped
    if (rvm.has_mapped_segment(segname)) {
        perror("ERROR : rvm_map : Try to map a segment that is already mapped");
        exit(2);
    }
    //if segment exists, is not mapped
    else {
        stat(segment_full_name.c_str(),&EDS_info);
        int segment_size = EDS_info.st_size;

        //if segment is shorter one resize it
        if (segment_size < size_to_create) {
            truncate(segment_full_name.c_str(), size_to_create);
            if (stat(redo_full_name.c_str(),&redo_info) == 0 ) {
                truncate(redo_full_name.c_str(), size_to_create);
            }
        }
        //if segment is bigger
        else if( segment_size > size_to_create) {
            perror("ERROR : rvm_map :The segment is bigger than size_to_create");
            exit(2);
        }
    }

    //if needed update segment and copy persistent data from disk to memory
    void * VM_range_start = malloc(size_to_create);
    if (stat(redo_full_name.c_str(),&redo_info)== 0 ) {
        apply_redoLog(redo_full_name.c_str(),segment_full_name.c_str()); 
    }

    std::ifstream is;
    is.open(segment_full_name.c_str());
    is.read( (char *) VM_range_start, size_to_create);
    is.close();

    Segment segment(segname, segment_full_name.c_str(), size_to_create, VM_range_start);
    rvm.mapped_segments.push_back(segment);
    VM_to_segname[VM_range_start] = segment_full_name;
    return VM_range_start;
}



void rvm_unmap(rvm_t rvm, void *segbase) {

    if( rvm.has_mapped_segbase(segbase)) {
      perror("unmap ERROR : this segment is not mapped by this rvm");
      exit(2);
      }

    //check is segbase is not already in transaction
    if (is_in_trans(segbase)) {
        perror("ERROR : rvm_unmap : This segment is in a transaction.");
        exit(2);
    }

    rvm.remove_segment(segbase);
    free(segbase);
    VM_to_segname.erase(segbase);
}



void rvm_destroy(rvm_t rvm, const char *segname) {
    //check if the segment exists in the directory
    const char* dir = rvm.dir;

    std::string str_segname(segname);
    std::string str_dir(dir);
    std::string segment_full_name = str_dir + "/" + str_segname;

    DIR* dirp = opendir(dir);
    struct dirent* entry;
    bool tmp = false;
    while( ((entry = readdir(dirp)) != NULL) && (tmp==false) ) {
        std::string full_name = entry->d_name;
        if (segment_full_name==full_name) {
            tmp=true;}
    }
    closedir(dirp);

    //if the segment exists and is mapped : ERROR
    if (rvm.has_mapped_segment(segname)) {
        perror("ERROR : rvm_destroy : Try to destroy mapped segment");
        exit(2);
    }
    //if the segment does not exist, do nothing
    if(tmp==false) {
    }
    //if segment is not mapped, destroy it and its redo log
    else {
        std::string redo_full_name = dir + '/' + str_segname + "_redo";
        remove(segment_full_name.c_str());
        remove(redo_full_name.c_str());
    }
}



trans_t rvm_begin_trans(rvm_t rvm, int numsegs, void **segbases) {
    int i=0;
    bool tmp_mapped = true;
    bool tmp_trans = false;

    while( (i<numsegs) && (tmp_mapped) && (!tmp_trans)) {
        tmp_mapped = rvm.has_mapped_segbase(segbases[i]);
        tmp_trans= is_in_trans(segbases[i]);
    }
    //check that all segment in segbases are mapped by this rvm
    if (tmp_mapped) {
      perror("ERROR : there is an unmapped segment in segbases");
      exit(2);
      }

    //check segment is not already in transaction
    if (tmp_trans) {
        perror("ERROR : rvm_begin_trans :segment already in transaction");
        exit(2);
    }

    int trans_id = gtid;
    gtid ++;
    TID tid(trans_id, numsegs, segbases);
    trans_list[trans_id] = tid;
    return (trans_t) trans_id;
}



void rvm_about_to_modify(trans_t tid, void *segbase, int offset, int size) {
    // check that the segment is in the segbases of this tid
    Trans_list::iterator it = trans_list.find(tid);
    bool tmp  = (it->second).is_segbase(segbase);
    if( !tmp) {
        perror("ERROR : rvm_about_to_modify : the segment is not part of this transaction");
    }
    else {
        //create undo_record, copy the VM region into it and add it to the undoLog of the transaction
        UndoRecord u;
        u.VM_start = segbase;
        u.offset = offset;
        u.size = size;
        u.region = (char*) malloc(size);
        memcpy(u.region, (char*) segbase+offset,size);
        (it->second).add_undoRecord(u);
    }
}



void rvm_commit_trans(trans_t tid) {
    Trans_list::iterator it = trans_list.find(tid);
    if(it == trans_list.end()) {
        perror("ERROR : rvm_commit_trans :this TID does not exist");
        exit(2);
    }

    UndoLog u = (it->second).undoLog;
    UndoLog::iterator uit;
    for (UndoLog::iterator uit = u.begin(); uit != u.end(); uit ++) {
        //set up
        struct stat EDS_info;
        UndoRecord ur = *uit;
        void* segbase = uit->VM_start;
        const char* segment_full_name = (VM_to_segname[segbase]).c_str();
        std::string str_segname(segment_full_name);
        const char* redo_full_name = (str_segname + "_redo").c_str();
        stat(segment_full_name,&EDS_info);
        int segment_size = EDS_info.st_size;

        //update
        std::ofstream os;
        os.open(redo_full_name);
        os.write( (char*) segbase, segment_size);
        os.close();

        //cleaning
        free(ur.region);     
    }
    trans_list.erase(it);
}



void rvm_abort_trans(trans_t tid) {

    Trans_list::iterator it = trans_list.find(tid);
    if(it == trans_list.end()) {
        perror("ERROR : rvm_abort_trans :this TID does not exist");
        exit(2);
    }

    UndoLog u = (it->second).undoLog;
    UndoLog::iterator uit;

    for (UndoLog::iterator uit = u.begin(); uit != u.end(); uit ++) {
        memcpy( (char*) uit->VM_start + uit->offset, uit->region, uit->size);
        free(uit->region);
    }
    trans_list.erase(it);    
}



void rvm_truncate_log(rvm_t rvm) {
    // for all redo logs whose EDS is not mapped, rewrite

    DIR* dirp = opendir(rvm.dir);
    struct dirent* entry;
    std::string str_dir(rvm.dir);

    while( (entry = readdir(dirp)) != NULL) {
        std::string full_name = str_dir + '/' + entry->d_name;

        //if it is a redo log, check if the segment mapped and if not truncate
        if(full_name.find("_redo") !=  std::string::npos ) {
            std::string redo_full_name = full_name;
            unsigned sz = full_name.size()-5;
            full_name.resize(sz); //segment_full_name
            if(rvm.has_mapped_segment(full_name.c_str()) ) { }
            else { apply_redoLog(redo_full_name.c_str(),full_name.c_str()); }
        }
    }
    closedir(dirp);
}
