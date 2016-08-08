#include <map>
#include <vector>
#include <list>
#include <algorithm>
#include<string>

typedef char* Segname;
typedef std::map<void*, std::string> VM_to_Segname;
typedef int trans_t;

/*SUMMARY :
 * typedef std::list<UndoRecord> UndoLog;
 * typedef std::map<int,TID> Trans_list; 
 * typedef std::list<Segment> MappedSegments;
 * typedef std::map<const char*, rvm_t> DIR_to_RVM;
*/



class UndoRecord {
    public:
	void* VM_start; //address of the beginning of the VM region
	int offset;
	int size;
	char* region; //VM region mapped to the EDS


    UndoRecord() {};
};



typedef std::list<UndoRecord> UndoLog;



class TID {
	public:
		int ID;
		char* dir;
		UndoLog undoLog;
		int numsegs;
		std::list<void*> VM_region_list;
          

        TID() {};


		TID(int id, int n, void** segbases) {
			ID = id;
			numsegs = n;
			for(int i=0; i<numsegs; i++) {
				VM_region_list.push_back(segbases[i]);
            }
		}


		bool is_segbase(void* segbase) {
			std::list<void*>::iterator it = find( VM_region_list.begin(),VM_region_list.end(), segbase);
			if(it!=VM_region_list.end()) { return true; }
			else { return false; }
		}


		void add_undoRecord (UndoRecord u) {
			undoLog.push_back(u);
		}
};



typedef std::map<int,TID> Trans_list;



class Segment {
	public:
		const char* local_name;
		const char* full_name;
		int size;
		void* VM_region;
		bool in_trans;

	public:
		Segment(const char* ln, const char* fn, int s, void* r) {
			local_name = ln;
			full_name = fn;
			size = s;
			VM_region = r;
			in_trans = false;
        }
};



typedef std::list<Segment> MappedSegments;



class rvm_t {
	public:
		const char* dir;
		MappedSegments mapped_segments;
		std::list<int> trans_list;
		int init;

	public:
		rvm_t() {
			init = -1;
		}

		rvm_t(const char* directory) {
			dir = directory;
			init = 0;
		}

		bool is_init() {
			if(init == -1) { return false; }
			else { return true; } 
		}	

        //return true is segment mapped
		bool has_mapped_segment(const char* segname) {
			bool tmp = false;
			MappedSegments::iterator it = mapped_segments.begin();
			while( (it !=  mapped_segments.end()) && (tmp!=true))  {
				if (it->local_name == segname) { tmp = true; }
				else { it++; }
			}
			return tmp;
		}


        //return true if segment mapped
		bool has_mapped_segbase(void* segbase) {
			bool tmp = false;
			MappedSegments::iterator it = mapped_segments.begin();
			while( (it !=  mapped_segments.end()) && (tmp!=true))  {
				if (it->VM_region == segbase) { tmp = true; }
				else { it++; }
			}
			return tmp;
		}

		
		bool remove_segment(void* segbase) {
			bool tmp = false;
			MappedSegments::iterator it = mapped_segments.begin();
			while( (it !=  mapped_segments.end()) && (tmp!=true))  {
				if (it->VM_region == segbase) { tmp = true;}
				else { it++; }
			}
			if (tmp) { mapped_segments.erase(it);}
			else{ return tmp;}
		}
				
};



typedef std::map<const char*, rvm_t> DIR_to_RVM;
