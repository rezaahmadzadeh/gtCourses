#RVM

This project implements a lightweight recoverable virtual memory based on the corresponding paper.

Index:
- rvm.cpp
- rvm.h
- Makefile

Compilation and execution:
A makefile is provided and the user must compile it using 'make'.


Rules of this LRVM library:

1. There is one RVM runtime per directory.
2. An EDS segment can be mapped only by an initialized RVM.
3. An EDS segment can be mapped only once.
4. An EDS segment can be unmapped only if it has been mapped.
5. An EDS segment can be unmapped only by the RVM which has mapped it.
6. An EDS segment can be unmapped only if it is not involved in a transaction.
7. An EDS segment can be destroyed only if it exists in the directory.
8. An EDS segment can be destroyed only if it is unmapped.
9. RVM can run a transaction only on segbases mapped and that it has mapped itself.
10. RVM can run a transaction only on segbase that is not already involved in
a transaction.
11. RVM can modify only segbase that it has announced in the transaction. We
assume that the RVM is "honest" i.e. it acts only on transactions is has
initiated.
12. We assume the user modify a segabse only within range announced at the 
mapping. RVM does not handle the case in which the modification goes beyond the announced limit.
13. RVM truncates only segments which are not mapped.


How you use logfiles to accomplish persistency plus transaction semantics ?
What goes in them? How do the files get cleaned up, so that they do not expand indefinitely ?

Every time a transaction commit, we create a file on disk for the segbase
that underwent a call to about_to_modify. This way, even if a segbase has been
mapped but not modified, there is no need to create and manage redoLogs for
it. All the redo files are stored in the directory specified y the library.
For each segbase modified, we store the region of this segbase the entire
segbase. So even even if a segbase undewent several modifications, we record
only its final state.
On the contrary, when the user call about_to_modify, we only store the part of
the segbase that is going to be modifed and that copy stays in memory.

Every time a user map a segment, we apply truncation to this specific segment
so that he gets the latest version of the segment and it is an oppurtunity to
clean the disk.
Elseway, the files get cleaned up when the user expressively call
truncate_log.




