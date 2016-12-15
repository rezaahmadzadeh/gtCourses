#!/bin/csh
set user = $1
set Expected = HelloFrom.abenbihi3.FMYbcJNRPaKyZLUteAwRhFmCJBzHrbMvRshFnmYm
set Actual = `./BreakRSA  3589763135383532911 43940595153509089  1550655505597288069 1167253584470367775 2049257334136825518 444355567217571341 3387138883261841651 53269650188955293 3385549031949600827 3387333131954743006 1873865792703532520 323982924407895918`
echo "expected is $Expected" 
echo "actal    is $Actual"
if ( "$Expected" == "$Actual" ) then
echo "Grade for user $user is 100"
else
echo "Grade for user $user is 50"
endif
