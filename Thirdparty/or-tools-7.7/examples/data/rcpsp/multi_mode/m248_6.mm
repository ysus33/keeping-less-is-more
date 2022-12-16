************************************************************************
file with basedata            : cm248_.bas
initial value random generator: 110884477
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  104
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       34       15       34
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        2          2           8   9
   3        2          3           5   9  10
   4        2          3           8  10  17
   5        2          2           6  11
   6        2          3           7   8  14
   7        2          3          13  15  17
   8        2          2          13  15
   9        2          1          12
  10        2          2          12  16
  11        2          3          12  13  17
  12        2          1          14
  13        2          1          16
  14        2          1          15
  15        2          1          18
  16        2          1          18
  17        2          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     1       4    4    9    8
         2     7       3    2    8    5
  3      1     3       8    7    7    6
         2     8       7    5    6    4
  4      1     4       9    4    8   10
         2     5       9    4    6    8
  5      1     4       7    7    5   10
         2     5       7    6    4    8
  6      1     6       9    8    8    9
         2    10       4    4    4    9
  7      1     1       9    9    6    5
         2     1      10    3    3    7
  8      1     9       9    7    5    7
         2    10       7    6    2    2
  9      1     2       5   10    7    6
         2     4       3    8    4    6
 10      1     1       8    7    6    8
         2    10       5    6    6    5
 11      1     5       3    2    7    8
         2     9       3    1    6    7
 12      1     2       9   10    1    5
         2     2       7    5    1    6
 13      1     7       5    5    8    6
         2     7       6    5    6    4
 14      1     5       8    8   10    5
         2     8       2    6    5    3
 15      1     3       4   10    8    5
         2     4       3   10    4    5
 16      1     5       4    9    7    5
         2     6       3    8    4    1
 17      1     3       5    5   10    9
         2     8       4    3   10    6
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   28   26   96   99
************************************************************************
