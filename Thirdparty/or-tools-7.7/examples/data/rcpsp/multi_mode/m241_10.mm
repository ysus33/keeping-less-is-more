************************************************************************
file with basedata            : cm241_.bas
initial value random generator: 232848095
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  124
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       19       13       19
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        2          2           6  14
   3        2          3           6   8   9
   4        2          3           5   7  11
   5        2          2          13  17
   6        2          3          13  16  17
   7        2          2          14  15
   8        2          3          11  12  13
   9        2          2          10  15
  10        2          2          11  12
  11        2          1          16
  12        2          1          14
  13        2          1          15
  14        2          2          16  17
  15        2          1          18
  16        2          1          18
  17        2          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     1       0    6    7    3
         2     1       0    9    3    3
  3      1     2       8    0    5    7
         2     8       0    2    4    5
  4      1     1       0    4   10    7
         2     9       8    0    9    3
  5      1     6       3    0    3    6
         2    10       0    3    2    4
  6      1     7       0    8    6    5
         2     9       2    0    1    4
  7      1     3       0    9    9    9
         2     4       7    0    1    9
  8      1     1       0    5    6    7
         2     7       3    0    6    2
  9      1     2       0    1    8   10
         2     8       0    1    5    6
 10      1     6       0    5    7    5
         2     7       0    4    6    4
 11      1     3       8    0    7    9
         2     6       6    0    6    6
 12      1     3       0    3    7    6
         2     7       0    3    1    6
 13      1     4       8    0    6    9
         2    10       4    0    5    4
 14      1     1       6    0    7    4
         2    10       4    0    3    4
 15      1     6       0    6    3    9
         2     9       8    0    2    5
 16      1     5       0    5    5    9
         2     9       6    0    4    7
 17      1     1       8    0    8    4
         2    10       0    7    8    4
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   10    9   85   93
************************************************************************
