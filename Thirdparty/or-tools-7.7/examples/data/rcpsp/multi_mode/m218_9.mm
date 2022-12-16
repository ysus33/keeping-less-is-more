************************************************************************
file with basedata            : cm218_.bas
initial value random generator: 437259177
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  115
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       23        5       23
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        2          2           6   8
   3        2          3           5   7  11
   4        2          3           6  16  17
   5        2          3          10  12  15
   6        2          1           9
   7        2          3           9  10  12
   8        2          3          10  13  14
   9        2          1          13
  10        2          2          16  17
  11        2          2          13  14
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
  2      1     4       7    0    6    0
         2     4       0    2    9    0
  3      1     4       2    0    4    0
         2     8       0    3    0    5
  4      1     6       0    3    6    0
         2     6       0    8    0    2
  5      1     4       0    4    0    5
         2     6       3    0    0    4
  6      1     7       2    0    0    9
         2     8       0    5    0    9
  7      1     2       0    7    3    0
         2    10       7    0    0   10
  8      1     5       3    0    7    0
         2     7       2    0    0    6
  9      1     3       8    0    0    7
         2     5       0    6    0    5
 10      1     1       0    5    0    7
         2     6       4    0    8    0
 11      1     1       6    0    7    0
         2     6       0    5    0    6
 12      1     6       0    4    3    0
         2     6       4    0    3    0
 13      1     6       9    0    0    6
         2     9       7    0    0    6
 14      1     4       9    0    7    0
         2     8       0    2    0    5
 15      1     1       4    0    9    0
         2    10       3    0    0    4
 16      1     1       0    7    0    1
         2     8       0    5    3    0
 17      1     2       0    7    0    5
         2     8       0    3    7    0
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   13   13   57   65
************************************************************************
