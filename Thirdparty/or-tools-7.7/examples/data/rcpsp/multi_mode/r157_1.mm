************************************************************************
file with basedata            : cr157_.bas
initial value random generator: 16856
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  137
RESOURCES
  - renewable                 :  1   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       20        7       20
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          2           9  11
   3        3          3           5   6   9
   4        3          3           7  10  14
   5        3          2          10  17
   6        3          3           7   8  11
   7        3          1          13
   8        3          1          12
   9        3          3          13  14  15
  10        3          1          16
  11        3          3          13  15  16
  12        3          3          14  15  16
  13        3          1          17
  14        3          1          17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0
  2      1     2       9    8    4
         2     3       7    8    3
         3    10       0    8    3
  3      1     3      10    4    8
         2     8       9    2    6
         3     8       8    3    6
  4      1     7       6    9    5
         2     8       4    6    4
         3     9       0    6    3
  5      1     2       6    5    8
         2     3       6    3    7
         3     7       5    1    4
  6      1     5       0    6    6
         2    10       9    2    4
         3    10       0    3    3
  7      1     1       0    7    7
         2     3       0    6    7
         3     5       5    6    6
  8      1     1       0    7    2
         2     7       8    5    1
         3    10       8    2    1
  9      1     1       0    4    3
         2     8       0    1    2
         3     8       3    3    1
 10      1     1       9    4    7
         2     2       6    4    7
         3     9       5    4    6
 11      1     2       0    4    7
         2     6       0    3    7
         3    10       0    3    4
 12      1     2       8    2    8
         2     7       4    2    6
         3     8       0    1    6
 13      1     7       0    5    6
         2     9       0    5    5
         3    10       0    5    4
 14      1     7       0    7    8
         2     8       9    6    8
         3     9       6    6    7
 15      1     3       7    6    9
         2     9       0    6    6
         3     9       0    5    7
 16      1     5       0    8   10
         2     7       0    6   10
         3     9       0    4   10
 17      1     2       6   10   10
         2     4       0    9   10
         3     6       0    9    9
 18      1     0       0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  N 1  N 2
   12   96  108
************************************************************************
