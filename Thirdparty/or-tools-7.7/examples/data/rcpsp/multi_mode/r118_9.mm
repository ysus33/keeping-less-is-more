************************************************************************
file with basedata            : cr118_.bas
initial value random generator: 1121051087
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  122
RESOURCES
  - renewable                 :  1   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       18       12       18
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          2           5   9
   3        3          2           5   9
   4        3          3           6   7  13
   5        3          3          10  11  12
   6        3          3          11  15  17
   7        3          2           8  10
   8        3          2           9  12
   9        3          3          11  14  15
  10        3          1          14
  11        3          1          16
  12        3          2          15  16
  13        3          1          17
  14        3          2          16  17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0
  2      1     1       2    9    0
         2     2       0    0    4
         3     8       0    8    0
  3      1     1      10    8    0
         2     6       0    0    2
         3     6       0    8    0
  4      1     1       9    4    0
         2     6       6    0    9
         3     6       7    3    0
  5      1     1       7    7    0
         2     5       2    5    0
         3     8       0    3    0
  6      1     4       0    5    0
         2     8       0    0    3
         3     9       0    4    0
  7      1     1       4    8    0
         2     2       0    0    8
         3     5       0    0    7
  8      1     7       0    4    0
         2     9       0    3    0
         3     9       0    0    3
  9      1     3       8    3    0
         2     5       5    3    0
         3    10       1    0    4
 10      1     3       0    0    9
         2     4       4    2    0
         3    10       0    0    5
 11      1     1       0    0    4
         2     5       6    5    0
         3     8       5    4    0
 12      1     2       0    5    0
         2     7       2    0   10
         3     7       0    3    0
 13      1     2       9    0    6
         2     4       7    8    0
         3     5       2    6    0
 14      1     1       1    0    3
         2     1       0    6    0
         3     8       0    2    0
 15      1     3       0    0    7
         2     6       0    0    4
         3     6       0    3    0
 16      1     2       9    0    6
         2     5       8    4    0
         3     7       5    3    0
 17      1     5       0    1    0
         2     7       2    0    4
         3    10       2    0    3
 18      1     0       0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  N 1  N 2
   14   62   62
************************************************************************
