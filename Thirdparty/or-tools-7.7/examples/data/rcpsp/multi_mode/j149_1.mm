************************************************************************
file with basedata            : md137_.bas
initial value random generator: 20247
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  16
horizon                       :  116
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     14      0       15       10       15
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           8  13  14
   3        3          2           5  10
   4        3          3           6   9  10
   5        3          3           6   7  14
   6        3          3          11  12  13
   7        3          1           8
   8        3          1           9
   9        3          2          11  12
  10        3          3          13  14  15
  11        3          1          15
  12        3          1          15
  13        3          1          16
  14        3          1          16
  15        3          1          16
  16        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     8       4    0   10    0
         2     8       0    8    9    0
         3    10       0    6    9    0
  3      1     6       9    0    7    0
         2     7       6    0    5    0
         3    10       0    9    0    9
  4      1     8       0    3    0    3
         2     9       4    0    6    0
         3     9       4    0    0    3
  5      1     1       0    5    0    4
         2     4       7    0    8    0
         3     9       0    2    8    0
  6      1     4       0    7    0    8
         2     6       0    7    0    6
         3     8       0    6    0    5
  7      1     1       0    8    0    7
         2     8       8    0    2    0
         3     9       0    6    0    6
  8      1     1       0    2    0    6
         2     3       8    0    0    6
         3     9       3    0    0    6
  9      1     1       0    6    0    8
         2     1       0    5    2    0
         3     8       1    0    0    8
 10      1     1       0    8    5    0
         2     7       6    0    0    6
         3     8       0    3    4    0
 11      1     2       9    0    0    6
         2     3       0    7    7    0
         3     7       8    0    0    2
 12      1     1       0    9    0   10
         2     6       5    0    4    0
         3     7       1    0    0    5
 13      1     1      10    0    0   10
         2     3       0    4    6    0
         3     7       8    0    4    0
 14      1     3       0    7    0    7
         2     6       5    0    0    4
         3     9       3    0    7    0
 15      1     1       0    6    0    5
         2     3       0    3    0    3
         3     6       7    0    7    0
 16      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
    6   10   40   50
************************************************************************