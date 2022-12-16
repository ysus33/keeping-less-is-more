************************************************************************
file with basedata            : md159_.bas
initial value random generator: 2030152759
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  16
horizon                       :  121
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     14      0       22        7       22
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   6   7
   3        3          2           9  10
   4        3          2           7   9
   5        3          1           9
   6        3          2          13  15
   7        3          3           8  10  12
   8        3          2          11  14
   9        3          3          12  13  15
  10        3          3          11  13  14
  11        3          1          15
  12        3          1          14
  13        3          1          16
  14        3          1          16
  15        3          1          16
  16        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     5       9    7    4    0
         2     6       8    4    4    0
         3    10       5    3    1    0
  3      1     1      10    5    0    9
         2     6       6    5    0    9
         3     9       4    4    7    0
  4      1     8       8    9    9    0
         2    10       8    7    8    0
         3    10       8    5    0    9
  5      1     9       2    1    0    8
         2     9       2    1    3    0
         3    10       1    1    0    9
  6      1     4       5    8    0    5
         2     7       4    6    0    4
         3     9       4    2    0    3
  7      1     1       9    8    0    6
         2     5       7    8    8    0
         3     9       4    8    6    0
  8      1     2       7    7    0    7
         2     2       6    6    8    0
         3     5       6    2    0    7
  9      1     1       3    7    0    4
         2     8       2    5    3    0
         3    10       2    1    0    3
 10      1     2      10    7    6    0
         2     4       4    7    0    6
         3     5       3    3    0    3
 11      1     1       8    3    0    4
         2     9       5    3   10    0
         3    10       3    2    0    3
 12      1     3       9    4   10    0
         2     5       8    3    0   10
         3    10       6    3    0    5
 13      1     4       7    9    8    0
         2     6       7    7    0    9
         3     8       7    2    0    6
 14      1     4       7    6    0    6
         2     5       7    6    3    0
         3     8       6    6    0    4
 15      1     4       3    5    9    0
         2     7       2    4    8    0
         3     8       2    4    7    0
 16      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   22   18   88   84
************************************************************************
