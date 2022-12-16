************************************************************************
file with basedata            : cr19_.bas
initial value random generator: 2006855241
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  130
RESOURCES
  - renewable                 :  1   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       19       15       19
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   9  14
   3        3          3           6  10  11
   4        3          3           6   7  12
   5        3          2           6  11
   6        3          3          13  15  16
   7        3          3           8   9  16
   8        3          2          13  15
   9        3          1          10
  10        3          1          17
  11        3          3          12  13  15
  12        3          1          17
  13        3          1          17
  14        3          1          16
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0
  2      1     3       9    0    2
         2    10       2    0    2
         3    10       6    9    0
  3      1     3       1    0    4
         2     6       1    3    0
         3    10       0    2    0
  4      1     2       3    8    0
         2     4       3    0    7
         3    10       0    0    2
  5      1     2       0    0    3
         2     7       0    0    2
         3     9       5    3    0
  6      1     9       7    0    9
         2    10       6    0    3
         3    10       0    9    0
  7      1     5       0    0    3
         2     6       0    6    0
         3     7       3    3    0
  8      1     1       8    0    7
         2     7       0    9    0
         3     9       0    0    5
  9      1     1      10    7    0
         2     7       0    7    0
         3    10       9    0    8
 10      1     1       0    2    0
         2     3       5    0    7
         3     5       0    0    6
 11      1     2       0   10    0
         2     4       9    9    0
         3     8       0    6    0
 12      1     1       0    0    5
         2     7       3    3    0
         3     8       0    0    3
 13      1     3       3    3    0
         2     4       0    0    1
         3     5       0    2    0
 14      1     3       6    0    4
         2     9       0    8    0
         3    10       0    7    0
 15      1     2       5    8    0
         2     6       0    0   10
         3     7       0    6    0
 16      1     1      10    0    3
         2     2       0    0    2
         3     5       5    6    0
 17      1     2       0    1    0
         2     3       4    0    3
         3     7       3    0    2
 18      1     0       0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  N 1  N 2
    7   51   38
************************************************************************
