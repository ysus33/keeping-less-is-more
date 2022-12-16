************************************************************************
file with basedata            : cr560_.bas
initial value random generator: 144622681
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  119
RESOURCES
  - renewable                 :  5   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       25        7       25
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          2          13  16
   3        3          3           6   7  11
   4        3          3           5   8  11
   5        3          3           6  15  16
   6        3          2           9  13
   7        3          3           9  12  16
   8        3          3          10  12  15
   9        3          1          10
  10        3          1          14
  11        3          3          12  14  15
  12        3          1          13
  13        3          1          17
  14        3          1          17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  R 3  R 4  R 5  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0    0    0    0
  2      1     4       0    0    0    2    0    4    6
         2     5       7    0   10    0    0    4    6
         3     6       0    0    4    2    0    2    6
  3      1     2       0    0    0    9    3    9    7
         2     3       7    1    0    9    0    9    5
         3     4       0    0    2    0    3    6    3
  4      1     1       0    5    0    0    4    4    7
         2     3       0    2    0    0    0    3    6
         3     4       9    1    8    7    0    3    3
  5      1     6       0    8    0    4    0    5    3
         2     9       9    7    1    0    0    4    3
         3    10       7    0    0    4    0    4    3
  6      1     3       1    0    1    0    2    8    5
         2     4       0    0    1    0    2    7    5
         3     6       0    0    1    3    0    7    4
  7      1     3       0    8    0    0    5   10    8
         2     5       0    0    6    0    4    6    6
         3     7       0    6    0    0    3    5    5
  8      1     3       8    8    0    0    0   10    7
         2     6       8    0    0    3    6    6    7
         3    10       6    6    0    0    0    2    5
  9      1     5       5    6    0    0    7    8    8
         2     8       3    4    7    0    6    5    4
         3     8       0    0    9    0    7    5    5
 10      1     1       0    9    9    0    6    8    8
         2     7       0    0    7    0    0    5    8
         3    10      10    3    2    0    0    3    7
 11      1     1       8    0    7    0    7   10    3
         2     3       0    0    4    9    6    8    3
         3     7       8    0    3    9    6    4    2
 12      1     6       6    9    3   10    0    7    8
         2     6       6    0    3    0    4    8    7
         3     9       6    8    1    0    4    6    4
 13      1     1       2    5    4    4    0    8    9
         2     3       2    4    3    0    0    6    6
         3     7       1    0    2    0    0    5    5
 14      1     3       7    0    0    1    6    9    9
         2     8       7    8    0    1    0    5    8
         3     8       6    0    3    0    0    2    9
 15      1     5       0    0    0    0    8    4    6
         2     6       4    0    0    0    6    3    4
         3     7       0    1    0    0    4    1    2
 16      1     3       7    0    0    0    0    4    9
         2     5       6    5    0    6    3    4    9
         3     6       5    0    3    0    0    4    8
 17      1     6       0    0    0    4    6   10    6
         2     8       7    0    0    3    5   10    5
         3    10       0    4    8    3    5    9    3
 18      1     0       0    0    0    0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  R 3  R 4  R 5  N 1  N 2
   32   24   24   19   22  119  109
************************************************************************
