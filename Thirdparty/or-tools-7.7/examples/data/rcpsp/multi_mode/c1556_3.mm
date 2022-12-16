************************************************************************
file with basedata            : c1556_.bas
initial value random generator: 1403569486
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
    1     16      0       21       11       21
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   7   9
   3        3          1           6
   4        3          2           8  10
   5        3          2          13  16
   6        3          3           8  13  15
   7        3          1          11
   8        3          2          16  17
   9        3          1          13
  10        3          2          11  12
  11        3          1          14
  12        3          1          15
  13        3          1          14
  14        3          1          17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     2       7    7    7    8
         2     6       6    6    5    6
         3     8       4    2    2    4
  3      1     4       9   10    5    7
         2     5       7    9    4    5
         3     8       7    8    3    2
  4      1     1       5    8    4    2
         2     1       6    6    6    2
         3     3       2    6    4    2
  5      1     6       4    7    7    7
         2     9       4    7    6    6
         3    10       4    7    5    4
  6      1     6       6    9    6    9
         2     6       7   10    6    8
         3     7       4    3    4    7
  7      1     3       6    7   10    7
         2     5       6    6   10    6
         3     8       5    5   10    6
  8      1     2       9    3    6    8
         2     5       9    2    5    4
         3     8       9    2    2    4
  9      1     1       9    7    6    4
         2     6       8    6    4    4
         3    10       8    6    1    3
 10      1     2       4    9    9    7
         2     3       2    9    8    7
         3     7       1    8    6    6
 11      1     1       8    8    2    9
         2     1      10    8    3    8
         3     4       3    3    2    4
 12      1     4       9    5    8    8
         2     8       7    5    5    7
         3     9       6    4    1    7
 13      1     6       7   10    8    8
         2     7       6    5    5    6
         3     9       3    2    3    3
 14      1     3       5    4    1    8
         2     8       4    2    1    4
         3     8       4    3    1    3
 15      1     1       8    5    9    7
         2     1       9    4    7    6
         3     9       4    4    4    1
 16      1     1       3    7    9    3
         2     3       2    6    8    2
         3     9       1    2    6    2
 17      1     2       3    6    9    6
         2     6       3    5    5    6
         3     7       2    5    3    6
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   32   40   96   97
************************************************************************
