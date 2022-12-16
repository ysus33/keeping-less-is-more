************************************************************************
file with basedata            : cr511_.bas
initial value random generator: 2128894637
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  146
RESOURCES
  - renewable                 :  5   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       24        1       24
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   6   7
   3        3          3           7  13  14
   4        3          3           6   8   9
   5        3          1          10
   6        3          2          10  11
   7        3          1          15
   8        3          2          10  17
   9        3          3          12  13  17
  10        3          2          12  13
  11        3          2          14  16
  12        3          2          15  16
  13        3          2          15  16
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
  2      1     4       3    0    2    0    0    7    0
         2     5       3    2    2    0    0    4    0
         3    10       3    0    0    9    6    3    0
  3      1     7       4    6    8    0    0    5    0
         2     9       2    0    0    4    8    4    0
         3     9       0    0    0    1    0    0    6
  4      1     4       0    7    5    0    9    3    0
         2     4       1    8    0    0    7    0    3
         3     9       1    0    0    2    0    0    2
  5      1     1       4    0    0    5    4    5    0
         2     2       0    0    5    2    3    1    0
         3    10       0    6    0    0    0    0    8
  6      1     3       7    5   10    0    0    0    8
         2     7       0    0    9    0    0    0    3
         3     7       0    0    0    5    0    5    0
  7      1     1       0    0    0    0    9    0   10
         2     8       0    4    9    4    8    2    0
         3     9       0    0    5    0    8    2    0
  8      1     1       0    0    0   10    8    0    8
         2     7       2    5    0    5    6    6    0
         3     7       0    0    3    8    0    0    6
  9      1     4       8    0   10    0    6    0    8
         2     7       8    8    0    3    0    0    6
         3    10       7    4    9    0    0    6    0
 10      1     7       0    4    0    0    6    0   10
         2     8       0    4    0   10    3    7    0
         3    10       0    3    0    0    0    0    9
 11      1     4       5    0    0    8    0    0    9
         2     7       0    5    0    7    3    0    9
         3     9       0    0    2    0    3    9    0
 12      1     1       0    7    8    0    9    5    0
         2     4       0    0    0    0    8    0    5
         3     8       9    3    0    3    0    0    2
 13      1     5       8    2    0   10    0    6    0
         2     7       0    0    2    9    0    4    0
         3    10       6    0    0    8    7    2    0
 14      1     4       6    4    0    5    0    0    2
         2     6       6    3    6    0    0    4    0
         3    10       6    2    0    5    0    0    2
 15      1     2       4    5    0    0    8    3    0
         2     7       0    0    6    0    8    0    7
         3     9       3    1    4    4    6    0    7
 16      1     5       6    8    7    7    2    0    9
         2     5       8   10    7    0    0    0    8
         3    10       0    0    0    7    0    0    7
 17      1     4       6    0    0    9    0    7    0
         2     8       3    0    0    4    0    4    0
         3     9       2    0    5    0    0    0    2
 18      1     0       0    0    0    0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  R 3  R 4  R 5  N 1  N 2
   21   23   27   19   23   43   51
************************************************************************
