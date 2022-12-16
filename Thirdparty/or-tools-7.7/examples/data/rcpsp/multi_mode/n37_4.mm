************************************************************************
file with basedata            : cn37_.bas
initial value random generator: 1434322217
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  137
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  3   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       26       14       26
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   7  12
   3        3          2           6   8
   4        3          1          16
   5        3          3           6   9  15
   6        3          3          10  11  13
   7        3          3          10  13  14
   8        3          3           9  14  15
   9        3          1          11
  10        3          1          17
  11        3          1          16
  12        3          3          13  14  15
  13        3          2          16  17
  14        3          1          17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2  N 3
------------------------------------------------------------------------
  1      1     0       0    0    0    0    0
  2      1     8       6    4    0    5    8
         2     8       6    4    9    0    0
         3     9       4    4    0    5    8
  3      1     3       2    8    0    2    2
         2     4       2    7    9    0    2
         3     5       1    6    4    0    1
  4      1     6       8    7    0    7    8
         2     6       8    8    4    0    9
         3     7       5    4    2    0    0
  5      1     4       7    9    7    9    1
         2     4       7    8    0   10    0
         3     5       7    6    0    8    0
  6      1     4       4    8    0    7   10
         2     5       4    7    0    0    7
         3     9       4    4    0    3    4
  7      1     8       7    7    3    0    0
         2     8      10    7    0    0    8
         3    10       6    6    3    3    0
  8      1     1       8    7   10    0    0
         2     6       8    6   10    0    2
         3     8       7    6    0    8    0
  9      1     2       7    4    7    8    4
         2     3       5    4    6    7    0
         3    10       4    4    0    3    2
 10      1     1       4    7    0    3    8
         2     2       3    6    0    0    8
         3     9       3    4    0    3    0
 11      1     5       5    7    0    0    5
         2     6       5    6    7    0    3
         3     8       2    5    7    0    2
 12      1     1       8    3    8    2    0
         2     2       8    2    0    2    0
         3     9       8    2    8    1    0
 13      1     4       5    9    6    0    0
         2     7       2    8    6    0    0
         3     9       1    8    4    0    0
 14      1     1       4    8    0   10    0
         2     5       4    5    6    7    0
         3     9       4    4    0    2    0
 15      1     1       8    7    9    3    0
         2     6       6    6    0    0    4
         3    10       6    3    9    0    0
 16      1     5       7   10    0   10    0
         2     5       8   10    0    0    2
         3    10       6   10    0    9    0
 17      1     1       6   10    0    2    0
         2     2       4    7    7    0    0
         3    10       2    3    6    0    4
 18      1     0       0    0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2  N 3
   24   26   26   31   22
************************************************************************
