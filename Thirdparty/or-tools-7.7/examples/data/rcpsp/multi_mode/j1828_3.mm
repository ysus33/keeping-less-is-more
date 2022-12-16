************************************************************************
file with basedata            : md284_.bas
initial value random generator: 1357347136
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  20
horizon                       :  131
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     18      0       12        0       12
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           9  15  17
   3        3          3           5   8  16
   4        3          3           6   8  10
   5        3          3          11  13  15
   6        3          3           7   9  14
   7        3          3          12  13  19
   8        3          3          11  15  18
   9        3          1          13
  10        3          1          11
  11        3          1          17
  12        3          1          16
  13        3          1          18
  14        3          2          16  19
  15        3          1          19
  16        3          1          17
  17        3          1          20
  18        3          1          20
  19        3          1          20
  20        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     2       0    8    0    9
         2     4       0    8    8    0
         3    10       5    0    0    9
  3      1     1       0    8    0    2
         2     3       0    5    0    2
         3     7       0    3    5    0
  4      1     1       0    3    0    8
         2     4       3    0   10    0
         3     4       0    3    0    4
  5      1     3       9    0    8    0
         2     7       0    9    7    0
         3     8       9    0    7    0
  6      1     2       0   10    0    8
         2     4       0   10    0    5
         3     5       4    0    9    0
  7      1     2       6    0    6    0
         2     6       0    2    0    6
         3     7       6    0    0    3
  8      1     3       0    6    0    5
         2     3       0   10    4    0
         3     8       8    0    0    5
  9      1     3       0    8    0    6
         2     8       9    0    3    0
         3     8       8    0    0    3
 10      1     4       0    2    7    0
         2     9       0    2    5    0
         3    10       8    0    0    8
 11      1     1       0    5    9    0
         2     1       3    0    0    6
         3     5       0    4    9    0
 12      1     2       0    6    6    0
         2     4       0    5    6    0
         3     6       0    3    5    0
 13      1     1       0    5    0    4
         2     4       5    0    0    2
         3     6       2    0    6    0
 14      1     5       0    8    0    1
         2     7       9    0    0    1
         3     7       7    0    7    0
 15      1     3       9    0    7    0
         2     6       7    0    7    0
         3    10       1    0    0    5
 16      1     1       0    6    6    0
         2     7       5    0    6    0
         3    10       2    0    5    0
 17      1     1       0    9    8    0
         2     1       4    0    0   10
         3     3       0    9    0    8
 18      1     1       7    0    0    7
         2     5       0    7    6    0
         3     8       6    0    5    0
 19      1     4       6    0    9    0
         2     4       0    5    0    6
         3     9       0    5    9    0
 20      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   40   39  124   91
************************************************************************
