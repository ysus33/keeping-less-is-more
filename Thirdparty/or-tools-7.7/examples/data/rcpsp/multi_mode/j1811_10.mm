************************************************************************
file with basedata            : md267_.bas
initial value random generator: 517972257
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  20
horizon                       :  148
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     18      0       19        2       19
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           7  10  12
   3        3          3           5   8  14
   4        3          3           6   7  18
   5        3          3           7   9  11
   6        3          3           8  13  16
   7        3          1          17
   8        3          1           9
   9        3          2          10  15
  10        3          1          19
  11        3          3          16  18  19
  12        3          2          14  16
  13        3          2          14  15
  14        3          1          17
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
  2      1     6       0    5    5    0
         2     6       5    0    5    0
         3    10       0    6    2    0
  3      1     2       5    0    0    5
         2     6       0    2    2    0
         3     9       5    0    0    3
  4      1     3       0    8    5    0
         2     7       0    7    1    0
         3     9       0    6    0    5
  5      1     5       9    0    7    0
         2     7       7    0    0   10
         3     9       4    0    6    0
  6      1     1       7    0    0    6
         2     2       6    0    0    6
         3     3       6    0    0    3
  7      1     8       0    3    1    0
         2     8       3    0    8    0
         3     8       4    0    4    0
  8      1     3      10    0    5    0
         2     5       0    1    0    8
         3     9       7    0    0    3
  9      1     7       7    0    0    7
         2     9       0   10   10    0
         3    10       0    4    5    0
 10      1     2       4    0    5    0
         2     4       0   10    0    4
         3     9       0    9    0    4
 11      1     2       0    2    7    0
         2     7      10    0    5    0
         3     9       0    1    0    4
 12      1     2       0    3    7    0
         2     8       9    0    6    0
         3    10       2    0    6    0
 13      1     1       7    0    0    8
         2     6       6    0    0    7
         3     6       0    1    0    5
 14      1     5       9    0    4    0
         2     8       7    0    0    4
         3    10       0    4    0    3
 15      1     1       3    0    6    0
         2     5       0    5    0    6
         3     5       0    8    0    1
 16      1     5       0    7    0    8
         2     6       0    5    0    8
         3     7       2    0    7    0
 17      1     4       5    0    2    0
         2     7       2    0    0    9
         3    10       0    5    0    5
 18      1     5       4    0    0    8
         2     5       0   10    8    0
         3    10       0    4    4    0
 19      1     1       0    6    7    0
         2     3       0    6    0    6
         3     5       3    0    0    4
 20      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   25   27   52   53
************************************************************************
