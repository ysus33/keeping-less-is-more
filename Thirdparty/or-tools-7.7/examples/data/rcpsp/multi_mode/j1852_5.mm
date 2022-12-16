************************************************************************
file with basedata            : md308_.bas
initial value random generator: 1415009286
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  20
horizon                       :  139
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     18      0       25        8       25
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           6  10  15
   3        3          3           7   8  12
   4        3          3           5   6  10
   5        3          2           7  11
   6        3          2          17  18
   7        3          1          19
   8        3          2           9  15
   9        3          3          14  16  17
  10        3          1          13
  11        3          2          12  17
  12        3          3          13  15  16
  13        3          1          14
  14        3          2          18  19
  15        3          1          19
  16        3          1          18
  17        3          1          20
  18        3          1          20
  19        3          1          20
  20        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     3       2    0    3   10
         2     4       2    0    2    8
         3     4       0    6    3    9
  3      1     1       0    9    8    5
         2     5       0    6    7    4
         3     8       0    5    6    3
  4      1     4       0    5    5    7
         2     8       0    2    5    7
         3    10       3    0    3    5
  5      1     4      10    0    8    6
         2     7       9    0    8    5
         3     8       6    0    7    4
  6      1     1       0    6    9   10
         2     4       0    4    5    5
         3     6       7    0    4    5
  7      1     4       2    0    7    9
         2    10       0   10    5    8
         3    10       1    0    5    9
  8      1     2       0    7    7    5
         2     7       7    0    5    3
         3    10       7    0    2    2
  9      1     3       2    0    4    6
         2     7       0    4    4    4
         3    10       0    3    1    3
 10      1     2       4    0    4    7
         2     7       0    8    3    4
         3    10       4    0    2    2
 11      1     2       0    7    6    9
         2     3       0    6    6    8
         3     4       3    0    1    7
 12      1     4       8    0    3    5
         2     4       0    2    3    7
         3     6       7    0    3    4
 13      1     7       0    9    4   10
         2     7       4    0    5   10
         3     9       0    9    3    9
 14      1     2       3    0    8    9
         2     8       3    0    7    6
         3     8       0    3    7    7
 15      1     1       0    8    6    6
         2     3       4    0    5    6
         3     9       4    0    5    4
 16      1     3       0    1    4    7
         2     4       7    0    2    6
         3     5       7    0    1    5
 17      1     3       5    0    2    7
         2     7       4    0    2    7
         3     9       4    0    2    4
 18      1     2      10    0    2    9
         2     4       9    0    1    8
         3     6       0    6    1    8
 19      1     1       2    0    6    8
         2     2       1    0    4    5
         3     7       0    6    3    1
 20      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   23   29   87  125
************************************************************************
