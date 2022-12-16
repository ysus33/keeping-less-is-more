************************************************************************
file with basedata            : cm435_.bas
initial value random generator: 1418583211
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  137
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       18        3       18
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        4          3           5   6  14
   3        4          3           6   7  14
   4        4          3           7   8   9
   5        4          1           8
   6        4          2           8   9
   7        4          1          10
   8        4          2          13  16
   9        4          2          12  13
  10        4          2          11  16
  11        4          1          12
  12        4          2          15  17
  13        4          2          15  17
  14        4          3          15  16  17
  15        4          1          18
  16        4          1          18
  17        4          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     6       7    0    5    5
         2     7       4    0    5    5
         3     7       0    8    5    5
         4     9       0    7    5    4
  3      1     2       0    5   10    6
         2     6       0    5   10    4
         3    10       5    0    9    2
         4    10       0    4    9    2
  4      1     2       0    6    4    2
         2     8       0    4    4    2
         3    10       7    0    3    2
         4    10       3    0    4    2
  5      1     3       0    7    9    8
         2     3       4    0   10   10
         3     4       0    7    8    8
         4     6       0    5    6    5
  6      1     3       8    0   10    6
         2     5       0    9   10    6
         3     8       0    9   10    5
         4    10       6    0    9    5
  7      1     3       8    0    3   10
         2     4       8    0    3    8
         3     5       6    0    3    4
         4     5       0    9    3    3
  8      1     6       0    7    4    5
         2     6       7    0    4    5
         3     7       0    7    3    3
         4    10       0    7    2    3
  9      1     3       6    0    3    5
         2     5       0    7    3    5
         3     8       0    3    2    4
         4     9       5    0    1    3
 10      1     2       5    0   10    7
         2     3       5    0    7    7
         3     3       0    4    7    7
         4     4       5    0    6    7
 11      1     1       0    6    9   10
         2     3       2    0    8    8
         3     7       0    6    6    5
         4    10       0    4    4    4
 12      1     3       0    4   10    8
         2     3      10    0   10    7
         3     4       9    0    7    7
         4     6       9    0    6    5
 13      1     1       0    3    6    2
         2     4       0    2    6    2
         3     6       4    0    6    1
         4    10       0    1    6    1
 14      1     3       5    0    8    1
         2     4       0    7    5    1
         3    10       0    7    3    1
         4    10       3    0    2    1
 15      1     2       0    9    8    9
         2     7       8    0    7    6
         3     8       0    9    5    6
         4    10       8    0    3    5
 16      1     1       9    0    8    7
         2     3       0    4    5    6
         3     5       0    2    3    6
         4    10       0    1    3    3
 17      1     2       8    0   10   10
         2     3       7    0   10    7
         3     7       7    0    9    6
         4     8       6    0    9    1
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   19   20   87   66
************************************************************************
