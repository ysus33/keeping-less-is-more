************************************************************************
file with basedata            : cm423_.bas
initial value random generator: 1891498492
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  129
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       19        7       19
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        4          3           5   7  10
   3        4          3          10  12  13
   4        4          3           5   6   7
   5        4          1          11
   6        4          1          11
   7        4          3           8   9  15
   8        4          3          11  12  17
   9        4          1          14
  10        4          3          15  16  17
  11        4          1          14
  12        4          1          16
  13        4          3          14  15  17
  14        4          1          16
  15        4          1          18
  16        4          1          18
  17        4          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     2       4    7    4    0
         2     3       3    7    0    2
         3     3       1    6    0    5
         4     3       3    7    3    0
  3      1     1       7    3    0    9
         2     1       8    3   10    0
         3    10       4    3    6    0
         4    10       5    3    0    8
  4      1     2       9    4    0    6
         2     3       8    4    6    0
         3     5       6    3    0    6
         4     7       5    3    0    6
  5      1     2       6   10    4    0
         2     4       5    9    0    9
         3     7       4    6    4    0
         4     8       2    5    2    0
  6      1     1       7    8    0    7
         2     2       7    6    0    6
         3     2       5    7    9    0
         4    10       4    4    5    0
  7      1     2       3    6    9    0
         2     7       2    5    7    0
         3     8       2    3    6    0
         4     8       2    1    0    5
  8      1     3       9    7    0    8
         2     6       6    7    6    0
         3     6       8    7    5    0
         4     8       5    4    5    0
  9      1     3       4    1    0   10
         2     7       4    1    0    7
         3     9       3    1    0    6
         4    10       2    1    0    6
 10      1     5       9   10    6    0
         2     6       9    9    0    6
         3     7       8    7    0    4
         4     9       7    6    4    0
 11      1     5       7    6    0    4
         2     6       5    5    0    4
         3     8       4    5    1    0
         4     9       2    4    0    4
 12      1     2       9   10    0    7
         2     3       6   10    0    5
         3     4       5    9    4    0
         4    10       1    9    0    2
 13      1     1       8    8    9    0
         2     2       8    6    5    0
         3     7       7    6    0    8
         4     8       7    4    0    8
 14      1     6      10    7    8    0
         2     6      10    7    0    5
         3     8       6    7    0    5
         4     9       5    6    0    5
 15      1     4       5    9    7    0
         2     5       5    8    6    0
         3     5       5    5    0    5
         4     6       5    4    5    0
 16      1     1       5    8    7    0
         2     6       5    8    5    0
         3     7       5    7    4    0
         4     8       5    6    3    0
 17      1     1       2    8    0    5
         2     1       2    8    9    0
         3     4       2    7    0    6
         4     6       1    3    7    0
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   21   28   75   77
************************************************************************
