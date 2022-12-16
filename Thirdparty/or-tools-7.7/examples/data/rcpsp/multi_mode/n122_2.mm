************************************************************************
file with basedata            : cn122_.bas
initial value random generator: 1393720815
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  135
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  1   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       18        7       18
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          2           6  10
   3        3          3           5   7   9
   4        3          3           5  11  17
   5        3          1          12
   6        3          3           8   9  13
   7        3          2          10  14
   8        3          3          11  12  16
   9        3          2          15  16
  10        3          3          11  12  13
  11        3          1          15
  12        3          1          15
  13        3          1          17
  14        3          2          16  17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1
------------------------------------------------------------------------
  1      1     0       0    0    0
  2      1     4       9    4    2
         2     5       7    3    2
         3     8       3    1    1
  3      1     1      10    6    3
         2     5       8    5    2
         3     5       9    4    0
  4      1     2       9    8    0
         2     4       4    7   10
         3     6       1    5    0
  5      1     4       7    6    0
         2     6       7    5    0
         3     8       7    3    0
  6      1     3      10    7    4
         2     4       4    7    3
         3     8       3    3    0
  7      1     1       7    7    0
         2     7       5    5    0
         3     8       4    3    0
  8      1     1       7    5    6
         2     3       7    5    5
         3     8       6    4    4
  9      1     2       4    4    7
         2     5       3    2    4
         3    10       3    1    0
 10      1     7       2    4    0
         2    10       1    2    0
         3    10       2    1    6
 11      1     1       6    9    0
         2     4       3    8    8
         3    10       3    8    5
 12      1     2       1    8   10
         2     3       1    5    8
         3    10       1    3    7
 13      1     5       3    7   10
         2     6       3    6    0
         3     9       1    6    0
 14      1     1       9    9    0
         2     3       8    4    0
         3     6       8    1    0
 15      1     1       8    8    9
         2     2       5    5    8
         3    10       5    4    0
 16      1     5       7    9    3
         2     8       4    8    0
         3    10       3    6    0
 17      1     2       5   10   10
         2     3       4    9    0
         3     9       4    8    0
 18      1     0       0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1
   18   21   69
************************************************************************
