************************************************************************
file with basedata            : cm239_.bas
initial value random generator: 2000023589
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
    1     16      0       34        5       34
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        2          3           8  11  16
   3        2          3           5   7  11
   4        2          3           7   8  13
   5        2          3           6   9  17
   6        2          3           8  12  14
   7        2          3          10  15  17
   8        2          1          15
   9        2          2          10  14
  10        2          1          16
  11        2          2          12  17
  12        2          1          13
  13        2          1          15
  14        2          1          16
  15        2          1          18
  16        2          1          18
  17        2          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     5       9    2    6    6
         2     6       8    2    2    3
  3      1     5       7    7   10    3
         2     7       6    7   10    3
  4      1     2       9    7    4    5
         2     6       8    4    3    2
  5      1     1       6    7    4    5
         2     9       5    7    2    4
  6      1     8       8    7    8    6
         2    10       6    5    5    6
  7      1     1      10    8    6    4
         2     6       9    8    3    2
  8      1     9       8    6    5    4
         2     9       2    6    9    4
  9      1     3       4    6    4    5
         2     8       3    6    4    4
 10      1     4       4    6    2    7
         2     9       2    6    1    4
 11      1     7       5    3    8    5
         2    10       2    3    4    5
 12      1     7       3    7    5    6
         2     9       3    5    3    1
 13      1     7       7    6    7    4
         2     9       4    4    7    1
 14      1     2       9    5    6    8
         2     9       7    2    4    7
 15      1     6       7    9   10    8
         2     9       7    9   10    5
 16      1     6       6    7    6    6
         2     9       3    7    6    4
 17      1     3       7    5    9    4
         2     4       6    2    4    2
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   23   20   81   64
************************************************************************
