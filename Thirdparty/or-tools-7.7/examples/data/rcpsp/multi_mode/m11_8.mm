************************************************************************
file with basedata            : cm11_.bas
initial value random generator: 147576511
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  94
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       35        2       35
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        1          3           6   9  10
   3        1          3          14  15  16
   4        1          3           5   6   7
   5        1          2           8  12
   6        1          3           8  11  17
   7        1          3           9  13  15
   8        1          2          13  15
   9        1          1          14
  10        1          2          11  17
  11        1          1          13
  12        1          2          14  16
  13        1          1          16
  14        1          1          17
  15        1          1          18
  16        1          1          18
  17        1          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     4       4    0    3    0
  3      1     5       4    0    2    0
  4      1    10       4    0    0    7
  5      1     5       3    0    0    2
  6      1     5       0   10    0    4
  7      1     5       0    7    9    0
  8      1     3       6    0    2    0
  9      1     8       0    6    0    2
 10      1     6       4    0    4    0
 11      1     6       0    9    0    3
 12      1    10       0    1    0    3
 13      1     5       0    8    0    9
 14      1     4       0    4    5    0
 15      1     4       0   10    7    0
 16      1     9       0    6    5    0
 17      1     5       2    0    0    7
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
    8   14   37   37
************************************************************************
