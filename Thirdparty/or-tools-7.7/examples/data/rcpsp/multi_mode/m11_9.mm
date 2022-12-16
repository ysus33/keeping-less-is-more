************************************************************************
file with basedata            : cm11_.bas
initial value random generator: 59564546
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  83
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       30       15       30
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        1          3           5   6   9
   3        1          2           8  11
   4        1          3           6  14  17
   5        1          3           7   8  13
   6        1          2          10  12
   7        1          3          11  14  15
   8        1          3          10  14  17
   9        1          1          13
  10        1          2          15  16
  11        1          2          16  17
  12        1          1          13
  13        1          1          15
  14        1          1          16
  15        1          1          18
  16        1          1          18
  17        1          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     7      10    0    0    7
  3      1    10       4    0   10    0
  4      1     4       0    4    0    3
  5      1     1       7    0    0    5
  6      1     6       3    0    9    0
  7      1    10       7    0   10    0
  8      1     6       0    7   10    0
  9      1     9       6    0    1    0
 10      1     4       0    8    5    0
 11      1     5       0    4    5    0
 12      1     2       0    8    7    0
 13      1     5       0    8    0    7
 14      1     2       4    0    5    0
 15      1     3       0   10    9    0
 16      1     7       0    3    4    0
 17      1     2       5    0    9    0
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   13   13   84   22
************************************************************************
