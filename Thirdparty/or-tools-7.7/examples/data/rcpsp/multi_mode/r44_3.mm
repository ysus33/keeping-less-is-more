************************************************************************
file with basedata            : cr44_.bas
initial value random generator: 1813220618
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  135
RESOURCES
  - renewable                 :  4   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       32        4       32
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   7  11
   3        3          1          15
   4        3          3           7   9  12
   5        3          3           6  10  12
   6        3          2           8  13
   7        3          2           8  10
   8        3          1          16
   9        3          2          10  11
  10        3          2          13  16
  11        3          3          15  16  17
  12        3          2          14  17
  13        3          2          14  17
  14        3          1          15
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  R 3  R 4  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0    0    0
  2      1     5       0    3    0    0    0    4
         2     5       0    3    0    3    5    0
         3     8       0    3    6    0    0    3
  3      1     2       6    0    6    7    4    0
         2     5       0    3    6    6    0    3
         3     7       0    0    2    5    0    3
  4      1    10       7    0    4    4    0    3
         2    10       0    0    8    5    0    3
         3    10       0    0    6    7    0    2
  5      1     3       0    0    6    0    0    4
         2     7       0    0    3    0    9    0
         3    10       0    0    2    0    9    0
  6      1     2       5    8    4    0    0    2
         2     8       0    0    2    8    5    0
         3    10       0    6    2    0    0    2
  7      1    10       1    4    0    1    1    0
         2    10       3    0    3    0    1    0
         3    10       0    3    0    0    0    9
  8      1     4       0   10    0    3    5    0
         2     6       2    3    0    0    3    0
         3     9       0    0    0    3    2    0
  9      1     1       0    0    7    7    6    0
         2     4      10    0    4    5    0    5
         3     8       0    0    2    3    4    0
 10      1     2       9    0    0    0    8    0
         2     4       7    0    0    0    7    0
         3     9       0    0    0    6    6    0
 11      1     7       8    0    6    7    6    0
         2     7       0    0    8    7    9    0
         3     8       9    4    4    7    0    4
 12      1     5       2    1    9    9    0    9
         2     5       3    0   10    0    8    0
         3     7       0    0    7    0    2    0
 13      1     2       8    3    4    0    8    0
         2     6       6    0    0    4    0    8
         3     9       0    3    3    0    4    0
 14      1     7       0    0    0    6    7    0
         2     9       8    0    6    0    0    6
         3     9       0    0    0    6    0    1
 15      1     1       0    9    9    0    0    9
         2     2       0    0    8    0    0    9
         3     5       3    7    7    2    0    9
 16      1     1       3    0    0    7    0    8
         2     1       0    6    0    7    0    8
         3     8       0    2    0    7    1    0
 17      1     1       0    7    8    0    8    0
         2     8       0    5    0    3    8    0
         3     8       8    0    0    0    8    0
 18      1     0       0    0    0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  R 3  R 4  N 1  N 2
   20   13   25   25   33   27
************************************************************************
