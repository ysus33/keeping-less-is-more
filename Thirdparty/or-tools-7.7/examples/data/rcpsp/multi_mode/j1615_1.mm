************************************************************************
file with basedata            : md207_.bas
initial value random generator: 22668
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  130
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       23        0       23
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           8   9  11
   3        3          3           6   7   8
   4        3          3           5   8  10
   5        3          2           9  12
   6        3          3          14  15  17
   7        3          1          17
   8        3          3          12  14  15
   9        3          1          16
  10        3          1          11
  11        3          3          12  14  15
  12        3          2          13  17
  13        3          1          16
  14        3          1          16
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     3      10    8    9    0
         2     6       7    4    0    1
         3     7       2    4    0    1
  3      1     2       6   10    9    0
         2     4       6    7    9    0
         3     7       5    4    0    6
  4      1     3      10    8    8    0
         2     6       9    8    0    5
         3     7       9    7    7    0
  5      1     2       9   10    0    8
         2     6       8    9    0    3
         3     8       7    8    0    3
  6      1     2       9   10    9    0
         2     5       7    9    9    0
         3    10       5    9    7    0
  7      1     1       7    7    0    8
         2     6       7    4    0    8
         3     9       3    4    2    0
  8      1     3       9   10    0    6
         2     6       8    7    0    4
         3    10       6    7    0    2
  9      1     4       7    8    0    8
         2     5       4    7    0    4
         3     7       3    6    0    2
 10      1     5       5   10    4    0
         2     5       4    9    0    9
         3     9       2    9    4    0
 11      1     1       9    7    0    9
         2     7       7    4    0    7
         3     7       4    5    7    0
 12      1     6       7    3    0    7
         2     6       7    4    9    0
         3     8       4    1    8    0
 13      1     3       4    8    0    4
         2     6       3    7    0    4
         3     9       3    3    6    0
 14      1     1       6    4    6    0
         2     4       5    3    5    0
         3     4       5    3    0    3
 15      1     6      10    6    0    6
         2     7       4    3    5    0
         3     9       2    1    0    3
 16      1     3       7    7    6    0
         2     5       7    5    4    0
         3     9       7    2    2    0
 17      1     8       3    8    0    8
         2     8       4    8    4    0
         3    10       1    8    0    8
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   29   32   47   48
************************************************************************
