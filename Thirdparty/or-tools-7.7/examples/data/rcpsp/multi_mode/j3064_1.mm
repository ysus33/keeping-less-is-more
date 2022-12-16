************************************************************************
file with basedata            : mf64_.bas
initial value random generator: 1855
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  32
horizon                       :  260
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     30      0       25       19       25
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           6   8  10
   3        3          3           9  15  17
   4        3          2           5   7
   5        3          3           9  12  13
   6        3          1          21
   7        3          3           9  18  23
   8        3          1          11
   9        3          2          16  22
  10        3          2          12  15
  11        3          3          12  16  20
  12        3          3          14  21  25
  13        3          3          16  19  30
  14        3          1          17
  15        3          2          23  28
  16        3          1          26
  17        3          1          23
  18        3          2          20  21
  19        3          2          29  31
  20        3          2          22  29
  21        3          1          24
  22        3          2          24  30
  23        3          1          27
  24        3          1          28
  25        3          3          26  27  28
  26        3          1          31
  27        3          2          29  30
  28        3          1          31
  29        3          1          32
  30        3          1          32
  31        3          1          32
  32        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     1       7    5    7   10
         2     3       6    4    6    9
         3     9       6    3    3    9
  3      1     2       8    4    8    8
         2     5       8    4    6    7
         3     6       7    3    4    5
  4      1     3       6    2   10    2
         2     6       6    1    9    2
         3    10       3    1    8    1
  5      1     5       5    4    9    7
         2    10       4    2    8    4
         3    10       5    1    7    4
  6      1     5       8    7    4   10
         2     7       7    5    3    9
         3    10       5    4    3    9
  7      1     1       1    4    9    6
         2     7       1    4    8    5
         3     8       1    4    4    5
  8      1     1       5    1    4    6
         2     9       4    1    3    5
         3     9       3    1    4    5
  9      1     2       7    6    9    5
         2     6       5    6    5    4
         3     9       3    5    4    2
 10      1     3       7    7    9    3
         2     7       7    7    5    3
         3     8       4    5    3    3
 11      1     5       2    9   10    8
         2     6       2    9    5    7
         3     9       2    8    2    5
 12      1     2       8    6    8    7
         2     6       5    4    6    4
         3     9       3    2    6    3
 13      1     1       3    8    8    8
         2     2       2    5    8    6
         3    10       1    4    6    4
 14      1     2       7    9    9    8
         2     5       5    8    6    7
         3     9       4    5    5    7
 15      1     2      10    9   10    9
         2     4       9    6    9    5
         3     9       9    5    8    2
 16      1     3       6    6    8    4
         2     4       6    5    7    4
         3     9       6    2    7    3
 17      1     2       9    5    5    6
         2     4       7    3    4    6
         3     9       4    3    4    5
 18      1     2       6    5    8    8
         2     6       6    4    7    4
         3    10       5    3    4    1
 19      1     6       4    5    4    6
         2     6       5    5    3    6
         3    10       2    5    1    4
 20      1     6       7    6    3    9
         2     7       5    5    2    9
         3     8       4    4    2    9
 21      1     1       4    6   10    7
         2     6       3    6    9    7
         3     9       3    6    9    4
 22      1     4       6    6    9    6
         2     7       5    5    9    4
         3    10       2    5    9    3
 23      1     2       5    7    9    5
         2     2       4    7   10    7
         3     3       3    7    4    2
 24      1     1       2    2    5    8
         2     3       2    1    4    8
         3     5       2    1    1    7
 25      1     6       8    2    3    9
         2     7       8    1    3    5
         3    10       7    1    2    3
 26      1     2       6    9    4    7
         2     5       6    7    3    6
         3     8       6    6    3    6
 27      1     4       4    8    6    8
         2     5       3    7    6    7
         3     7       2    7    3    6
 28      1     1       7    7    7    7
         2     4       7    6    5    7
         3     8       6    6    2    5
 29      1     4       9   10   10    9
         2     7       7    7    8    8
         3    10       4    5    7    8
 30      1     4       9    7    9    9
         2     8       8    5    7    6
         3    10       5    3    2    6
 31      1     6       2    7    7    5
         2     8       1    7    5    5
         3     9       1    6    3    4
 32      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   37   34  222  212
************************************************************************
