************************************************************************
file with basedata            : mf12_.bas
initial value random generator: 1526308859
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  32
horizon                       :  254
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     30      0       31        5       31
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           5   6   8
   3        3          1          14
   4        3          3           9  12  21
   5        3          3           9  13  22
   6        3          2           7  13
   7        3          3          10  12  15
   8        3          2          11  26
   9        3          3          17  25  28
  10        3          3          19  23  31
  11        3          2          20  30
  12        3          2          19  20
  13        3          1          18
  14        3          2          16  22
  15        3          2          18  21
  16        3          1          28
  17        3          1          18
  18        3          3          23  24  27
  19        3          1          28
  20        3          3          24  25  29
  21        3          3          22  23  24
  22        3          2          25  29
  23        3          1          26
  24        3          1          31
  25        3          1          31
  26        3          1          30
  27        3          1          30
  28        3          1          29
  29        3          1          32
  30        3          1          32
  31        3          1          32
  32        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     2       0    9    0    6
         2     5       7    0    5    0
         3     6       0    6    4    0
  3      1     2       2    0    3    0
         2     3       2    0    2    0
         3     6       1    0    2    0
  4      1     7       0    4    0    2
         2     8       0    4    4    0
         3     9       0    3    3    0
  5      1     3       0    1    8    0
         2     4       9    0    0    3
         3    10       0    1    7    0
  6      1     2       0    3    0    4
         2     3       8    0    0    4
         3     7       0    2    3    0
  7      1     2       7    0    0   10
         2     3       0    8    0    8
         3     7       0    8    7    0
  8      1     3       5    0    0    4
         2     4       0    3    4    0
         3     8       3    0    3    0
  9      1     2       1    0    8    0
         2     8       1    0    0    4
         3     8       0    2    0    4
 10      1     3       0    8    9    0
         2     7       3    0    9    0
         3     7      10    0    0    2
 11      1     1       4    0    7    0
         2     5       0    7    0    9
         3    10       0    6    0    8
 12      1     3       0    7    4    0
         2     7       0    6    0    7
         3    10       0    3    0    2
 13      1     2       7    0    0    8
         2     6       7    0    0    5
         3     8       7    0    6    0
 14      1     1       0    6    0    6
         2     6       0    5    0    5
         3     8       0    5    4    0
 15      1     2       0    4    0    9
         2     7       0    3    0    8
         3     9       0    2    0    7
 16      1     3       0    4    0    6
         2     5       2    0    5    0
         3     5       0    4    0    1
 17      1     3       5    0    6    0
         2     9       0    7    5    0
         3    10       0    1    0    2
 18      1     3       0    8    0    9
         2     5       0    5    0    5
         3     7       8    0    0    2
 19      1     1       0    7    3    0
         2     2       9    0    3    0
         3     7       0    5    0    4
 20      1     1       0    3    5    0
         2     4       5    0    0   10
         3    10       0    2    0    8
 21      1     1       1    0    5    0
         2     3       0    1    3    0
         3     9       0    1    0    9
 22      1     6       6    0    6    0
         2     7       0    7    0    7
         3    10       0    6    0    5
 23      1     6      10    0    8    0
         2     8       0    8    0    5
         3     9       0    8    0    1
 24      1     8       5    0    2    0
         2     8       0    3    0    3
         3    10       4    0    2    0
 25      1     8       9    0   10    0
         2     9       0    4   10    0
         3     9       7    0   10    0
 26      1     1       6    0    6    0
         2     4       0    7    6    0
         3     9       0    6    0    3
 27      1     2       7    0    0   10
         2     3       7    0    8    0
         3     8       7    0    0    4
 28      1     3       0    7    0    8
         2     7       2    0    7    0
         3     9       0    4    7    0
 29      1     1       0    5    9    0
         2     3       5    0    5    0
         3     9       0    4    0    2
 30      1     3       0    8    7    0
         2     6       9    0    4    0
         3    10       0    8    0    9
 31      1     8       0   10    0    8
         2     9       0    9    0    7
         3    10       0    9    5    0
 32      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   33   28   88   89
************************************************************************
