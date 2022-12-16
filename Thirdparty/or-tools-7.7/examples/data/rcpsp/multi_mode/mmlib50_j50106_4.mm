jobs  (incl. supersource/sink ):	52
RESOURCES
- renewable                 : 2 R
- nonrenewable              : 2 N
- doubly constrained        : 0 D
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
1	1	5		2 3 4 5 9 
2	3	3		13 12 6 
3	3	4		14 11 8 7 
4	3	4		14 11 10 7 
5	3	3		12 10 7 
6	3	2		10 7 
7	3	6		23 21 19 17 16 15 
8	3	4		21 18 17 10 
9	3	2		17 10 
10	3	4		30 23 19 15 
11	3	4		30 19 18 15 
12	3	3		30 15 14 
13	3	4		25 21 20 17 
14	3	5		27 25 24 23 22 
15	3	4		25 24 22 20 
16	3	2		24 18 
17	3	4		30 29 27 24 
18	3	3		27 26 22 
19	3	2		25 22 
20	3	2		27 26 
21	3	2		29 26 
22	3	3		35 29 28 
23	3	4		36 35 32 29 
24	3	1		26 
25	3	1		26 
26	3	3		37 35 28 
27	3	3		35 31 28 
28	3	7		40 39 38 36 34 33 32 
29	3	5		40 39 37 34 31 
30	3	5		40 39 36 34 31 
31	3	4		50 41 38 33 
32	3	4		51 47 43 41 
33	3	4		51 44 43 42 
34	3	5		51 50 48 45 42 
35	3	4		49 45 43 42 
36	3	5		49 48 47 46 45 
37	3	5		50 49 48 47 46 
38	3	2		44 42 
39	3	4		49 48 47 45 
40	3	3		50 46 44 
41	3	2		49 44 
42	3	2		47 46 
43	3	2		48 46 
44	3	1		45 
45	3	1		52 
46	3	1		52 
47	3	1		52 
48	3	1		52 
49	3	1		52 
50	3	1		52 
51	3	1		52 
52	1	0		
************************************************************************
REQUESTS/DURATIONS
jobnr.	mode	dur	R1	R2	N1	N2	
------------------------------------------------------------------------
1	1	0	0	0	0	0	
2	1	1	2	5	9	4	
	2	6	1	3	6	3	
	3	9	1	2	6	2	
3	1	1	6	10	6	7	
	2	8	4	9	4	4	
	3	9	4	7	2	3	
4	1	5	3	7	4	3	
	2	7	3	6	3	2	
	3	10	3	4	2	2	
5	1	6	5	7	7	7	
	2	7	2	6	5	4	
	3	9	2	5	1	2	
6	1	2	7	6	7	9	
	2	6	7	4	7	7	
	3	8	7	3	6	7	
7	1	1	1	6	5	6	
	2	3	1	6	5	5	
	3	6	1	6	5	4	
8	1	5	6	6	8	4	
	2	8	3	4	7	3	
	3	10	2	3	7	2	
9	1	6	10	6	3	7	
	2	7	9	5	2	5	
	3	8	8	3	2	3	
10	1	3	5	3	6	4	
	2	8	5	3	5	3	
	3	9	5	3	5	2	
11	1	4	4	8	10	3	
	2	8	3	8	8	3	
	3	9	3	8	6	1	
12	1	1	9	4	4	5	
	2	2	7	4	4	4	
	3	5	6	3	3	2	
13	1	1	1	6	5	9	
	2	7	1	4	5	7	
	3	8	1	2	3	3	
14	1	5	6	5	8	6	
	2	7	4	3	8	2	
	3	9	4	3	8	1	
15	1	2	6	4	4	9	
	2	3	4	3	4	5	
	3	9	2	3	4	2	
16	1	2	7	5	10	7	
	2	4	6	3	9	5	
	3	5	2	2	7	4	
17	1	1	3	5	8	7	
	2	2	2	4	6	7	
	3	9	2	3	3	6	
18	1	3	3	7	8	7	
	2	7	3	5	7	6	
	3	8	2	5	7	6	
19	1	3	5	10	8	7	
	2	5	3	7	7	7	
	3	8	1	7	7	6	
20	1	3	6	9	6	7	
	2	4	6	9	4	6	
	3	9	5	8	4	6	
21	1	2	8	6	6	4	
	2	5	8	4	6	4	
	3	6	8	2	5	3	
22	1	1	7	10	3	5	
	2	3	6	7	1	4	
	3	5	5	6	1	2	
23	1	1	4	9	7	9	
	2	4	4	5	3	6	
	3	9	4	3	2	5	
24	1	8	4	9	8	8	
	2	9	2	7	8	8	
	3	10	2	3	8	8	
25	1	5	4	8	9	4	
	2	6	3	8	5	3	
	3	7	1	6	2	3	
26	1	4	10	4	9	7	
	2	6	9	2	7	7	
	3	10	9	2	4	6	
27	1	3	7	9	8	8	
	2	7	3	6	6	6	
	3	8	1	3	4	4	
28	1	4	9	7	8	8	
	2	9	7	3	8	8	
	3	10	7	2	6	8	
29	1	2	8	8	8	7	
	2	4	7	8	6	5	
	3	9	6	6	6	3	
30	1	1	3	8	5	6	
	2	2	3	4	3	6	
	3	10	3	3	3	2	
31	1	2	8	9	2	8	
	2	3	4	4	1	5	
	3	10	4	2	1	1	
32	1	3	4	7	7	8	
	2	6	4	5	7	8	
	3	9	3	4	7	8	
33	1	3	5	7	9	7	
	2	9	5	7	7	4	
	3	10	5	4	7	4	
34	1	2	8	6	7	3	
	2	3	8	4	4	3	
	3	8	7	1	2	2	
35	1	1	3	10	5	5	
	2	2	3	9	4	5	
	3	8	2	8	1	4	
36	1	3	7	8	6	5	
	2	9	6	8	6	4	
	3	10	6	6	5	3	
37	1	1	8	5	9	3	
	2	4	5	4	9	3	
	3	7	2	4	9	3	
38	1	2	10	4	5	9	
	2	3	8	3	5	7	
	3	9	8	2	3	6	
39	1	6	10	5	4	7	
	2	7	9	5	3	6	
	3	8	9	3	3	5	
40	1	1	5	7	3	5	
	2	9	3	6	2	5	
	3	10	3	6	1	4	
41	1	3	10	3	5	5	
	2	8	9	2	4	5	
	3	10	8	1	3	5	
42	1	8	6	9	3	10	
	2	9	4	7	2	7	
	3	10	4	6	2	2	
43	1	2	5	7	5	3	
	2	3	5	5	5	2	
	3	6	5	5	5	1	
44	1	8	9	5	7	6	
	2	9	7	2	4	4	
	3	10	6	2	3	3	
45	1	1	8	9	5	8	
	2	5	7	8	5	7	
	3	7	6	8	5	6	
46	1	5	1	5	6	6	
	2	6	1	3	5	5	
	3	9	1	3	4	4	
47	1	1	8	7	7	10	
	2	2	4	6	7	5	
	3	9	2	6	5	1	
48	1	5	9	9	8	6	
	2	6	7	6	6	3	
	3	8	7	5	6	2	
49	1	2	2	9	5	9	
	2	3	1	6	5	8	
	3	10	1	4	4	8	
50	1	3	7	10	8	4	
	2	8	6	9	6	4	
	3	9	5	9	4	2	
51	1	1	5	9	8	6	
	2	6	3	8	7	5	
	3	7	3	8	6	5	
52	1	0	0	0	0	0	
************************************************************************

 RESOURCE AVAILABILITIES 
	R 1	R 2	N 1	N 2
	39	36	268	253

************************************************************************
