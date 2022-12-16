jobs  (incl. supersource/sink ):	52
RESOURCES
- renewable                 : 2 R
- nonrenewable              : 2 N
- doubly constrained        : 0 D
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
1	1	12		2 3 4 5 6 7 9 10 14 15 16 26 
2	3	8		31 24 23 22 17 13 12 11 
3	3	7		27 25 24 23 22 20 18 
4	3	3		24 19 8 
5	3	3		31 23 8 
6	3	7		33 28 27 24 21 19 13 
7	3	9		43 38 37 31 27 25 24 21 18 
8	3	8		43 38 37 33 27 25 21 18 
9	3	5		37 27 23 19 18 
10	3	7		43 38 37 30 28 25 24 
11	3	4		38 37 29 18 
12	3	8		38 37 35 33 32 30 29 27 
13	3	7		43 42 38 37 35 30 25 
14	3	8		43 42 41 35 33 32 30 29 
15	3	4		34 33 30 23 
16	3	5		43 34 30 28 27 
17	3	3		38 32 21 
18	3	5		41 34 32 30 28 
19	3	5		42 41 32 30 25 
20	3	8		51 50 49 43 42 36 33 32 
21	3	5		49 42 40 35 29 
22	3	4		49 37 33 29 
23	3	5		51 43 42 38 32 
24	3	5		51 50 41 36 32 
25	3	3		49 40 29 
26	3	4		51 41 36 32 
27	3	7		50 48 45 44 41 40 39 
28	3	6		51 50 48 42 39 35 
29	3	4		51 50 36 34 
30	3	3		50 40 36 
31	3	4		45 42 41 39 
32	3	4		45 44 40 39 
33	3	4		46 45 44 40 
34	3	4		48 47 45 44 
35	3	4		47 46 45 44 
36	3	3		48 45 39 
37	3	2		50 44 
38	3	1		41 
39	3	1		46 
40	3	1		47 
41	3	1		49 
42	3	1		44 
43	3	1		48 
44	3	1		52 
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
2	1	4	2	0	0	9	
	2	9	2	0	7	0	
	3	10	2	0	0	8	
3	1	1	0	3	0	10	
	2	5	0	2	0	8	
	3	8	2	0	6	0	
4	1	3	0	7	10	0	
	2	6	0	7	0	4	
	3	8	0	7	5	0	
5	1	1	6	0	8	0	
	2	3	0	8	0	1	
	3	7	4	0	0	1	
6	1	4	9	0	3	0	
	2	5	8	0	3	0	
	3	7	7	0	2	0	
7	1	2	9	0	8	0	
	2	5	6	0	7	0	
	3	7	0	6	7	0	
8	1	1	0	4	8	0	
	2	2	0	3	5	0	
	3	7	2	0	0	2	
9	1	5	0	10	6	0	
	2	6	7	0	0	9	
	3	8	0	8	4	0	
10	1	8	0	10	6	0	
	2	8	4	0	0	6	
	3	9	2	0	0	3	
11	1	1	7	0	6	0	
	2	5	7	0	0	7	
	3	6	0	6	0	5	
12	1	3	1	0	3	0	
	2	5	1	0	0	2	
	3	6	1	0	0	1	
13	1	5	0	8	0	5	
	2	5	1	0	0	4	
	3	8	0	4	3	0	
14	1	1	0	6	0	9	
	2	3	6	0	8	0	
	3	10	3	0	8	0	
15	1	3	0	7	0	6	
	2	5	7	0	0	6	
	3	9	0	3	0	6	
16	1	1	6	0	3	0	
	2	5	6	0	0	6	
	3	7	0	4	0	4	
17	1	3	0	8	0	3	
	2	6	0	6	0	1	
	3	7	1	0	0	1	
18	1	1	5	0	4	0	
	2	3	3	0	3	0	
	3	9	3	0	0	8	
19	1	4	0	7	0	5	
	2	6	0	6	9	0	
	3	8	4	0	9	0	
20	1	1	5	0	2	0	
	2	2	5	0	1	0	
	3	10	0	4	0	2	
21	1	3	9	0	4	0	
	2	3	0	2	4	0	
	3	5	0	1	0	3	
22	1	1	5	0	0	10	
	2	6	5	0	0	4	
	3	8	0	8	4	0	
23	1	1	7	0	0	3	
	2	8	7	0	3	0	
	3	9	6	0	3	0	
24	1	3	4	0	0	9	
	2	8	0	7	0	8	
	3	9	3	0	5	0	
25	1	4	0	7	4	0	
	2	9	3	0	2	0	
	3	10	0	2	2	0	
26	1	2	7	0	0	6	
	2	6	6	0	1	0	
	3	8	0	1	1	0	
27	1	2	9	0	6	0	
	2	2	0	7	0	4	
	3	10	6	0	0	4	
28	1	2	0	9	0	3	
	2	8	6	0	0	4	
	3	9	6	0	0	3	
29	1	1	9	0	3	0	
	2	3	0	8	3	0	
	3	9	0	7	0	5	
30	1	3	9	0	0	3	
	2	7	5	0	0	2	
	3	8	3	0	0	2	
31	1	2	8	0	3	0	
	2	2	7	0	0	2	
	3	2	0	3	0	1	
32	1	1	0	9	0	2	
	2	5	8	0	0	2	
	3	10	0	8	0	2	
33	1	1	0	10	4	0	
	2	1	5	0	0	3	
	3	1	0	5	0	2	
34	1	5	5	0	0	5	
	2	7	3	0	0	5	
	3	8	2	0	0	4	
35	1	7	0	8	9	0	
	2	9	0	7	8	0	
	3	10	5	0	0	7	
36	1	3	7	0	0	5	
	2	4	0	3	5	0	
	3	8	5	0	0	4	
37	1	3	6	0	6	0	
	2	5	0	4	0	2	
	3	6	5	0	0	2	
38	1	2	5	0	6	0	
	2	7	0	2	0	8	
	3	9	0	2	3	0	
39	1	3	6	0	0	9	
	2	10	0	9	2	0	
	3	10	0	7	0	5	
40	1	3	7	0	3	0	
	2	5	7	0	2	0	
	3	5	0	3	3	0	
41	1	3	0	7	0	4	
	2	6	0	7	0	3	
	3	8	0	5	0	3	
42	1	1	0	9	0	7	
	2	2	0	8	0	5	
	3	8	1	0	1	0	
43	1	2	0	8	0	5	
	2	5	0	7	0	4	
	3	5	0	7	1	0	
44	1	1	0	8	7	0	
	2	3	0	7	0	4	
	3	6	0	7	0	3	
45	1	2	9	0	8	0	
	2	3	0	4	8	0	
	3	10	9	0	0	5	
46	1	1	6	0	0	2	
	2	4	5	0	0	1	
	3	6	0	2	1	0	
47	1	1	0	6	0	5	
	2	1	5	0	0	5	
	3	2	0	3	0	1	
48	1	2	7	0	0	1	
	2	7	5	0	0	1	
	3	8	4	0	0	1	
49	1	2	3	0	0	6	
	2	3	2	0	2	0	
	3	6	2	0	1	0	
50	1	5	10	0	0	7	
	2	6	5	0	0	4	
	3	8	0	6	0	3	
51	1	1	0	8	0	4	
	2	5	0	7	3	0	
	3	6	6	0	0	4	
52	1	0	0	0	0	0	
************************************************************************

 RESOURCE AVAILABILITIES 
	R 1	R 2	N 1	N 2
	37	37	102	129

************************************************************************
