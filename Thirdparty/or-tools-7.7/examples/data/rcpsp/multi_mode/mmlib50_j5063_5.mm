jobs  (incl. supersource/sink ):	52
RESOURCES
- renewable                 : 2 R
- nonrenewable              : 2 N
- doubly constrained        : 0 D
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
1	1	10		2 3 4 5 6 7 9 11 16 21 
2	3	7		25 23 19 18 12 10 8 
3	3	7		23 22 20 18 17 15 13 
4	3	5		23 15 14 12 10 
5	3	5		25 18 13 12 10 
6	3	7		37 35 32 29 26 22 18 
7	3	7		37 35 29 26 24 19 18 
8	3	4		32 26 22 13 
9	3	10		40 36 35 32 31 30 29 28 25 23 
10	3	8		36 32 29 28 27 26 24 20 
11	3	8		40 37 36 31 29 28 27 22 
12	3	6		37 30 29 28 26 22 
13	3	10		51 50 38 37 36 35 33 30 29 27 
14	3	7		40 39 36 35 32 28 24 
15	3	5		37 36 31 26 25 
16	3	6		42 38 37 32 31 26 
17	3	7		39 35 33 32 31 29 28 
18	3	5		38 36 31 28 27 
19	3	6		50 49 40 39 31 28 
20	3	8		51 50 48 42 39 38 35 34 
21	3	5		50 49 47 32 28 
22	3	3		48 42 24 
23	3	2		42 24 
24	3	6		50 49 47 45 38 33 
25	3	5		50 47 42 38 33 
26	3	7		51 50 49 47 41 40 39 
27	3	7		49 48 47 46 42 41 39 
28	3	4		51 48 42 34 
29	3	6		49 48 47 46 45 41 
30	3	4		46 44 42 41 
31	3	2		47 34 
32	3	2		48 34 
33	3	1		34 
34	3	3		46 44 41 
35	3	3		47 45 41 
36	3	3		47 45 43 
37	3	3		49 48 47 
38	3	2		44 41 
39	3	2		44 43 
40	3	2		45 43 
41	3	1		43 
42	3	1		45 
43	3	1		52 
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
2	1	3	7	0	0	8	
	2	7	7	0	0	5	
	3	8	7	0	0	4	
3	1	1	9	0	7	0	
	2	3	0	5	0	3	
	3	4	7	0	3	0	
4	1	2	10	0	8	0	
	2	7	9	0	8	0	
	3	8	0	5	8	0	
5	1	4	9	0	0	10	
	2	4	0	3	0	9	
	3	5	8	0	0	8	
6	1	3	6	0	7	0	
	2	3	0	5	5	0	
	3	10	5	0	0	5	
7	1	3	0	6	0	8	
	2	5	3	0	4	0	
	3	9	0	3	3	0	
8	1	5	5	0	0	2	
	2	7	5	0	0	1	
	3	10	0	6	0	1	
9	1	3	0	9	0	5	
	2	7	7	0	0	4	
	3	10	0	9	0	4	
10	1	4	8	0	0	6	
	2	9	7	0	0	6	
	3	10	4	0	0	6	
11	1	1	0	5	2	0	
	2	2	7	0	0	5	
	3	8	0	2	0	4	
12	1	5	0	3	3	0	
	2	7	7	0	0	9	
	3	7	5	0	3	0	
13	1	2	0	8	8	0	
	2	7	3	0	0	6	
	3	9	0	4	0	3	
14	1	8	5	0	7	0	
	2	9	3	0	0	5	
	3	10	3	0	0	4	
15	1	4	0	6	8	0	
	2	6	0	3	4	0	
	3	10	6	0	3	0	
16	1	1	5	0	4	0	
	2	5	4	0	0	6	
	3	9	0	4	0	6	
17	1	5	8	0	3	0	
	2	6	7	0	3	0	
	3	7	4	0	3	0	
18	1	1	0	4	9	0	
	2	3	0	2	0	5	
	3	6	6	0	0	5	
19	1	1	6	0	5	0	
	2	2	3	0	3	0	
	3	9	0	7	0	1	
20	1	3	6	0	3	0	
	2	3	0	5	2	0	
	3	10	0	3	2	0	
21	1	4	0	3	0	4	
	2	5	9	0	4	0	
	3	6	0	3	0	1	
22	1	6	7	0	0	5	
	2	9	0	5	5	0	
	3	10	0	2	5	0	
23	1	4	0	6	10	0	
	2	5	4	0	7	0	
	3	9	0	5	6	0	
24	1	4	0	6	9	0	
	2	7	9	0	9	0	
	3	8	0	5	9	0	
25	1	1	8	0	8	0	
	2	7	0	7	6	0	
	3	9	0	7	0	2	
26	1	3	5	0	9	0	
	2	6	3	0	8	0	
	3	9	1	0	8	0	
27	1	3	0	5	0	4	
	2	8	0	5	8	0	
	3	10	1	0	7	0	
28	1	1	0	4	3	0	
	2	5	7	0	3	0	
	3	6	5	0	3	0	
29	1	2	0	8	0	3	
	2	5	6	0	0	3	
	3	9	5	0	0	3	
30	1	6	0	6	5	0	
	2	8	0	3	3	0	
	3	10	1	0	0	5	
31	1	2	8	0	9	0	
	2	4	4	0	8	0	
	3	5	0	1	0	4	
32	1	1	8	0	10	0	
	2	5	0	2	0	3	
	3	10	6	0	0	1	
33	1	4	0	5	0	7	
	2	6	0	5	0	6	
	3	8	4	0	0	5	
34	1	2	7	0	0	3	
	2	5	0	2	0	2	
	3	6	0	2	0	1	
35	1	3	0	9	7	0	
	2	5	5	0	0	1	
	3	9	0	3	0	1	
36	1	1	6	0	10	0	
	2	9	6	0	9	0	
	3	9	0	4	0	4	
37	1	3	0	9	4	0	
	2	4	0	8	3	0	
	3	10	0	7	0	5	
38	1	4	9	0	0	4	
	2	4	0	3	7	0	
	3	10	0	2	6	0	
39	1	3	0	3	6	0	
	2	3	0	3	0	9	
	3	4	0	3	0	8	
40	1	4	9	0	7	0	
	2	5	0	8	0	7	
	3	8	0	7	0	6	
41	1	1	0	6	0	9	
	2	2	8	0	6	0	
	3	10	7	0	0	5	
42	1	2	0	5	2	0	
	2	3	0	3	2	0	
	3	5	2	0	1	0	
43	1	6	8	0	0	10	
	2	9	0	3	0	8	
	3	10	7	0	8	0	
44	1	4	0	1	9	0	
	2	5	0	1	8	0	
	3	7	5	0	0	4	
45	1	1	0	5	0	8	
	2	4	9	0	0	8	
	3	9	0	2	3	0	
46	1	1	5	0	0	2	
	2	2	4	0	8	0	
	3	4	0	3	8	0	
47	1	4	2	0	9	0	
	2	6	0	3	0	8	
	3	7	0	3	7	0	
48	1	9	0	8	4	0	
	2	9	9	0	4	0	
	3	10	9	0	0	4	
49	1	2	9	0	0	2	
	2	5	0	7	0	1	
	3	7	4	0	0	1	
50	1	2	0	7	0	6	
	2	4	8	0	0	5	
	3	10	7	0	5	0	
51	1	8	0	8	2	0	
	2	9	0	5	0	8	
	3	9	0	3	2	0	
52	1	0	0	0	0	0	
************************************************************************

 RESOURCE AVAILABILITIES 
	R 1	R 2	N 1	N 2
	56	53	149	124

************************************************************************
