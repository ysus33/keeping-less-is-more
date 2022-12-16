jobs  (incl. supersource/sink ):	52
RESOURCES
- renewable                 : 2 R
- nonrenewable              : 2 N
- doubly constrained        : 0 D
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
1	1	5		2 3 4 5 6 
2	6	5		14 12 9 8 7 
3	6	4		18 14 9 7 
4	6	7		19 18 17 16 15 14 13 
5	6	4		19 18 14 9 
6	6	4		19 13 12 10 
7	6	5		19 17 16 13 11 
8	6	5		30 24 19 18 17 
9	6	5		27 24 17 16 15 
10	6	4		24 20 18 15 
11	6	4		27 24 20 15 
12	6	3		27 20 15 
13	6	4		27 25 24 20 
14	6	4		30 25 23 21 
15	6	4		30 28 25 22 
16	6	2		30 21 
17	6	1		20 
18	6	4		32 31 25 23 
19	6	3		28 25 22 
20	6	1		21 
21	6	2		28 22 
22	6	6		36 35 33 32 31 26 
23	6	2		27 26 
24	6	4		41 32 31 28 
25	6	5		41 37 36 35 33 
26	6	3		41 34 29 
27	6	3		41 33 29 
28	6	3		38 37 33 
29	6	3		39 38 37 
30	6	5		46 45 41 40 39 
31	6	4		46 40 39 38 
32	6	4		46 40 39 38 
33	6	3		46 45 39 
34	6	5		50 45 44 42 40 
35	6	4		46 44 42 40 
36	6	4		50 44 42 40 
37	6	3		46 45 40 
38	6	4		50 45 44 43 
39	6	3		50 44 42 
40	6	2		51 43 
41	6	2		50 43 
42	6	1		43 
43	6	3		49 48 47 
44	6	3		51 49 47 
45	6	2		48 47 
46	6	1		50 
47	6	1		52 
48	6	1		52 
49	6	1		52 
50	6	1		52 
51	6	1		52 
52	1	0		
************************************************************************
REQUESTS/DURATIONS
jobnr.	mode	dur	R1	R2	N1	N2	
------------------------------------------------------------------------
1	1	0	0	0	0	0	
2	1	1	11	8	10	12	
	2	3	10	7	7	10	
	3	12	8	7	6	10	
	4	13	8	6	6	8	
	5	19	7	6	4	7	
	6	20	6	5	1	7	
3	1	4	15	12	8	19	
	2	9	13	12	6	19	
	3	13	11	10	6	18	
	4	14	8	10	6	18	
	5	15	6	9	5	16	
	6	19	3	7	4	16	
4	1	3	18	18	6	17	
	2	8	18	17	6	16	
	3	12	18	16	6	14	
	4	13	18	16	5	12	
	5	16	18	15	5	8	
	6	18	18	14	5	8	
5	1	1	20	17	9	14	
	2	9	19	16	7	14	
	3	12	19	16	7	12	
	4	16	19	15	5	10	
	5	17	18	14	4	7	
	6	19	18	14	2	7	
6	1	4	11	18	12	14	
	2	5	8	14	11	11	
	3	6	8	13	11	11	
	4	7	6	11	10	8	
	5	12	6	9	8	8	
	6	19	4	7	8	6	
7	1	1	16	10	13	17	
	2	2	13	9	12	14	
	3	3	11	8	11	13	
	4	11	8	8	11	12	
	5	12	8	7	10	11	
	6	13	5	7	10	10	
8	1	1	9	13	14	4	
	2	3	9	12	12	4	
	3	4	9	11	12	4	
	4	7	8	8	11	3	
	5	11	8	7	9	3	
	6	17	8	5	7	2	
9	1	3	19	8	19	13	
	2	10	17	7	18	12	
	3	16	16	7	17	10	
	4	17	14	6	17	9	
	5	18	13	6	16	8	
	6	19	13	6	16	7	
10	1	3	6	17	11	6	
	2	7	6	15	11	6	
	3	9	6	15	9	6	
	4	11	6	13	9	6	
	5	14	6	12	7	6	
	6	17	6	12	6	6	
11	1	3	7	14	11	11	
	2	5	6	13	11	10	
	3	10	5	13	11	10	
	4	12	5	12	11	10	
	5	13	3	12	11	10	
	6	20	2	12	11	10	
12	1	3	17	14	12	14	
	2	4	16	13	11	12	
	3	7	14	12	11	12	
	4	11	12	10	10	9	
	5	15	11	8	10	8	
	6	16	11	7	9	5	
13	1	1	17	12	18	15	
	2	8	17	9	15	15	
	3	9	17	8	13	12	
	4	10	16	8	11	11	
	5	12	16	7	9	10	
	6	20	15	5	8	8	
14	1	2	9	13	16	19	
	2	9	9	13	16	17	
	3	11	6	13	15	15	
	4	12	6	13	15	13	
	5	13	3	12	15	9	
	6	14	3	12	14	8	
15	1	5	10	11	15	18	
	2	7	8	11	14	16	
	3	9	8	10	13	10	
	4	12	7	10	13	7	
	5	13	6	10	11	7	
	6	14	5	9	11	3	
16	1	6	10	11	20	18	
	2	8	9	10	19	18	
	3	13	9	8	19	18	
	4	15	8	7	18	18	
	5	17	8	6	18	18	
	6	19	8	6	18	17	
17	1	1	8	6	16	16	
	2	10	8	5	15	16	
	3	13	7	5	14	15	
	4	16	7	3	13	13	
	5	18	7	3	12	13	
	6	19	6	2	12	12	
18	1	1	13	11	14	19	
	2	7	12	10	12	17	
	3	8	9	9	9	15	
	4	16	8	8	8	13	
	5	17	6	7	5	9	
	6	18	4	7	3	8	
19	1	7	16	6	15	15	
	2	10	13	4	12	14	
	3	11	13	3	10	14	
	4	14	11	3	8	13	
	5	16	11	1	8	13	
	6	18	9	1	4	13	
20	1	1	13	11	5	7	
	2	3	13	10	5	6	
	3	11	13	9	5	6	
	4	12	12	8	4	4	
	5	17	12	8	4	3	
	6	20	11	6	3	2	
21	1	1	13	8	17	14	
	2	2	12	7	17	13	
	3	6	12	7	17	9	
	4	16	12	7	17	7	
	5	19	12	6	17	7	
	6	20	12	5	17	4	
22	1	1	6	16	15	19	
	2	12	4	14	15	18	
	3	16	3	10	15	18	
	4	17	3	9	15	18	
	5	19	2	7	15	18	
	6	20	1	4	15	18	
23	1	3	15	6	12	14	
	2	6	14	6	11	13	
	3	7	14	6	11	12	
	4	9	12	5	11	12	
	5	13	11	5	10	10	
	6	14	8	4	10	10	
24	1	1	18	9	11	12	
	2	4	18	8	11	11	
	3	5	17	7	10	11	
	4	14	14	7	10	10	
	5	16	14	5	9	10	
	6	17	11	5	8	9	
25	1	1	10	4	14	18	
	2	2	9	4	13	14	
	3	12	8	4	13	12	
	4	15	6	4	12	8	
	5	16	4	4	12	5	
	6	19	2	4	12	3	
26	1	1	7	20	13	10	
	2	5	7	18	13	9	
	3	11	7	16	13	7	
	4	12	7	15	13	7	
	5	15	7	13	13	5	
	6	18	7	12	13	5	
27	1	2	18	14	14	6	
	2	6	17	12	13	5	
	3	7	16	9	12	5	
	4	8	16	8	9	5	
	5	12	15	5	8	5	
	6	17	15	4	6	5	
28	1	2	16	12	16	6	
	2	5	14	11	16	6	
	3	11	12	11	14	6	
	4	14	9	11	14	6	
	5	15	6	11	12	5	
	6	16	4	11	12	5	
29	1	3	18	11	18	15	
	2	9	18	11	16	15	
	3	11	17	9	15	14	
	4	13	16	8	14	14	
	5	15	16	7	12	14	
	6	16	15	7	11	13	
30	1	6	17	8	12	5	
	2	11	14	8	10	5	
	3	16	13	6	9	4	
	4	17	12	6	7	2	
	5	19	11	5	5	1	
	6	20	11	4	4	1	
31	1	1	18	17	18	11	
	2	6	16	15	18	9	
	3	9	15	14	18	7	
	4	17	15	13	17	6	
	5	18	14	9	17	4	
	6	19	12	8	17	1	
32	1	1	7	15	20	18	
	2	2	6	12	16	17	
	3	12	5	11	16	16	
	4	14	4	10	13	15	
	5	15	2	6	11	14	
	6	16	2	3	10	14	
33	1	1	17	13	11	6	
	2	3	17	11	10	5	
	3	8	15	11	7	4	
	4	14	14	10	7	3	
	5	17	14	7	3	2	
	6	18	13	6	2	2	
34	1	3	13	13	20	18	
	2	5	13	13	18	16	
	3	8	11	10	18	15	
	4	10	10	10	18	12	
	5	12	9	7	17	12	
	6	16	9	6	16	10	
35	1	3	14	19	9	19	
	2	5	13	19	8	17	
	3	6	11	19	8	14	
	4	8	11	19	8	12	
	5	14	10	18	6	10	
	6	15	8	18	6	7	
36	1	1	15	16	11	17	
	2	2	13	13	8	17	
	3	3	12	12	7	16	
	4	6	12	11	6	15	
	5	8	10	6	4	14	
	6	18	10	4	3	12	
37	1	1	4	11	11	19	
	2	8	3	10	11	18	
	3	10	3	7	10	17	
	4	12	3	7	9	17	
	5	14	1	5	8	16	
	6	16	1	3	7	15	
38	1	1	18	15	16	15	
	2	4	15	15	14	12	
	3	8	13	14	14	11	
	4	16	10	14	12	9	
	5	19	8	13	11	7	
	6	20	8	13	11	5	
39	1	1	1	17	9	19	
	2	2	1	15	8	19	
	3	6	1	14	8	19	
	4	13	1	13	6	19	
	5	16	1	13	5	19	
	6	20	1	12	4	19	
40	1	1	14	19	13	10	
	2	7	12	19	12	8	
	3	9	11	18	11	7	
	4	10	9	16	10	6	
	5	13	8	15	9	5	
	6	14	6	14	8	3	
41	1	5	17	9	9	4	
	2	8	16	8	9	3	
	3	11	16	7	9	3	
	4	12	15	7	9	3	
	5	17	15	7	9	2	
	6	20	14	6	9	3	
42	1	3	18	15	5	13	
	2	9	18	14	4	12	
	3	10	14	11	3	11	
	4	13	14	11	3	10	
	5	17	11	8	2	11	
	6	20	9	7	1	10	
43	1	4	12	4	2	10	
	2	6	12	4	2	9	
	3	7	12	4	2	8	
	4	12	12	4	2	7	
	5	16	12	4	2	6	
	6	17	12	4	2	5	
44	1	2	18	17	12	13	
	2	3	13	15	11	12	
	3	8	13	13	10	11	
	4	9	9	10	8	10	
	5	12	6	8	7	9	
	6	20	4	6	7	8	
45	1	3	13	15	12	17	
	2	6	12	15	11	17	
	3	7	11	15	10	15	
	4	14	11	15	9	12	
	5	15	10	15	9	11	
	6	18	9	15	8	8	
46	1	4	4	16	10	10	
	2	6	4	14	9	10	
	3	8	4	12	8	9	
	4	13	3	10	7	8	
	5	15	3	8	5	7	
	6	20	3	6	5	6	
47	1	2	15	8	17	6	
	2	3	12	6	16	5	
	3	8	9	6	13	5	
	4	11	8	5	12	4	
	5	12	6	3	9	2	
	6	18	4	2	4	1	
48	1	1	17	17	16	11	
	2	2	16	16	13	9	
	3	4	15	15	12	9	
	4	11	14	15	10	7	
	5	16	13	14	9	5	
	6	19	13	12	7	4	
49	1	3	9	15	20	16	
	2	4	8	15	16	15	
	3	8	8	14	14	13	
	4	10	8	14	8	11	
	5	14	8	12	6	10	
	6	20	8	12	5	7	
50	1	4	8	11	15	17	
	2	10	7	10	14	15	
	3	11	7	7	14	13	
	4	13	6	6	14	10	
	5	14	4	5	14	10	
	6	18	3	2	14	7	
51	1	3	19	12	13	15	
	2	4	18	10	11	14	
	3	5	18	10	9	13	
	4	6	18	9	8	10	
	5	17	17	8	7	7	
	6	18	17	8	6	5	
52	1	0	0	0	0	0	
************************************************************************

 RESOURCE AVAILABILITIES 
	R 1	R 2	N 1	N 2
	75	85	594	599

************************************************************************
