jobs  (incl. supersource/sink ):	52
RESOURCES
- renewable                 : 4 R
- nonrenewable              : 2 N
- doubly constrained        : 0 D
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
1	1	5		2 3 4 6 8 
2	6	7		15 13 12 11 10 9 5 
3	6	6		15 13 11 10 9 5 
4	6	3		16 13 7 
5	6	5		24 20 17 16 14 
6	6	3		22 12 9 
7	6	2		11 10 
8	6	7		29 24 22 20 19 17 16 
9	6	6		29 24 20 19 18 17 
10	6	7		29 25 24 22 20 19 18 
11	6	5		29 24 22 21 17 
12	6	5		29 25 24 21 19 
13	6	5		36 29 24 23 20 
14	6	4		36 29 22 19 
15	6	4		25 23 20 19 
16	6	2		21 18 
17	6	4		36 28 27 25 
18	6	3		28 27 23 
19	6	4		30 28 27 26 
20	6	4		30 28 27 26 
21	6	4		36 30 28 26 
22	6	2		28 23 
23	6	3		31 30 26 
24	6	5		43 37 34 32 31 
25	6	3		32 31 30 
26	6	7		43 42 38 37 34 33 32 
27	6	7		43 42 38 37 34 33 32 
28	6	3		43 34 31 
29	6	2		43 31 
30	6	5		43 42 40 34 33 
31	6	4		42 40 38 33 
32	6	3		40 39 35 
33	6	2		39 35 
34	6	5		51 46 45 41 39 
35	6	5		51 46 45 44 41 
36	6	5		51 50 46 45 39 
37	6	4		51 45 44 41 
38	6	3		51 50 39 
39	6	2		47 44 
40	6	2		47 44 
41	6	2		50 47 
42	6	2		50 47 
43	6	2		50 47 
44	6	1		48 
45	6	1		47 
46	6	1		47 
47	6	1		49 
48	6	1		49 
49	6	1		52 
50	6	1		52 
51	6	1		52 
52	1	0		
************************************************************************
REQUESTS/DURATIONS
jobnr.	mode	dur	R1	R2	R3	R4	N1	N2	
------------------------------------------------------------------------
1	1	0	0	0	0	0	0	0	
2	1	8	18	24	8	26	27	8	
	2	14	17	23	7	22	27	6	
	3	19	16	23	7	20	26	6	
	4	20	16	22	7	16	24	5	
	5	22	14	22	6	12	24	4	
	6	30	14	21	6	9	23	4	
3	1	5	25	17	25	28	21	7	
	2	13	24	15	25	24	20	6	
	3	19	22	14	24	22	18	6	
	4	23	22	12	22	20	17	5	
	5	25	21	11	21	16	14	5	
	6	28	20	9	21	14	14	5	
4	1	8	23	17	24	15	25	28	
	2	11	21	17	19	13	22	24	
	3	19	18	17	15	10	20	21	
	4	20	18	16	13	7	16	16	
	5	21	14	16	9	5	14	14	
	6	26	13	16	7	5	12	13	
5	1	8	6	21	9	5	11	26	
	2	10	6	21	7	3	9	18	
	3	25	6	15	7	3	7	18	
	4	26	6	13	6	2	7	13	
	5	27	5	12	6	2	5	8	
	6	30	5	9	5	1	4	3	
6	1	5	26	29	15	14	28	26	
	2	9	26	29	14	13	24	25	
	3	10	24	28	14	13	20	24	
	4	20	24	28	13	13	15	24	
	5	21	23	28	12	13	11	23	
	6	30	22	27	10	13	7	22	
7	1	6	26	26	14	13	21	21	
	2	7	21	20	10	12	19	19	
	3	16	18	16	9	10	18	19	
	4	19	10	15	7	9	17	16	
	5	20	5	11	5	9	17	14	
	6	26	3	7	1	8	16	12	
8	1	3	17	29	23	17	19	28	
	2	8	16	25	22	16	17	28	
	3	11	14	23	22	15	15	28	
	4	13	12	20	22	15	13	28	
	5	19	9	20	21	14	7	28	
	6	21	8	16	20	14	6	28	
9	1	2	26	18	16	15	13	29	
	2	5	24	14	15	15	13	28	
	3	11	19	12	13	14	12	28	
	4	16	18	8	12	12	9	28	
	5	22	13	5	11	12	8	27	
	6	23	10	3	10	11	8	27	
10	1	1	23	14	20	12	28	15	
	2	9	21	11	20	11	28	15	
	3	19	18	9	18	11	28	15	
	4	25	11	7	17	10	27	14	
	5	26	11	5	13	10	27	14	
	6	29	6	2	12	10	27	13	
11	1	5	20	7	20	25	16	15	
	2	15	18	7	18	25	14	12	
	3	16	17	5	14	25	13	8	
	4	18	16	4	10	25	13	7	
	5	22	14	4	8	25	11	3	
	6	26	10	3	5	25	11	2	
12	1	1	21	9	19	21	24	26	
	2	3	19	9	18	20	24	22	
	3	6	17	9	17	20	24	19	
	4	12	16	9	16	19	24	14	
	5	15	14	9	15	18	24	10	
	6	29	12	9	13	16	24	8	
13	1	4	28	22	29	30	25	27	
	2	6	28	15	27	29	24	22	
	3	9	28	14	27	28	22	20	
	4	21	27	11	25	27	20	16	
	5	29	27	7	25	27	18	9	
	6	30	27	2	24	26	16	7	
14	1	9	9	17	11	26	28	25	
	2	14	8	17	11	25	27	24	
	3	26	7	15	10	24	27	24	
	4	28	6	13	9	24	27	24	
	5	29	6	10	9	23	27	24	
	6	30	5	9	8	22	27	24	
15	1	3	26	22	22	27	25	20	
	2	6	20	19	20	25	24	17	
	3	7	20	14	19	23	24	16	
	4	8	15	12	16	20	21	14	
	5	22	10	9	15	18	20	11	
	6	23	6	4	14	18	20	6	
16	1	6	25	22	21	28	26	11	
	2	12	25	21	21	27	26	11	
	3	16	24	19	19	27	26	11	
	4	22	24	16	16	25	25	11	
	5	23	22	14	13	25	25	11	
	6	26	22	13	12	23	25	11	
17	1	2	25	15	15	13	10	18	
	2	3	25	14	12	11	10	18	
	3	6	25	14	12	10	10	18	
	4	7	25	13	10	9	10	18	
	5	10	25	13	7	8	10	18	
	6	24	25	13	6	8	10	18	
18	1	1	14	18	16	27	24	14	
	2	2	13	18	12	26	21	13	
	3	6	11	18	10	23	17	11	
	4	7	10	18	6	23	15	10	
	5	8	10	18	5	19	8	9	
	6	17	9	18	2	18	8	9	
19	1	2	15	13	20	20	24	5	
	2	4	14	13	17	17	21	5	
	3	11	13	12	16	15	19	5	
	4	23	12	9	15	11	18	4	
	5	24	11	9	12	9	15	4	
	6	25	9	7	11	6	14	4	
20	1	9	24	25	15	17	14	16	
	2	13	21	24	13	12	13	15	
	3	16	20	24	10	12	13	11	
	4	20	17	24	10	9	13	9	
	5	28	15	24	9	4	12	5	
	6	29	14	24	7	3	11	3	
21	1	5	14	22	28	6	19	25	
	2	15	10	19	22	6	15	24	
	3	16	8	17	17	6	14	19	
	4	17	6	11	15	6	11	18	
	5	19	5	10	9	6	9	12	
	6	25	4	7	6	6	7	11	
22	1	10	28	25	22	29	20	27	
	2	11	28	25	20	27	19	27	
	3	13	28	25	18	24	15	27	
	4	17	28	25	17	24	10	26	
	5	23	27	25	14	20	10	26	
	6	27	27	25	12	19	6	25	
23	1	4	20	21	18	25	24	27	
	2	5	18	21	16	21	19	25	
	3	8	14	21	14	16	18	15	
	4	16	13	20	13	16	13	14	
	5	22	12	20	12	12	9	7	
	6	28	10	19	11	10	6	3	
24	1	1	15	26	22	4	19	6	
	2	2	14	25	21	4	19	6	
	3	4	14	24	21	4	16	6	
	4	13	13	21	20	4	15	6	
	5	20	13	20	19	3	14	5	
	6	27	12	19	18	3	11	5	
25	1	5	18	7	15	11	15	8	
	2	7	18	7	12	10	14	8	
	3	8	17	6	9	10	14	8	
	4	9	17	5	9	10	12	7	
	5	18	16	4	7	10	11	6	
	6	27	16	4	5	10	10	6	
26	1	3	19	26	18	22	7	9	
	2	4	19	25	17	22	6	7	
	3	11	18	25	15	21	5	6	
	4	18	18	25	14	20	4	5	
	5	23	18	24	13	19	4	5	
	6	24	17	24	12	17	3	4	
27	1	1	18	29	23	24	29	21	
	2	6	16	29	22	22	29	17	
	3	10	16	29	21	21	28	15	
	4	17	15	28	21	20	28	12	
	5	20	15	28	19	18	27	10	
	6	22	14	28	19	17	26	6	
28	1	9	17	23	17	21	22	19	
	2	11	16	19	13	18	17	18	
	3	14	15	19	13	18	15	18	
	4	26	13	11	11	17	13	18	
	5	27	10	8	8	13	6	17	
	6	28	9	5	7	12	5	17	
29	1	15	16	30	3	2	19	12	
	2	16	15	26	2	1	16	10	
	3	19	15	22	2	1	11	10	
	4	22	15	18	1	1	9	9	
	5	23	15	16	1	1	6	7	
	6	25	15	15	1	1	3	7	
30	1	12	16	24	24	8	29	25	
	2	19	15	23	22	7	27	24	
	3	22	15	21	21	5	27	24	
	4	23	15	19	20	4	26	22	
	5	29	15	16	16	2	25	21	
	6	30	15	16	15	2	25	21	
31	1	8	16	19	28	20	16	17	
	2	9	15	17	28	19	13	17	
	3	18	12	15	25	18	9	17	
	4	19	11	14	25	18	7	17	
	5	20	10	13	23	16	7	17	
	6	21	8	12	21	16	4	17	
32	1	1	17	28	29	19	16	23	
	2	5	17	23	27	17	14	23	
	3	7	17	23	26	15	13	23	
	4	13	17	17	24	12	12	23	
	5	23	16	16	24	9	12	23	
	6	30	16	14	22	9	11	23	
33	1	9	3	28	11	19	25	25	
	2	11	3	25	9	18	24	21	
	3	15	3	16	8	17	23	16	
	4	18	3	13	8	16	23	13	
	5	23	3	8	4	15	21	7	
	6	26	3	2	4	13	21	4	
34	1	8	17	6	29	17	13	7	
	2	10	16	6	25	16	11	7	
	3	11	16	4	23	14	9	7	
	4	18	16	4	21	13	8	7	
	5	19	15	3	16	9	7	7	
	6	21	15	2	15	8	5	7	
35	1	5	10	20	29	20	28	21	
	2	9	8	19	29	20	23	20	
	3	13	7	17	28	20	21	18	
	4	16	4	14	27	20	21	17	
	5	22	4	12	26	20	17	16	
	6	25	1	7	26	20	16	14	
36	1	10	1	15	6	27	20	12	
	2	21	1	11	5	23	20	11	
	3	22	1	9	5	18	19	11	
	4	24	1	8	4	16	19	11	
	5	28	1	5	2	12	18	11	
	6	29	1	2	2	4	17	11	
37	1	3	21	22	30	23	18	28	
	2	11	20	20	26	19	16	28	
	3	14	17	19	25	18	14	27	
	4	15	16	18	23	15	12	27	
	5	16	13	16	20	12	10	27	
	6	29	13	13	19	8	9	26	
38	1	1	28	22	23	22	22	22	
	2	2	25	22	21	20	20	21	
	3	5	23	20	21	18	20	16	
	4	7	21	17	20	15	18	12	
	5	15	17	16	20	13	18	7	
	6	26	14	12	19	11	17	4	
39	1	2	19	14	15	10	11	21	
	2	3	17	14	15	9	11	20	
	3	19	17	12	15	9	9	20	
	4	21	15	10	14	9	7	20	
	5	23	14	7	14	9	5	20	
	6	28	13	5	13	9	1	20	
40	1	2	19	11	13	10	19	8	
	2	8	19	10	10	9	18	8	
	3	9	18	10	9	8	17	6	
	4	22	18	10	6	8	12	5	
	5	24	18	10	3	6	12	4	
	6	28	17	10	1	4	7	2	
41	1	4	27	28	7	13	29	21	
	2	7	26	25	6	11	28	20	
	3	8	23	20	6	9	28	20	
	4	20	23	20	5	7	28	19	
	5	21	21	13	4	6	26	17	
	6	22	19	10	3	3	26	15	
42	1	3	27	18	22	15	30	16	
	2	9	26	15	20	12	24	15	
	3	14	25	13	16	10	23	14	
	4	21	25	10	13	9	17	14	
	5	23	24	9	9	6	14	14	
	6	28	24	8	3	3	12	13	
43	1	3	11	9	16	22	25	25	
	2	4	9	7	15	21	20	20	
	3	7	9	6	13	20	20	16	
	4	16	7	4	10	19	15	11	
	5	18	4	3	9	19	9	7	
	6	27	2	3	8	17	2	3	
44	1	2	12	7	13	20	18	26	
	2	17	11	7	13	19	17	24	
	3	19	9	6	13	16	14	21	
	4	21	9	6	12	10	12	14	
	5	22	6	4	12	6	12	12	
	6	28	5	4	12	4	8	9	
45	1	12	11	14	28	16	1	22	
	2	14	10	10	25	15	1	20	
	3	15	7	9	24	14	1	18	
	4	24	6	7	21	13	1	13	
	5	25	3	5	19	12	1	9	
	6	27	2	2	19	12	1	6	
46	1	2	27	11	27	24	27	24	
	2	5	22	11	25	24	23	20	
	3	14	20	11	19	24	19	15	
	4	16	18	10	17	24	12	10	
	5	23	14	10	13	24	10	5	
	6	27	13	10	9	24	4	5	
47	1	5	29	7	9	25	19	9	
	2	12	23	6	8	21	17	9	
	3	24	23	6	8	14	14	9	
	4	25	21	3	8	13	10	9	
	5	26	15	2	8	9	9	9	
	6	30	15	2	8	2	5	9	
48	1	6	15	23	28	13	18	15	
	2	7	13	22	25	10	17	11	
	3	9	9	22	25	9	17	10	
	4	12	8	22	23	8	16	10	
	5	17	5	21	21	7	16	6	
	6	22	5	21	20	7	16	5	
49	1	1	21	21	17	29	18	26	
	2	8	21	20	17	26	16	22	
	3	11	21	16	16	23	15	20	
	4	15	21	12	15	19	13	11	
	5	27	21	10	14	18	12	8	
	6	30	21	7	14	16	10	2	
50	1	1	22	12	26	27	19	21	
	2	2	17	10	22	22	18	18	
	3	9	16	10	15	19	18	15	
	4	16	13	8	11	16	17	15	
	5	18	7	8	7	13	17	13	
	6	24	6	6	2	9	16	10	
51	1	9	11	9	27	26	27	19	
	2	10	11	8	26	25	25	15	
	3	12	11	8	25	25	22	11	
	4	15	11	8	25	24	22	8	
	5	20	10	8	25	24	19	5	
	6	27	10	8	24	24	18	1	
52	1	0	0	0	0	0	0	0	
************************************************************************

 RESOURCE AVAILABILITIES 
	R 1	R 2	R 3	R 4	N 1	N 2
	81	74	72	70	716	635

************************************************************************
