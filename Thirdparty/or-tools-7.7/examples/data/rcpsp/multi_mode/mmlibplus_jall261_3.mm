jobs  (incl. supersource/sink ):	52
RESOURCES
- renewable                 : 2 R
- nonrenewable              : 4 N
- doubly constrained        : 0 D
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
1	1	9		2 3 4 5 6 7 8 13 23 
2	9	6		22 21 17 14 11 10 
3	9	4		28 17 10 9 
4	9	4		25 16 11 10 
5	9	2		21 9 
6	9	7		31 30 27 26 17 15 14 
7	9	4		30 25 15 11 
8	9	7		31 29 27 26 24 18 17 
9	9	5		27 26 24 19 12 
10	9	6		41 31 29 26 19 18 
11	9	7		34 32 31 28 27 26 20 
12	9	7		51 34 32 31 30 25 20 
13	9	7		38 33 31 30 29 28 24 
14	9	7		51 36 35 33 32 29 20 
15	9	9		51 48 41 38 36 35 33 32 29 
16	9	9		51 48 47 41 38 36 35 32 29 
17	9	8		51 50 49 41 40 38 32 25 
18	9	10		51 50 49 48 46 38 37 33 32 30 
19	9	7		51 46 40 38 37 33 30 
20	9	9		50 49 48 46 45 41 40 38 37 
21	9	7		50 49 48 36 35 34 33 
22	9	4		48 36 33 27 
23	9	5		51 48 33 32 30 
24	9	7		51 50 40 39 37 36 32 
25	9	6		48 46 44 36 35 33 
26	9	6		51 46 44 40 39 36 
27	9	6		50 49 46 45 44 35 
28	9	7		48 47 46 45 44 43 39 
29	9	5		50 49 43 40 34 
30	9	4		47 44 36 35 
31	9	5		49 46 45 43 39 
32	9	4		45 44 43 42 
33	9	3		47 45 43 
34	9	3		46 44 42 
35	9	2		43 39 
36	9	2		45 43 
37	9	2		47 44 
38	9	1		39 
39	9	1		42 
40	9	1		42 
41	9	1		43 
42	9	1		52 
43	9	1		52 
44	9	1		52 
45	9	1		52 
46	9	1		52 
47	9	1		52 
48	9	1		52 
49	9	1		52 
50	9	1		52 
51	9	1		52 
52	1	0		
************************************************************************
REQUESTS/DURATIONS
jobnr.	mode	dur	R1	R2	N1	N2	N3	N4	
------------------------------------------------------------------------
1	1	0	0	0	0	0	0	0	
2	1	2	4	3	13	19	11	10	
	2	3	4	3	13	18	10	10	
	3	14	4	3	13	18	9	8	
	4	15	3	3	13	17	8	8	
	5	18	3	3	12	14	8	5	
	6	19	3	3	12	13	8	4	
	7	20	2	3	11	12	7	3	
	8	27	2	3	11	12	6	3	
	9	28	2	3	11	10	6	2	
3	1	3	5	3	10	30	17	28	
	2	5	4	2	8	29	17	23	
	3	13	4	2	8	28	17	20	
	4	17	4	2	8	28	17	18	
	5	18	3	2	7	27	17	15	
	6	22	3	1	6	26	17	13	
	7	26	3	1	5	25	17	10	
	8	28	3	1	4	25	17	6	
	9	29	3	1	4	24	17	5	
4	1	5	4	1	19	12	13	30	
	2	6	4	1	18	12	12	26	
	3	7	4	1	17	12	12	24	
	4	11	4	1	17	12	11	23	
	5	12	4	1	16	12	9	22	
	6	13	4	1	15	12	7	18	
	7	20	4	1	15	12	5	18	
	8	23	4	1	14	12	5	15	
	9	28	4	1	14	12	3	12	
5	1	4	3	2	29	22	26	26	
	2	8	2	1	29	21	24	26	
	3	13	2	1	28	21	21	24	
	4	14	2	1	27	21	21	21	
	5	16	1	1	26	20	16	21	
	6	20	1	1	26	20	14	18	
	7	24	1	1	25	19	11	18	
	8	25	1	1	24	19	8	14	
	9	26	1	1	24	19	7	14	
6	1	3	3	5	28	29	16	17	
	2	13	3	4	28	29	13	13	
	3	15	3	4	28	29	13	12	
	4	19	3	4	28	29	11	11	
	5	21	3	3	28	29	10	10	
	6	23	3	3	28	29	7	8	
	7	24	3	3	28	29	6	6	
	8	27	3	3	28	29	3	4	
	9	28	3	3	28	29	2	3	
7	1	2	3	2	24	17	22	14	
	2	13	3	1	24	17	21	14	
	3	16	3	1	24	16	17	14	
	4	17	2	1	24	15	15	13	
	5	20	2	1	24	13	14	13	
	6	22	2	1	24	12	12	12	
	7	25	1	1	24	11	7	12	
	8	29	1	1	24	10	7	11	
	9	30	1	1	24	10	3	11	
8	1	3	5	3	23	23	28	30	
	2	4	4	3	22	21	27	27	
	3	9	4	3	20	21	27	26	
	4	11	4	2	17	21	26	25	
	5	15	4	2	15	20	26	23	
	6	16	4	2	12	19	26	23	
	7	26	4	1	10	19	25	22	
	8	28	4	1	9	18	25	21	
	9	30	4	1	6	18	25	19	
9	1	4	3	3	13	26	14	14	
	2	5	3	3	13	24	13	12	
	3	6	3	3	13	23	13	12	
	4	7	3	3	13	22	12	9	
	5	8	3	3	13	21	12	9	
	6	14	2	3	13	18	11	6	
	7	18	2	3	13	16	11	5	
	8	26	2	3	13	15	10	3	
	9	27	2	3	13	14	10	2	
10	1	4	4	4	23	16	15	21	
	2	9	3	4	20	15	14	21	
	3	10	3	4	16	15	12	20	
	4	11	2	3	14	15	12	18	
	5	12	2	2	11	13	11	18	
	6	15	2	2	9	13	9	18	
	7	19	1	2	9	12	7	16	
	8	23	1	1	6	12	5	15	
	9	27	1	1	5	11	4	15	
11	1	9	4	4	23	14	19	18	
	2	11	4	4	22	14	17	17	
	3	17	4	3	18	13	16	15	
	4	18	4	3	18	12	16	15	
	5	19	4	2	13	11	15	14	
	6	20	4	2	12	10	14	13	
	7	21	4	2	9	8	12	12	
	8	25	4	1	5	8	12	10	
	9	26	4	1	5	7	11	9	
12	1	3	5	1	28	10	13	25	
	2	5	5	1	25	9	12	22	
	3	7	5	1	25	9	11	20	
	4	12	5	1	22	9	10	20	
	5	16	5	1	21	9	10	19	
	6	17	5	1	20	9	8	17	
	7	24	5	1	19	9	7	14	
	8	26	5	1	18	9	6	14	
	9	29	5	1	16	9	6	12	
13	1	1	3	4	23	20	19	28	
	2	2	2	4	23	19	19	25	
	3	7	2	4	21	16	19	23	
	4	8	2	4	20	15	18	20	
	5	10	2	3	19	13	18	14	
	6	11	2	3	19	11	18	13	
	7	18	2	3	18	10	18	10	
	8	25	2	3	17	9	17	5	
	9	26	2	3	16	6	17	3	
14	1	3	3	4	24	14	20	27	
	2	5	2	4	22	12	19	24	
	3	12	2	4	21	10	19	22	
	4	13	2	4	20	9	18	17	
	5	15	2	3	17	8	17	16	
	6	26	1	3	16	7	15	12	
	7	27	1	2	14	7	15	12	
	8	28	1	2	13	5	13	9	
	9	29	1	2	12	5	13	5	
15	1	6	3	4	14	14	9	26	
	2	8	3	3	13	12	9	23	
	3	10	3	3	13	12	7	21	
	4	13	3	3	12	11	6	19	
	5	14	3	2	10	9	6	19	
	6	17	3	2	9	9	4	17	
	7	21	3	2	8	7	3	14	
	8	27	3	2	8	6	2	13	
	9	30	3	2	7	6	2	10	
16	1	2	3	5	18	10	17	24	
	2	7	3	4	17	9	14	22	
	3	8	3	4	17	8	11	21	
	4	12	3	4	17	8	9	21	
	5	13	3	4	16	8	8	18	
	6	14	3	4	16	7	8	17	
	7	15	3	4	16	7	6	17	
	8	18	3	4	16	6	2	15	
	9	27	3	4	16	6	2	14	
17	1	5	4	4	18	14	10	22	
	2	7	3	4	17	13	9	21	
	3	8	3	4	13	13	8	20	
	4	9	2	3	11	13	7	18	
	5	14	2	3	10	12	7	18	
	6	16	2	3	8	12	6	17	
	7	17	1	3	5	11	6	17	
	8	18	1	2	3	11	5	16	
	9	21	1	2	2	10	5	15	
18	1	1	3	4	29	20	22	25	
	2	9	3	3	28	18	20	25	
	3	10	3	3	28	18	18	23	
	4	12	3	3	28	16	14	21	
	5	13	3	3	28	15	13	19	
	6	14	3	2	27	15	11	16	
	7	19	3	2	27	14	7	13	
	8	21	3	2	27	13	5	11	
	9	23	3	2	27	11	3	8	
19	1	2	2	4	25	27	25	23	
	2	5	2	4	25	25	23	23	
	3	8	2	3	22	24	21	20	
	4	10	2	3	20	22	16	17	
	5	11	1	2	18	20	12	15	
	6	21	1	2	14	20	12	10	
	7	25	1	2	14	17	6	10	
	8	28	1	1	12	15	4	8	
	9	29	1	1	8	13	2	3	
20	1	4	3	5	18	28	13	19	
	2	5	3	4	16	23	12	19	
	3	6	3	4	15	22	12	19	
	4	7	3	4	13	16	12	19	
	5	11	3	3	11	16	11	18	
	6	20	3	3	10	12	10	18	
	7	22	3	2	8	10	10	17	
	8	27	3	2	7	5	9	17	
	9	28	3	2	6	4	8	17	
21	1	13	2	3	19	12	26	21	
	2	18	2	3	19	12	25	19	
	3	19	2	3	19	12	24	19	
	4	20	2	3	19	12	24	18	
	5	25	1	3	19	11	24	16	
	6	26	1	3	19	11	23	16	
	7	27	1	3	19	11	22	15	
	8	28	1	3	19	10	22	14	
	9	30	1	3	19	10	22	12	
22	1	9	1	3	16	27	26	12	
	2	10	1	3	16	22	23	12	
	3	11	1	3	15	21	20	11	
	4	16	1	3	14	20	17	9	
	5	21	1	3	14	17	17	7	
	6	22	1	3	13	14	11	6	
	7	23	1	3	12	12	10	3	
	8	24	1	3	12	10	4	3	
	9	28	1	3	11	10	4	1	
23	1	2	5	5	25	16	28	18	
	2	5	4	4	20	14	28	17	
	3	7	4	4	19	12	27	15	
	4	8	3	3	16	10	26	14	
	5	9	3	2	16	10	26	12	
	6	12	3	2	13	9	26	11	
	7	17	2	1	11	7	25	11	
	8	22	2	1	9	6	24	8	
	9	23	2	1	6	3	24	7	
24	1	6	4	1	7	16	18	27	
	2	8	3	1	6	15	15	23	
	3	11	3	1	6	15	13	20	
	4	13	3	1	6	15	12	18	
	5	16	3	1	5	15	10	17	
	6	23	2	1	4	15	9	16	
	7	25	2	1	3	15	8	13	
	8	27	2	1	2	15	7	11	
	9	28	2	1	2	15	5	10	
25	1	3	4	1	30	20	22	23	
	2	5	3	1	28	20	19	21	
	3	7	3	1	26	20	17	21	
	4	9	3	1	23	20	14	20	
	5	10	2	1	22	20	14	20	
	6	11	2	1	20	19	12	19	
	7	15	1	1	19	19	9	18	
	8	18	1	1	16	19	7	18	
	9	26	1	1	16	19	7	17	
26	1	1	4	5	28	28	23	27	
	2	5	3	4	26	25	22	23	
	3	6	3	4	24	22	19	21	
	4	11	2	4	24	20	17	20	
	5	12	2	3	22	19	17	15	
	6	13	2	3	21	15	16	13	
	7	14	1	2	20	14	12	9	
	8	18	1	2	20	10	10	7	
	9	27	1	2	18	7	9	5	
27	1	3	5	3	23	27	23	18	
	2	4	4	2	22	24	23	18	
	3	5	3	2	21	19	21	17	
	4	6	3	2	19	15	20	16	
	5	7	3	2	18	15	19	16	
	6	10	2	2	18	10	18	15	
	7	17	2	2	17	9	18	15	
	8	21	1	2	16	6	17	13	
	9	24	1	2	15	4	16	13	
28	1	1	1	4	20	30	25	3	
	2	5	1	4	19	23	23	2	
	3	9	1	4	16	21	21	2	
	4	14	1	4	15	19	21	2	
	5	15	1	4	13	16	20	2	
	6	16	1	3	11	10	18	2	
	7	19	1	3	10	7	17	2	
	8	23	1	3	8	7	16	2	
	9	25	1	3	7	2	14	2	
29	1	3	4	2	22	18	26	15	
	2	8	3	1	22	15	25	14	
	3	9	3	1	20	14	25	13	
	4	14	2	1	18	12	25	12	
	5	18	2	1	16	10	24	12	
	6	23	2	1	15	10	23	12	
	7	24	1	1	13	8	23	11	
	8	25	1	1	12	7	22	10	
	9	27	1	1	12	5	21	10	
30	1	2	4	3	15	19	26	23	
	2	5	4	2	13	16	25	23	
	3	9	4	2	13	16	24	23	
	4	15	4	2	11	15	23	22	
	5	18	4	2	11	13	23	22	
	6	24	4	2	11	13	23	21	
	7	28	4	2	9	12	21	22	
	8	29	4	2	9	10	20	21	
	9	30	4	2	8	9	20	21	
31	1	7	4	1	29	18	24	23	
	2	8	3	1	25	18	19	22	
	3	9	3	1	19	17	19	20	
	4	10	3	1	19	17	14	19	
	5	17	3	1	13	16	14	17	
	6	18	3	1	13	16	10	17	
	7	19	3	1	9	15	9	16	
	8	20	3	1	6	15	5	14	
	9	22	3	1	2	14	3	12	
32	1	4	4	3	23	22	24	3	
	2	5	4	3	23	21	21	3	
	3	6	4	3	22	20	21	3	
	4	10	4	2	19	17	20	2	
	5	14	4	2	19	14	19	2	
	6	15	3	2	17	14	16	2	
	7	22	3	2	16	10	15	1	
	8	23	3	1	14	8	14	1	
	9	24	3	1	13	8	13	1	
33	1	1	1	4	24	20	20	24	
	2	4	1	3	23	18	18	24	
	3	7	1	3	21	17	18	24	
	4	11	1	3	18	16	18	24	
	5	14	1	3	16	14	17	24	
	6	17	1	3	15	14	16	24	
	7	18	1	3	13	13	16	24	
	8	22	1	3	11	12	14	24	
	9	24	1	3	7	11	14	24	
34	1	3	2	1	30	15	24	14	
	2	4	2	1	29	14	22	14	
	3	6	2	1	28	14	19	12	
	4	9	2	1	28	13	16	12	
	5	11	1	1	28	12	16	11	
	6	20	1	1	27	11	11	8	
	7	24	1	1	27	9	10	8	
	8	25	1	1	26	8	8	6	
	9	28	1	1	26	8	5	5	
35	1	1	4	5	16	22	25	27	
	2	5	4	4	15	20	25	26	
	3	7	4	4	15	18	22	24	
	4	8	3	4	15	18	20	23	
	5	10	2	4	14	15	19	23	
	6	18	2	4	14	14	16	22	
	7	20	1	4	13	12	15	22	
	8	24	1	4	13	9	11	21	
	9	27	1	4	13	8	11	20	
36	1	3	4	2	16	16	27	20	
	2	10	4	2	14	15	26	18	
	3	11	4	2	12	13	26	15	
	4	18	4	2	9	12	25	14	
	5	20	3	2	8	10	24	14	
	6	21	3	2	6	9	24	12	
	7	22	3	2	5	6	23	11	
	8	29	3	2	4	6	23	9	
	9	30	3	2	1	3	23	8	
37	1	3	3	3	13	21	22	25	
	2	7	3	3	12	21	22	22	
	3	10	3	3	11	18	21	21	
	4	19	3	3	11	17	19	19	
	5	22	3	3	9	14	19	19	
	6	24	3	3	9	13	18	18	
	7	25	3	3	7	11	16	15	
	8	28	3	3	7	9	16	14	
	9	30	3	3	6	8	14	14	
38	1	3	3	5	20	29	7	25	
	2	4	3	5	18	29	6	24	
	3	5	3	5	18	29	5	23	
	4	9	3	5	16	29	5	23	
	5	10	2	5	13	29	5	22	
	6	18	2	5	13	29	4	21	
	7	19	1	5	12	29	4	19	
	8	24	1	5	9	29	3	19	
	9	27	1	5	8	29	3	18	
39	1	5	3	3	27	7	22	10	
	2	9	3	2	27	6	19	8	
	3	10	3	2	25	5	19	7	
	4	12	2	2	24	4	18	7	
	5	13	2	2	22	4	16	6	
	6	14	2	2	21	3	15	4	
	7	15	2	2	18	3	15	4	
	8	26	1	2	17	3	14	2	
	9	29	1	2	17	2	13	2	
40	1	10	3	4	24	24	23	26	
	2	11	3	4	24	24	23	24	
	3	13	3	4	22	24	21	24	
	4	17	2	4	19	23	20	23	
	5	18	2	3	14	23	20	22	
	6	20	2	3	13	23	19	22	
	7	21	2	3	11	22	19	21	
	8	27	1	3	8	22	17	21	
	9	28	1	3	5	22	17	20	
41	1	2	2	2	27	28	19	17	
	2	8	1	2	27	25	18	16	
	3	10	1	2	23	23	18	16	
	4	12	1	2	20	21	17	15	
	5	13	1	2	16	17	16	14	
	6	16	1	2	16	15	16	13	
	7	19	1	2	10	15	15	11	
	8	21	1	2	10	12	15	10	
	9	29	1	2	7	11	15	9	
42	1	7	2	2	26	26	23	28	
	2	8	1	2	24	25	21	25	
	3	9	1	2	22	23	18	24	
	4	14	1	2	19	21	17	23	
	5	15	1	1	18	20	16	21	
	6	18	1	1	16	17	13	19	
	7	19	1	1	15	15	8	18	
	8	24	1	1	15	14	7	16	
	9	29	1	1	13	12	4	15	
43	1	1	5	5	19	12	14	21	
	2	4	5	4	17	12	14	18	
	3	13	5	4	16	11	13	18	
	4	22	5	3	14	10	13	15	
	5	26	5	3	13	10	12	15	
	6	27	5	2	11	9	11	14	
	7	28	5	1	10	9	11	11	
	8	29	5	1	8	9	10	10	
	9	30	5	1	7	8	10	10	
44	1	1	4	5	23	26	24	23	
	2	2	4	5	21	24	23	22	
	3	6	4	5	20	23	23	22	
	4	8	4	5	18	22	22	20	
	5	9	4	5	16	22	21	19	
	6	12	3	5	16	21	20	19	
	7	16	3	5	14	20	19	18	
	8	21	3	5	13	19	18	16	
	9	27	3	5	13	19	18	15	
45	1	5	5	4	27	10	22	26	
	2	11	4	4	27	9	20	25	
	3	14	4	4	26	9	19	25	
	4	18	4	4	25	8	17	25	
	5	19	3	4	23	7	15	25	
	6	21	3	4	23	6	14	24	
	7	23	3	4	21	5	12	24	
	8	24	2	4	21	3	12	24	
	9	27	2	4	20	3	10	24	
46	1	4	4	3	18	30	22	14	
	2	8	4	2	15	29	21	13	
	3	9	4	2	14	28	21	13	
	4	12	4	2	14	28	20	12	
	5	15	3	2	12	28	20	12	
	6	17	3	2	11	27	20	11	
	7	18	2	2	9	27	20	11	
	8	21	2	2	7	26	19	11	
	9	25	2	2	6	26	19	10	
47	1	1	2	3	21	22	29	22	
	2	2	2	3	20	21	25	21	
	3	3	2	3	20	20	24	17	
	4	4	2	3	20	19	19	14	
	5	5	2	2	20	19	16	13	
	6	10	2	2	19	19	14	12	
	7	23	2	2	19	18	13	9	
	8	28	2	1	19	17	11	4	
	9	29	2	1	19	17	7	3	
48	1	8	5	3	23	27	15	24	
	2	9	5	3	20	25	15	23	
	3	10	5	3	18	25	15	22	
	4	11	5	3	15	25	15	22	
	5	12	5	3	13	23	15	21	
	6	21	5	3	11	23	15	21	
	7	25	5	3	10	22	15	20	
	8	29	5	3	7	22	15	20	
	9	30	5	3	3	21	15	20	
49	1	12	2	5	25	26	28	17	
	2	18	2	5	22	25	28	16	
	3	19	2	5	18	21	28	16	
	4	23	2	5	15	19	27	16	
	5	24	2	5	12	16	27	16	
	6	25	1	5	11	16	27	16	
	7	28	1	5	9	13	26	16	
	8	29	1	5	6	10	26	16	
	9	30	1	5	1	10	26	16	
50	1	1	3	4	26	15	18	30	
	2	2	2	4	24	14	18	29	
	3	3	2	4	20	14	17	28	
	4	4	2	4	19	13	15	26	
	5	9	2	4	16	13	14	26	
	6	21	1	4	14	13	12	25	
	7	25	1	4	14	12	12	24	
	8	26	1	4	11	12	9	22	
	9	29	1	4	9	12	8	22	
51	1	6	2	4	30	24	25	29	
	2	11	2	3	28	19	20	28	
	3	12	2	3	27	17	19	28	
	4	13	2	3	26	16	16	27	
	5	17	2	2	23	14	13	27	
	6	18	2	2	23	11	10	26	
	7	20	2	1	20	8	8	25	
	8	26	2	1	20	6	5	25	
	9	30	2	1	19	6	4	25	
52	1	0	0	0	0	0	0	0	
************************************************************************

 RESOURCE AVAILABILITIES 
	R 1	R 2	N 1	N 2	N 3	N 4
	27	30	963	905	906	939

************************************************************************
