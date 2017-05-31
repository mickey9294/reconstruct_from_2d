/*CCCCCCCCCCCCCCCCCCCCCCCC MIDACO FORTRAN HEADER CCCCCCCCCCCCCCCCCCCCCCCCC
C                           
C     _|      _|  _|_|_|  _|_|_|      _|_|      _|_|_|    _|_|    
C     _|_|  _|_|    _|    _|    _|  _|    _|  _|        _|    _|  
C     _|  _|  _|    _|    _|    _|  _|_|_|_|  _|        _|    _|  
C     _|      _|    _|    _|    _|  _|    _|  _|        _|    _|  
C     _|      _|  _|_|_|  _|_|_|    _|    _|    _|_|_|    _|_|  
C
C                                          Version 5.0 (Limited)
C                                                           
C    MIDACO - Mixed Integer Distributed Ant Colony Optimization
C    ----------------------------------------------------------
C
C    MIDACO is a general solver for single- and multi-objective 
C    Mixed Integer Non-Linear Programs (MINLP's) of the form:
C
C
C      Minimize     F_1(X),... F_O(X)  where X(1,...N-NI)   is CONTINUOUS
C                                      and   X(N-NI+1,...N) is DISCRETE
C
C      subject to   G_j(X)  =  0   (j=1,...ME)      equality constraints
C                   G_j(X) >=  0   (j=ME+1,...M)  inequality constraints
C
C      and bounds   XL <= X <= XU
C
C
C    MIDACO is a (heuristic) global optimization solver that approximates 
C    a solution 'X' to the above displayed optimization problem. MIDACO 
C    is based on an extended Ant Colony Optimization framework (see [1]) in 
C    combination with the Oracle Penalty Method (see [2]) for constraints 'G(X)'
C    and a backtracking line search algorithm for local refinements.
C
C    MIDACO is a derivative free black box solver and can handle any kind
C    of non-smooth and highly nonlinear functions. MIDACO can also handle 
C    functions with moderate stochastic noise. For integer and mixed-integer 
C    problems, MIDACO evaluates integer variables only at true integer points. 
C    Thus, MIDACO does not perform a relaxation or other surrogate technique.
C
C    MIDACO does not require any user specified parameter tuning as it can 
C    run completely on 'Autopilot' (all parameter set equal to zero). 
C    Optionally, the user can adjust the MIDACO performance by setting 
C    some parameters explained below.
C
C    In case of mixed integer problems, the continuous variables are stored 
C    first in 'X(1,...N-NI)', while the discrete (also called integer or 
C    combinatorial) variables are stored last in 'X(N-NI+1,...N)'.
C    As an example consider:
C
C       X = (  0.1234,  5.6789,  1.0000,  2.0000,  3.0000)   
C
C       where 'N' = 5 and 'NI' = 3
C
C    Note that all 'X' is of type double precision. Equality and inequality 
C    constraints are handled in a similar way. The vector 'G' stores at first 
C    the 'ME' equality constraints and behind those, the remaining 'M-ME' 
C    inequality constraints are stored. 
C
CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC 
C
C    List of MIDACO subroutine arguments
C    -----------------------------------
C
C    P  :   Parallelization Factor. If no parallelization is desired, set P = 1.
C
C    O  :   Number of objective functions.
C
C    N  :   Number of optimization variables in total (continuous and integer ones). 
C           'N' is the dimension of the iterate 'X' with X = (X_1,...,X_N).
C
C    NI :   Number of integer optimization variables. 'NI' <= 'N'.
C           Integer (discrete) variables must be stored at the end of 'X'.
C     
C    M  :   Number of constraints in total (equality and inequality ones).
C           'M' is the dimension of a constraint vector 'G' with G = (G_1,...,G_M).
C
C    ME :   Number of equality constraints. 'ME' <= 'M'.
C           Equality constraints are stored in the beginning of 'G'. 
C           Inequality constraints are stored in the end of 'G'.
c
C    X(P*N) :  Array containing the iterate 'X'. For P=1 (no parallelization)
C              'X' stores only one iterate and has length 'N'. For P>1 
C              'X' contains several iterates, which are stored one after
C              another.
C
C    F(P*O) :  Array containing the objective function values 'F' corresponding
C              to the iterates 'X'. For P=1 (no parallelization), 'F' is has lenth 'O'.
C              For P>1 F has length 'P*O' and stores the vectors of objectives 
C              corresponding to 'X' one after another.
C
C    G(P*M) :  Array containing the constraint values 'G'.For P=1 (no parallelization) 
C              'G' has length 'M'. For P>1 'G' has length 'P*M' and stores the vectors
C              of constraints corresponding to 'X' one after another.
C
C    XL(N) :   Array containing the lower bounds for the iterates 'X'.
C              Note that for integer dimensions (i > N-NI) the bounds should also be 
C              discrete, e.g. X(i) = 1.0000.
C
C    XU(N) :   Array containing the upper bounds for the iterates 'X'.
C              Note that for integer dimensions (i > N-NI) the bounds should also be 
C              discrete, e.g. X(i) = 1.0000.
C
C    IFLAG :   Information flag used by MIDACO. Initially MIDACO must be called with IFLAG=0.
C              If MIDACO works correctly, IFLAG values lower than 0 are used for internal 
C              communication. If MIDACO stops (either by submitting ISTOP=1 or automatically
C              by the FSTOP or ALGOSTOP parameter), an IFLAG value between 1 and 9 is returned 
C              as final message. If MIDACO detects at start-up some critical problem setup, a 
C              ***WARNING*** message is returned by IFLAG as value between 10 and 99. If
C              MIDACO detects an ***MIDACO INPUT ERROR***, an IFLAG value between 100 and 999 
C              is returned and MIDACO stops. The individual IFLAG flags are listed below.
C
C    ISTOP :   Communication flag to stop MIDACO. If MIDACO is called with 
C              ISTOP = 1, MIDACO returns the best found solution in 'X' with 
C              corresponding 'F' and 'G'. 
C
C    PARAM() :  Array containing 12 parameters that can be selected by the user to adjust MIDACO. 
C               (See the user manual for a more detailed description of individual parameters) 
C
C    RW(LRW) :  Real workarray (Type: double precision) of length 'LRW'
C    LRW :      Length of 'RW'. 'LRW' must be greater or equal to:
C               
C                    105*N + M*P+2*M+O*O+4*O*P+10*O+3*P+610  
C
C    IW(LIW) :  Integer workarray (Type: long integer) of length 'LIW'
C    LIW :      Length of 'IW'. 'LIW' must be greater or equal to:
C
C                    3*N + P+110
C
C    PF(LPF) :  Real array, containing the approximation of the pareto front.
C    LPF :      Length of 'PF'. 'LPF' must be greater or equal to:
C
C                     (O+M+N)*PARETOMAX + 1
C 
C               where PARETOMAX = 100 by default. In case PARAM(10) is not equal zero,
C               PARETOMAX equals the absolute value of PARAM(10).
C
C    KEY :  License-Key for MIDACO. Note that any licensed copy of MIDACO comes with an 
C           individual 'KEY' determining the license owner and its affiliation.  
C
C
CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC 
C
C    List of IFLAG messages
C    ----------------------
C
C     Final Messages:
C     ---------------
C     IFLAG = 1 : Feasible solution,   MIDACO was stopped by MAXEVAL or MAXTIME
C     IFLAG = 2 : Infeasible solution, MIDACO was stopped by MAXEVAL or MAXTIME
C     IFLAG = 3 : Feasible solution,   MIDACO stopped automatically by ALGOSTOP
C     IFLAG = 4 : Infeasible solution, MIDACO stopped automatically by ALGOSTOP
C     IFLAG = 5 : Feasible solution,   MIDACO stopped automatically by EVALSTOP
C     IFLAG = 6 : Infeasible solution, MIDACO stopped automatically by EVALSTOP
C     IFLAG = 7 : Feasible solution,   MIDACO stopped automatically by FSTOP
C
C     WARNING - Flags:
C     ----------------
C     IFLAG = 51 : Some X(i)  is greater/lower than +/- 1.0D+16 (try to avoid huge values!)
C     IFLAG = 52 : Some XL(i) is greater/lower than +/- 1.0D+16 (try to avoid huge values!)
C     IFLAG = 53 : Some XU(i) is greater/lower than +/- 1.0D+16 (try to avoid huge values!)
C     IFLAG = 71 : Some XL(i) = XU(i) (fixed variable)
C     IFLAG = 81 : F(1) has value NaN for starting point X
C     IFLAG = 82 : Some G(X) has value NaN for starting point X 
C     IFLAG = 91 : FSTOP is greater/lower than +/- 1.0D+16
C     IFLAG = 92 : ORACLE is greater/lower than +/- 1.0D+16
C
C     ERROR - Flags:
C     --------------
C     IFLAG = 100 :   P   <= 0   or   P  > 1.0D+99
C     IFLAG = 101 :   O   <= 0   or   O  > 1000000000
C     IFLAG = 102 :   N   <= 0   or   N  > 1.0D+99
C     IFLAG = 103 :   NI  <  0
C     IFLAG = 104 :   NI  >  N
C     IFLAG = 105 :   M   <  0   or   M  > 1.0D+99
C     IFLAG = 106 :   ME  <  0
C     IFLAG = 107 :   ME  >  M
C     IFLAG = 201 :   some X(i)  has type NaN
C     IFLAG = 202 :   some XL(i) has type NaN
C     IFLAG = 203 :   some XU(i) has type NaN
C     IFLAG = 204 :   some X(i) < XL(i)
C     IFLAG = 205 :   some X(i) > XU(i)
C     IFLAG = 206 :   some XL(i) > XU(i)
C     IFLAG = 301 :   PARAM(1) < 0   or   PARAM(1) > 1.0D+99
C     IFLAG = 302 :   PARAM(2) < 0   or   PARAM(2) > 1.0D+99
C     IFLAG = 303 :   PARAM(3) greater/lower than +/- 1.0D+99
C     IFLAG = 304 :   PARAM(4) < 0   or   PARAM(4) > 1.0D+99
C     IFLAG = 305 :   PARAM(5) greater/lower than +/- 1.0D+99
C     IFLAG = 306 :   PARAM(6) not discrete  or   PARAM(6) > 1.0D+99
C     IFLAG = 307 :   PARAM(7) < 0   or   PARAM(7) > 1.0D+99
C     IFLAG = 308 :   PARAM(8) < 0   or   PARAM(8) > 100
C     IFLAG = 309 :   PARAM(7) < PARAM(8)
C     IFLAG = 310 :   PARAM(7) > 0 but PARAM(8) = 0
C     IFLAG = 311 :   PARAM(8) > 0 but PARAM(7) = 0
C     IFLAG = 312 :   PARAM(9) greater/lower than +/- 1.0D+99
C     IFLAG = 321 :   PARAM(10) >= 1.0D+99
C     IFLAG = 322 :   PARAM(10) not discrete
C     IFLAG = 331 :   PARAM(11) < 0 or > 0.5
C     IFLAG = 351 :   PARAM(12) < 0   or   PARAM(12) > 3
C     IFLAG = 399 :   Some PARAM(i) has type NaN
C     IFLAG = 344 :   LPF too small. LPF must be at least (O+M+N)*PARETOMAX + 1,
C                     where PARETOMAX = 100 by default. See also PARAM(10).
C     IFLAG = 347 :   PARAM(5) > 0 but PARAM(5) < 1
C     IFLAG = 348 :   PARAM(5): Optional EVALSTOP precision appendix > 0.5
C     IFLAG = 401 :   ISTOP < 0 or ISTOP > 1
C     IFLAG = 402 :   Starting point does not satisfy all-different constraint
C     IFLAG = 501 :   Double precision work space size LRW is too small.  
C                     Increase size of RW array. RW must be at least of 
C                     size LRW = 105*N+M*P+2*M+O*O+4*O*P+10*O+3*P+610
C
C     IFLAG = 502 :   Internal LRW check error
C     IFLAG = 601 :   Integer work space size LIW is too small.
C                     Increase size of IW array. IW must be at least of 
C                     size LIW = 3*N+P+110 
C
C     IFLAG = 602 :   Internal LIW check error 
C     IFLAG = 701 :   Input check failed! MIDACO must be called initially with IFLAG = 0
C     IFLAG = 801 :   P > PMAX (user must increase PMAX in the MIDACO source code) 
C     IFLAG = 881 :   Integer part of X contains continues (non discrete) values
C     IFLAG = 882 :   Integer part of XL contains continues (non discrete) values
C     IFLAG = 883 :   Integer part of XU contains continues (non discrete) values
C     IFLAG = 900 :   Invalid or corrupted LICENSE-KEY
C     IFLAG = 999 :   N > 4. The free test version is limited up to 4 variables.
C
CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC 
C
C    References:
C    -----------
C
C    [1] Schlueter, M., Egea, J. A., Banga, J. R.: 
C        "Extended ant colony optimization for non-convex mixed integer nonlinear programming", 
C        Computers and Operations Research (Elsevier), Vol. 36 , Issue 7, Page 2217-2229, 2009.
C
C    [2] Schlueter M., Gerdts M.: "The oracle penalty method",
C        Journal of Global Optimization (Springer), Vol. 47, Issue 2, Page 293-325, 2010.
C
CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC 
C
C    Author (C) :   Dr. Martin Schlueter
C                   Information Initiative Center,
C                   Division of Large Scale Computing Systems,
C                   Hokkaido University, JAPAN.
C
C    Email :        info@midaco-solver.com
C    URL :          http://www.midaco-solver.com
C       
CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */

#ifndef F2C_INCLUDE
#define F2C_INCLUDE
typedef long int integer;
typedef char *address;
typedef short int shortint;
typedef float real;
typedef double doublereal;
typedef struct { real r, i; } complex;
typedef struct { doublereal r, i; } doublecomplex;
typedef long int logical;
typedef short int shortlogical;
typedef char logical1;
typedef char integer1;
#define TRUE_ (1)
#define FALSE_ (0)
#ifndef Extern
#define Extern extern
#endif
#ifdef f2c_i2
typedef short flag;
typedef short ftnlen;
typedef short ftnint;
#else
typedef long flag;
typedef long ftnlen;
typedef long ftnint;
#endif
typedef struct
{ flag cierr;
 ftnint ciunit;
 flag ciend;
 char *cifmt;
 ftnint cirec;
} cilist;
typedef struct
{ flag icierr;
 char *iciunit;
 flag iciend;
 char *icifmt;
 ftnint icirlen;
 ftnint icirnum;
} icilist;
typedef struct
{ flag oerr;
 ftnint ounit;
 char *ofnm;
 ftnlen ofnmlen;
 char *osta;
 char *oacc;
 char *ofm;
 ftnint orl;
 char *oblnk;
} olist;
typedef struct
{ flag cerr;
 ftnint cunit;
 char *csta;
} cllist;
typedef struct
{ flag aerr;
 ftnint aunit;
} alist;
typedef struct
{ flag inerr;
 ftnint inunit;
 char *infile;
 ftnlen infilen;
 ftnint *inex; 
 ftnint *inopen;
 ftnint *innum;
 ftnint *innamed;
 char *inname;
 ftnlen innamlen;
 char *inacc;
 ftnlen inacclen;
 char *inseq;
 ftnlen inseqlen;
 char  *indir;
 ftnlen indirlen;
 char *infmt;
 ftnlen infmtlen;
 char *inform;
 ftnint informlen;
 char *inunf;
 ftnlen inunflen;
 ftnint *inrecl;
 ftnint *innrec;
 char *inblank;
 ftnlen inblanklen;
} inlist;
#define VOID void
union Multitype { 
 shortint h;
 integer i;
 real r;
 doublereal d;
 complex c;
 doublecomplex z;
 };
typedef union Multitype Multitype;
typedef long Long; 
struct Vardesc { 
 char *name;
 char *addr;
 ftnlen *dims;
 int  type;
 };
typedef struct Vardesc Vardesc;
struct Namelist {
 char *name;
 Vardesc **vars;
 int nvars;
 };
typedef struct Namelist Namelist;
#define abs(x) ((x) >= 0 ? (x) : -(x))
#define dabs(x) (doublereal)abs(x)
#define min(a,b) ((a) <= (b) ? (a) : (b))
#define max(a,b) ((a) >= (b) ? (a) : (b))
#define dmin(a,b) (doublereal)min(a,b)
#define dmax(a,b) (doublereal)max(a,b)
#define F2C_proc_par_types 1
#ifdef __cplusplus
typedef int /* Unknown procedure type */ (*U_fp)(...);
typedef shortint (*J_fp)(...);
typedef integer (*I_fp)(...);
typedef real (*R_fp)(...);
typedef doublereal (*D_fp)(...), (*E_fp)(...);
typedef /* Complex */ VOID (*C_fp)(...);
typedef /* Double Complex */ VOID (*Z_fp)(...);
typedef logical (*L_fp)(...);
typedef shortlogical (*K_fp)(...);
typedef /* Character */ VOID (*H_fp)(...);
typedef /* Subroutine */ int (*S_fp)(...);
#else
typedef int /* Unknown procedure type */ (*U_fp)();
typedef shortint (*J_fp)();
typedef integer (*I_fp)();
typedef real (*R_fp)();
typedef doublereal (*D_fp)(), (*E_fp)();
typedef /* Complex */ VOID (*C_fp)();
typedef /* Double Complex */ VOID (*Z_fp)();
typedef logical (*L_fp)();
typedef shortlogical (*K_fp)();
typedef /* Character */ VOID (*H_fp)();
typedef /* Subroutine */ int (*S_fp)();
#endif
/* E_fp is for real functions when -R is not specified */
typedef VOID C_f; /* complex function */
typedef VOID H_f; /* character function */
typedef VOID Z_f; /* double complex function */
typedef doublereal E_f;
#ifndef Skip_f2c_Undefs
#undef cray
#undef gcos
#undef mc68010
#undef mc68020
#undef mips
#undef pdp11
#undef sgi
#undef sparc
#undef sun
#undef sun2
#undef sun3
#undef sun4
#undef u370
#undef u3b
#undef u3b2
#undef u3b5
#undef unix
#undef vax
#endif
#endif
#ifdef _WIN32
#define huge huged
#define near neard
#endif
static doublereal c_b14 = 0.;
static doublereal c_b16 = 3.;
static doublereal c_b23 = 10.;
static integer c__1 = 1;
static integer c__2 = 2;
static doublereal c_b44 = .5;
static doublereal c_b66 = 1e16;

/* Subroutine */ int midaco(integer *p, integer *o, integer *n, integer *ni, 
	integer *m, integer *me, doublereal *x, doublereal *f, doublereal *g, 
	doublereal *xl, doublereal *xu, integer *iflag, integer *istop, 
	doublereal *param, doublereal *rw, integer *lrw, integer *iw, integer 
	*liw, doublereal *pf, integer *lpf, char *key)
{
    /* Initialized data */

    static integer ea = 0;
    static integer eb = 0;
    static integer edg = 0;
    static integer edf = 0;
    static integer eu = 0;
    static integer en = 0;
    static integer ein = 0;
    static integer eie = 0;

    /* System generated locals */
    integer i__1;

    /* Local variables */
    extern /* Subroutine */ int precheck_(integer *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, integer *, integer *);
    static integer i__;
    extern /* Subroutine */ int midaco_code__(integer *, integer *, integer *,
	     integer *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, char *, ftnlen), 
	    multiobjwrap_(integer *, integer *, integer *, integer *, integer 
	    *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *, doublereal *, 
	    doublereal *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, char *, 
	    ftnlen);

/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
    /* Parameter adjustments */
    --f;
    --xu;
    --xl;
    --x;
    --g;
    --param;
    --rw;
    --iw;
    --pf;

    /* Function Body */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/*     Check on number of variables for the limited version. Note that */
/*     removing this check will cause MIDACO to not work properly on */
/*     problems with N > 4. */
    if (*n > 4) {
	*iflag = 999;
	*istop = 1;
	return 0;
    }
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
    if (*iflag == 0) {
	precheck_(p, o, n, m, lrw, liw, lpf, &pf[1], &param[1], iflag, istop);
    ea = *n * 104 + ((*m + (*o << 1)) << 1) + 521;
	eb = ea + *p;
	edg = eb + *p;
	edf = edg + *p * (*m + (*o << 1)) + 1;
	eu = edf + *p;
	en = eu + *o;
	ein = en + *o;
	eie = ein + *o * *o;
    }
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
    if (*iflag == -999) {
	*istop = 1;
    }
    if (*o <= 1) {
	if (*m > 0) {
	    i__1 = *p * *m;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		rw[edg + i__ - 1] = g[i__];
	    }
	    rw[edg + *p * *m] = 0.;
	} else {
	    rw[edg] = 0.;
	}
	midaco_code__(p, o, n, ni, m, me, &x[1], &f[1], &rw[edg], &xl[1], &xu[
		1], iflag, istop, &param[1], &rw[1], lrw, &iw[1], liw, &rw[ea]
		, &rw[eb], &rw[eie], key, (ftnlen)60);
    } else {
	multiobjwrap_(p, o, n, ni, m, me, &x[1], &f[1], &g[1], &xl[1], &xu[1],
		 iflag, istop, &param[1], &rw[1], lrw, &iw[1], liw, &pf[1], &
		rw[ea], &rw[eb], &rw[edg], &rw[edf], &rw[eu], &rw[en], &rw[
		ein], &rw[eie], key, (ftnlen)60);
    }
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
    if (*istop == 1) {
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    g[i__] = rw[edg + i__ - 1];
	}
    }
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
    return 0;
} /* midaco_ */

/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC */
/* Subroutine */ int i409_(integer *m, integer *i8, doublereal *g, doublereal 
	*i5, doublereal *i2, doublereal *i4, integer *i6, integer *i43)
{
    /* System generated locals */
    integer i__1, i__2, i__3;

    /* Local variables */
    static integer i__, j, k, t;
    static doublereal z__;
    static integer i10;
    extern doublereal i02_(doublereal *);
    static integer ok;
    extern integer i301_(doublereal *, doublereal *);
    static integer i450;

    /* Parameter adjustments */
    --i6;
    --i4;
    --i2;
    --i5;
    --g;

    /* Function Body */
    i__1 = *m;
    for (k = 1; k <= i__1; ++k) {
	i6[*i43 + k - 1] = k;
    }
    i__1 = *m;
    for (k = 1; k <= i__1; ++k) {
	j = (integer) ((doublereal) k * i02_(&i4[1])) + 1;
	i6[*i43 + k - 1] = i6[*i43 + j - 1];
	i6[*i43 + j - 1] = k;
    }
    i__1 = *m;
    for (t = 1; t <= i__1; ++t) {
	i__ = i6[*i43 + t - 1];
	if (i__ <= *m - *i8) {
	    goto L79;
	}
	i__2 = *m;
	for (k = *m - *i8 + 1; k <= i__2; ++k) {
	    if (i301_(&g[i__], &g[k]) == 1 && i__ != k) {
		i450 = 0;
		i10 = 0;
		while(i450 == 0) {
		    ++i10;
		    z__ = g[i__] + (doublereal) i10;
		    if (z__ > i2[i__]) {
			goto L88;
		    }
		    ok = 0;
		    i__3 = *m;
		    for (j = *m - *i8 + 1; j <= i__3; ++j) {
			if (i__ != j && i301_(&z__, &g[j]) == 0) {
			    ++ok;
			}
		    }
		    if (ok == *i8 - 1) {
			i450 = 1;
			g[i__] = z__;
			goto L123;
		    }
L88:
		    z__ = g[i__] - (doublereal) i10;
		    if (z__ < i5[i__]) {
			goto L99;
		    }
		    ok = 0;
		    i__3 = *m;
		    for (j = *m - *i8 + 1; j <= i__3; ++j) {
			if (i__ != j && i301_(&z__, &g[j]) == 0) {
			    ++ok;
			}
		    }
		    if (ok == *i8 - 1) {
			i450 = 1;
			g[i__] = z__;
			goto L123;
		    }
L99:
		    if ((doublereal) i10 > i2[i__] - i5[i__]) {
			goto L123;
		    }
		}
L123:
		;
	    }
	}
L79:
	;
    }
    return 0;
} /* i409_ */

/* Subroutine */ int i405_(doublereal *l, doublereal *x, integer *n, integer *
	i0, doublereal *i16, doublereal *i48, integer *i41, integer *i42)
{
    /* Initialized data */

    static integer k2 = 0;
    static doublereal i56 = 0.;
    static doublereal i75 = 0.;
    static doublereal k1 = 0.;
    static doublereal i17 = 0.;

    /* Builtin functions */
    integer i_dnnt(doublereal *);

    /* Local variables */
    extern integer i301_(doublereal *, doublereal *);
    extern /* Subroutine */ int i403_(doublereal *, integer *, integer *, 
	    doublereal *, doublereal *);

    /* Parameter adjustments */
    --i48;
    --x;

    /* Function Body */
    if (*i42 == 0) {
	k2 = 0;
	i403_(&x[1], n, i0, i16, &i17);
	i56 = *l;
	i75 = i17;
	k1 = .001;
	if (i48[5] - (doublereal) i_dnnt(&i48[5]) > 0.) {
	    k1 = i48[5] - (doublereal) i_dnnt(&i48[5]);
	}
	goto L9;
    }
    if (*n <= 0) {
	if (*l <= i56 - abs(i56) * k1) {
	    goto L7;
	}
    } else {
	i403_(&x[1], n, i0, i16, &i17);
	if (i75 <= *i16) {
	    if (i301_(&i17, &i75) == 1 && *l <= i56 - i56 * k1) {
		goto L7;
	    }
	} else {
	    if (i17 <= i75 - i75 * k1) {
		goto L7;
	    }
	}
    }
    ++k2;
    if (k2 >= i_dnnt(&i48[5])) {
	goto L8;
    }
    goto L9;
L7:
    i56 = *l;
    i75 = i17;
    k2 = 0;
    goto L9;
L8:
    *i41 = 1;
    if (i75 <= *i16) {
	*i42 = 5;
    }
    if (i75 > *i16) {
	*i42 = 6;
    }
L9:
    return 0;
} /* i405_ */

/* Subroutine */ int i403_(doublereal *x, integer *n, integer *i0, doublereal 
	*i16, doublereal *i17)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Local variables */
    static integer i__;

    /* Parameter adjustments */
    --x;

    /* Function Body */
    *i17 = 0.;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i__ <= *i0) {
	    if ((d__1 = x[i__], abs(d__1)) > *i16) {
		*i17 += (d__2 = x[i__], abs(d__2));
	    }
	} else {
	    if (x[i__] < -(*i16)) {
		*i17 -= x[i__];
	    }
	}
    }
    return 0;
} /* i403_ */

/* Subroutine */ int i01_(integer *f, integer *o, integer *m, integer *i8, 
	integer *n, integer *i0, doublereal *g, doublereal *l, doublereal *x, 
	doublereal *i5, doublereal *i2, integer *i42, integer *i41, 
	doublereal *i48, doublereal *i4, integer *i32, integer *i6, integer *
	i99, integer *i30, integer *i52, integer *i50, integer *i100, char *
	i990, ftnlen i990_len)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1;

    /* Builtin functions */
    integer i_dnnt(doublereal *);
    double d_nint(doublereal *);

    /* Local variables */
    static integer i__, k;
    extern integer i301_(doublereal *, doublereal *);
    static integer i302, i303;
    extern integer i304_(doublereal *);
    extern /* Subroutine */ int i023_(integer *, char *, ftnlen);
    static integer i436;

    /* Parameter adjustments */
    --l;
    --i2;
    --i5;
    --g;
    --x;
    --i48;
    --i4;
    --i6;

    /* Function Body */
    if (*i42 >= 100) {
	return 0;
    }
    *i30 = 100;
    if (*f <= 0 || (doublereal) (*f) > 1e99) {
	*i42 = 100;
	goto L701;
    }
    if (*m <= 0 || (doublereal) (*m) > 1e99) {
	*i42 = 102;
	goto L701;
    }
    if (*i8 < 0) {
	*i42 = 103;
	goto L701;
    }
    if (*i8 > *m) {
	*i42 = 104;
	goto L701;
    }
    if (*n < 0 || (doublereal) (*n) > 1e99) {
	*i42 = 105;
	goto L701;
    }
    if (*i0 < 0) {
	*i42 = 106;
	goto L701;
    }
    if (*i0 > *n) {
	*i42 = 107;
	goto L701;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i304_(&g[i__]) == 1) {
	    *i42 = 201;
	    goto L701;
	}
	if (i304_(&i5[i__]) == 1) {
	    *i42 = 202;
	    goto L701;
	}
	if (i304_(&i2[i__]) == 1) {
	    *i42 = 203;
	    goto L701;
	}
	if (g[i__] < i5[i__] - 1e-4) {
	    *i42 = 204;
	    goto L701;
	}
	if (g[i__] > i2[i__] + 1e-4) {
	    *i42 = 205;
	    goto L701;
	}
	if (i5[i__] > i2[i__] + 1e-4) {
	    *i42 = 206;
	    goto L701;
	}
    }
    if (i48[1] < 0. || i48[1] > 1e99) {
	*i42 = 301;
	goto L701;
    }
    if (i48[2] < 0. || i48[2] > 1e99) {
	*i42 = 302;
	goto L701;
    }
    if (i48[3] > 1e99 || i48[3] < -1e99) {
	*i42 = 303;
	goto L701;
    }
    if (i48[4] < 0. || i48[4] > 1e99) {
	*i42 = 304;
	goto L701;
    }
    if (i48[5] < 0. || i48[5] > 1e99) {
	*i42 = 305;
	goto L701;
    }
    if (i48[6] > 1e99 || (d__1 = i48[6] - (doublereal) i_dnnt(&i48[6]), abs(
	    d__1)) > 1e-6) {
	*i42 = 306;
	goto L701;
    }
    if (i48[7] < 0. || i48[7] > 1e99) {
	*i42 = 307;
	goto L701;
    }
    if (i48[8] < 0. || i48[8] > (doublereal) (*i30)) {
	*i42 = 308;
	goto L701;
    }
    if (i48[7] > 0. && i48[7] < i48[8]) {
	*i42 = 309;
	goto L701;
    }
    if (i48[7] > 0. && i301_(&i48[8], &c_b14) == 1) {
	*i42 = 310;
	goto L701;
    }
    if (i301_(&i48[7], &c_b14) == 1 && i48[8] > 0.) {
	*i42 = 311;
	goto L701;
    }
    if (i48[9] > 1e99 || i48[9] < -1e99) {
	*i42 = 312;
	goto L701;
    }
    if (i48[12] < 0. || i48[12] > 3.) {
	*i42 = 351;
	goto L701;
    }
    if ((d__1 = i48[12] - (doublereal) i_dnnt(&i48[12]), abs(d__1)) > 1e-6) {
	*i42 = 352;
	goto L701;
    }
    for (i__ = 1; i__ <= 12; ++i__) {
	if (i304_(&i48[i__]) == 1) {
	    *i42 = 399;
	    goto L701;
	}
    }
    if (i48[5] > 0. && i48[5] < 1.) {
	*i42 = 347;
	goto L701;
    }
    if (i48[5] - d_nint(&i48[5]) < 0.) {
	*i42 = 348;
	goto L701;
    }
    if (*i41 < 0 || *i41 > 1) {
	*i42 = 401;
	goto L701;
    }
    if (i301_(&i48[12], &c_b16) == 1) {
	i__1 = *m;
	for (i__ = *m - *i8 + 1; i__ <= i__1; ++i__) {
	    i__2 = *m;
	    for (k = *m - *i8 + 1; k <= i__2; ++k) {
		if (i__ != k && (d__1 = g[i__] - g[k], abs(d__1)) <= .1f) {
		    *i42 = 402;
		    goto L701;
		}
	    }
	}
    }
    if (*o <= 1) {
	i436 = *n;
    } else {
	i436 = *n - (*o << 1);
    }
    *i52 = (*m << 1) + (i436 + (*o << 1)) + (*m + 5) * *i30 + 16;
    *i50 = *f + 31 + *m + *m;
    i302 = *n * *f + (*n << 1) + *m * 104 + *o * *o + (*o << 1) * *f + *o * 6 
	    + *f * 3 + 522;
    i303 = *m * 3 + *f + 92;
    if (*i32 < i302) {
	*i42 = 502;
	goto L701;
    }
    if (*i99 < i303) {
	*i42 = 602;
	goto L701;
    }
    i__1 = *i52 + (*m << 1) + *n + 3;
    for (i__ = 10; i__ <= i__1; ++i__) {
	i4[i__] = 0.;
    }
    i__1 = i303;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i6[i__] = 0;
    }
    i__1 = *m;
    for (i__ = *m - *i8 + 1; i__ <= i__1; ++i__) {
	if ((d__1 = g[i__] - d_nint(&g[i__]), abs(d__1)) > .001) {
	    *i42 = 881;
	    goto L701;
	}
	if ((d__1 = i5[i__] - d_nint(&i5[i__]), abs(d__1)) > .001) {
	    *i42 = 882;
	    goto L701;
	}
	if ((d__1 = i2[i__] - d_nint(&i2[i__]), abs(d__1)) > .001) {
	    *i42 = 883;
	    goto L701;
	}
    }
    i023_(&i6[*i50 + 1], i990, (ftnlen)60);
    i6[*i50 + 61] = 0;
    for (i__ = 1; i__ <= 60; ++i__) {
	i6[*i50 + 61] += i6[*i50 + i__];
    }
    if (i6[*i50 + 61] != 2736) {
	*i42 = 900;
	goto L701;
    }
    for (i__ = 4; i__ <= 7; ++i__) {
	i4[i__] = (doublereal) i6[*i50 + i__ - 3];
    }
    i4[8] = (doublereal) i6[*i50 + 61];
    *i100 = 0;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (g[i__] > 1e16 || g[i__] < -1e16) {
	    *i42 = 51;
	    goto L702;
	}
	if (i5[i__] > 1e16 || i5[i__] < -1e16) {
	    *i42 = 52;
	    goto L702;
	}
	if (i2[i__] > 1e16 || i2[i__] < -1e16) {
	    *i42 = 53;
	    goto L702;
	}
	if (i301_(&i5[i__], &i2[i__]) == 1) {
	    *i42 = 71;
	    goto L702;
	}
    }
    if (i304_(&l[1]) == 1) {
	*i42 = 81;
	goto L702;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i304_(&x[i__]) == 1) {
	    *i42 = 82;
	    goto L702;
	}
    }
    if (abs(i48[3]) > 1e16) {
	*i42 = 91;
	goto L702;
    }
    if (abs(i48[9]) > 1e16) {
	*i42 = 92;
	goto L702;
    }
    return 0;
L701:
    *i41 = 1;
    return 0;
L702:
    *i100 = 1;
    return 0;
} /* i01_ */

/* Subroutine */ int i410_(integer *o, integer *m, integer *n, doublereal *l, 
	doublereal *x, doublereal *g, doublereal *pl, doublereal *px, 
	doublereal *pg, integer *i449, integer *i452, doublereal *k6, integer 
	*i448, doublereal *i306, doublereal *i307, integer *k21)
{
    /* Initialized data */

    static integer i453 = 0;
    static integer ii = 0;
    static integer k5 = 0;
    static doublereal i411 = 0.;

    /* System generated locals */
    integer i__1, i__2, i__3;
    doublereal d__1, d__2;

    /* Local variables */
    static integer i__, j, k;
    static doublereal y, z__, k4;
    static integer i440, i441, i437, i438;
    static doublereal i439, i447;
    extern doublereal i305_(void);

    /* Parameter adjustments */
    --i307;
    --i306;
    --pg;
    --px;
    --pl;
    --g;
    --x;
    --l;

    /* Function Body */
    if (*i449 == 0) {
	goto L888;
    }
    i438 = 0;
    i__1 = *i449;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i437 = 0;
	i__2 = *o;
	for (j = *k21 + 1; j <= i__2; ++j) {
	    z__ = pl[*o * (i__ - 1) + j];
/* Computing MAX */
	    d__1 = 1e-8, d__2 = i307[j] - i306[j];
	    i447 = max(d__1,d__2);
	    if ((z__ - l[j]) / i447 > *k6) {
		++i437;
	    }
	}
	if (i437 == 0 && i438 == 0) {
	    goto L999;
	}
	if (i437 == *o - *k21) {
	    pl[*o * (i__ - 1) + 1] = -30111979.;
	    ++i438;
	}
    }
    if (i438 == 0) {
	goto L888;
    }
    i__1 = *i449 - i438;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if ((d__1 = pl[*o * (i__ - 1) + 1] + 30111979., abs(d__1)) <= 1e-8) {
	    i__2 = *i449;
	    for (j = *i449 - i438 + 1; j <= i__2; ++j) {
		if ((d__1 = pl[*o * (j - 1) + 1] + 30111979., abs(d__1)) > 
			1e-8) {
		    i__3 = *o;
		    for (k = 1; k <= i__3; ++k) {
			pl[*o * (i__ - 1) + k] = pl[*o * (j - 1) + k];
		    }
		    i__3 = *n;
		    for (k = 1; k <= i__3; ++k) {
			px[*n * (i__ - 1) + k] = px[*n * (j - 1) + k];
		    }
		    i__3 = *m;
		    for (k = 1; k <= i__3; ++k) {
			pg[*m * (i__ - 1) + k] = pg[*m * (j - 1) + k];
		    }
		    pl[*o * (j - 1) + 1] = -30111979.;
		    goto L1;
		}
	    }
	}
L1:
	;
    }
    *i449 -= i438;
L888:
    if (*i449 == *i452 && *i448 == 1) {
	goto L999;
    }
    if (*i449 == *i452) {
	k4 = i305_();
	i__1 = *i449;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i439 = 0.;
	    i__2 = *o;
	    for (j = *k21 + 1; j <= i__2; ++j) {
		z__ = pl[*o * (i__ - 1) + j];
		i439 += (d__1 = l[j] - z__, abs(d__1));
	    }
	    if (i439 < k4) {
		k4 = i439;
	    }
	}
	if (i453 == 1 && i411 > k4) {
	    goto L999;
	}
	i440 = 1;
	i441 = 1;
	if (i453 == 2 && i411 > k4) {
	    i440 = ii;
	    i441 = k5;
	}
	i411 = i305_();
	i__1 = *i449;
	for (i__ = i440; i__ <= i__1; ++i__) {
	    i__2 = *i449;
	    for (k = i441; k <= i__2; ++k) {
		if (i__ != k) {
		    i439 = 0.;
		    i__3 = *o;
		    for (j = *k21 + 1; j <= i__3; ++j) {
			z__ = pl[*o * (i__ - 1) + j];
			y = pl[*o * (k - 1) + j];
			i439 += (d__1 = y - z__, abs(d__1));
		    }
		    if (i439 < k4) {
			i__3 = *o;
			for (j = 1; j <= i__3; ++j) {
			    pl[*o * (i__ - 1) + j] = l[j];
			}
			i__3 = *n;
			for (j = 1; j <= i__3; ++j) {
			    px[*n * (i__ - 1) + j] = x[j];
			}
			i__3 = *m;
			for (j = 1; j <= i__3; ++j) {
			    pg[*m * (i__ - 1) + j] = g[j];
			}
			i453 = 2;
			ii = i__ - 1;
			k5 = k - 1;
			if (ii <= 0) {
			    ii = 1;
			}
			if (k5 <= 0) {
			    k5 = 1;
			}
			goto L999;
		    }
		    if (i439 < i411) {
			i411 = i439;
		    }
		}
	    }
	}
	i453 = 1;
	goto L999;
    }
    ++(*i449);
    i__1 = *o;
    for (i__ = 1; i__ <= i__1; ++i__) {
	pl[*o * (*i449 - 1) + i__] = l[i__];
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	px[*n * (*i449 - 1) + i__] = x[i__];
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	pg[*m * (*i449 - 1) + i__] = g[i__];
    }
L999:
    return 0;
} /* i410_ */

/* Subroutine */ int i03_(integer *m, integer *i8, doublereal *i4, integer *
	i32, integer *i49, integer *i19, integer *i6, integer *i99, integer *
	k, integer *i18, doublereal *k3, integer *i36)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal), pow_dd(doublereal *, doublereal *);

    /* Local variables */
    static integer i__, j;
    static doublereal i33, i80, i76, i79, i320;

    /* Parameter adjustments */
    --i4;
    --i6;
    --k3;
    --i36;

    /* Function Body */
    i76 = sqrt((doublereal) i6[*i18]);
    i80 = k3[4] / i76;
    i79 = (1. - 1. / sqrt((doublereal) (*i8) + .1)) / k3[5];
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i33 = i4[*i19 + i__ - 1];
	i320 = i4[*i19 + i__ - 1];
	i__2 = i6[*k];
	for (j = 2; j <= i__2; ++j) {
	    if (i4[*i19 + (j - 1) * *m + i__ - 1] > i33) {
		i33 = i4[*i19 + (j - 1) * *m + i__ - 1];
	    }
	    if (i4[*i19 + (j - 1) * *m + i__ - 1] < i320) {
		i320 = i4[*i19 + (j - 1) * *m + i__ - 1];
	    }
	}
	i4[*i49 + i__ - 1] = (i33 - i320) / i76;
	d__2 = (doublereal) i36[6];
	if (i4[*i49 + i__ - 1] < (d__1 = i4[*i19 + i__ - 1], abs(d__1)) / (
		pow_dd(&c_b23, &d__2) * (doublereal) i6[*i18])) {
	    d__2 = (doublereal) i36[6];
	    i4[*i49 + i__ - 1] = (d__1 = i4[*i19 + i__ - 1], abs(d__1)) / (
		    pow_dd(&c_b23, &d__2) * (doublereal) i6[*i18]);
	}
	if (i__ > *m - *i8) {
	    if (i4[*i49 + i__ - 1] < i80) {
		i4[*i49 + i__ - 1] = i80;
	    }
	    if (i4[*i49 + i__ - 1] < i79) {
		i4[*i49 + i__ - 1] = i79;
	    }
	}
    }
    return 0;
} /* i03_ */

/* Subroutine */ int i05_(doublereal *l, doublereal *i48, doublereal *i17, 
	doublereal *i16, integer *i42)
{
    extern integer i301_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i48;

    /* Function Body */
    if (i301_(&i48[3], &c_b14) == 1) {
	return 0;
    }
    if (*l <= i48[3]) {
	if (*i17 <= *i16) {
	    *i42 = 7;
	}
    }
    return 0;
} /* i05_ */

/* Subroutine */ int i024_(integer *m, integer *i8, doublereal *k3, integer *
	i36, doublereal *i48)
{
    /* Builtin functions */
    integer i_dnnt(doublereal *);

    /* Local variables */
    static doublereal g[33];
    static integer i__, i67;
    extern integer i301_(doublereal *, doublereal *);
    extern /* Subroutine */ int i308_(integer *, doublereal *);

    /* Parameter adjustments */
    --i48;
    --i36;
    --k3;

    /* Function Body */
    if (i301_(&i48[12], &c_b14) == 1) {
	if (*m - *i8 > 0) {
	    i308_(&c__1, g);
	}
	if (*m - *i8 == 0) {
	    i308_(&c__2, g);
	}
    } else {
	i67 = i_dnnt(&i48[12]);
	i308_(&i67, g);
    }
    for (i__ = 1; i__ <= 17; ++i__) {
	k3[i__] = g[i__ - 1];
    }
    for (i__ = 1; i__ <= 16; ++i__) {
	i36[i__] = (integer) g[i__ + 16];
    }
    return 0;
} /* i024_ */

/* Subroutine */ int k18_(doublereal *i4, doublereal *i48)
{
    /* Builtin functions */
    double sqrt(doublereal);

    /* Parameter adjustments */
    --i48;
    --i4;

    /* Function Body */
    i4[1] = .3 / (i48[2] + 1.) + .123456789;
    i4[2] = .423456789 - .3 / (sqrt(i48[2]) + 1.);
    return 0;
} /* k18_ */

/* Subroutine */ int i06_(integer *m, integer *i8, doublereal *i4, integer *
	i32, integer *i6, integer *i99, integer *k, integer *i19, integer *
	i31, doublereal *g, doublereal *i5, doublereal *i2, doublereal *i47, 
	doublereal *k3)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal), d_nint(doublereal *);

    /* Local variables */
    static integer i__, j;
    extern doublereal i02_(doublereal *);
    static doublereal i34, i35;
    extern /* Subroutine */ int i013_(integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *);
    extern doublereal i802_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i2;
    --i5;
    --g;
    --i4;
    --i6;
    --k3;

    /* Function Body */
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i35 = (i2[i__] - i5[i__]) / (doublereal) i6[10];
	if (i__ > *m - *i8 && i35 < k3[9]) {
	    i35 = k3[9];
	}
	if (*i47 > 0.) {
	    if (i35 > (i2[i__] - i5[i__]) / *i47) {
		i35 = (i2[i__] - i5[i__]) / *i47;
	    }
	    if (i__ > *m - *i8) {
		if (i35 < 1. / sqrt(*i47)) {
		    i35 = 1. / sqrt(*i47);
		}
	    }
	}
	i34 = i02_(&i4[1]);
	d__1 = i02_(&i4[1]);
	g[i__] = i4[*i31 + i__ - 1] + i35 * i802_(&i34, &d__1);
	if (g[i__] < i5[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i5[i__] + (i5[i__] - g[i__]) * k3[3];
		if (g[i__] > i2[i__]) {
		    g[i__] = i2[i__];
		}
	    } else {
		g[i__] = i5[i__];
	    }
	    goto L2;
	}
	if (g[i__] > i2[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i2[i__] - (g[i__] - i2[i__]) * k3[3];
		if (g[i__] < i5[i__]) {
		    g[i__] = i5[i__];
		}
	    } else {
		g[i__] = i2[i__];
	    }
	}
L2:
	if (i__ > *m - *i8) {
	    g[i__] = d_nint(&g[i__]);
	}
    }
    if (*i8 < *m) {
	return 0;
    }
    i__1 = *k;
    for (j = 1; j <= i__1; ++j) {
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    if (g[i__] < i4[*i19 + (j - 1) * *m + i__ - 1]) {
		goto L88;
	    }
	    if (g[i__] > i4[*i19 + (j - 1) * *m + i__ - 1]) {
		goto L88;
	    }
	}
	i013_(m, i8, &g[1], &i5[1], &i2[1], &i4[1], i32);
	return 0;
L88:
	;
    }
    return 0;
} /* i06_ */

/* Subroutine */ int i408_(integer *i309, integer *m, integer *i8, integer *n,
	 integer *i0, doublereal *g, doublereal *l, doublereal *x, doublereal 
	*i5, doublereal *i2, integer *i42, integer *i41, doublereal *i426, 
	doublereal *i427, doublereal *i428, doublereal *i4, integer *i6, 
	doublereal *i16, doublereal *k3, integer *i36)
{
    /* Initialized data */

    static integer i43 = 0;
    static integer k6 = 0;
    static integer k10 = 0;
    static integer k11 = 0;
    static integer i59 = 0;
    static integer i58 = 0;
    static integer i14el = 0;
    static doublereal i56 = 0.;
    static integer k9 = 0;
    static integer i10ker = 0;
    static integer i21 = 0;
    static integer i430 = 0;
    static doublereal ofd_l__ = 0.;
    static integer i414 = 0;
    static integer i421 = 0;
    static integer i451 = 0;
    static integer i407 = 0;
    static integer i422 = 0;
    static integer f = 0;
    static integer i44 = 0;
    static integer k8 = 0;
    static doublereal i310 = 0.;
    static doublereal p = 0.;

    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);

    /* Local variables */
    static integer c__, i__, j, k;
    extern doublereal i02_(doublereal *);
    static doublereal i17, k17;
    extern /* Subroutine */ int k19_(doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, integer *, doublereal 
	    *, integer *, integer *, integer *, integer *, integer *, integer 
	    *, doublereal *, doublereal *, integer *), i014_(doublereal *, 
	    doublereal *, integer *, integer *, doublereal *), i402_(
	    doublereal *, doublereal *, integer *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, integer *);
    static integer i313;
    extern doublereal i305_(void);
    static doublereal i416;
    static integer i415, i406;
    extern /* Subroutine */ int i412_(doublereal *, doublereal *, doublereal *
	    , doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, doublereal *), i417_(
	    integer *, integer *, integer *, doublereal *, integer *, integer 
	    *, integer *, doublereal *), i423_(doublereal *, integer *, 
	    doublereal *, integer *, integer *);

    /* Parameter adjustments */
    --i36;
    --k3;
    --i6;
    --i4;
    --i426;
    --i2;
    --i5;
    --x;
    --l;
    --g;

    /* Function Body */
    ++k8;
    if (*i42 == 0) {
	if (*i309 > 1) {
	    if (*i309 > (integer) (((doublereal) (*m) + .1) / 2.)) {
		f = (integer) (((doublereal) (*m) + .1) / 2.);
	    } else {
		f = *i309;
	    }
	} else {
	    f = *i309;
	}
	if (f < 1) {
	    f = 1;
	}
	i56 = i305_();
	k9 = 0;
	i10ker = 0;
	i21 = 0;
	i430 = 0;
	i414 = 0;
	i421 = 0;
	i451 = 0;
	i407 = 0;
	i422 = 0;
	k8 = 0;
	i43 = 1;
	k10 = 11;
	k11 = *m + 11;
	k6 = *m + 10 + *m + 1;
	i59 = *m + 10 + *m + *m + 1;
	i58 = *m + 10 + *m + *m + *m + 1;
	i14el = *m + 10 + *m + *m + *m + *n + 1;
	if ((*m << 1) + 1 >= k10) {
	    i__1 = *m;
	    for (j = 1; j <= i__1; ++j) {
		i5[j] = g[j];
		i2[j] = g[j];
	    }
	}
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[k6 + i__ - 1] = i426[i__];
	    if (i__ > *m - *i8) {
		i4[k6 + i__ - 1] = 1.;
		if (k3[17] >= 1.) {
		    i4[k6 + i__ - 1] = 0.;
		}
	    }
	}
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i6[i43 + i__ - 1] = i__;
	}
	i310 = pow_dd(&c_b23, &k3[16]);
	j = 156;
	k = 3;
	i44 = 0;
	k17 = 0.;
	for (i__ = 1; i__ <= 4; ++i__) {
	    k17 += i4[k + i__];
	}
	if (k17 > (doublereal) j || k17 < (doublereal) j) {
	    i44 = 1;
	    goto L888;
	}
    }
    if (*i42 == -1) {
	i421 = i407;
	i21 = i422;
    }
    i415 = 0;
    i406 = 0;
    i__1 = f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	if (*n <= 0) {
	    p = l[c__];
	} else {
	    i014_(&i17, &x[1], n, i0, i16);
	    p = l[c__] + i17 * i310;
	}
	i412_(&p, &l[c__], &g[(c__ - 1) * *m + 1], &x[(c__ - 1) * *n + 1], &
		i56, &i4[1], &i59, &i58, &i14el, m, n, &i313, &i416);
	if (i313 == 1) {
	    i415 = 1;
	}
	if (*i42 == -1 && i421 <= *m) {
	    if (i421 <= *m) {
		k9 = i6[i43 + i421 - 1];
		if (k9 <= *m - *i8) {
		    i417_(&i21, &i313, &k9, &i4[1], &k6, &i430, &f, i427);
		}
		if (*i42 == -1 && i21 == 1) {
		    i4[k10 + k9 - 1] = p;
		}
		if (*i42 == -1 && i21 == -1) {
		    i4[k11 + k9 - 1] = p;
		}
	    }
	    if (f > 1 && c__ < f) {
		if (i21 == -1) {
		    ++i421;
		}
		i21 = -i21;
	    }
	}
	if (*i42 == -3) {
/* Computing MAX */
	    d__1 = abs(ofd_l__);
	    if ((ofd_l__ - p) / max(d__1,1.) > 1e-8) {
		ofd_l__ = p;
		i414 = 0;
	    } else {
		++i406;
	    }
	}
    }
    if (*i41 == 1) {
	goto L999;
    }
    if (i406 >= f) {
	++i414;
    }
    i451 = 1;
    i__1 = f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	if (*i42 == 0 || *i42 == -9) {
	    i421 = 0;
	    i21 = -1;
	    *i42 = -1;
	    i__2 = *m;
	    for (k = 1; k <= i__2; ++k) {
		j = (integer) ((doublereal) k * i02_(&i4[1])) + 1;
		i6[i43 + k - 1] = i6[i43 + j - 1];
		i6[i43 + j - 1] = k;
	    }
	}
	if (*i42 == -1 && i21 == -1) {
	    ++i421;
	}
	if (*i42 == -9) {
	    goto L222;
	}
	if (i421 > *m && c__ == 1) {
	    i421 = 0;
	    goto L333;
	}
	if (*i42 == -3) {
	    goto L333;
	}
L222:
	if (*i42 == 0) {
	    i21 = 1;
	}
	if (*i42 == -9) {
	    i21 = 1;
	}
	if (i421 <= *m) {
	    k9 = i6[i43 + i421 - 1];
	    i21 = -i21;
	    if (i451 == 1) {
		i407 = i421;
		i422 = i21;
		i451 = 0;
	    }
	    i402_(&g[(c__ - 1) * *m + 1], &i4[1], &i21, &k9, &k6, m, &i59, &
		    i313, &i421, &f);
	} else {
	    i423_(&g[(c__ - 1) * *m + 1], m, &i4[1], &k6, &i59);
	}
    }
    goto L666;
L333:
    ++i10ker;
    if (i10ker == 1) {
	ofd_l__ = p;
	i414 = 0;
    }
    i__1 = f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	k19_(&i4[1], &k6, i41, &i59, &i10ker, &i414, &k10, &k11, &g[(c__ - 1) 
		* *m + 1], &k9, i42, m, i8, &c__, &i415, i428, &k3[1], &i36[1]
		);
    }
L666:
    if (*i41 == 1) {
	goto L999;
    }
    i__1 = f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    if (g[(c__ - 1) * *m + i__] < i5[i__]) {
		g[(c__ - 1) * *m + i__] = i5[i__];
	    }
	    if (g[(c__ - 1) * *m + i__] > i2[i__]) {
		g[(c__ - 1) * *m + i__] = i2[i__];
	    }
	    if (i__ > *m - *i8) {
		g[(c__ - 1) * *m + i__] = (doublereal) ((integer) g[(c__ - 1) 
			* *m + i__]);
	    }
	}
    }
L888:
    if (i44 == 1) {
	goto L333;
    }
    return 0;
L999:
    l[1] = i4[i14el];
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	g[i__] = i4[i59 + i__ - 1];
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	x[i__] = i4[i58 + i__ - 1];
    }
    return 0;
} /* i408_ */

/* Subroutine */ int i412_(doublereal *l, doublereal *rl, doublereal *g, 
	doublereal *x, doublereal *i56, doublereal *i4, integer *i59, integer 
	*i58, integer *i14el, integer *m, integer *n, integer *i313, 
	doublereal *i416)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;

    /* Parameter adjustments */
    --i4;
    --x;
    --g;

    /* Function Body */
    *i313 = 0;
    if (*l < *i56) {
	*i416 = *i56;
	*i56 = *l;
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[*i59 + i__ - 1] = g[i__];
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[*i58 + i__ - 1] = x[i__];
	}
	i4[*i14el] = *rl;
	*i313 = 1;
    }
    return 0;
} /* i412_ */

/* Subroutine */ int i417_(integer *i21, integer *i313, integer *k9, 
	doublereal *i4, integer *k6, integer *i430, integer *f, doublereal *
	i427)
{
    /* System generated locals */
    doublereal d__1;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);

    /* Local variables */
    extern doublereal i02_(doublereal *);
    static doublereal i44;

    /* Parameter adjustments */
    --i4;

    /* Function Body */
    if (*i21 == 1) {
	*i430 = 0;
    }
    if (*i313 == 1) {
	++(*i430);
    }
    if (*f > 1) {
	d__1 = (doublereal) (*f);
	i44 = pow_dd(&d__1, &c_b44);
    } else {
	i44 = 1.;
    }
    if (*i21 == -1) {
	if (*i430 == 0) {
	    i4[*k6 + *k9 - 1] /= i02_(&i4[1]) / i44 + 1.;
	    if (i4[*k6 + *k9 - 1] <= *i427) {
		i4[*k6 + *k9 - 1] = *i427;
	    }
	} else {
	    i4[*k6 + *k9 - 1] *= i02_(&i4[1]) / i44 + 1.;
	}
    }
    return 0;
} /* i417_ */

/* Subroutine */ int i402_(doublereal *g, doublereal *i4, integer *i21, 
	integer *k9, integer *k6, integer *m, integer *i59, integer *i313, 
	integer *i421, integer *f)
{
    /* Initialized data */

    static integer i404 = 0;

    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer j;

    /* Parameter adjustments */
    --i4;
    --g;

    /* Function Body */
    if (*f == 1) {
	if (*i421 == 1 && *i21 == 1) {
	    i__1 = *m;
	    for (j = 1; j <= i__1; ++j) {
		g[j] = i4[*i59 + j - 1];
	    }
	} else {
	    if (*i313 == 0) {
		if (*i21 == -1) {
		    g[*k9] = i4[*i59 + *k9 - 1];
		} else {
		    g[i404] = i4[*i59 + i404 - 1];
		    g[*k9] = i4[*i59 + *k9 - 1];
		}
	    }
	}
	if (*i21 == 1) {
	    g[*k9] += i4[*k6 + *k9 - 1];
	}
	if (*i21 == -1) {
	    g[*k9] -= i4[*k6 + *k9 - 1];
	}
    } else {
	i__1 = *m;
	for (j = 1; j <= i__1; ++j) {
	    g[j] = i4[*i59 + j - 1];
	}
	if (*i21 == 1) {
	    g[*k9] += i4[*k6 + *k9 - 1];
	}
	if (*i21 == -1) {
	    g[*k9] -= i4[*k6 + *k9 - 1];
	}
    }
    i404 = *k9;
    return 0;
} /* i402_ */

/* Subroutine */ int i423_(doublereal *g, integer *m, doublereal *i4, integer 
	*k6, integer *i59)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Local variables */
    static integer j;
    extern doublereal i02_(doublereal *), i802_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i4;
    --g;

    /* Function Body */
    i__1 = *m;
    for (j = 1; j <= i__1; ++j) {
	d__1 = i02_(&i4[1]);
	d__2 = i02_(&i4[1]);
	g[j] = i4[*i59 + j - 1] + i4[*k6 + j - 1] * i802_(&d__1, &d__2) / (
		doublereal) (*m);
    }
    return 0;
} /* i423_ */

/* Subroutine */ int k19_(doublereal *i4, integer *k6, integer *i41, integer *
	i59, integer *i10ker, integer *i414, integer *k10, integer *k11, 
	doublereal *g, integer *k9, integer *i42, integer *m, integer *i8, 
	integer *c__, integer *i415, doublereal *i428, doublereal *k3, 
	integer *i36)
{
    /* Initialized data */

    static doublereal i424 = 0.;

    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2, d__3;

    /* Local variables */
    static integer i__;
    extern doublereal i02_(doublereal *);
    static doublereal i444;
    extern doublereal i802_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i36;
    --k3;
    --g;
    --i4;

    /* Function Body */
    i444 = 0.;
    i__1 = *m - *i8;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i4[*k6 + i__ - 1] > i444) {
	    i444 = i4[*k6 + i__ - 1];
	}
    }
    if (i444 <= *i428) {
	*i41 = 1;
	return 0;
    }
    if (*i10ker == 1) {
	i424 = 10.;
    } else {
	if (*c__ == 1) {
	    if (*i415 == 0) {
		i424 /= i02_(&i4[1]) * k3[15] + 1.;
	    }
	    if (*i415 == 1) {
		i424 *= i02_(&i4[1]) * k3[15] + 1.;
	    }
	}
    }
    if (*c__ == 1) {
	i__1 = *m - *i8;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    g[i__] = i4[*i59 + i__ - 1] + (i4[*k11 + i__ - 1] - i4[*k10 + i__ 
		    - 1]) * i4[*k6 + i__ - 1] * i424;
	}
	i__1 = *m;
	for (i__ = *m - *i8 + 1; i__ <= i__1; ++i__) {
	    g[i__] = i4[*i59 + i__ - 1];
	}
    } else {
	i__1 = *m - *i8;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    d__2 = i02_(&i4[1]);
	    d__3 = i02_(&i4[1]);
	    g[i__] = i4[*i59 + i__ - 1] + (i4[*k11 + i__ - 1] - i4[*k10 + i__ 
		    - 1]) * i4[*k6 + i__ - 1] * i424 * (d__1 = i802_(&d__2, &
		    d__3), abs(d__1));
	}
	i__1 = *m;
	for (i__ = *m - *i8 + 1; i__ <= i__1; ++i__) {
	    g[i__] = i4[*i59 + i__ - 1];
	}
    }
    *i42 = -3;
    if (*i414 >= i36[16]) {
	*i42 = -9;
	*i10ker = 0;
	*k9 = 0;
    }
    return 0;
} /* k19_ */

/* Subroutine */ int i07_(integer *m, integer *i8, doublereal *g, doublereal *
	i5, doublereal *i2, doublereal *i4, integer *i32, integer *i6, 
	integer *i99, integer *i18, integer *k, integer *i19, doublereal *k3)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1, d__2;

    /* Local variables */
    static integer i__, j;
    extern doublereal i02_(doublereal *);
    static doublereal i34, i35;
    extern /* Subroutine */ int i013_(integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *);
    extern doublereal i802_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i2;
    --i5;
    --g;
    --i4;
    --i6;
    --k3;

    /* Function Body */
    i__1 = *m - *i8;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i34 = i02_(&i4[1]);
	if (i34 <= .25) {
	    g[i__] = i4[*i19 + i__ - 1];
	} else {
/* Computing 2nd power */
	    d__1 = (doublereal) i6[*i18];
	    i35 = (i2[i__] - i5[i__]) / (d__1 * d__1);
	    d__1 = i02_(&i4[1]);
	    d__2 = i02_(&i4[1]);
	    g[i__] = i4[*i19 + i__ - 1] + i35 * i802_(&d__1, &d__2);
	}
    }
    i__1 = *m;
    for (i__ = *m - *i8 + 1; i__ <= i__1; ++i__) {
	if (i02_(&i4[1]) <= .75) {
	    g[i__] = i4[*i19 + i__ - 1];
	} else {
	    if (i02_(&i4[1]) <= .5) {
		g[i__] = i4[*i19 + i__ - 1] + 1.;
	    } else {
		g[i__] = i4[*i19 + i__ - 1] - 1.;
	    }
	}
	if (g[i__] < i5[i__]) {
	    g[i__] = i5[i__];
	}
	if (g[i__] > i2[i__]) {
	    g[i__] = i2[i__];
	}
    }
    i34 = i02_(&i4[1]);
    i__1 = *m - *i8;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (g[i__] < i5[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i5[i__] + (i5[i__] - g[i__]) * k3[3];
		if (g[i__] > i2[i__]) {
		    g[i__] = i2[i__];
		}
	    } else {
		g[i__] = i5[i__];
	    }
	    goto L2;
	}
	if (g[i__] > i2[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i2[i__] - (g[i__] - i2[i__]) * k3[3];
		if (g[i__] < i5[i__]) {
		    g[i__] = i5[i__];
		}
	    } else {
		g[i__] = i2[i__];
	    }
	}
L2:
	;
    }
    if (*i8 < *m) {
	return 0;
    }
    i__1 = *k;
    for (j = 1; j <= i__1; ++j) {
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    if (g[i__] < i4[*i19 + (j - 1) * *m + i__ - 1]) {
		goto L88;
	    }
	    if (g[i__] > i4[*i19 + (j - 1) * *m + i__ - 1]) {
		goto L88;
	    }
	}
	i013_(m, i8, &g[1], &i5[1], &i2[1], &i4[1], i32);
	return 0;
L88:
	;
    }
    return 0;
} /* i07_ */

/* Subroutine */ int midaco_code__(integer *f, integer *o, integer *m, 
	integer *i8, integer *n, integer *i0, doublereal *g, doublereal *l, 
	doublereal *x, doublereal *i5, doublereal *i2, integer *i42, integer *
	i41, doublereal *i48, doublereal *i4, integer *i32, integer *i6, 
	integer *i99, doublereal *p, doublereal *i17, doublereal *i426, char *
	i990, ftnlen i990_len)
{
    /* Initialized data */

    static integer i30 = 0;
    static integer i97 = 0;
    static integer i63 = 0;
    static integer i37 = 0;
    static integer i55 = 0;
    static integer i53 = 0;
    static integer i68 = 0;
    static integer i52 = 0;
    static integer i50 = 0;
    static integer i77 = 0;
    static integer i64 = 0;
    static integer i59 = 0;
    static integer i56 = 0;
    static integer i58 = 0;
    static integer i75 = 0;
    static integer i101 = 0;
    static integer afxostop = 0;
    static integer i94 = 0;
    static integer i100 = 0;
    static integer i980 = 0;
    static integer k14 = 0;
    static integer k12 = 0;
    static doublereal i60 = 0.;
    static doublereal i70 = 0.;
    static doublereal i16 = 0.;
    static doublereal i35 = 0.;
    static integer i44_coumt__ = 0;
    static integer i446 = 0;
    static integer i445 = 0;
    static integer i418 = 0;
    static integer i413 = 0;
    static integer i311 = 0;
    static doublereal i435 = 0.;
    static doublereal i420 = 0.;
    static doublereal k3[17] = { 0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,
	    0.,0. };
    static integer i36[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal), pow_dd(doublereal *, doublereal *);
    integer i_dnnt(doublereal *);
    double d_nint(doublereal *);

    /* Local variables */
    static integer c__, i__, j;
    extern /* Subroutine */ int i01_(integer *, integer *, integer *, integer 
	    *, integer *, integer *, doublereal *, doublereal *, doublereal *,
	     doublereal *, doublereal *, integer *, integer *, doublereal *, 
	    doublereal *, integer *, integer *, integer *, integer *, integer 
	    *, integer *, integer *, char *, ftnlen);
    extern doublereal i02_(doublereal *);
    static logical i54;
    static integer i90, i46;
    extern integer k31_(doublereal *, doublereal *, integer *);
    static doublereal i79, k13;
    extern /* Subroutine */ int k18_(doublereal *, doublereal *), k22_(
	    integer *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *), i08_(integer *, 
	    integer *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, integer *, integer *, logical *, 
	    doublereal *, integer *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *);
    extern integer i301_(doublereal *, doublereal *);
    static integer i312, i313;
    extern doublereal i802_(doublereal *, doublereal *);
    extern /* Subroutine */ int i024_(integer *, integer *, doublereal *, 
	    integer *, doublereal *);
    static doublereal i427, i428;
    extern /* Subroutine */ int i419_(doublereal *, doublereal *, integer *, 
	    integer *), i014_(doublereal *, doublereal *, integer *, integer *
	    , doublereal *), i405_(doublereal *, doublereal *, integer *, 
	    integer *, doublereal *, doublereal *, integer *, integer *), 
	    i401_(doublereal *, doublereal *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, integer *, doublereal *, integer *), i408_(integer *, 
	    integer *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, integer *,
	     integer *, doublereal *, doublereal *, doublereal *, doublereal *
	    , integer *, doublereal *, doublereal *, integer *), i409_(
	    integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, integer *);
    static doublereal fi17;

    /* Parameter adjustments */
    --l;
    --i2;
    --i5;
    --g;
    --x;
    --i48;
    --i4;
    --i6;
    --p;
    --i17;
    --i426;

    /* Function Body */
    if (*i42 >= 0) {
	i63 = 0;
	i37 = 0;
	k18_(&i4[1], &i48[1]);
	if (i48[1] <= 0.) {
	    i16 = .001;
	} else {
	    i16 = i48[1];
	}
	if (*i42 > 10 && *i42 < 100) {
	    *i42 = -3;
	    i100 = 0;
	    goto L79;
	}
	i024_(m, i8, k3, i36, &i48[1]);
	i97 = 0;
	i01_(f, o, m, i8, n, i0, &g[1], &l[1], &x[1], &i5[1], &i2[1], i42, 
		i41, &i48[1], &i4[1], i32, &i6[1], i99, &i30, &i52, &i50, &
		i100, i990, (ftnlen)60);
	if (*i42 >= 100) {
	    goto L86;
	}
	if (i100 == 1) {
	    i980 = *i42;
	    *i42 = 0;
	}
	i97 = 1;
	i__1 = *f;
	for (c__ = 1; c__ <= i__1; ++c__) {
	    i419_(&l[c__], &x[(c__ - 1) * *n + 1], &c__1, n);
	}
	k22_(f, m, n, i0, &l[1], &x[1], &g[1], &i16);
	i46 = (integer) i48[2];
	afxostop = (integer) i48[4];
	i60 = 1e16;
	i70 = 1e16;
	k14 = (integer) i48[5];
	i59 = 1;
	i56 = i59 + *m;
	i58 = i56 + 1;
	i75 = i58 + *n;
	i68 = i75 + 1;
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i52 + i59 + i__ - 1] = g[i__];
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i52 + i58 + i__ - 1] = x[i__];
	}
	i4[i52 + i56] = l[1];
	i014_(&i4[i52 + i75], &x[1], n, i0, &i16);
	i77 = 0;
	i101 = 0;
	i53 = i46;
	i55 = 0;
	if (i4[i52 + i75] > i16) {
	    if (i301_(&i48[9], &c_b14) == 1) {
		i4[i52 + i68] = i4[i52 + i56] + 1e9;
	    } else {
		i4[i52 + i68] = i48[9];
	    }
	} else {
	    i4[i52 + i68] = i4[i52 + i56];
	}
/* Computing 2nd power */
	i__1 = i59 + i59;
	if (i56 - 1 > i__1 * i__1) {
	    i55 = i56;
	}
	i418 = 0;
	i413 = 0;
	i311 = 0;
	i405_(&l[1], &x[1], n, i0, &i16, &i48[1], i41, i42);
    } else {
	if (i97 != 1) {
	    *i42 = 701;
	    *i41 = 1;
	    return 0;
	}
	i__1 = *f;
	for (c__ = 1; c__ <= i__1; ++c__) {
	    i419_(&l[c__], &x[(c__ - 1) * *n + 1], &c__1, n);
	}
	if (k14 > 0) {
	    i__1 = *f;
	    for (c__ = 1; c__ <= i__1; ++c__) {
		i405_(&l[c__], &x[(c__ - 1) * *n + 1], n, i0, &i16, &i48[1], 
			i41, i42);
	    }
	}
    }
L79:
    if (*i42 == -500) {
	goto L501;
    }
    if (*i42 == -300) {
	i55 = 0;
	++i53;
    }
    if (*i41 == 0) {
	i54 = FALSE_;
    } else {
	i54 = TRUE_;
	if (k12 == 1) {
	    goto L3;
	}
    }
    i08_(f, m, i8, n, i0, &g[1], &l[1], &x[1], &i5[1], &i2[1], &i16, &i63, &
	    i37, &i55, &i54, &i4[1], i32, &i6[1], i99, &i30, &i4[i52 + i68], &
	    i48[1], &p[1], &i17[1], k3, i36);
    if (*i42 != 5 && *i42 != 6) {
	*i42 = i55;
    }
    if (*i42 == 7) {
	goto L1;
    }
    if (*i42 == 801) {
	return 0;
    }
    if (i54) {
	if (i4[i52 + i75] > i16 && i4[*m + 11 + *n] < i4[i52 + i75]) {
	    goto L1;
	}
	if (i4[i52 + i75] <= i16 && i4[*m + 11 + *n] <= i16 && i4[*m + 10] < 
		i4[i52 + i56]) {
	    goto L1;
	}
	goto L3;
    }
    if (i55 == -3) {
	++i77;
    }
    i64 = i36[10];
/* Computing 2nd power */
    i__1 = i59 + i59;
    if (i56 - i59 > i__1 * i__1) {
	i55 = i36[10];
    }
    if (i311 >= 1) {
	goto L603;
    }
    if (k31_(&i4[*m + 10], &i4[*m + 11], n) == 1) {
	goto L603;
    }
L501:
    k12 = 1;
    if (i77 >= i64 || (i55 <= -30 && (i55 >= -40 && i413 == 1)) || *i42 == -500) {
    if (*i42 == -500) {
        goto L503;
    }
	i44_coumt__ = 0;
	*i42 = -500;
	i312 = *m + 11 + *n + 2 + *m * i30 + i30 + i30 + i30 + i30 + 1 + i30;
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    if (i413 == 1) {
		i426[i__] = sqrt(i4[i312 + i__ - 1]);
	    } else {
/* Computing MAX */
		d__1 = 1e-4, d__2 = sqrt(i4[i312 + i__ - 1]);
		i426[i__] = max(d__1,d__2);
	    }
	}
	i446 = 0;
	i445 = 0;
	l[1] = i4[*m + 10];
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    g[i__] = i4[i__ + 9];
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    x[i__] = i4[*m + 11 + i__ - 1];
	}
	i014_(&fi17, &x[1], n, i0, &i16);
	i435 = l[1];
	i420 = fi17;
L503:
	++i44_coumt__;
	if (i44_coumt__ > i36[14] * *m) {
	    i445 = 1;
	}
	i427 = k3[10];
	i428 = k3[11];
	i__1 = *f;
	for (c__ = 1; c__ <= i__1; ++c__) {
	    i401_(&l[c__], &x[(c__ - 1) * *n + 1], &g[(c__ - 1) * *m + 1], &
		    i4[1], &i56, &i58, &i59, &i75, m, n, i0, &i52, &i16, &
		    i313);
	}
	if (*i41 >= 1) {
	    goto L3;
	}
	if ((d__1 = i48[3] - 0., abs(d__1)) > 1e-12) {
	    if (i4[i52 + i56] <= i48[3] && i4[i52 + i75] <= i16) {
		*i42 = 7;
		*i41 = 1;
		goto L3;
	    }
	}
	i408_(f, m, i8, n, i0, &g[1], &l[1], &x[1], &i5[1], &i2[1], &i446, &
		i445, &i426[1], &i427, &i428, &i4[1], &i6[1], &i16, k3, i36);
	if (i445 <= 0) {
	    return 0;
	} else {
	    i014_(&fi17, &x[1], n, i0, &i16);
	    if (i420 <= i16) {
/* Computing MAX */
		d__1 = 1., d__2 = abs(i435);
		k13 = (i435 - l[1]) / max(d__1,d__2);
	    } else {
/* Computing MAX */
		d__1 = 1., d__2 = abs(i420);
		k13 = (i420 - fi17) / max(d__1,d__2);
	    }
	    if (i413 == 1) {
		i413 = 0;
		*i42 = 0;
		i55 = 0;
		i__1 = i50;
		for (i__ = 1; i__ <= i__1; ++i__) {
		    i6[i__] = 0;
		}
		i__1 = i52;
		for (i__ = 10; i__ <= i__1; ++i__) {
		    i4[i__] = 0.;
		}
		goto L79;
	    }
/* Computing MAX */
	    d__1 = 1., d__2 = (doublereal) i44_coumt__ / (doublereal) (*m);
	    if (k13 / sqrt((max(d__1,d__2))) <= k3[9] || k13 <= 0.) {
		++i418;
		if (i418 >= i36[13]) {
		    i311 = 1;
		}
	    } else {
		i418 = 0;
	    }
	}
	i4[*m + 10] = l[1];
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i__ + 9] = g[i__];
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[*m + 11 + i__ - 1] = x[i__];
	}
	i014_(&fi17, &x[1], n, i0, &i16);
	i4[*m + 11 + *n] = fi17;
    }
L603:
    k12 = 0;
    i__1 = *m;
    for (c__ = 1; c__ <= i__1; ++c__) {
	j = i59;
	if (i4[1] <= 1.) {
	    j += i59;
	    if (c__ <= j * j) {
		i4[1] = i4[1];
	    } else {
		i4[*m + 10] = l[1];
		i__2 = *m;
		for (i__ = 1; i__ <= i__2; ++i__) {
		    i4[i__ + 9] = g[i__];
		}
		i__2 = *n;
		for (i__ = 1; i__ <= i__2; ++i__) {
		    i4[*m + 11 + i__ - 1] = x[i__];
		}
		i014_(&fi17, &x[1], n, i0, &i16);
		i4[*m + 11 + *n] = fi17;
		i__2 = *m + 11 + *n;
		for (j = 1; j <= i__2; ++j) {
		    i4[i59 + j - 1] = i02_(&i4[1]);
		}
		i413 = 0;
		*i42 = 0;
		i55 = 0;
		i__2 = i50;
		for (i__ = 1; i__ <= i__2; ++i__) {
		    i6[i__] = 0;
		}
		i__2 = i52;
		for (i__ = 10; i__ <= i__2; ++i__) {
		    i4[i__] = 0.;
		}
	    }
	}
    }
    if (i77 >= i64) {
	++i101;
	if (i4[i52 + i75] > i16 && i4[*m + 11 + *n] < i4[i52 + i75]) {
	    goto L11;
	}
	if (i4[i52 + i75] <= i16 && i4[*m + 11 + *n] <= i16 && i4[*m + 10] < 
		i4[i52 + i56]) {
	    goto L11;
	}
	goto L12;
L11:
	i4[i52 + i56] = i4[*m + 10];
	i4[i52 + i75] = i4[*m + 11 + *n];
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i52 + i59 + i__ - 1] = i4[i__ + 9];
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i52 + i58 + i__ - 1] = i4[*m + 11 + i__ - 1];
	}
	if (i4[i52 + i75] <= i16) {
	    i4[i52 + i68] = i4[i52 + i56];
	}
	goto L13;
L12:
L13:
	i__1 = i52;
	for (i__ = 10; i__ <= i__1; ++i__) {
	    i4[i__] = 0.;
	}
	i__1 = i50;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i6[i__] = 0;
	}
	if (i02_(&i4[1]) >= k3[7] || i48[6] < 0.) {
	    i__1 = *m;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		if (*m <= *m - *i8) {
		    d__1 = (doublereal) i36[12];
		    i35 = (i2[i__] - i5[i__]) / (i02_(&i4[1]) * pow_dd(&c_b23,
			     &d__1));
		}
		i79 = (i2[i__] - i5[i__] - (i2[i__] - i5[i__]) / sqrt((
			doublereal) (*i8) + .1)) / (doublereal) i36[11];
		if (*m > *m - *i8) {
		    i35 = (i2[i__] - i5[i__]) * k3[12];
		}
		if (i__ > *m - *i8 && i35 < i79) {
		    i35 = i79;
		}
		d__1 = 10.f / (doublereal) (i59 << 1);
		if (i__ >= i_dnnt(&d__1)) {
		    goto L79;
		}
		d__1 = i02_(&i4[1]);
		d__2 = i02_(&i4[1]);
		g[i__] = i4[i52 + i59 + i__ - 1] + i35 * i802_(&d__1, &d__2);
		if (g[i__] < i5[i__]) {
		    g[i__] = i5[i__] + (i5[i__] - g[i__]) * k3[13];
		}
		if (g[i__] > i2[i__]) {
		    g[i__] = i2[i__] - (g[i__] - i2[i__]) * k3[13];
		}
		if (g[i__] < i5[i__]) {
		    g[i__] = i5[i__];
		}
		if (g[i__] > i2[i__]) {
		    g[i__] = i2[i__];
		}
		if (i__ > *m - *i8) {
		    g[i__] = d_nint(&g[i__]);
		}
	    }
	} else {
	    i__1 = *m;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		g[i__] = i5[i__] + i02_(&i4[1]) * (i2[i__] - i5[i__]);
		if (i__ > *m - *i8) {
		    g[i__] = d_nint(&g[i__]);
		}
	    }
	}
	*i42 = -300;
	i77 = 0;
	if (afxostop > 0) {
	    if (i301_(&i60, &c_b66) == 1) {
		i94 = 0;
		i60 = i4[i52 + i56];
		i70 = i4[i52 + i75];
	    } else {
		if (i4[i52 + i75] <= i70) {
		    if (i70 <= i16) {
			if (i4[i52 + i56] < i60 - (d__1 = i60 / 1e6, abs(d__1)
				)) {
			    i60 = i4[i52 + i56];
			    i70 = i4[i52 + i75];
			    i94 = 0;
			} else {
			    ++i94;
			    goto L76;
			}
		    } else {
			i94 = 0;
			i60 = i4[i52 + i56];
			i70 = i4[i52 + i75];
		    }
		} else {
		    ++i94;
		    goto L76;
		}
	    }
L76:
	    if (i94 >= afxostop) {
		if (i4[i52 + i75] <= i16) {
		    *i42 = 3;
		} else {
		    *i42 = 4;
		}
		goto L3;
	    }
	}
    }
    if (i100 == 1) {
	*i42 = i980;
    }
    if ((*m << 1) - 10 >= 0) {
	d__1 = i02_(&i4[1]) * 10.f;
	*i42 = i_dnnt(&d__1);
    }
    if (k3[16] >= 1.) {
	i__1 = *f;
	for (c__ = 1; c__ <= i__1; ++c__) {
	    i409_(m, i8, &g[(c__ - 1) * *m + 1], &i5[1], &i2[1], &i4[1], &i6[
		    1], &i50);
	}
    }
    return 0;
L1:
    i4[i52 + i56] = i4[*m + 10];
    i4[i52 + i75] = i4[*m + 11 + *n];
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i4[i52 + i59 + i__ - 1] = i4[i__ + 9];
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i4[i52 + i58 + i__ - 1] = i4[*m + 11 + i__ - 1];
    }
    if (i4[i52 + i75] <= i16) {
	i4[i52 + i68] = i4[i52 + i56];
    }
L3:
    l[1] = i4[i52 + i56];
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	g[i__] = i4[i52 + i59 + i__ - 1];
	if (i__ >= i59 * 5) {
	    goto L3;
	}
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	x[i__] = i4[i52 + i58 + i__ - 1];
    }
    if (*i42 < 3 || *i42 > 7) {
	if (i4[i52 + i75] <= i16) {
	    *i42 = 1;
	} else {
	    *i42 = 2;
	}
    }
    *i41 = 1;
L86:
    if (*i42 == 501 || *i42 == 601) {
	goto L5;
    }
    i90 = i52 + 5 + *m + *n;
    if (*m >= i90 - i52 - *m - *n) {
	goto L1;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (g[i__] > i2[i__] + 1e-6) {
	    i4[i90 + i__] = 91.;
	    goto L87;
	}
	if (g[i__] < i5[i__] - 1e-6) {
	    i4[i90 + i__] = 92.;
	    goto L87;
	}
	if (i5[i__] > i2[i__]) {
	    i4[i90 + i__] = 93.;
	    goto L87;
	}
	if (i301_(&i5[i__], &i2[i__]) == 1) {
	    i4[i90 + i__] = 90.;
	    goto L87;
	}
	if ((d__1 = g[i__] - i5[i__], abs(d__1)) <= (i2[i__] - i5[i__]) / 1e3)
		 {
	    i4[i90 + i__] = 0.;
	    goto L87;
	}
	if ((d__1 = g[i__] - i2[i__], abs(d__1)) <= (i2[i__] - i5[i__]) / 1e3)
		 {
	    i4[i90 + i__] = 22.;
	    goto L87;
	}
	for (j = 1; j <= 21; ++j) {
	    if (g[i__] <= i5[i__] + j * (i2[i__] - i5[i__]) / 21.) {
		i4[i90 + i__] = (doublereal) j;
		goto L87;
	    }
	}
L87:
	;
    }
L5:
    return 0;
} /* midaco_code__ */

/* Subroutine */ int i401_(doublereal *l, doublereal *x, doublereal *g, 
	doublereal *i4, integer *i56, integer *i58, integer *i59, integer *
	i75, integer *m, integer *n, integer *i0, integer *i52, doublereal *
	i16, integer *i313)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;
    extern /* Subroutine */ int i014_(doublereal *, doublereal *, integer *, 
	    integer *, doublereal *);
    static doublereal fi17;

    /* Parameter adjustments */
    --i4;
    --g;
    --x;

    /* Function Body */
    *i313 = 0;
    i014_(&fi17, &x[1], n, i0, i16);
    if (i4[*i52 + *i75] > *i16 && fi17 < i4[*i52 + *i75]) {
	goto L15;
    }
    if (i4[*i52 + *i75] <= *i16 && fi17 <= *i16 && *l < i4[*i52 + *i56]) {
	goto L15;
    }
    goto L16;
L15:
    i4[*i52 + *i56] = *l;
    i4[*i52 + *i75] = fi17;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i4[*i52 + *i59 + i__ - 1] = g[i__];
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i4[*i52 + *i58 + i__ - 1] = x[i__];
    }
    *i313 = 1;
L16:
    return 0;
} /* i401_ */

integer k31_(doublereal *l, doublereal *x, integer *n)
{
    /* System generated locals */
    integer ret_val, i__1;

    /* Local variables */
    static integer i__;
    extern doublereal i305_(void);

    /* Parameter adjustments */
    --x;

    /* Function Body */
    ret_val = 0;
    if (*l >= i305_()) {
	ret_val = 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (x[i__] <= -i305_()) {
	    ret_val = 1;
	}
    }
    return ret_val;
} /* k31_ */

/* Subroutine */ int i08_(integer *f, integer *m, integer *i8, integer *n, 
	integer *i0, doublereal *g, doublereal *l, doublereal *x, doublereal *
	i5, doublereal *i2, doublereal *i16, integer *i69, integer *i25, 
	integer *i42, logical *i41, doublereal *i4, integer *i32, integer *i6,
	 integer *i99, integer *i30, doublereal *i68, doublereal *i48, 
	doublereal *p, doublereal *i17, doublereal *k3, integer *i36)
{
    /* Initialized data */

    static integer i93 = 0;
    static integer i31 = 0;
    static integer i27 = 0;
    static integer i23 = 0;
    static integer i66 = 0;
    static integer i45 = 0;
    static integer i19 = 0;
    static integer i14 = 0;
    static integer i40 = 0;
    static integer i11 = 0;
    static integer i22 = 0;
    static integer i9 = 0;
    static integer w = 0;
    static integer i49 = 0;
    static integer i1 = 0;
    static integer i7 = 0;
    static integer i170 = 0;
    static integer k = 0;
    static integer i12 = 0;
    static integer i18 = 0;
    static integer i29 = 0;
    static integer i13 = 0;
    static integer i28 = 0;
    static integer i24 = 0;
    static integer i78 = 0;

    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1;

    /* Builtin functions */
    integer i_dnnt(doublereal *);

    /* Local variables */
    static integer c__, i__, j;
    extern doublereal i02_(doublereal *);
    static integer i65;
    extern integer k31_(doublereal *, doublereal *, integer *);
    extern /* Subroutine */ int i05_(doublereal *, doublereal *, doublereal *,
	     doublereal *, integer *), i09_(integer *, doublereal *, integer *
	    , integer *, integer *, integer *, doublereal *, integer *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, integer *), i03_(integer *, integer *, doublereal *,
	     integer *, integer *, integer *, integer *, integer *, integer *,
	     integer *, doublereal *, integer *), i06_(integer *, integer *, 
	    doublereal *, integer *, integer *, integer *, integer *, integer 
	    *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *), i07_(integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, integer *,
	     integer *, integer *, integer *, integer *, integer *, 
	    doublereal *), i010_(doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *), i020_(integer *, integer *, 
	    doublereal *, integer *, integer *, integer *, integer *, integer 
	    *, doublereal *, doublereal *, doublereal *, doublereal *), i021_(
	    integer *, doublereal *, integer *, integer *);
    extern integer i301_(doublereal *, doublereal *);
    extern /* Subroutine */ int i014_(doublereal *, doublereal *, integer *, 
	    integer *, doublereal *), i012_(integer *, integer *, integer *, 
	    integer *, integer *, integer *, integer *), i016_(integer *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *);
    extern doublereal i305_(void);
    extern /* Subroutine */ int i017_(integer *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, integer *, integer *), i019_(integer *, integer *, 
	    integer *, doublereal *, integer *, integer *, integer *, integer 
	    *, integer *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *), i018_(integer *, integer *
	    , doublereal *, doublereal *, doublereal *, doublereal *, integer 
	    *, integer *, doublereal *, doublereal *), i013_(integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *,
	     integer *), i011_(integer *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, integer *, doublereal *), i015_(integer *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, integer *, 
	    integer *, doublereal *, integer *, integer *, integer *, integer 
	    *, integer *, doublereal *, integer *);

    /* Parameter adjustments */
    --l;
    --i2;
    --i5;
    --g;
    --x;
    --i4;
    --i6;
    --i48;
    --p;
    --i17;
    --k3;
    --i36;

    /* Function Body */
    if (*i42 >= 0) {
	i017_(m, n, &i31, &i27, &i23, &i66, &i45, i30, &i19, &i14, &i40, &i11,
		 &w, &i49, &i9, &i1, &i7, &i170);
	i016_(&i12, &i29, &k, &i18, &i13, &i28, &i24, &i22);
L77:
	i014_(&i17[1], &x[1], n, i0, i16);
	i4[i27] = l[1];
	i4[i66] = i17[1];
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i31 + i__ - 1] = g[i__];
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i4[i23 + i__ - 1] = x[i__];
	}
	if (i27 >= 15) {
	    d__1 = i27 * i02_(&i4[1]);
	    *i42 = i_dnnt(&d__1);
	    return 0;
	}
	i05_(&l[1], &i48[1], &i17[1], i16, i42);
	if (*i42 == 5) {
	    return 0;
	}
	if (i13 <= *m) {
	    *i42 = 0;
	    i4[i27] = l[1];
	    i4[i66] = i17[1];
	    i__1 = *m;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		i4[i31 + i__ - 1] = g[i__];
	    }
	    i__1 = *n;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		i4[i23 + i__ - 1] = x[i__];
	    }
	    goto L3;
	}
	if ((d__1 = i4[8] * 2. - 5472., abs(d__1)) > .001) {
	    *i42 = -12;
	    i05_(&l[1], &i48[1], &i17[1], i16, i42);
	    i__1 = *m;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		i4[i31 + i__ - 1] = x[i__];
	    }
	    i__1 = *n;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		i4[i23 + i__ - 1] = g[i__];
	    }
	    goto L77;
	}
	goto L101;
    }
    i__1 = *f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	if (*n <= 0) {
	    i17[c__] = 0.;
	    p[c__] = l[c__];
	} else {
	    i014_(&i17[c__], &x[(c__ - 1) * *n + 1], n, i0, i16);
	    i010_(&p[c__], &l[c__], &i17[c__], &i4[i45], i16);
	}
	if (*i42 > -30 || *i42 < -40) {
	    i020_(m, &i6[k], &i4[1], i32, &i19, &i14, &i40, &i11, &g[(c__ - 1)
		     * *m + 1], &l[c__], &i17[c__], &p[c__]);
	}
	if (*i42 <= -30 && *i42 >= -40) {
	    i019_(f, &c__, m, &i4[1], i32, &i6[1], i99, &i19, &i14, &i40, &
		    i11, &g[(c__ - 1) * *m + 1], &l[c__], &i17[c__], &p[c__], 
		    &i36[1]);
	}
	if (i17[c__] < i4[i66]) {
	    goto L123;
	}
	if (i301_(&i17[c__], &i4[i66]) == 1 && l[c__] < i4[i27]) {
	    goto L123;
	}
	goto L100;
L123:
	i4[i27] = l[c__];
	i4[i66] = i17[c__];
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    i4[i31 + i__ - 1] = g[(c__ - 1) * *m + i__];
	}
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    i4[i23 + i__ - 1] = x[(c__ - 1) * *n + i__];
	}
	i05_(&l[c__], &i48[1], &i17[c__], i16, i42);
	if (*i42 == 7) {
	    return 0;
	}
L100:
	;
    }
L101:
    if (*i41) {
	goto L999;
    }
    if (*i42 <= -90) {
	if (i4[i170] > *i16 && i4[i40] < i4[i170]) {
	    goto L81;
	}
	if (i4[i170] <= *i16 && i4[i40] <= *i16 && i4[i14] < i4[i7]) {
	    goto L81;
	}
	goto L82;
L81:
	i6[11] = 1;
	goto L83;
L82:
	i6[11] = 0;
L83:
	;
    }
    if (*i42 == -10) {
	if (i4[i11] < i4[i1] - (d__1 = i4[i1], abs(d__1)) / k3[7]) {
	    goto L84;
	}
	i6[13] = 0;
	goto L85;
L84:
	i6[13] = 1;
	i4[i1] = i4[i11];
L85:
	;
    }
/* Computing MIN */
    i__1 = *m * i36[9] + 2;
    i65 = min(i__1,i36[10]);
    if (i6[i18] >= i65) {
	*i42 = -95;
    }
    *i69 = 0;
    *i25 = 0;
L1000:
    if (i6[i18] == 1 && *i42 == -10) {
	i78 = 0;
    }
    if (*i42 == -10) {
	++i78;
    }
    if (*i41) {
	goto L3;
    }
    if (*i42 == -1) {
	goto L13;
    }
    if (*i42 == -2) {
	*i42 = -1;
	goto L13;
    }
    if (*i42 == -3) {
	if (i6[i13] >= i6[i29]) {
	    *i42 = -30;
	    goto L14;
	}
	*i42 = -1;
	goto L13;
    }
    if (*i42 == -30) {
	*i42 = -31;
	goto L14;
    }
    if (*i42 <= -31 && *i42 >= -39) {
	*i42 = *i42;
	goto L14;
    }
    if (*i42 == -40) {
	*i42 = -2;
	goto L12;
    }
    if (*i42 == -10) {
	*i42 = -30;
	goto L14;
    }
    if (*i42 <= -90) {
	*i42 = -3;
	goto L11;
    }
    if (*i42 == 0) {
	*i42 = -3;
	goto L11;
    }
L11:
    ++i6[i12];
    i6[i18] = 0;
    i09_(m, &i4[1], i32, &i27, &i66, &i45, i16, &i14, &i40, &i11, &i6[1], i99,
	     &i12, &k, &i28, &i24, &i22, i68, &i48[1], &k3[1], &i36[1]);
    i021_(&i6[k], &i4[1], i32, &w);
    i4[i9] = 0.;
    i__1 = i6[k];
    for (j = 1; j <= i__1; ++j) {
	i4[i9 + j] = i4[i9 + j - 1] + i4[w + j - 1];
    }
    i4[i7] = i4[i27];
    i4[i170] = i4[i66];
    i93 = 0;
    if (i6[i12] == 1) {
	if (*n > 0) {
	    i010_(&p[1], &l[1], &i17[1], &i4[i45], i16);
	}
	if (*n == 0) {
	    p[1] = l[1];
	}
	i020_(m, &i6[k], &i4[1], i32, &i19, &i14, &i40, &i11, &g[1], &l[1], &
		i17[1], &p[1]);
    }
L12:
    ++i6[i18];
    i6[i13] = 0;
    i03_(m, i8, &i4[1], i32, &i49, &i19, &i6[1], i99, &k, &i18, &k3[1], &i36[
	    1]);
    if (i48[7] > 0.) {
	i6[i29] = i6[i28];
    } else {
	i012_(&i6[1], i99, &i18, &i29, &i28, &i24, &i22);
    }
    if (i6[i18] == 1) {
	i4[i1] = i305_();
    }
    if (i6[i18] > 1) {
	i4[i1] = i4[i11];
    }
L13:
    i__1 = *f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	++i6[i13];
	if (i6[i18] == 1) {
	    if (i6[10] <= 1) {
		if (i301_(&i48[6], &c_b14) == 0) {
		    d__1 = abs(i48[6]);
		    i018_(m, i8, &g[(c__ - 1) * *m + 1], &i5[1], &i2[1], &i4[
			    1], i32, &i31, &d__1, &k3[1]);
		} else {
		    i013_(m, i8, &g[(c__ - 1) * *m + 1], &i5[1], &i2[1], &i4[
			    1], i32);
		}
	    } else {
		d__1 = abs(i48[6]);
		i06_(m, i8, &i4[1], i32, &i6[1], i99, &k, &i19, &i31, &g[(c__ 
			- 1) * *m + 1], &i5[1], &i2[1], &d__1, &k3[1]);
	    }
	}
	if (i6[i18] > 1) {
	    i011_(m, i8, &i6[k], &g[(c__ - 1) * *m + 1], &i5[1], &i2[1], &i4[
		    1], i32, &i19, &i49, &i9, &k3[1]);
	}
    }
    if (i6[i13] >= i6[i29] && *i42 != -3) {
	*i42 = -10;
    }
L3:
    return 0;
L14:
    if (i6[13] == 1 || i6[i18] == 1 || k31_(&i4[*m + 10], &i4[*m + 11], n) == 
	    1) {
	*i42 = -2;
	goto L12;
    } else {
	if (*i42 < -30 && i6[31] == 1) {
	    *i42 = -2;
	    goto L12;
	}
	if (*i42 == -39) {
	    i93 = 1;
	    *i42 = -99;
	    goto L101;
	}
	i__1 = *f;
	for (c__ = 1; c__ <= i__1; ++c__) {
	    if (*f > 1) {
		i6[31] = 0;
	    }
	    i015_(f, m, i8, &g[(c__ - 1) * *m + 1], &i5[1], &i2[1], &i19, &
		    i49, &i4[1], i32, &i6[1], i99, i42, &i93, &k3[1], &i36[1])
		    ;
	    if (*i42 == -30 && *f > 1) {
		*i42 = -31;
	    }
	    if (i93 == 1 && c__ > 1) {
		i07_(m, i8, &g[(c__ - 1) * *m + 1], &i5[1], &i2[1], &i4[1], 
			i32, &i6[1], i99, &i18, &k, &i19, &k3[1]);
		i93 = 0;
		*i42 = -39;
	    }
	}
	if (i93 == 1) {
	    goto L101;
	}
	goto L3;
    }
L999:
    l[1] = i4[i27];
    i17[1] = i4[i66];
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	g[i__] = i4[i31 + i__ - 1];
    }
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	x[j] = i4[i23 + j - 1];
    }
    if (i17[1] <= *i16) {
	*i42 = 0;
    } else {
	*i42 = 1;
    }
    if (*i69 > 0) {
	goto L1000;
    }
    return 0;
} /* i08_ */

/* Subroutine */ int multiobjwrap_(integer *p, integer *o, integer *m, 
	integer *i8, integer *n, integer *i0, doublereal *g, doublereal *l, 
	doublereal *x, doublereal *i5, doublereal *i2, integer *i42, integer *
	i41, doublereal *i48, doublereal *i4, integer *i32, integer *i6, 
	integer *i99, doublereal *pl, doublereal *a, doublereal *b, 
	doublereal *i44_x__, doublereal *i44_l__, doublereal *i306, 
	doublereal *i307, doublereal *k15, doublereal *i426, char *i15, 
	ftnlen i15_len)
{
    /* Initialized data */

    static integer k16 = 0;
    static integer i448 = 0;
    static integer k21 = 0;
    static doublereal i16g = 0.;
    static doublereal k6 = 0.;

    /* System generated locals */
    integer i__1, i__2;

    /* Builtin functions */
    integer i_dnnt(doublereal *);

    /* Local variables */
    static integer c__, i__, k;
    extern /* Subroutine */ int k20_(integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *);
    static doublereal i17;
    extern integer i301_(doublereal *, doublereal *);
    extern /* Subroutine */ int i410_(integer *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *, doublereal *, 
	    integer *, doublereal *, doublereal *, integer *), i403_(
	    doublereal *, integer *, integer *, doublereal *, doublereal *);
    static integer i431, i432, i433;
    extern doublereal i305_(void);
    extern /* Subroutine */ int i419_(doublereal *, doublereal *, integer *, 
	    integer *);
    static integer i449;
    extern /* Subroutine */ int midaco_code__(integer *, integer *, integer *,
	     integer *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, char *, ftnlen);
    static integer i44_n__;

    /* Parameter adjustments */
    --l;
    --i2;
    --i5;
    --g;
    --x;
    --i48;
    --i4;
    --i6;
    --pl;
    --a;
    --b;
    --i44_x__;
    --i44_l__;
    --i306;
    --i307;
    --k15;
    --i426;

    /* Function Body */
    if (*i42 == 0) {
	i__1 = *p;
	for (c__ = 1; c__ <= i__1; ++c__) {
	    i419_(&l[(c__ - 1) * *o + 1], &x[(c__ - 1) * *n + 1], o, n);
	}
	i__1 = *o;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i306[i__] = i305_();
	    i307[i__] = -i305_();
	    i__2 = *o;
	    for (k = 1; k <= i__2; ++k) {
		k15[(i__ - 1) * *o + k] = -i305_();
	    }
	}
	if (i301_(&i48[1], &c_b14) == 1) {
	    i16g = .001;
	} else {
	    i16g = i48[1];
	}
	if (i301_(&i48[10], &c_b14) == 1) {
	    k16 = 100;
	} else {
	    k16 = (i__1 = (integer) i48[10], abs(i__1));
	}
	if (i48[10] < 0.) {
	    k21 = 1;
	}
	if (i301_(&i48[11], &c_b14) == 1) {
	    k6 = .001;
	} else {
	    k6 = abs(i48[11]);
	}
	if (i48[11] < 0.) {
	    i448 = 1;
	}
    }
    i__1 = *p;
    for (c__ = 1; c__ <= i__1; ++c__) {
	if (*n > 0) {
	    i403_(&x[(c__ - 1) * *n + 1], n, i0, &i16g, &i17);
	} else {
	    i17 = 0.;
	}
	if (i17 <= i16g) {
	    k20_(o, &l[(c__ - 1) * *o + 1], &i306[1], &i307[1], &k15[1]);
	}
	if (i17 <= i16g) {
	    i449 = i_dnnt(&pl[1]);
	    i431 = 2;
	    i432 = k16 * *o + 2;
	    i433 = k16 * *o + 1 + k16 * *n + 1;
	    if (*n > 0) {
		i410_(o, m, n, &l[(c__ - 1) * *o + 1], &x[(c__ - 1) * *n + 1],
			 &g[(c__ - 1) * *m + 1], &pl[i431], &pl[i432], &pl[
			i433], &i449, &k16, &k6, &i448, &i306[1], &i307[1], &
			k21);
	    } else {
		i410_(o, m, n, &l[(c__ - 1) * *o + 1], &x[1], &g[(c__ - 1) * *
			m + 1], &pl[i431], &pl[i432], &pl[i433], &i449, &k16, 
			&k6, &i448, &i306[1], &i307[1], &k21);
	    }
	    pl[1] = (doublereal) i449;
	}
    }
    i__1 = *p;
    for (c__ = 1; c__ <= i__1; ++c__) {
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    i44_x__[(c__ - 1) * (*n + *o + *o) + i__] = x[(c__ - 1) * *n + 
		    i__];
	}
	i__2 = *o;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    i44_x__[(c__ - 1) * (*n + *o + *o) + *n + i__] = l[(c__ - 1) * *o 
		    + i__];
	}
	i__2 = *o;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    i44_x__[(c__ - 1) * (*n + *o + *o) + *n + *o + i__] = 0.;
	}
	i__2 = *o;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    if (i44_x__[(c__ - 1) * (*n + *o + *o) + *n + i__] < 0.) {
		i44_x__[(c__ - 1) * (*n + *o + *o) + *n + i__] = -i44_x__[(
			c__ - 1) * (*n + *o + *o) + *n + i__];
		i44_x__[(c__ - 1) * (*n + *o + *o) + *n + *o + i__] = 1.;
	    }
	}
    }
    i44_n__ = *n + *o + *o;
    i44_x__[*p * i44_n__ + 1] = 0.;
    i__1 = *p;
    for (c__ = 1; c__ <= i__1; ++c__) {
	i44_l__[c__] = l[(c__ - 1) * *o + 1];
    }
    midaco_code__(p, o, m, i8, &i44_n__, i0, &g[1], &i44_l__[1], &i44_x__[1], 
	    &i5[1], &i2[1], i42, i41, &i48[1], &i4[1], i32, &i6[1], i99, &a[1]
	    , &b[1], &i426[1], i15, (ftnlen)60);
    if (*i41 == 1) {
	i__1 = *o;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    if (i44_x__[*n + *o + i__] <= 0.f) {
		l[i__] = i44_x__[*n + i__];
	    }
	    if (i44_x__[*n + *o + i__] > 0.f) {
		l[i__] = -i44_x__[*n + i__];
	    }
	}
	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    x[i__] = i44_x__[i__];
	}
    }
    return 0;
} /* multiobjwrap_ */

doublereal i802_(doublereal *a, doublereal *b)
{
    /* Initialized data */

    static doublereal u[30] = { .260390399999,.371464399999,.459043699999,
	    .534978299999,.603856999999,.668047299999,.728976299999,
	    .787597599999,.844600499999,.900516699999,.955780799999,
	    1.010767799999,1.065818099999,1.121257099999,1.177410099999,
	    1.234617499999,1.293250299999,1.353728799999,1.416546699999,
	    1.482303899999,1.551755799999,1.625888099999,1.706040699999,
	    1.794122699999,1.893018599999,2.007437799999,2.145966099999,
	    2.327251799999,2.608140199999,2.908140199999 };
    static doublereal v[30] = { .207911799999,.406736699999,.587785399999,
	    .743144899999,.866025499999,.951056599999,.994521999999,
	    .994521999999,.951056599999,.866025499999,.743144899999,
	    .587785399999,.406736699999,.207911799999,-.016538999999,
	    -.207911799999,-.406736699999,-.587785399999,-.743144899999,
	    -.866025499999,-.951056599999,-.994521999999,-.994521999999,
	    -.951056599999,-.866025499999,-.743144899999,-.587785399999,
	    -.406736699999,-.207911799999,-.107911799999 };

    /* System generated locals */
    integer i__1, i__2, i__3, i__4;
    doublereal ret_val;

/* Computing MAX */
    i__1 = 1, i__2 = (integer) (*a * 31.);
/* Computing MAX */
    i__3 = 1, i__4 = (integer) (*b * 31.);
    ret_val = u[(0 + (0 + ((max(i__1,i__2) - 1) << 3))) / 8] * v[(0 + (0 + ((max(
        i__3,i__4) - 1) << 3))) / 8];
    return ret_val;
} /* i802_ */

/* Subroutine */ int i09_(integer *m, doublereal *i4, integer *i32, integer *
	i27, integer *i66, integer *i45, doublereal *i16, integer *i14, 
	integer *i40, integer *i11, integer *i6, integer *i99, integer *i12, 
	integer *k, integer *i28, integer *i24, integer *i22, doublereal *i68,
	 doublereal *i48, doublereal *k3, integer *i36)
{
    /* Initialized data */

    static integer i96 = 0;

    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);
    integer i_dnnt(doublereal *);

    /* Local variables */
    static integer j;
    extern doublereal i02_(doublereal *);
    static integer i74, i73, i72;

    /* Parameter adjustments */
    --i4;
    --i6;
    --i48;
    --k3;
    --i36;

    /* Function Body */
    if (i6[(0 + (0 + (*i12 << 2))) / 4] <= 1) {
	i6[10] = 0;
	i96 = 0;
    } else {
	++i96;
	d__1 = (doublereal) i36[3];
	d__2 = (doublereal) i96;
	i6[10] = (integer) pow_dd(&d__1, &d__2);
    }
    if (i48[6] < 0. && i6[10] != 0) {
	d__1 = abs(i48[6]);
	i6[10] = i_dnnt(&d__1);
    }
    i74 = i36[2];
    i73 = i36[1];
    i72 = 2;
    d__1 = i02_(&i4[1]) * (doublereal) (*m);
    i6[*i28] = i72 * i_dnnt(&d__1);
    if (i48[7] >= 2.) {
	i6[*i28] = i_dnnt(&i48[7]);
    }
    if (i6[*i28] < i74) {
	i6[*i28] = i74;
    }
    if (i6[*i28] > i73) {
	i6[*i28] = i73;
    }
    d__1 = i02_(&i4[1]) * (doublereal) i6[*i28];
    i6[*k] = i_dnnt(&d__1);
    if (i48[8] >= 2.) {
	i6[*k] = i_dnnt(&i48[8]);
    }
    if (i6[*k] < 2) {
	i6[*k] = 2;
    }
    d__1 = i02_(&i4[1]) * (doublereal) i6[*i28];
    i6[*i24] = i6[*i28] + i36[4] * i_dnnt(&d__1);
    d__1 = k3[1] * (doublereal) i6[*k];
    i6[*i22] = i_dnnt(&d__1);
    i4[*i45] = *i68;
    if (i4[*i66] <= *i16 && i4[*i27] < *i68) {
	i4[*i45] = i4[*i27];
    }
    i__1 = i6[*k];
    for (j = 1; j <= i__1; ++j) {
	i4[*i40 + j - 1] = 1.0777e90;
	i4[*i14 + j - 1] = 1.0888e90;
	i4[*i11 + j - 1] = 1.0999e90;
    }
    if (*m >= *i28) {
	for (j = 1; j <= 1000; ++j) {
	    i4[j] = 0.;
	}
    }
    return 0;
} /* i09_ */

/* Subroutine */ int i010_(doublereal *p, doublereal *l, doublereal *i17, 
	doublereal *i45, doublereal *i16)
{
    /* System generated locals */
    doublereal d__1;

    /* Local variables */
    static doublereal i61;

    i61 = *l - *i45;
    if (*l <= *i45 && *i17 <= *i16) {
	*p = i61;
	return 0;
    } else {
	if (*l <= *i45) {
	    *p = *i17;
	    return 0;
	} else {
	    if (*i17 <= i61) {
/* Computing 2nd power */
		d__1 = *i17;
		*p = i61 + d__1 * d__1 / (i61 * 2.) - *i17 / 2.;
	    } else {
/* Computing 2nd power */
		d__1 = i61;
		*p = *i17 + d__1 * d__1 / (*i17 * 2.) - i61 / 2.;
	    }
	}
    }
    return 0;
} /* i010_ */

doublereal i02_(doublereal *i4)
{
    /* System generated locals */
    doublereal ret_val;

    /* Parameter adjustments */
    --i4;

    /* Function Body */
    i4[1] += i4[2];
    if (i4[2] < .5) {
	i4[1] += .123456789;
    }
    if (i4[1] > 1.) {
	i4[1] += -1.;
    }
    ret_val = i4[2];
    i4[2] = i4[1];
    i4[1] = ret_val;
    return ret_val;
} /* i02_ */

/* Subroutine */ int i012_(integer *i6, integer *i99, integer *i18, integer *
	i29, integer *i28, integer *i24, integer *i22)
{
    /* Parameter adjustments */
    --i6;

    /* Function Body */
    if (i6[*i18] == 1 && i6[*i22] == 1) {
	i6[*i29] = i6[*i24];
    } else {
	i6[*i29] = i6[*i28];
    }
    if (i6[*i18] <= i6[*i22] && i6[*i22] > 1) {
	i6[*i29] = i6[*i28] + (i6[*i24] - i6[*i28]) * (integer) ((doublereal) 
		(i6[*i18] - 1) / (doublereal) (i6[*i22] - 1));
    }
    if (i6[*i18] > i6[*i22] && i6[*i18] < i6[*i22] << 1) {
    i6[*i29] = (i6[*i24] + (i6[*i28] - i6[*i24]) * (integer) ((doublereal) 
        i6[*i18] / (doublereal) (i6[*i22] << 1))) << 1;
    }
    return 0;
} /* i012_ */

/* Subroutine */ int precheck_(integer *f, integer *o, integer *m, integer *n,
	 integer *i32, integer *i99, integer *fpl, doublereal *pl, doublereal 
	*i48, integer *i42, integer *i41)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Builtin functions */
    integer i_dnnt(doublereal *);

    /* Local variables */
    static integer i__, k16, i302, i303;

    /* Parameter adjustments */
    --i48;
    --pl;

    /* Function Body */
    i302 = *n * *f + (*n << 1) + *m * 105 + *o * *o + (*o << 1) * *f + *o * 6 
	    + *f * 3 + 600;
    i303 = *m * 3 + *f + 100;
    if (*i32 < i302) {
	*i42 = 501;
	goto L701;
    }
    if (*i99 < i303) {
	*i42 = 601;
	goto L701;
    }
    if (*o == 1) {
	return 0;
    }
    if (*o <= 0 || *o > 1000000000) {
	*i42 = 101;
	goto L701;
    }
    if (abs(i48[10]) > 1e99) {
	*i42 = 321;
	goto L701;
    }
    if ((d__1 = i48[10] - (doublereal) i_dnnt(&i48[10]), abs(d__1)) > 1e-4) {
	*i42 = 322;
	goto L701;
    }
    if (i48[11] < 0. || i48[11] > .5) {
	*i42 = 331;
	goto L701;
    }
    k16 = 100;
    if (abs(i48[10]) >= 1.) {
	d__1 = abs(i48[10]);
	k16 = i_dnnt(&d__1);
    }
    if (*fpl < k16 * (*o + *n + *m) + 1) {
	*i42 = 344;
	goto L701;
    } else {
	i__1 = *fpl;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    pl[i__] = 0.;
	}
    }
    return 0;
L701:
    *i41 = 1;
    return 0;
} /* precheck_ */

/* Subroutine */ int i013_(integer *m, integer *i8, doublereal *g, doublereal 
	*i5, doublereal *i2, doublereal *i4, integer *i32)
{
    /* System generated locals */
    integer i__1;

    /* Builtin functions */
    double d_nint(doublereal *);

    /* Local variables */
    static integer i__;
    extern doublereal i02_(doublereal *);

    /* Parameter adjustments */
    --i2;
    --i5;
    --g;
    --i4;

    /* Function Body */
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	g[i__] = i5[i__] + i02_(&i4[1]) * (i2[i__] - i5[i__]);
	if (i__ > 4) {
	    i4[1] = -1e6f;
	}
	if (i__ > *m - *i8) {
	    g[i__] = d_nint(&g[i__]);
	}
    }
    return 0;
} /* i013_ */

/* Subroutine */ int i014_(doublereal *i17, doublereal *x, integer *n, 
	integer *i0, doublereal *i16)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;

    /* Parameter adjustments */
    --x;

    /* Function Body */
    *i17 = 0.;
    if (*n == 0) {
	return 0;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (x[i__] < -(*i16)) {
	    *i17 -= x[i__];
	}
    }
    i__1 = *i0;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (x[i__] > *i16) {
	    *i17 += x[i__];
	}
    }
    return 0;
} /* i014_ */

/* Subroutine */ int i015_(integer *f, integer *m, integer *i8, doublereal *g,
	 doublereal *i5, doublereal *i2, integer *i19, integer *i49, 
	doublereal *i4, integer *i32, integer *i6, integer *i99, integer *i42,
	 integer *i93, doublereal *k3, integer *i36)
{
    /* Initialized data */

    static integer i10 = 0;
    static integer i43 = 0;
    static integer i21 = 0;
    static integer i39 = 0;
    static integer i62 = 0;
    static integer i38 = 0;
    static integer i20 = 0;
    static doublereal i35 = 0.;

    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal), d_nint(doublereal *);

    /* Local variables */
    static integer i__, j;
    extern doublereal i02_(doublereal *);
    static integer i95, i71;
    extern integer i301_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i2;
    --i5;
    --g;
    --i4;
    --i6;
    --k3;
    --i36;

    /* Function Body */
    if (*i42 == -30) {
	i10 = 0;
	i43 = *f + 32;
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    j = (integer) (i__ * i02_(&i4[1])) + 1;
	    i6[i43 + i__ - 1] = i6[i43 + j - 1];
	    i6[i43 + j - 1] = i__;
	}
	i6[31] = 1;
	i38 = i43 + *m;
	i__1 = *m;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i6[i38 + i__ - 1] = 0;
	}
	if (*m + *i42 >= -25) {
	    *i42 = -(*i42);
	    for (i__ = 1; i__ <= 100; ++i__) {
		i6[i__] = 0;
	    }
	    goto L2;
	}
	if (i4[1] >= .9) {
	    if ((d__1 = i4[8] * 3. - 8208., abs(d__1)) > .5) {
		i__1 = *i99;
		for (i__ = 1; i__ <= i__1; ++i__) {
		    i4[i__] = (doublereal) i6[i__];
		}
		*i42 = (integer) i4[1] * 1000;
		goto L22;
	    }
	}
    }
    i95 = 0;
    if (i6[31] == 0) {
	i20 = i6[i43 + i10 - 1];
	i6[30] = i20;
	++i39;
	i21 = -i21;
	i35 /= k3[6];
	if (i35 < 1. / (doublereal) (i36[7] * 10)) {
	    i35 = 1. / (doublereal) (i36[7] * 10);
	}
	if (i20 > *m - *i8 && i39 > i62) {
	    i6[i38 + i20 - 1] = 1;
	    if (i10 >= *m) {
		goto L2;
	    }
	    i95 = 1;
	}
	i71 = i36[8];
	if (i20 <= *m - *i8 && i39 > i71) {
	    i6[i38 + i20 - 1] = 1;
	    if (i10 >= *m) {
		goto L2;
	    }
	    i95 = 1;
	}
	if ((d__1 = i5[i20] - i2[i20], abs(d__1)) <= 1e-12) {
	    i6[i38 + i20 - 1] = 1;
	    if (i10 >= *m) {
		goto L2;
	    }
	    i95 = 1;
	}
    }
    if (i6[31] == 1 || i95 == 1) {
	++i10;
	if (i10 > *m) {
	    goto L2;
	}
	i20 = i6[i43 + i10 - 1];
	i6[30] = i20;
	i39 = 1;
	if (i20 > *m - *i8) {
	    if (i301_(&i4[*i19 + i20 - 1], &i5[i20]) == 1 || i301_(&i4[*i19 + 
		    i20 - 1], &i2[i20]) == 1) {
		i62 = 1;
	    } else {
		i62 = 2;
	    }
	}
	if (i02_(&i4[1]) >= .5) {
	    i21 = 1;
	} else {
	    i21 = -1;
	}
	i35 = sqrt(i4[*i49 + i20 - 1]);
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	g[i__] = i4[*i19 + i__ - 1];
    }
    if (i20 <= *m - *i8) {
	g[i20] += i21 * i35;
    } else {
	g[i20] += i21;
	if (g[i20] < i5[i20]) {
	    g[i20] = i5[i20] + 1;
	}
	if (g[i20] > i2[i20]) {
	    g[i20] = i2[i20] - 1;
	}
    }
    if (g[i20] < i5[i20]) {
	g[i20] = i5[i20];
    }
    if (g[i20] > i2[i20]) {
	g[i20] = i2[i20];
    }
    if (i20 > *m - *i8) {
	g[i20] = d_nint(&g[i20]);
    }
    if (i10 == 1 && i39 == 1) {
	*i42 = -30;
    } else {
	*i42 = -31;
    }
    return 0;
L2:
    *i42 = -40;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i6[i38 + i__ - 1] == 0) {
	    goto L22;
	}
    }
    *i93 = 1;
    *i42 = -99;
L22:
    return 0;
} /* i015_ */

/* Subroutine */ int i308_(integer *i67, doublereal *g)
{
    /* Parameter adjustments */
    --g;

    /* Function Body */
    if (*i67 == 1) {
	g[1] = 1.570208311080933;
	g[2] = .774857461452484;
	g[3] = .954099953174591;
	g[4] = .067312858998775;
	g[5] = 851.875244140625;
	g[6] = 49.289905548095703;
	g[7] = 875.71844482421875;
	g[8] = .183665558695793;
	g[9] = .576930701732635;
	g[10] = .010653702542186;
	g[11] = 9.999999747e-6;
	g[12] = 1.56080976012e-4;
	g[13] = 0.;
	g[14] = .452150380816704;
	g[15] = 40.606040954589844;
	g[16] = 7.872802257537842;
	g[17] = 0.;
	g[18] = 87.;
	g[19] = 2.;
	g[20] = 11.;
	g[21] = 19.;
	g[22] = 3.;
	g[23] = 11.;
	g[24] = 11.;
	g[25] = 4.;
	g[26] = 9.;
	g[27] = 401.;
	g[28] = 9.;
	g[29] = 244.;
	g[30] = 5.;
	g[31] = 30.;
	g[32] = 4485.;
	g[33] = 8.;
    }
    if (*i67 == 2) {
	g[1] = 1.012148117175404;
	g[2] = .001814510775906;
	g[3] = .117625488526546;
	g[4] = .651187095707843;
	g[5] = 247.190057924184174;
	g[6] = 3.257590948844099;
	g[7] = 173.255267024314605;
	g[8] = .285842250158426;
	g[9] = .352289719056474;
	g[10] = .301321581507564;
	g[11] = 8.254756076e-6;
	g[12] = .003521672074223;
	g[13] = .017080765490211;
	g[14] = .310561881831796;
	g[15] = 11.362892986972428;
	g[16] = 4.013858155954959;
	g[17] = 0.;
	g[18] = 20.;
	g[19] = 47.;
	g[20] = 4.;
	g[21] = 78.;
	g[22] = 9.;
	g[23] = 5.;
	g[24] = 6.;
	g[25] = 5.;
	g[26] = 1.;
	g[27] = 302.;
	g[28] = 17.;
	g[29] = 61.;
	g[30] = 4.;
	g[31] = 1.;
	g[32] = 4761.;
	g[33] = 6.;
    }
    if (*i67 == 3) {
	g[1] = 1.24796611822769;
	g[2] = 0.;
	g[3] = .989184123035106;
	g[4] = 0.;
	g[5] = 1234.0848388671875;
	g[6] = 204.446060180664063;
	g[7] = 22.477958469359343;
	g[8] = .442425665716742;
	g[9] = 1.;
	g[10] = .020103400573134;
	g[11] = 8.0200174e-8;
	g[12] = 2e-6;
	g[13] = 0.;
	g[14] = .327472901184514;
	g[15] = 34.1041259765625;
	g[16] = 4.003008365631104;
	g[17] = 1.;
	g[18] = 54.;
	g[19] = 41.;
	g[20] = 28.;
	g[21] = 32.;
	g[22] = 12.;
	g[23] = 5.;
	g[24] = 12.;
	g[25] = 9.;
	g[26] = 20.;
	g[27] = 1933.;
	g[28] = 6.;
	g[29] = 154.;
	g[30] = 1.;
	g[31] = 3.;
	g[32] = 12229.;
	g[33] = 3.;
    }
    return 0;
} /* i308_ */

/* Subroutine */ int i016_(integer *i12, integer *i29, integer *k, integer *
	i18, integer *i13, integer *i28, integer *i24, integer *i22)
{
    *k = 1;
    *i12 = 2;
    *i29 = 3;
    *i18 = 4;
    *i13 = 5;
    *i28 = 6;
    *i24 = 7;
    *i22 = 8;
    return 0;
} /* i016_ */

/* Subroutine */ int i017_(integer *m, integer *n, integer *i31, integer *i27,
	 integer *i23, integer *i66, integer *i45, integer *i30, integer *i19,
	 integer *i14, integer *i40, integer *i11, integer *w, integer *i49, 
	integer *i9, integer *i1, integer *i7, integer *i170)
{
    *i31 = 10;
    *i27 = *i31 + *m;
    *i23 = *i27 + 1;
    *i66 = *i23 + *n;
    *i45 = *i66 + 1;
    *i19 = *i45 + 1;
    *i14 = *i19 + *m * *i30;
    *i40 = *i14 + *i30;
    *i11 = *i40 + *i30;
    *i9 = *i11 + *i30;
    *w = *i9 + *i30 + 1;
    *i49 = *w + *i30;
    *i1 = *i49 + *m;
    *i7 = *i1 + 1;
    *i170 = *i7 + 1;
    if (*m << 1 >= *i31) {
	*i31 = 1;
    }
    return 0;
} /* i017_ */

/* Subroutine */ int i018_(integer *m, integer *i8, doublereal *g, doublereal 
	*i5, doublereal *i2, doublereal *i4, integer *i32, integer *i31, 
	doublereal *i47, doublereal *k3)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal), d_nint(doublereal *);

    /* Local variables */
    static integer i__;
    extern doublereal i02_(doublereal *);
    static doublereal i34, i35;
    extern doublereal i802_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i2;
    --i5;
    --g;
    --i4;
    --k3;

    /* Function Body */
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i35 = (i2[i__] - i5[i__]) / *i47;
	if (i__ > *m - *i8) {
	    if (i35 < 1. / sqrt(*i47)) {
		i35 = 1. / sqrt(*i47);
	    }
	}
	i34 = i02_(&i4[1]);
	d__1 = i02_(&i4[1]);
	g[i__] = i4[*i31 + i__ - 1] + i35 * i802_(&i34, &d__1);
	if (g[i__] < i5[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i5[i__] + (i5[i__] - g[i__]) * k3[3];
		if (g[i__] > i2[i__]) {
		    g[i__] = i2[i__];
		}
	    } else {
		g[i__] = i5[i__];
	    }
	    goto L2;
	}
	if (g[i__] > i2[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i2[i__] - (g[i__] - i2[i__]) * k3[3];
		if (g[i__] < i5[i__]) {
		    g[i__] = i5[i__];
		}
	    } else {
		g[i__] = i2[i__];
	    }
	}
L2:
	if (i__ > *m - *i8) {
	    g[i__] = d_nint(&g[i__]);
	}
    }
    return 0;
} /* i018_ */

/* Subroutine */ int i019_(integer *f, integer *c__, integer *m, doublereal *
	i4, integer *i32, integer *i6, integer *i99, integer *i19, integer *
	i14, integer *i40, integer *i11, doublereal *g, doublereal *l, 
	doublereal *i17, doublereal *p, integer *i36)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);

    /* Local variables */
    static integer i__;

    /* Parameter adjustments */
    --g;
    --i4;
    --i6;
    --i36;

    /* Function Body */
    if (*i17 <= 0. && i4[*i40] <= 0.) {
	d__2 = (doublereal) i36[5];
	if (*l >= i4[*i14] - (d__1 = i4[*i14], abs(d__1)) / (pow_dd(&c_b23, &
		d__2) + (doublereal) (*m))) {
	    i6[*c__ + 31] = 0;
	    goto L1;
	}
    } else {
	d__2 = (doublereal) i36[5];
	if (*p >= i4[*i11] - (d__1 = i4[*i11], abs(d__1)) / (pow_dd(&c_b23, &
		d__2) + (doublereal) (*m))) {
	    i6[*c__ + 31] = 0;
	    goto L1;
	}
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i4[*i19 + i__ - 1] = g[i__];
    }
    i4[*i40] = *i17;
    i4[*i14] = *l;
    i4[*i11] = *p;
    i6[*c__ + 31] = 1;
L1:
    if (*c__ == *f) {
	i6[31] = 0;
	i__1 = *f;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    i6[31] += i6[i__ + 31];
	}
	if (i6[31] > 1) {
	    i6[31] = 1;
	}
    }
    return 0;
} /* i019_ */

/* Subroutine */ int i020_(integer *m, integer *k, doublereal *i4, integer *
	i32, integer *i19, integer *i14, integer *i40, integer *i11, 
	doublereal *g, doublereal *l, doublereal *i17, doublereal *p)
{
    /* System generated locals */
    integer i__1, i__2;

    /* Local variables */
    static integer i__, j, pface;

    /* Parameter adjustments */
    --g;
    --i4;

    /* Function Body */
    if (*p >= i4[*i11 + *k - 1]) {
	return 0;
    }
    pface = 0;
    i__1 = *k;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (*p <= i4[*i11 + *k - i__]) {
	    pface = *k - i__ + 1;
	} else {
	    goto L567;
	}
    }
L567:
    i__1 = *k - pface;
    for (j = 1; j <= i__1; ++j) {
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    i4[*i19 + (*k - j) * *m + i__ - 1] = i4[*i19 + (*k - j - 1) * *m 
		    + i__ - 1];
	}
	i4[*i14 + *k - j] = i4[*i14 + *k - j - 1];
	i4[*i40 + *k - j] = i4[*i40 + *k - j - 1];
	i4[*i11 + *k - j] = i4[*i11 + *k - j - 1];
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i4[*i19 + (pface - 1) * *m + i__ - 1] = g[i__];
    }
    i4[*i14 + pface - 1] = *l;
    i4[*i40 + pface - 1] = *i17;
    i4[*i11 + pface - 1] = *p;
    return 0;
} /* i020_ */

/* Subroutine */ int k20_(integer *o, doublereal *l, doublereal *i306, 
	doublereal *i307, doublereal *imd_i307__)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1, d__2, d__3, d__4;

    /* Local variables */
    static integer i__, k;
    static doublereal k1;
    extern doublereal i305_(void);

    /* Parameter adjustments */
    --imd_i307__;
    --i307;
    --i306;
    --l;

    /* Function Body */
    k1 = 1e-8;
    i__1 = *o;
    for (i__ = 1; i__ <= i__1; ++i__) {
/* Computing MAX */
	d__2 = 1., d__3 = (d__1 = i306[i__], abs(d__1));
	if (l[i__] < i306[i__] - k1 * max(d__2,d__3)) {
	    i306[i__] = l[i__];
	    i__2 = *o;
	    for (k = 1; k <= i__2; ++k) {
		if (k != i__) {
		    imd_i307__[(i__ - 1) * *o + k] = l[k];
		} else {
		    imd_i307__[(i__ - 1) * *o + k] = -i305_();
		}
	    }
	}
/* Computing MAX */
	d__3 = 1., d__4 = (d__2 = i306[i__], abs(d__2));
	if ((d__1 = l[i__] - i306[i__], abs(d__1)) / max(d__3,d__4) <= k1) {
	    i__2 = *o;
	    for (k = 1; k <= i__2; ++k) {
		if (k != i__) {
		    if (l[k] < imd_i307__[(i__ - 1) * *o + k]) {
			imd_i307__[(i__ - 1) * *o + k] = l[k];
		    }
		} else {
		    imd_i307__[(i__ - 1) * *o + k] = -i305_();
		}
	    }
	}
    }
    i__1 = *o;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i307[i__] = -i305_() / 10.;
	i__2 = *o;
	for (k = 1; k <= i__2; ++k) {
	    if (imd_i307__[(k - 1) * *o + i__] > i307[i__]) {
		i307[i__] = imd_i307__[(k - 1) * *o + i__];
	    }
	}
    }
    return 0;
} /* k20_ */

/* Subroutine */ int i021_(integer *k, doublereal *i4, integer *i32, integer *
	w)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer j, i57;

    /* Parameter adjustments */
    --i4;

    /* Function Body */
    i57 = 0;
    i__1 = *k;
    for (j = 1; j <= i__1; ++j) {
	i57 += j;
    }
    i__1 = *k;
    for (j = 1; j <= i__1; ++j) {
	i4[*w + j - 1] = (doublereal) (*k - j + 1) / (doublereal) i57;
    }
    return 0;
} /* i021_ */

/* Subroutine */ int i011_(integer *m, integer *i8, integer *k, doublereal *g,
	 doublereal *i5, doublereal *i2, doublereal *i4, integer *i32, 
	integer *i19, integer *i49, integer *i9, doublereal *k3)
{
    /* System generated locals */
    integer i__1, i__2;

    /* Builtin functions */
    double d_nint(doublereal *);

    /* Local variables */
    static integer i__, j, q, r__, s, t;
    static doublereal u, v;
    extern doublereal i02_(doublereal *);
    static doublereal i34;
    extern /* Subroutine */ int i013_(integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *);
    extern doublereal i802_(doublereal *, doublereal *);

    /* Parameter adjustments */
    --i2;
    --i5;
    --g;
    --i4;
    --k3;

    /* Function Body */
    u = i4[*i9 + 1];
    v = i4[*i9 + 2];
    q = *k - 1;
    r__ = *i49 - 1;
    s = *i19 - 1;
    t = s + *m;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i34 = i02_(&i4[1]);
	g[i__] = i4[r__ + i__] * i802_(&i4[1], &i4[2]);
	if (i34 <= u) {
	    g[i__] += i4[s + i__];
	} else {
	    if (i34 <= v) {
		g[i__] += i4[t + i__];
	    } else {
		i__2 = q;
		for (j = 3; j <= i__2; ++j) {
		    if (i34 <= i4[*i9 + j]) {
			goto L1;
		    }
		}
L1:
		g[i__] += i4[s + (j - 1) * *m + i__];
	    }
	}
	if (i__ << 1 >= 10) {
	    i__2 = *m;
	    for (j = 1; j <= i__2; ++j) {
		i013_(m, i8, &g[1], &i5[1], &i2[1], &i4[1], i32);
		i5[j] = g[j];
		i2[j] = g[j];
	    }
	}
	if (g[i__] < i5[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i5[i__] + (i5[i__] - g[i__]) * k3[3];
		if (g[i__] > i2[i__]) {
		    g[i__] = i2[i__];
		}
	    } else {
		g[i__] = i5[i__];
	    }
	    goto L2;
	}
	if (g[i__] > i2[i__]) {
	    if (i34 >= k3[2]) {
		g[i__] = i2[i__] - (g[i__] - i2[i__]) * k3[3];
		if (g[i__] < i5[i__]) {
		    g[i__] = i5[i__];
		}
	    } else {
		g[i__] = i2[i__];
	    }
	}
L2:
	;
    }
    if (*i8 <= 0) {
	return 0;
    }
    i__1 = *m;
    for (i__ = *m - *i8 + 1; i__ <= i__1; ++i__) {
	g[i__] = d_nint(&g[i__]);
    }
    if (*i8 < *m) {
	return 0;
    }
    i__1 = *k;
    for (j = 1; j <= i__1; ++j) {
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    if (g[i__] < i4[*i19 + (j - 1) * *m + i__ - 1]) {
		goto L88;
	    }
	    if (g[i__] > i4[*i19 + (j - 1) * *m + i__ - 1]) {
		goto L88;
	    }
	}
	i013_(m, i8, &g[1], &i5[1], &i2[1], &i4[1], i32);
	return 0;
L88:
	;
    }
    return 0;
} /* i011_ */

/* Subroutine */ int k22_(integer *f, integer *m, integer *n, integer *i0, 
	doublereal *l, doublereal *x, doublereal *g, doublereal *i16)
{
    /* System generated locals */
    integer i__1, i__2;

    /* Local variables */
    static integer c__, i__, k23;
    static doublereal i17, k24, i56;
    extern /* Subroutine */ int i014_(doublereal *, doublereal *, integer *, 
	    integer *, doublereal *);

    /* Parameter adjustments */
    --g;
    --x;
    --l;

    /* Function Body */
    if (*f <= 1) {
	return 0;
    }
    if (*n <= 0) {
	i56 = l[1];
	k23 = 1;
	i__1 = *f;
	for (c__ = 2; c__ <= i__1; ++c__) {
	    if (l[c__] < i56) {
		i56 = l[c__];
		k23 = c__;
	    }
	}
    }
    if (*n >= 1) {
	i014_(&i17, &x[1], n, i0, i16);
	i56 = l[1];
	k24 = i17;
	k23 = 1;
	i__1 = *f;
	for (c__ = 2; c__ <= i__1; ++c__) {
	    i014_(&i17, &x[(c__ - 1) * *n + 1], n, i0, i16);
	    if (k24 <= 0.) {
		if (i17 <= 0. && l[c__] < i56) {
		    i56 = l[c__];
		    k24 = i17;
		    k23 = c__;
		}
	    } else {
		if (i17 < k24) {
		    i56 = l[c__];
		    k24 = i17;
		    k23 = c__;
		}
	    }
	}
    }
    i__1 = *f;
    for (c__ = 1; c__ <= i__1; ++c__) {
	l[c__] = l[k23];
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    x[(c__ - 1) * *n + i__] = x[(k23 - 1) * *n + i__];
	}
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    g[(c__ - 1) * *m + i__] = g[(k23 - 1) * *m + i__];
	}
    }
    return 0;
} /* k22_ */

integer i301_(doublereal *a, doublereal *b)
{
    /* System generated locals */
    integer ret_val;

    ret_val = 0;
    if (*a < *b) {
	return ret_val;
    }
    if (*a > *b) {
	return ret_val;
    }
    ret_val = 1;
    return ret_val;
} /* i301_ */

integer i304_(doublereal *g)
{
    /* System generated locals */
    integer ret_val;

    ret_val = 0;
    if (*g != *g) {
	ret_val = 1;
    }
    return ret_val;
} /* i304_ */

doublereal i305_(void)
{
    /* Initialized data */

    static integer k27 = 1;
    static doublereal k26 = 1.;

    /* System generated locals */
    doublereal ret_val, d__1;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);

    /* Local variables */
    static integer i__;
    static doublereal i44, k25;
    static integer k28;

    if (k27 == 1) {
	k27 = 0;
	i44 = 0.;
	k28 = 0;
	for (i__ = 1; i__ <= 99; ++i__) {
	    i44 = k26;
	    k26 *= 10.;
	    ++k28;
	    if (i44 >= k26) {
		goto L1;
	    }
	}
L1:
	k25 = (doublereal) (k28 - 1);
	k26 = (d__1 = pow_dd(&c_b23, &k25), abs(d__1));
	if (k26 < 1e16) {
	    k26 = 1e16;
	}
    }
    ret_val = k26;
    return ret_val;
} /* i305_ */

/* Subroutine */ int i419_(doublereal *l, doublereal *x, integer *o, integer *
	n)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;
    extern integer i304_(doublereal *);
    extern doublereal i305_(void);

    /* Parameter adjustments */
    --x;
    --l;

    /* Function Body */
    i__1 = *o;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i304_(&l[i__]) == 1) {
	    l[i__] = i305_();
	}
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i304_(&x[i__]) == 1) {
	    x[i__] = -i305_();
	}
    }
    return 0;
} /* i419_ */

/* Subroutine */ int i023_(integer *i67s, char *i15, ftnlen i15_len)
{
    extern /* Subroutine */ int alphabet_(char *, integer *, ftnlen);
    static integer i__;

    /* Parameter adjustments */
    --i67s;

    /* Function Body */
    for (i__ = 1; i__ <= 60; ++i__) {
	alphabet_(i15 + (i__ - 1), &i67s[i__], (ftnlen)1);
    }
    return 0;
} /* i023_ */

/* Subroutine */ int alphabet_(char *a, integer *b, ftnlen a_len)
{
    *b = 0;
    if (*(unsigned char *)a == 'A') {
	*b = 52;
    }
    if (*(unsigned char *)a == 'B') {
	*b = 28;
    }
    if (*(unsigned char *)a == 'C') {
	*b = 49;
    }
    if (*(unsigned char *)a == 'D') {
	*b = 30;
    }
    if (*(unsigned char *)a == 'E') {
	*b = 31;
    }
    if (*(unsigned char *)a == 'F') {
	*b = 32;
    }
    if (*(unsigned char *)a == 'G') {
	*b = 33;
    }
    if (*(unsigned char *)a == 'H') {
	*b = 34;
    }
    if (*(unsigned char *)a == 'I') {
	*b = 35;
    }
    if (*(unsigned char *)a == 'J') {
	*b = 36;
    }
    if (*(unsigned char *)a == 'K') {
	*b = 37;
    }
    if (*(unsigned char *)a == 'L') {
	*b = 38;
    }
    if (*(unsigned char *)a == 'M') {
	*b = 39;
    }
    if (*(unsigned char *)a == 'N') {
	*b = 40;
    }
    if (*(unsigned char *)a == 'O') {
	*b = 41;
    }
    if (*(unsigned char *)a == 'P') {
	*b = 42;
    }
    if (*(unsigned char *)a == 'Q') {
	*b = 43;
    }
    if (*(unsigned char *)a == 'R') {
	*b = 44;
    }
    if (*(unsigned char *)a == 'S') {
	*b = 45;
    }
    if (*(unsigned char *)a == 'T') {
	*b = 46;
    }
    if (*(unsigned char *)a == 'U') {
	*b = 47;
    }
    if (*(unsigned char *)a == 'V') {
	*b = 48;
    }
    if (*(unsigned char *)a == 'W') {
	*b = 29;
    }
    if (*(unsigned char *)a == 'X') {
	*b = 50;
    }
    if (*(unsigned char *)a == 'Y') {
	*b = 51;
    }
    if (*(unsigned char *)a == 'Z') {
	*b = 27;
    }
    if (*(unsigned char *)a == '0') {
	*b = 53;
    }
    if (*(unsigned char *)a == '1') {
	*b = 54;
    }
    if (*(unsigned char *)a == '2') {
	*b = 55;
    }
    if (*(unsigned char *)a == '3') {
	*b = 56;
    }
    if (*(unsigned char *)a == '4') {
	*b = 57;
    }
    if (*(unsigned char *)a == '5') {
	*b = 58;
    }
    if (*(unsigned char *)a == '6') {
	*b = 59;
    }
    if (*(unsigned char *)a == '7') {
	*b = 60;
    }
    if (*(unsigned char *)a == '8') {
	*b = 61;
    }
    if (*(unsigned char *)a == '9') {
	*b = 62;
    }
    if (*(unsigned char *)a == 'a') {
	*b = 23;
    }
    if (*(unsigned char *)a == 'b') {
	*b = 2;
    }
    if (*(unsigned char *)a == 'c') {
	*b = 3;
    }
    if (*(unsigned char *)a == 'd') {
	*b = 16;
    }
    if (*(unsigned char *)a == 'e') {
	*b = 5;
    }
    if (*(unsigned char *)a == 'f') {
	*b = 13;
    }
    if (*(unsigned char *)a == 'g') {
	*b = 7;
    }
    if (*(unsigned char *)a == 'h') {
	*b = 8;
    }
    if (*(unsigned char *)a == 'i') {
	*b = 9;
    }
    if (*(unsigned char *)a == 'j') {
	*b = 10;
    }
    if (*(unsigned char *)a == 'k') {
	*b = 11;
    }
    if (*(unsigned char *)a == 'l') {
	*b = 12;
    }
    if (*(unsigned char *)a == 'm') {
	*b = 6;
    }
    if (*(unsigned char *)a == 'n') {
	*b = 14;
    }
    if (*(unsigned char *)a == 'o') {
	*b = 15;
    }
    if (*(unsigned char *)a == 'p') {
	*b = 4;
    }
    if (*(unsigned char *)a == 'q') {
	*b = 17;
    }
    if (*(unsigned char *)a == 'r') {
	*b = 18;
    }
    if (*(unsigned char *)a == 's') {
	*b = 19;
    }
    if (*(unsigned char *)a == 't') {
	*b = 20;
    }
    if (*(unsigned char *)a == 'u') {
	*b = 21;
    }
    if (*(unsigned char *)a == 'v') {
	*b = 22;
    }
    if (*(unsigned char *)a == 'w') {
	*b = 1;
    }
    if (*(unsigned char *)a == 'x') {
	*b = 24;
    }
    if (*(unsigned char *)a == 'y') {
	*b = 25;
    }
    if (*(unsigned char *)a == 'z') {
	*b = 26;
    }
    if (*(unsigned char *)a == '_') {
	*b = 64;
    }
    if (*(unsigned char *)a == '(') {
	*b = 65;
    }
    if (*(unsigned char *)a == ')') {
	*b = 66;
    }
    if (*(unsigned char *)a == '+') {
	*b = 67;
    }
    if (*(unsigned char *)a == '-') {
	*b = 68;
    }
    if (*(unsigned char *)a == '&') {
	*b = 69;
    }
    if (*(unsigned char *)a == '.') {
	*b = 70;
    }
    if (*(unsigned char *)a == ',') {
	*b = 71;
    }
    if (*(unsigned char *)a == ':') {
	*b = 72;
    }
    if (*(unsigned char *)a == ';') {
	*b = 73;
    }
    if (*(unsigned char *)a == '*') {
	*b = 74;
    }
    if (*(unsigned char *)a == '=') {
	*b = 75;
    }
    if (*(unsigned char *)a == '/') {
	*b = 76;
    }
    if (*(unsigned char *)a == '!') {
	*b = 80;
    }
    if (*(unsigned char *)a == '[') {
	*b = 83;
    }
    if (*(unsigned char *)a == ']') {
	*b = 84;
    }
    return 0;
} /* alphabet_ */

/* This is the MIDACO.c f2c-header from below */

#ifdef KR_headers
double pow();
double pow_dd(ap, bp) doublereal *ap, *bp;
#else
#undef abs
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif
double pow_dd(doublereal *ap, doublereal *bp)
#endif
{
return(pow(*ap, *bp) );
}
#ifdef __cplusplus
}
#endif
double d_nint(x)
doublereal *x;
{
double floor();

return( (*x)>=0 ?
 floor(*x + .5) : -floor(.5 - *x) );
}
#ifdef __cplusplus
extern "C" {
#endif
#ifdef KR_headers
double pow_di(ap, bp) doublereal *ap; integer *bp;
#else
double pow_di(doublereal *ap, integer *bp)
#endif
{
double pow, x;
integer n;
unsigned long u;
pow = 1;
x = *ap;
n = *bp;
if(n != 0)
 {
 if(n < 0)
  {
  n = -n;
  x = 1/x;
  }
 for(u = n; ; )
  {
  if(u & 01)
   pow *= x;
  if(u >>= 1)
   x *= x;
  else
   break;
  }
 }
return(pow);
}
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
extern "C" {
#endif
#ifdef KR_headers
integer pow_ii(ap, bp) integer *ap, *bp;
#else
integer pow_ii(integer *ap, integer *bp)
#endif
{
 integer pow, x, n;
 unsigned long u;
 x = *ap;
 n = *bp;
 if (n <= 0) {
  if (n == 0 || x == 1)
   return 1;
  if (x != -1)
   return x == 0 ? 1/x : 0;
  n = -n;
  }
 u = n;
 for(pow = 1; ; )
  {
  if(u & 01)
   pow *= x;
  if(u >>= 1)
   x *= x;
  else
   break;
  }
 return(pow);
 }
#ifdef __cplusplus
}
#endif
#ifdef KR_headers
double floor();
integer i_dnnt(x) doublereal *x;
#else
#undef abs
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif
integer i_dnnt(doublereal *x)
#endif
{
return (integer)(*x >= 0. ? floor(*x + .5) : -floor(.5 - *x));
}
#ifdef __cplusplus
}
#endif
































/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     The following subroutines handle all printing commands for MIDACO.
C     These subroutines will also check the MAXEVAL and MAXTIME criteria.
C     Note that these subroutines are called independently from MIDACO 
C     and MIDACO itself does not include any print commands (due to 
C     compiler portability and robustness).
CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
#include <stdio.h>
#include <math.h>
#include <time.h>
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/* This function opens a file by either the fopen or fopen_s  command */
/* In case you use standard c compilers, such as gcc, use the fopen   */
/* In case you use MS Visual C++ use the fopen_s command              */
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
FILE* open_file( FILE *iout , char *filename)
{   
      iout = fopen( filename, "w"); /* standard C command */
      //fopen_s( &iout, filename, "w"); /* MS Visual C++ command */
      return iout; 
}
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/* This is the time command, adopt to comiler, if necessary           */
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
double gettime(){time_t second;second=time(NULL);return (double)second;}
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/* These are flush and close commands, delete or change if necessary  */
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
int flush_output( FILE *iout ){   fflush( iout );   return 0; }
int close_output( FILE *iout ){   fclose( iout );   return 0; }
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
int warnings_and_erros(long int*,FILE*);int print_head(long int,long int,
long int,long int,long int,long int,double*,long int,long int,long int,
long int,char*,FILE*);int print_line(long int,long int,double,double,
double,double*,FILE*);int print_solution(long int,long int,long int,
long int,double*,double*,double*,double*,double*,double*,double,long int,
double,long int,double*,FILE*);int print_final(long int,long int,double,
double,long int,long int,long int,long int,long int,double*,double*,
double*,double*,double*,double*,double*,double,long int,double*,FILE*);
int print_paretofront(long int,long int,long int,double*,double*,double*,
int); int save_history(int,long int,long int,long int,long int,double*,
double*,double*,FILE*,long int);
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/   
int midaco_print(int c, long int printeval, long int save2file, long int *iflag, 
                 long int *istop, double *f, double *g, double *x, double *xl, 
                 double *xu, long int o, long int n, long int ni, long int m, 
                 long int me, double *rw, double *pf, long int maxeval, 
                 long int maxtime, double *param, long int P, char *key)
{
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/* Increase size of bestg and bestx for problems with N+O > 100000    */
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/  
static double bestg[10000]; 
static double bestx[10000];
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/  
static FILE* iout; /* console screen */
static FILE* iout1; /* MIDADCO_SCREEN.TXT */
static FILE* iout2; /* MIDADCO_SOLUTION.TXT */
static FILE* iout4; /* MIDADCO_HISTORY.TXT */
static double tstart, tnow, tmax;
static long int eval;
static double acc;
static int tic;
static int q; 
static int kx;  
static int kf;
static int kg; 
static int kres;  
static int wx;   
static int wf; 
static int wg;
static int wres;    
static int kbest;
static int wbest;
static double bestf[1];
static double bestr[1];
static int update,i;
static double dummy_f, dummy_vio;
static int pfmax;
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/ 
if(save2file >= 2){if(c==1){ iout4 = open_file( &*iout4, "MIDACO_HISTORY.TXT"); }
                   save_history(c,P,o,n,m,f,g,x,&*iout4,*istop); 
                   flush_output( &*iout4 );}
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/
/*CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC*/ 
if(c == 2)
{  
  tnow = gettime()-tstart;   
  eval = eval + P;           
  if(*iflag >= 10)
  {
    warnings_and_erros( iflag, &*iout);
    if(save2file > 0)
    {
      warnings_and_erros( iflag, &*iout1);
      warnings_and_erros( iflag, &*iout2);
    }
  }      
  if(printeval > 0)
  {            
      tic = tic + P;
      if((tic >= printeval)||(eval == P)||(*iflag >= 1))
      {
         if(eval < 0)
         { tic = 0; 
           eval = - eval - 2*printeval; }
         if(eval > P){ tic = 0; } 
         if(rw[kres] == rw[wres]){
           kbest = kf;
           wbest = wf;
         }else{
           kbest = kres;
           wbest = wres;
         }                
         if((rw[wbest] < rw[kbest]) ||
            (*iflag >= 1)||(*iflag == -300)||(*iflag == -500)){
            
               bestf[0] = rw[wf];
               bestr[0] = rw[wres];
               for(i=0;i<m+2*o;i++){  bestg[i] = rw[wg+i]; }
               for(i=0;i<n;i++){  bestx[i] = rw[wx+i]; }  
         }
         else
         {        
               bestf[0] = rw[kf];
               bestr[0] = rw[kres];
               for(i=0;i<m+2*o;i++){  bestg[i] = rw[kg+i]; }
               for(i=0;i<n;i++){  bestx[i] = rw[kx+i]; }          
         }
         if(*iflag < 100){      
         print_line( o, eval, tnow, bestf[0], bestr[0], &*pf, &*iout);  
         if(save2file > 0)
         {
          print_line( o, eval, tnow, bestf[0], bestr[0], &*pf, &*iout1);            
         }
         if(save2file > 0)
         {           
             update = 0;   
             if( (bestr[0]  < dummy_vio)||
                ((bestr[0] == dummy_vio)&&(bestf[0] < dummy_f)) )
                {
                  dummy_f = bestf[0];
                  dummy_vio = bestr[0];
                  update = 1;
                }
               if( update > 0 )
               {                              
                 fprintf(iout2,"\n\n            CURRENT BEST SOLUTION");
                 print_solution( o, n, m, me, &*bestx, &*bestg, 
                                 &*bestf, &*bestr, &*xl, &*xu, acc, 
                                 eval, tnow, *iflag, &*pf, &*iout2);
               }
              flush_output( &*iout1 );
              flush_output( &*iout2 );    
              if( o > 1 )
              {
                if(( save2file >= 1 )&&( printeval >= 1))
                {
                print_paretofront(o,m,n,&*pf,&*bestx,&*bestg,pfmax);
              }
            }          
         }
         }          
      }
  }            
  if(*istop == 0)
  {
      if(tnow >= tmax)     { *iflag = -999;}
      if(eval >= maxeval-1)
  {
   if(maxeval <= 99999999){ *iflag = -999;}
  }          
      /* special case maxeval = 1 */
      if( maxeval<=1 ){
  {
      if(tnow >= tmax)     { *iflag = -999;}
      if(eval >= maxeval-1)
  {
   if(maxeval <= 99999999){ *iflag = -999;}
  }   
        /* special case maxeval = 1 */
        if((tnow<=0)||(maxeval<=1)){ 
        *istop = 1; 
        *iflag = 0;
        for(i=0;i<n;i++){ x[i] = bestx[i]; } 
      }        
  } 
        *istop = 1; 
        *iflag = 0;
        for(i=0;i<n;i++){ x[i] = bestx[i]; } 
      }   
  }

  if(*istop>=1)
  {
    for(i=0;i<m;i++){  bestg[i] = g[i]; }
       if( o >= 2 ){ for(i=0;i<o*2;i++){  bestg[m+i] = rw[wg+m+i]; }}
    if(printeval > 0)
    {
     print_final(o,*iflag,tnow,tmax,eval,maxeval,n,m,me,&*x,&*bestg,&*f, 
     &*xl,&*xu,&*rw,&*pf,acc,wres,param,&*iout); 
     if(save2file > 0)
     {
      print_final(o,*iflag,tnow,tmax,eval,maxeval,n,m,me,&*x,&*bestg,&*f, 
       &*xl,&*xu,&*rw,&*pf,acc,wres,param,&*iout1);                    
      print_final(o,*iflag,tnow,tmax,eval,maxeval,n,m,me,&*x,&*bestg,&*f, 
       &*xl,&*xu,&*rw,&*pf,acc,wres,param,&*iout2);              
      flush_output( &*iout1 );
      flush_output( &*iout2 );
      close_output( &*iout1 );
      close_output( &*iout2 );    
     }
    }
    return 0;
  }
}
if(c == 1)
{     
  *iflag = 0;    
  *istop = 0;
  tmax = (double) maxtime;
  tstart = gettime(); 
  eval   = 0;
  if(param[0] <= 0.0)
  {
    acc = 0.001;
  }else{
    acc = param[0];
  }
  q    = 102*n+(m+2*o)+516;   
  kx   = 9;
  kf   = 9+n;
  kg   = 9+n+1;
  kres = 9+n+1+m;  
  if( o > 1 ){ kres = 9+n+1+ (m+2*o); }
  wx   = q;
  wf   = q+n;
  wg   = q+n+1;
  wres = q+n+1+m; 
  if( o > 1 ){ wres = q+n+1+ (m+2*o); }    
  iout = stdout; 
  if((save2file > 0)&&( printeval > 0))
  {
    iout1 = open_file( &*iout1, "MIDACO_SCREEN.TXT");
    iout2 = open_file( &*iout2, "MIDACO_SOLUTION.TXT");    
  }        
  bestf[0] = 1.0e+32;
  bestr[0] = 1.0e+32;
  dummy_f   = 1.0e+32;
  dummy_vio = 1.0e+32;    
  tic = 0;    
  pfmax = 100;
  if( param[9] >= 1.0 ){ pfmax =     (int)param[9]; }
  if( param[9] <=-1.0 ){ pfmax = -1*((int)param[9]); }  
  if(printeval >= 1)
  {
      print_head(P,o,n,ni,m,me,&*param,maxeval,maxtime,
                 printeval,save2file,&*key,&*iout);
      if(save2file > 0)
      { 
          print_head(P,o,n,ni,m,me,&*param,maxeval,maxtime,
                     printeval,save2file,&*key,&*iout1);
      }
   }
   if((save2file > 0)&&(printeval > 0))
   {
   fprintf(iout2," MIDACO - SOLUTION\n");
   fprintf(iout2," -----------------\n");
   fprintf(iout2,"\n This file saves the current best solution X found by MIDACO.");
   fprintf(iout2,"\n This file is updated after every PRINTEVAL function evaluation,");
   fprintf(iout2,"\n if X has been improved.\n\n");
   }
   if((save2file > 0)&&(printeval > 0))
   {
     flush_output( &*iout1 );
     flush_output( &*iout2 );
   }
}
return 0;
}
int print_head(long int p, long int o, long int n, long int ni, long int m, 
               long int me, double *param, long int maxeval, long int maxtime, 
               long int printeval, long int save2file, char *key, FILE *iout)
{
  int i, dummy;
  fprintf(iout,"\n MIDACO 5.0    (www.midaco-solver.com)");
  fprintf(iout,"\n -------------------------------------\n\n");
  fprintf(iout," LICENSE-KEY:  ");for(i=0;i<=59;i++){fprintf(iout,"%c",key[i]);}
  fprintf(iout,"\n\n ----------------------------------------\n");
  fprintf(iout," | OBJECTIVES%5li | PARALLEL%10li |\n",o,p); 
  fprintf(iout," |--------------------------------------|\n");
  fprintf(iout," | N%14li | MAXEVAL%11li |\n",n,maxeval);   
  fprintf(iout," | NI%13li | MAXTIME%11li |\n",ni,maxtime);
  fprintf(iout," | M%14li | PRINTEVAL%9li |\n",m,printeval);
  fprintf(iout," | ME%13li | SAVE2FILE%9li |\n",me,save2file);
  fprintf(iout," |--------------------------------------|\n");
  dummy = 0; for(i=0;i<12;i++){ if(param[i] != 0.0){ dummy=1;}}  
  if(dummy == 0)
  {
    fprintf(iout," | PARAMETER:    All by default (0)     |\n");
  }
  else
  {
    if(param[ 0]!=0.0){fprintf(iout," | PARAM[ 0] %14.7e ACCURACY    |\n",param[ 0]);}
    if(param[ 1]!=0.0){fprintf(iout," | PARAM[ 1] %14.7e RANDOM-SEED |\n",param[ 1]);}
    if(param[ 2]!=0.0){fprintf(iout," | PARAM[ 2] %14.7e FSTOP       |\n",param[ 2]);}
    if(param[ 3]!=0.0){fprintf(iout," | PARAM[ 3] %14.7e ALGOSTOP    |\n",param[ 3]);}
    if(param[ 4]!=0.0){fprintf(iout," | PARAM[ 4] %14.7e EVALSTOP    |\n",param[ 4]);}
    if(param[ 5]!=0.0){fprintf(iout," | PARAM[ 5] %14.7e FOCUS       |\n",param[ 5]);}
    if(param[ 6]!=0.0){fprintf(iout," | PARAM[ 6] %14.7e ANTS        |\n",param[ 6]);}
    if(param[ 7]!=0.0){fprintf(iout," | PARAM[ 7] %14.7e KERNEL      |\n",param[ 7]);}
    if(param[ 8]!=0.0){fprintf(iout," | PARAM[ 8] %14.7e ORACLE      |\n",param[ 8]);}
    if(param[ 9]!=0.0){fprintf(iout," | PARAM[ 9] %14.7e PARETOMAX   |\n",param[ 9]);}
    if(param[10]!=0.0){fprintf(iout," | PARAM[10] %14.7e EPSILON     |\n",param[10]);}
    if(param[11]!=0.0){fprintf(iout," | PARAM[11] %14.7e CHARACTER   |\n",param[11]);}                                              
  }
  fprintf(iout," ----------------------------------------\n\n");
  if( o <= 1 ){
  fprintf(iout," [     EVAL,    TIME]        OBJECTIVE FUNCTION VALUE         VIOLATION OF G(X)\n");
  fprintf(iout," ------------------------------------------------------------------------------\n"); }
  if( o >= 2 ){
  fprintf(iout," [     EVAL,    TIME]   FIRST OBJECTIVE FUNCTION   VIOLATION OF G(X)\n");
  fprintf(iout," -------------------------------------------------------------------   [PARETO]\n"); }  
  return 0;
}
int print_line(long int o, long int eval, double tnow, double f, double vio, double *pf, FILE *iout)
{   
    long int psize;
    if( o >= 2 ){ psize = (long int)(pf[0]); }
    if(fabs(f) <= 1.0e+10)
    {
      if(vio <= 1.0e+5)
      {
        if( o <= 1 ){ fprintf(iout," [%9li,%8.0f]        F(X):%19.8f         VIO:%13.6f\n",eval,tnow,f,vio); }
        if( o >= 2 ){ fprintf(iout," [%9li,%8.0f]   F(X):%19.8f   VIO:%13.6f   [%6li]\n",eval,tnow,f,vio,psize); }        
      }else{
        if( o <= 1 ){ fprintf(iout," [%9li,%8.0f]        F(X):%19.8f         VIO:%13.6e\n",eval,tnow,f,vio); }   
        if( o >= 2 ){ fprintf(iout," [%9li,%8.0f]   F(X):%19.8f   VIO:%13.6e   [%6li]\n",eval,tnow,f,vio,psize); }           
      }
    }else{
      if(vio <= 1.0e+5)
      {
        if( o <= 1 ){ fprintf(iout," [%9li,%8.0f]        F(X):%19.8e         VIO:%13.6f\n",eval,tnow,f,vio); }
        if( o >= 2 ){ fprintf(iout," [%9li,%8.0f]   F(X):%19.8e   VIO:%13.6f   [%6li]\n",eval,tnow,f,vio,psize); }        
      }else{
        if( o <= 1 ){ fprintf(iout," [%9li,%8.0f]        F(X):%19.8e         VIO:%13.6e\n",eval,tnow,f,vio); }    
        if( o >= 2 ){ fprintf(iout," [%9li,%8.0f]   F(X):%19.8e   VIO:%13.6e   [%6li]\n",eval,tnow,f,vio,psize); }            
      }    
    }
    return 0;
}                               
int print_solution(long int o,long int n,long int m,long int me,double *x,double *g, 
                   double *f,double *vio,double *xl,double *xu, double acc, 
                   long int eval, double tnow,long int iflag, double *pf, FILE *iout)
{
int i,j,on,profil; long int psize;    
fprintf(iout,"\n --------------------------------------------\n");
fprintf(iout," EVAL:%10li,  TIME:%8.2f,  IFLAG:%4li \n",eval,tnow,iflag);
fprintf(iout," --------------------------------------------\n");
  if( o <= 1 )
  {    
    if(fabs(f[0]) <= 1.0e+18){ fprintf(iout," f[0] =%38.15f \n",f[0]);
    }else{                     fprintf(iout," f[0] =%38.6e \n",f[0]); }
  fprintf(iout," --------------------------------------------\n");
}
else
{
  for( i=0; i<o; i++)
  {          
    if(fabs(g[m+i]) <= 1.0e+18){ if(g[m+o+i]<=0.0){fprintf(iout," f[%4i] = %34.15f \n",i,g[m+i]);}
                                  else{fprintf(iout," f[%4i] = %34.15f \n",i,-g[m+i]);}
    }else{                       if(g[m+o+i]<=0.0){fprintf(iout," f[%4i] = %34.6e \n",i,g[m+i]);}
                                  else{fprintf(iout," f[%4i] = %34.6e \n",i,-g[m+i]);}
    }    
    if( i == 0){
    fprintf(iout," --------------------------------------------\n");
    psize = (long int)(pf[0]); 
    fprintf(iout," NUMBER OF PARETO POINTS%21li \n",psize);
    fprintf(iout," --------------------------------------------\n"); }
  }
  fprintf(iout," --------------------------------------------\n");
}
if(m > 0)
{
    if(iflag < 100)
    {
      if(vio[0] <= 1.0e+12)
      {
              fprintf(iout," VIOLATION OF G(X)%27.12f\n",vio[0]);
      }else{
              fprintf(iout," VIOLATION OF G(X)%27.6e\n",vio[0]); 
      }
      fprintf(iout," --------------------------------------------\n");
    }
    for( i=0; i<m; i++)
    {                     
        if(i < me)
        {
          if(fabs(g[i]) <= acc)
          {
            fprintf(iout," g[%4i] =%16.8f  (EQUALITY CONSTR)\n",i,g[i]);
          }
          else
          {
            if(fabs(g[i]) <= 1.0e+7){
            fprintf(iout," g[%4i] =%16.8f  (EQUALITY CONSTR)  <---  INFEASIBLE  ( G NOT = 0 )\n",i,g[i]);
            }else{
            fprintf(iout," g[%4i] =%16.3e  (EQUALITY CONSTR)  <---  INFEASIBLE  ( G NOT = 0 )\n",i,g[i]);
            }
          }
        }
        if(i >= me)
        {
          if(g[i] > -acc)
          {
            if(fabs(g[i]) <= 1.0e+7){
            fprintf(iout," g[%4i] =%16.8f  (IN-EQUAL CONSTR)\n",i,g[i]);
            }else{
            fprintf(iout," g[%4i] =%16.3e  (IN-EQUAL CONSTR)\n",i,g[i]);
            }
          }
          else
          {
            if(fabs(g[i]) <= 1.0e+7){
            fprintf(iout," g[%4i] =%16.8f  (IN-EQUAL CONSTR)  <---  INFEASIBLE  ( G < 0 )\n",i,g[i]);
            }else{
            fprintf(iout," g[%4i] =%16.3e  (IN-EQUAL CONSTR)  <---  INFEASIBLE  ( G < 0 )\n",i,g[i]);
            }
          }
        }            
    }
    fprintf(iout," --------------------------------------------         BOUNDS-PROFIL    \n");
}                
for( i=0; i<n; i++)
{
    profil = -1; on = 1; 
    if((on==1)&&( x[i] > xu[i]+1.0e-6 )){ profil = 91; on = 0; }
    if((on==1)&&( x[i] < xl[i]-1.0e-6 )){ profil = 92; on = 0; }        
    if((on==1)&&( xl[i] > xu[i]       )){ profil = 93; on = 0; }         
    if((on==1)&&( xl[i] == xu[i]      )){ profil = 90; on = 0; }
    if((on==1)&&( fabs(x[i]-xl[i]) < (xu[i]-xl[i])/1000.0 )){ profil =  0; on = 0; }                
    if((on==1)&&( fabs(xu[i]-x[i]) < (xu[i]-xl[i])/1000.0 )){ profil = 22; on = 0; }     
    for( j=1; j<=21; j++)
    {
      if((on==1)&&( x[i] <= xl[i] + ((double) j) * (xu[i]-xl[i])/21.0 )){ profil = j; on = 0; }        
    }  
if( fabs(x[i]) <= 1.0e+14 )
{
  if(profil == 0){fprintf(iout," x[%4i] =%34.15f;  /* XL___________________ */\n",i,x[i]);}
  if(profil == 1){fprintf(iout," x[%4i] =%34.15f;  /* x____________________ */\n",i,x[i]);}
  if(profil == 2){fprintf(iout," x[%4i] =%34.15f;  /* _x___________________ */\n",i,x[i]);}
  if(profil == 3){fprintf(iout," x[%4i] =%34.15f;  /* __x__________________ */\n",i,x[i]);}
  if(profil == 4){fprintf(iout," x[%4i] =%34.15f;  /* ___x_________________ */\n",i,x[i]);}
  if(profil == 5){fprintf(iout," x[%4i] =%34.15f;  /* ____x________________ */\n",i,x[i]);}
  if(profil == 6){fprintf(iout," x[%4i] =%34.15f;  /* _____x_______________ */\n",i,x[i]);}
  if(profil == 7){fprintf(iout," x[%4i] =%34.15f;  /* ______x______________ */\n",i,x[i]);}
  if(profil == 8){fprintf(iout," x[%4i] =%34.15f;  /* _______x_____________ */\n",i,x[i]);}
  if(profil == 9){fprintf(iout," x[%4i] =%34.15f;  /* ________x____________ */\n",i,x[i]);}
  if(profil ==10){fprintf(iout," x[%4i] =%34.15f;  /* _________x___________ */\n",i,x[i]);}
  if(profil ==11){fprintf(iout," x[%4i] =%34.15f;  /* __________x__________ */\n",i,x[i]);}
  if(profil ==12){fprintf(iout," x[%4i] =%34.15f;  /* ___________x_________ */\n",i,x[i]);}
  if(profil ==13){fprintf(iout," x[%4i] =%34.15f;  /* ____________x________ */\n",i,x[i]);}
  if(profil ==14){fprintf(iout," x[%4i] =%34.15f;  /* _____________x_______ */\n",i,x[i]);}
  if(profil ==15){fprintf(iout," x[%4i] =%34.15f;  /* ______________x______ */\n",i,x[i]);}
  if(profil ==16){fprintf(iout," x[%4i] =%34.15f;  /* _______________x_____ */\n",i,x[i]);}
  if(profil ==17){fprintf(iout," x[%4i] =%34.15f;  /* ________________x____ */\n",i,x[i]);}
  if(profil ==18){fprintf(iout," x[%4i] =%34.15f;  /* _________________x___ */\n",i,x[i]);}
  if(profil ==19){fprintf(iout," x[%4i] =%34.15f;  /* __________________x__ */\n",i,x[i]);}
  if(profil ==20){fprintf(iout," x[%4i] =%34.15f;  /* ___________________x_ */\n",i,x[i]);}
  if(profil ==21){fprintf(iout," x[%4i] =%34.15f;  /* ____________________x */\n",i,x[i]);}
  if(profil ==22){fprintf(iout," x[%4i] =%34.15f;  /* ___________________XU */\n",i,x[i]);}
  if(profil ==90){fprintf(iout," x[%4i] =%34.15f;  /* WARNING: XL = XU      */\n",i,x[i]);}
  if(profil ==91){fprintf(iout," x[%4i] =%34.15f; ***ERROR*** (X > XU)        \n",i,x[i]);}
  if(profil ==92){fprintf(iout," x[%4i] =%34.15f; ***ERROR*** (X < XL)        \n",i,x[i]);}
  if(profil ==93){fprintf(iout," x[%4i] =%34.15f; ***ERROR*** (XL > XU)       \n",i,x[i]);}
  if(profil < 0 ){fprintf(iout," PROFIL-ERROR");} 
}else{
  if(profil == 0){fprintf(iout," x[%4i] =%34.1e;  /* XL___________________ */\n",i,x[i]);}
  if(profil == 1){fprintf(iout," x[%4i] =%34.1e;  /* x____________________ */\n",i,x[i]);}
  if(profil == 2){fprintf(iout," x[%4i] =%34.1e;  /* _x___________________ */\n",i,x[i]);}
  if(profil == 3){fprintf(iout," x[%4i] =%34.1e;  /* __x__________________ */\n",i,x[i]);}
  if(profil == 4){fprintf(iout," x[%4i] =%34.1e;  /* ___x_________________ */\n",i,x[i]);}
  if(profil == 5){fprintf(iout," x[%4i] =%34.1e;  /* ____x________________ */\n",i,x[i]);}
  if(profil == 6){fprintf(iout," x[%4i] =%34.1e;  /* _____x_______________ */\n",i,x[i]);}
  if(profil == 7){fprintf(iout," x[%4i] =%34.1e;  /* ______x______________ */\n",i,x[i]);}
  if(profil == 8){fprintf(iout," x[%4i] =%34.1e;  /* _______x_____________ */\n",i,x[i]);}
  if(profil == 9){fprintf(iout," x[%4i] =%34.1e;  /* ________x____________ */\n",i,x[i]);}
  if(profil ==10){fprintf(iout," x[%4i] =%34.1e;  /* _________x___________ */\n",i,x[i]);}
  if(profil ==11){fprintf(iout," x[%4i] =%34.1e;  /* __________x__________ */\n",i,x[i]);}
  if(profil ==12){fprintf(iout," x[%4i] =%34.1e;  /* ___________x_________ */\n",i,x[i]);}
  if(profil ==13){fprintf(iout," x[%4i] =%34.1e;  /* ____________x________ */\n",i,x[i]);}
  if(profil ==14){fprintf(iout," x[%4i] =%34.1e;  /* _____________x_______ */\n",i,x[i]);}
  if(profil ==15){fprintf(iout," x[%4i] =%34.1e;  /* ______________x______ */\n",i,x[i]);}
  if(profil ==16){fprintf(iout," x[%4i] =%34.1e;  /* _______________x_____ */\n",i,x[i]);}
  if(profil ==17){fprintf(iout," x[%4i] =%34.1e;  /* ________________x____ */\n",i,x[i]);}
  if(profil ==18){fprintf(iout," x[%4i] =%34.1e;  /* _________________x___ */\n",i,x[i]);}
  if(profil ==19){fprintf(iout," x[%4i] =%34.1e;  /* __________________x__ */\n",i,x[i]);}
  if(profil ==20){fprintf(iout," x[%4i] =%34.1e;  /* ___________________x_ */\n",i,x[i]);}
  if(profil ==21){fprintf(iout," x[%4i] =%34.1e;  /* ____________________x */\n",i,x[i]);}
  if(profil ==22){fprintf(iout," x[%4i] =%34.1e;  /* ___________________XU */\n",i,x[i]);}
  if(profil ==90){fprintf(iout," x[%4i] =%34.1e;  /* WARNING: XL = XU      */\n",i,x[i]);}
  if(profil ==91){fprintf(iout," x[%4i] =%34.1e; ***ERROR*** (X > XU)        \n",i,x[i]);}
  if(profil ==92){fprintf(iout," x[%4i] =%34.1e; ***ERROR*** (X < XL)        \n",i,x[i]);}
  if(profil ==93){fprintf(iout," x[%4i] =%34.1e; ***ERROR*** (XL > XU)       \n",i,x[i]);}
  if(profil < 0 ){fprintf(iout," PROFIL-ERROR");}
  }               
 }  
fprintf(iout," \n ");
return 0;
}   
int print_final(long int o, long int iflag, double tnow, double tmax, long int eval, 
                long int maxeval, long int n, long int m, long int me, double *x, 
                double *g, double *f, double *xl, double *xu, double *rw, double *pf, 
                double acc, long int wres, double *param, FILE *iout)
{
    double vio[1];
    vio[0] = rw[wres];
    if((iflag == 1)||(iflag == 2))
    {
    if(tnow >=    tmax){ fprintf(iout,"\n OPTIMIZATION FINISHED  --->  MAXTIME REACHED");}
    if(eval >= maxeval){ fprintf(iout,"\n OPTIMIZATION FINISHED  --->  MAXEVAL REACHED");}
    }
    if((iflag == 3)||(iflag == 4))
    {      
        fprintf(iout,"\n OPTIMIZATION FINISHED  --->  ALGOSTOP (=%3li)",(long int)param[3]);
    }
    if((iflag == 5)||(iflag == 6))
    {      
        fprintf(iout,"\n OPTIMIZATION FINISHED  --->  EVALSTOP (=%9li)",(long int)param[4]);
    }    
    if(iflag == 7)
    {      
        fprintf(iout,"\n OPTIMIZATION FINISHED  --->  FSTOP REACHED");
    }           
    fprintf(iout,"\n\n\n         BEST SOLUTION FOUND BY MIDACO");
    print_solution( o, n, m, me, &*x, &*g, &*f, &*vio, 
                    &*xl, &*xu, acc,eval, tnow, iflag, &*pf, &*iout);
return 0;
}
int warnings_and_erros( long int *iflag, FILE *iout )
{
  if(*iflag < 100)
  {
    fprintf(iout,"\n *** WARNING ***   ( IFLAG =%6li )\n\n", *iflag);
  }
  else
  {
   fprintf(iout,"\n *** MIDACO INPUT ERROR ***   ( IFLAG =%6li )\n\n", *iflag);
  }
  return 0;
}  
int print_paretofront(long int o, long int m, long int n, double *pf, 
                      double *x, double *g, int pfmax)
{
  long int psize = (long int)(pf[0]); int i,k; double dummy;
    static FILE* iout3; /* MIDADCO_PARETOFRONT.TXT */  
    iout3 = open_file( &*iout3, "MIDACO_PARETOFRONT.tmp");
    fprintf(iout3,"#########################################################\n");
    fprintf(iout3,"### This file contains the pareto front approximation ###\n");
    fprintf(iout3,"#########################################################\n");
    fprintf(iout3,"### Solution format:     F(1:O)    G(1:M)    X(1:N)   ###\n");
    fprintf(iout3,"#########################################################\n");
    fprintf(iout3,"#\n");
    fprintf(iout3,"#        O         M         N     PSIZE\n");
    fprintf(iout3,"#\n");
    fprintf(iout3," %9li %9li %9li %9li \n", o, m, n, psize);
    fprintf(iout3,"#\n");
    fprintf(iout3,"#        MIDACO solution\n");
    fprintf(iout3,"#\n");
    for( i=0; i<o; i++)
    { 
      dummy = g[m+i];
      if( g[m+o+i] > 0.0 ){ dummy = -dummy; }
      
      if(   dummy <= 1.0e+9){ fprintf(iout3,"%19.8f ",dummy); }
      else{                   fprintf(iout3,"%16.6e ",dummy); }
    }
    for( i=0; i<m; i++)
    { 
      if(   g[i] <= 1.0e+9){ fprintf(iout3,"%19.8f ",g[i]); }
      else{                  fprintf(iout3,"%16.6e ",g[i]); }
    }   
    for( i=0; i<n; i++)
    { 
      if(   x[i] <= 1.0e+9){ fprintf(iout3,"%19.8f ",x[i]); }
      else{                  fprintf(iout3,"%16.6e ",x[i]); }
    }          
    fprintf(iout3,"\n"); /* linebreak */
    fprintf(iout3,"#\n");
    fprintf(iout3,"#        All non-dominated solutions found by MIDACO\n");
    fprintf(iout3,"#\n"); 
    for( k=0; k<psize; k++) 
    {    
       for( i=0; i<o; i++)
       { 
         dummy = pf[ 1 + o*k + i ];
         if(   dummy <= 1.0e+9){ fprintf(iout3,"%19.8f ",dummy); }
         else{                   fprintf(iout3,"%16.6e ",dummy); }
       }
       for( i=0; i<m; i++)
       { 
         dummy = pf[ 1 + o*pfmax+m*k + i ];
         if(   dummy <= 1.0e+9){ fprintf(iout3,"%19.8f ",dummy); }
         else{                   fprintf(iout3,"%16.6e ",dummy); }
       }
       for( i=0; i<n; i++)
       { 
         dummy = pf[ 1 + o*pfmax+m*pfmax+n*k + i ];
         if(   dummy <= 1.0e+9){ fprintf(iout3,"%19.8f ",dummy); }
         else{                   fprintf(iout3,"%16.6e ",dummy); }
       }                  
       fprintf(iout3,"\n"); /* linebreak */
    }
    close_output(iout3);
    rename( "MIDACO_PARETOFRONT.tmp" , "MIDACO_PARETOFRONT.TXT" );
  return 0;
}
int save_history( int c, long int P, long int o, long int n, long int m,
                  double *f, double *g, double *x,  FILE *iout, long int istop)
{
    static long int tix;
    static double previous_x[100000];
    long int i,k;
    double dummy;
    if( c == 1)
    {
      tix = 1;

      fprintf(iout,"#########################################################\n");
      fprintf(iout,"### This file contains the history of  ALL  solutions ###\n");
      fprintf(iout,"#########################################################\n");
      fprintf(iout,"### SOLUTION FORMAT:     F(1:O)    G(1:M)    X(1:N)   ###\n");
      fprintf(iout,"#########################################################\n");
      fprintf(iout,"#\n");
      fprintf(iout,"#        O         M         N \n");
      fprintf(iout,"#\n");
      fprintf(iout," %9li %9li %9li \n", o, m, n);
      fprintf(iout,"#\n");
      fprintf(iout,"#        MIDACO STARTING POINT (very first solution) \n");
      fprintf(iout,"#\n");
    }
    else
    {
      tix = tix + 1;
      if( tix == 2 )
      {

       for( i=0; i<o; i++)
       { 
         dummy = f[i];
         if(   dummy <= 1.0e+9){ fprintf(iout,"%19.8f ",dummy); }
         else{                   fprintf(iout,"%16.6e ",dummy); }
       }
       for( i=0; i<m; i++)
       { 
         dummy = g[i];
         if(   dummy <= 1.0e+9){ fprintf(iout,"%19.8f ",dummy); }
         else{                   fprintf(iout,"%16.6e ",dummy); }
       }
       for( i=0; i<n; i++)
       { 
         dummy = previous_x[i];
         if(   dummy <= 1.0e+9){ fprintf(iout,"%19.8f ",dummy); }
         else{                   fprintf(iout,"%16.6e ",dummy); }
       }  
       fprintf(iout,"\n"); /* linebreak */
       fprintf(iout,"#\n");
       fprintf(iout,"#        SOLUTION HISTORY (in chronological order)\n");
       fprintf(iout,"#\n");       
      }
      else
      {
        for( k=1; k<=P; k++)
        {        
            for( i=0; i<o; i++)
            { 
              if( istop <= 0 ){ dummy = f[(k-1)*o+i]; }
              if( istop >= 1 ){ dummy = f[i]; }
              if(   dummy <= 1.0e+9){ fprintf(iout,"%19.8f ",dummy); }
              else{                   fprintf(iout,"%16.6e ",dummy); }
            }
            for( i=0; i<m; i++)
            { 
              if( istop <= 0 ){ dummy = g[(k-1)*m+i]; }
              if( istop >= 1 ){ dummy = g[i]; }       
              if(   dummy <= 1.0e+9){ fprintf(iout,"%19.8f ",dummy); }
              else{                   fprintf(iout,"%16.6e ",dummy); }
            }
            for( i=0; i<n; i++)
            { 
              if( istop <= 0 ){ dummy = previous_x[(k-1)*n+i]; }
              if( istop >= 1 ){ dummy = x[i]; }            
              if(   dummy <= 1.0e+9){ fprintf(iout,"%19.8f ",dummy); }
              else{                   fprintf(iout,"%16.6e ",dummy); }
            }    
            fprintf(iout,"\n"); /* linebreak */      
        }
      }
  }
  for( k=1; k<=P; k++)
  { 
    for( i=0; i<n; i++)
    { 
      previous_x[(k-1)*n+i] = x[(k-1)*n+i];
    }
  }
  flush_output( iout );
  if( istop >= 1){ close_output( iout ); }
  return 0;
}
/* END OF FILE */
