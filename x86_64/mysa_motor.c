/* Created by Language version: 7.5.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__mysa_motor
#define _nrn_initial _nrn_initial__mysa_motor
#define nrn_cur _nrn_cur__mysa_motor
#define _nrn_current _nrn_current__mysa_motor
#define nrn_jacob _nrn_jacob__mysa_motor
#define nrn_state _nrn_state__mysa_motor
#define _net_receive _net_receive__mysa_motor 
#define evaluate_fct evaluate_fct__mysa_motor 
#define states states__mysa_motor 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gkbar _p[0]
#define gl _p[1]
#define gq _p[2]
#define gkf _p[3]
#define ek _p[4]
#define el _p[5]
#define eq _p[6]
#define ekf _p[7]
#define ik _p[8]
#define il _p[9]
#define iq _p[10]
#define ikf _p[11]
#define s_inf _p[12]
#define q_inf _p[13]
#define n_inf _p[14]
#define tau_s _p[15]
#define tau_q _p[16]
#define tau_n _p[17]
#define s _p[18]
#define q _p[19]
#define n _p[20]
#define Ds _p[21]
#define Dq _p[22]
#define Dn _p[23]
#define q10_1 _p[24]
#define q10_2 _p[25]
#define q10_3 _p[26]
#define v _p[27]
#define _g _p[28]
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_Exp(void);
 static void _hoc_evaluate_fct(void);
 static void _hoc_vtrapNB(void);
 static void _hoc_vtrapNA(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_mysa_motor", _hoc_setdata,
 "Exp_mysa_motor", _hoc_Exp,
 "evaluate_fct_mysa_motor", _hoc_evaluate_fct,
 "vtrapNB_mysa_motor", _hoc_vtrapNB,
 "vtrapNA_mysa_motor", _hoc_vtrapNA,
 0, 0
};
#define Exp Exp_mysa_motor
#define vtrapNB vtrapNB_mysa_motor
#define vtrapNA vtrapNA_mysa_motor
 extern double Exp( _threadargsprotocomma_ double );
 extern double vtrapNB( _threadargsprotocomma_ double );
 extern double vtrapNA( _threadargsprotocomma_ double );
 /* declare global and static user variables */
#define anC anC_mysa_motor
 double anC = 1.1;
#define anB anB_mysa_motor
 double anB = -83.2;
#define anA anA_mysa_motor
 double anA = 0.0462;
#define aqC aqC_mysa_motor
 double aqC = -12.2;
#define aqB aqB_mysa_motor
 double aqB = -107.3;
#define aqA aqA_mysa_motor
 double aqA = 0.00522;
#define asC asC_mysa_motor
 double asC = -5;
#define asB asB_mysa_motor
 double asB = -27;
#define asA asA_mysa_motor
 double asA = 0.3;
#define bnC bnC_mysa_motor
 double bnC = 10.5;
#define bnB bnB_mysa_motor
 double bnB = -66;
#define bnA bnA_mysa_motor
 double bnA = 0.0824;
#define bqC bqC_mysa_motor
 double bqC = -12.2;
#define bqB bqB_mysa_motor
 double bqB = -107.3;
#define bqA bqA_mysa_motor
 double bqA = 0.00522;
#define bsC bsC_mysa_motor
 double bsC = -1;
#define bsB bsB_mysa_motor
 double bsB = 10;
#define bsA bsA_mysa_motor
 double bsA = 0.03;
#define vtraub vtraub_mysa_motor
 double vtraub = -80;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gkbar_mysa_motor", "mho/cm2",
 "gl_mysa_motor", "mho/cm2",
 "gq_mysa_motor", "mho/cm2",
 "gkf_mysa_motor", "mho/cm2",
 "ek_mysa_motor", "mV",
 "el_mysa_motor", "mV",
 "eq_mysa_motor", "mV",
 "ekf_mysa_motor", "mV",
 "ik_mysa_motor", "mA/cm2",
 "il_mysa_motor", "mA/cm2",
 "iq_mysa_motor", "mA/cm2",
 "ikf_mysa_motor", "mA/cm2",
 "tau_s_mysa_motor", "ms",
 "tau_q_mysa_motor", "ms",
 "tau_n_mysa_motor", "ms",
 0,0
};
 static double delta_t = 1;
 static double n0 = 0;
 static double q0 = 0;
 static double s0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "vtraub_mysa_motor", &vtraub_mysa_motor,
 "asA_mysa_motor", &asA_mysa_motor,
 "asB_mysa_motor", &asB_mysa_motor,
 "asC_mysa_motor", &asC_mysa_motor,
 "bsA_mysa_motor", &bsA_mysa_motor,
 "bsB_mysa_motor", &bsB_mysa_motor,
 "bsC_mysa_motor", &bsC_mysa_motor,
 "aqA_mysa_motor", &aqA_mysa_motor,
 "aqB_mysa_motor", &aqB_mysa_motor,
 "aqC_mysa_motor", &aqC_mysa_motor,
 "bqA_mysa_motor", &bqA_mysa_motor,
 "bqB_mysa_motor", &bqB_mysa_motor,
 "bqC_mysa_motor", &bqC_mysa_motor,
 "anA_mysa_motor", &anA_mysa_motor,
 "anB_mysa_motor", &anB_mysa_motor,
 "anC_mysa_motor", &anC_mysa_motor,
 "bnA_mysa_motor", &bnA_mysa_motor,
 "bnB_mysa_motor", &bnB_mysa_motor,
 "bnC_mysa_motor", &bnC_mysa_motor,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[0]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.5.0",
"mysa_motor",
 "gkbar_mysa_motor",
 "gl_mysa_motor",
 "gq_mysa_motor",
 "gkf_mysa_motor",
 "ek_mysa_motor",
 "el_mysa_motor",
 "eq_mysa_motor",
 "ekf_mysa_motor",
 0,
 "ik_mysa_motor",
 "il_mysa_motor",
 "iq_mysa_motor",
 "ikf_mysa_motor",
 "s_inf_mysa_motor",
 "q_inf_mysa_motor",
 "n_inf_mysa_motor",
 "tau_s_mysa_motor",
 "tau_q_mysa_motor",
 "tau_n_mysa_motor",
 0,
 "s_mysa_motor",
 "q_mysa_motor",
 "n_mysa_motor",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 29, _prop);
 	/*initialize range parameters*/
 	gkbar = 0.002581;
 	gl = 0.002;
 	gq = 0.002232;
 	gkf = 0.15074;
 	ek = -90;
 	el = -90;
 	eq = -54.9;
 	ekf = -90;
 	_prop->param = _p;
 	_prop->param_size = 29;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _mysa_motor_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 29, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 mysa_motor /Users/rjjarvis/Downloads/morteza/x86_64/mysa_motor.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Motor Axon MYSA channels";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[3], _dlist1[3];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Ds = ( s_inf - s ) / tau_s ;
   Dq = ( q_inf - q ) / tau_q ;
   Dn = ( n_inf - n ) / tau_n ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Ds = Ds  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_s )) ;
 Dq = Dq  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_q )) ;
 Dn = Dn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_n )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    s = s + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_s)))*(- ( ( ( s_inf ) ) / tau_s ) / ( ( ( ( - 1.0 ) ) ) / tau_s ) - s) ;
    q = q + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_q)))*(- ( ( ( q_inf ) ) / tau_q ) / ( ( ( ( - 1.0 ) ) ) / tau_q ) - q) ;
    n = n + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_n)))*(- ( ( ( n_inf ) ) / tau_n ) / ( ( ( ( - 1.0 ) ) ) / tau_n ) - n) ;
   }
  return 0;
}
 
static int  evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   double _la , _lb , _lv2 ;
 _lv2 = _lv - vtraub ;
   _la = q10_3 * asA / ( Exp ( _threadargscomma_ ( _lv2 + asB ) / asC ) + 1.0 ) ;
   _lb = q10_3 * bsA / ( Exp ( _threadargscomma_ ( _lv2 + bsB ) / bsC ) + 1.0 ) ;
   tau_s = 1.0 / ( _la + _lb ) ;
   s_inf = _la / ( _la + _lb ) ;
   _la = q10_3 * aqA * ( Exp ( _threadargscomma_ ( _lv - aqB ) / aqC ) ) ;
   _lb = q10_3 * bqA / ( Exp ( _threadargscomma_ ( _lv - bqB ) / bqC ) ) ;
   tau_q = 1.0 / ( _la + _lb ) ;
   q_inf = _la / ( _la + _lb ) ;
   _la = q10_3 * vtrapNA ( _threadargscomma_ _lv ) ;
   _lb = q10_3 * vtrapNB ( _threadargscomma_ _lv ) ;
   tau_n = 1.0 / ( _la + _lb ) ;
   n_inf = _la / ( _la + _lb ) ;
    return 0; }
 
static void _hoc_evaluate_fct(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 evaluate_fct ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double vtrapNA ( _threadargsprotocomma_ double _lx ) {
   double _lvtrapNA;
 if ( fabs ( ( anB - _lx ) / anC ) < 1e-6 ) {
     _lvtrapNA = anA * anC ;
     }
   else {
     _lvtrapNA = anA * ( v - anB ) / ( 1.0 - Exp ( _threadargscomma_ ( anB - v ) / anC ) ) ;
     }
   
return _lvtrapNA;
 }
 
static void _hoc_vtrapNA(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  vtrapNA ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double vtrapNB ( _threadargsprotocomma_ double _lx ) {
   double _lvtrapNB;
 if ( fabs ( ( _lx - bnB ) / bnC ) < 1e-6 ) {
     _lvtrapNB = bnA * bnC ;
     }
   else {
     _lvtrapNB = bnA * ( bnB - v ) / ( 1.0 - Exp ( _threadargscomma_ ( v - bnB ) / bnC ) ) ;
     }
   
return _lvtrapNB;
 }
 
static void _hoc_vtrapNB(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  vtrapNB ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double Exp ( _threadargsprotocomma_ double _lx ) {
   double _lExp;
 if ( _lx < - 100.0 ) {
     _lExp = 0.0 ;
     }
   else {
     _lExp = exp ( _lx ) ;
     }
   
return _lExp;
 }
 
static void _hoc_Exp(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  Exp ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 3;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 3; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  n = n0;
  q = q0;
  s = s0;
 {
   q10_1 = pow( 2.2 , ( ( celsius - 20.0 ) / 10.0 ) ) ;
   q10_2 = pow( 2.9 , ( ( celsius - 20.0 ) / 10.0 ) ) ;
   q10_3 = pow( 3.0 , ( ( celsius - 36.0 ) / 10.0 ) ) ;
   evaluate_fct ( _threadargscomma_ v ) ;
   s = s_inf ;
   q = q_inf ;
   n = n_inf ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   ik = gkbar * s * ( v - ek ) ;
   il = gl * ( v - el ) ;
   iq = gq * q * ( v - eq ) ;
   ikf = gkf * n * n * n * n * ( v - ekf ) ;
   }
 _current += ik;
 _current += il;
 _current += iq;
 _current += ikf;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(s) - _p;  _dlist1[0] = &(Ds) - _p;
 _slist1[1] = &(q) - _p;  _dlist1[1] = &(Dq) - _p;
 _slist1[2] = &(n) - _p;  _dlist1[2] = &(Dn) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
