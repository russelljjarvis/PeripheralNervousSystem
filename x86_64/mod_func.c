#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _flut_motor_reg(void);
extern void _flut_sensory_reg(void);
extern void _hd_reg(void);
extern void _hd__vhalflmin73_reg(void);
extern void _hdmin73_reg(void);
extern void _kad_reg(void);
extern void _kap_reg(void);
extern void _kdr_reg(void);
extern void _mysa_motor_reg(void);
extern void _mysa_sensory_reg(void);
extern void _na3_reg(void);
extern void _nax_reg(void);
extern void _node_motor_reg(void);
extern void _node_sensory_reg(void);
extern void _pas_nml2_reg(void);
extern void _stin_motor_reg(void);
extern void _stin_sensory_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," flut_motor.mod");
    fprintf(stderr," flut_sensory.mod");
    fprintf(stderr," hd.mod");
    fprintf(stderr," hd__vhalflmin73.mod");
    fprintf(stderr," hdmin73.mod");
    fprintf(stderr," kad.mod");
    fprintf(stderr," kap.mod");
    fprintf(stderr," kdr.mod");
    fprintf(stderr," mysa_motor.mod");
    fprintf(stderr," mysa_sensory.mod");
    fprintf(stderr," na3.mod");
    fprintf(stderr," nax.mod");
    fprintf(stderr," node_motor.mod");
    fprintf(stderr," node_sensory.mod");
    fprintf(stderr," pas_nml2.mod");
    fprintf(stderr," stin_motor.mod");
    fprintf(stderr," stin_sensory.mod");
    fprintf(stderr, "\n");
  }
  _flut_motor_reg();
  _flut_sensory_reg();
  _hd_reg();
  _hd__vhalflmin73_reg();
  _hdmin73_reg();
  _kad_reg();
  _kap_reg();
  _kdr_reg();
  _mysa_motor_reg();
  _mysa_sensory_reg();
  _na3_reg();
  _nax_reg();
  _node_motor_reg();
  _node_sensory_reg();
  _pas_nml2_reg();
  _stin_motor_reg();
  _stin_sensory_reg();
}
