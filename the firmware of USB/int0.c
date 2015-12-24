#include "fx2.h"
#include "fx2regs.h"

extern BOOL hpi_int;

void int0_isr (void) interrupt 0
{ 
  hpi_int = TRUE; // HPI interrupted the FX2
  EX0 = 0;        // disable INT0/ interrupt, let foreground re-enable it
}