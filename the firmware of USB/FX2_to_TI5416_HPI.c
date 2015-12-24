#pragma NOIV               // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:       FX2_to_extsyncFIFO.c
//   Contents:   Hooks required to implement FX2 GPIF interface to a TI
//               5416 DSP via it's HPI (Host Port Interface)
//
//   Copyright (c) 2002 Cypress Semiconductor, Inc. All rights reserved
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"            // SYNCDELAY macro, see Section 15.14 of FX2 Tech.
                                // Ref. Manual for usage details.

#define HPI_RDY       GPIFREADYSTAT & bmBIT0 // RDY0
#define LED_ALL       (bmBIT0 | bmBIT1 | bmBIT2 | bmBIT3)
#define bmEP0BSY      0x01
#define bmEP1OUTBSY   0x02
#define bmEP1INBSY    0x04

#define bmHPIC        0x00 // HCNTL[1:0] = 00
#define bmHPID_AUTO   0x04 // HCNTL[1:0] = 01
#define bmHPIA        0x08 // HCNTL[1:0] = 10
#define bmHPID_MANUAL 0x0C // HCNTL[1:0] = 11

#define GPIFTRIGRD 4

#define GPIF_EP2 0
#define GPIF_EP4 1
#define GPIF_EP6 2
#define GPIF_EP8 3

extern BOOL GotSUD;               // Received setup data flag
extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

BYTE Configuration;               // Current configuration
BYTE AlternateSetting;            // Alternate settings
static WORD xdata LED_Count = 0;
static BYTE xdata LED_Status = 0;
BOOL in_enable = FALSE;           // flag to enable IN transfers
BOOL hpi_int = FALSE;             // HPI interrupt flag
static WORD xdata Tcount = 0;     // transaction count
BOOL enum_high_speed = FALSE;     // flag to let firmware know FX2 enumerated at high speed
static WORD xFIFOBC_IN = 0x0000;  // variable that contains EP6FIFOBCH/L value

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
// The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------
void LED_Off (BYTE LED_Mask);
void LED_On (BYTE LED_Mask);
void GpifInit ();

void GPIF_SingleByteWrite (BYTE gdata)
{
  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }

  XGPIFSGLDATLX = gdata;         // trigger GPIF Single Byte Write transaction
}


void GPIF_SingleWordWrite (BYTE gdath,BYTE gdatl)
{
  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }
  XGPIFSGLDATH	= gdath;
  XGPIFSGLDATLX = gdatl;         // trigger GPIF Single Byte Write transaction
 }

void TD_Init(void)             // Called once at startup
{
  // set the CPU clock to 48MHz
  CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1);
  SYNCDELAY;

  EP1OUTCFG = 0xA0;  // always OUT, valid, bulk
  EP1INCFG = 0xA0;   // always IN, valid, bulk
  SYNCDELAY;
  EP2CFG = 0xA0;     // EP2OUT, bulk, size 512, 4x buffered
  SYNCDELAY;                    
  EP4CFG = 0x00;     // EP4 not valid
  SYNCDELAY;                    
  EP6CFG = 0xE0;     // EP6IN, bulk, size 512, 4x buffered
  SYNCDELAY;                    
  EP8CFG = 0x00;     // EP8 not valid
  SYNCDELAY;

  FIFORESET = 0x80;  // set NAKALL bit to NAK all transfers from host
  SYNCDELAY;
  FIFORESET = 0x02;  // reset EP2 FIFO
  SYNCDELAY;
  FIFORESET = 0x06;  // reset EP6 FIFO
  SYNCDELAY;
  FIFORESET = 0x00;  // clear NAKALL bit to resume normal operation
  SYNCDELAY;

  EP2FIFOCFG = 0x01; // allow core to see zero to one transition of auto out bit
  SYNCDELAY;
  EP2FIFOCFG = 0x11; // auto out mode, disable PKTEND zero length send, byte ops
  SYNCDELAY;
  EP6FIFOCFG = 0x09; // auto in mode, disable PKTEND zero length send, byte ops
  SYNCDELAY;
  EP1OUTBC = 0x00;   // arm EP1OUT by writing any value to EP1OUTBC register

  GpifInit ();       // initialize GPIF registers  
   
  PORTACFG = bmBIT0; // PA0 takes on INT0/ alternate function
  OEA  |= 0x1C;      // initialize PA3 and PA2 port i/o pins as outputs	 
  OEC |= 0x03;		 //定义PC0和PC1作为输出引脚
  EX0 = 1;           // Enable INT0/ interrupt
  IT0 = 1;           // Detect INT0/ on falling edge
}

void TD_Poll(void)
{

  if( GPIFTRIG & 0x80 )              // if GPIF interface IDLE
  {
    if ( ! ( EP24FIFOFLGS & 0x02 ) ) // if there's a packet in the peripheral domain for EP2
    {
      IOA = bmHPID_AUTO;             // select HPID register with address auto-increment
	  while(!HPI_RDY);               // wait for HPI to complete internal portion of previous transfer
    
	  SYNCDELAY;
      GPIFTCB1 = EP2FIFOBCH;         // setup transaction count with number of bytes in the EP2 FIFO
      SYNCDELAY;
      GPIFTCB0 = EP2FIFOBCL;
      SYNCDELAY;
      GPIFTRIG = GPIF_EP2;           // launch GPIF FIFO WRITE Transaction from EP2 FIFO
      SYNCDELAY;

   	  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 GPIF Done bit
      {
        ;
      }
      SYNCDELAY;      
    
    }
  }
    
  if(in_enable)                             // if IN transfers are enabled,
  {     
    if(Tcount)                              // if Tcount is not zero
	{
      if( GPIFTRIG & 0x80 )                 // if GPIF interface IDLE
      {
        if( !( EP68FIFOFLGS & 0x01 ) )      // if EP6 FIFO is not full
        {
	      IOA = bmHPID_AUTO;                // select HPID register with address auto-increment              
          while(!HPI_RDY);                  // wait for HPI to complete internal portion of previous transfer
        	   
          SYNCDELAY;
	      GPIFTCB1 = MSB(Tcount);           // setup transaction count with Tcount value     
          SYNCDELAY;
          GPIFTCB0 = LSB(Tcount);
          SYNCDELAY;   

          GPIFTRIG = GPIFTRIGRD | GPIF_EP6; // launch GPIF FIFO READ Transaction to EP6IN
	      SYNCDELAY;

	      while( !( GPIFTRIG & 0x80 ) )     // poll GPIFTRIG.7 GPIF Done bit
          {
            ;
          }
    
	      SYNCDELAY;

		  xFIFOBC_IN = ( ( EP6FIFOBCH << 8 ) + EP6FIFOBCL ); // get EP6FIFOBCH/L value

		  if( xFIFOBC_IN < 0x0200 )         // if pkt is short,
		  {
		    INPKTEND = 0x06;                // force a commit to the host
          }
		  Tcount = 0;                       // set Tcount to zero to cease reading from DSP HPI RAM
        }    		    
      }
    }
  }  

  if(!(EP01STAT & bmEP1OUTBSY))
  {
    // handle OUTs to EP1OUT
  }

  if(!(EP01STAT & bmEP1INBSY))
  {
    // handle INs to EP1IN
  }

  if (hpi_int)
  {
    hpi_int = FALSE; // clear HPI interrupt flag
	EX0 = 1;         // enable INT0 interrupt again
	LED_On (bmBIT1); // turn on LED1 to alert user HPI interrupt occurred
  }
  
  // blink LED0 to indicate firmware is running

  if (++LED_Count == 10000)
  {
    if (LED_Status)
    {
      LED_Off (bmBIT0);
      LED_Status = 0;
    }
    else
    {
      LED_On (bmBIT0);
      LED_Status = 1;
    }
    LED_Count = 0;
  }

}

BOOL TD_Suspend(void)          // Called before the device goes into suspend mode
{
   return(TRUE);
}

BOOL TD_Resume(void)          // Called after the device resumes
{
   return(TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}

BOOL DR_SetConfiguration(void)  // Called when a Set Configuration command is received
{

  if( EZUSB_HIGHSPEED( ) )
  { // FX2 enumerated at high speed
    SYNCDELAY;                  
    EP6AUTOINLENH = 0x02;       // set AUTOIN commit length to 512 bytes
    SYNCDELAY;                  
    EP6AUTOINLENL = 0x00;
    SYNCDELAY;                  
    enum_high_speed = TRUE;
  }
  else
  { // FX2 enumerated at full speed
    SYNCDELAY;                   
    EP6AUTOINLENH = 0x00;       // set AUTOIN commit length to 64 bytes
    SYNCDELAY;                   
    EP6AUTOINLENL = 0x40;
    SYNCDELAY;                  
    enum_high_speed = FALSE;
  }
  
  Configuration = SETUPDAT[2];
  return(TRUE);                 // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{
   AlternateSetting = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}

#define VX_B2 0xB2 // turn off LED1
#define VX_B3 0xB3 // enable IN transfers
#define VX_B4 0xB4 // disable IN transfers
#define VX_B5 0xB5 // set Tcount value
#define VX_B6 0xB6 // write to HPIC register
#define VX_B7 0xB7 // write to HPIA register
#define VX_B8 0xB8 // reset EP6 FIFO
#define VX_B9 0xB9 // read GPIFTRIG register
#define VX_BA 0xBA // read GPIFTC registers
#define	VX_BB 0xBB
BOOL DR_VendorCmnd(void)
{

  switch (SETUPDAT[1])
  {
    case VX_B2: // turn off LED1
    {       
      LED_Off (bmBIT1);

      *EP0BUF = VX_B2;
	  EP0BCH = 0;
	  EP0BCL = 1;                   // Arm endpoint with # bytes to transfer
	  EP0CS |= bmHSNAK;             // Acknowledge handshake phase of device request
      break;
    }
	case VX_B3: // enable IN transfers
	{
	  in_enable = TRUE;

      *EP0BUF = VX_B3;
  	  EP0BCH = 0;
	  EP0BCL = 1;
	  EP0CS |= bmHSNAK;
	  break;
    }
	case VX_B4: // disable IN transfers
	{
	  in_enable = FALSE;

      *EP0BUF = VX_B4;
  	  EP0BCH = 0;
	  EP0BCL = 1;
	  EP0CS |= bmHSNAK;
	  break;
    }
	case VX_B5: // set Tcount value
	{
	  EP0BCL = 0;
	  while(EP01STAT & bmEP0BSY);             // wait until EP0 is available to be accessed by CPU
	  Tcount = (EP0BUF[0] << 8) + EP0BUF[1];  // load transaction count with EP0 values
	  
	  break;
    }
	case VX_B6: // write to HPIC register
	{	  
	  EP0BCL = 0;                         // re-arm EP0
	  while(EP01STAT & bmEP0BSY);         // wait until EP0 is available to be accessed by CPU
	  while(!HPI_RDY);                    // wait for HPI to complete internal portion of previous transfer
      IOA = bmHPIC;                       // select HPIC register
	  GPIFWFSELECT = 0x1E;                // point to waveforms that write first byte of HPI protocol
     // GPIF_SingleByteWrite(EP0BUF[0]);    // write LSB of DSP address
	  GPIF_SingleWordWrite(EP0BUF[1],EP0BUF[0]);
	  GPIFWFSELECT = 0x4E;                // point to waveforms that write second byte of HPI protocol
	 // GPIF_SingleByteWrite(EP0BUF[1]);    // write MSB of DSP address      	  
	  GPIF_SingleWordWrite(EP0BUF[3],EP0BUF[2]);
	  break;
    }
	case VX_B7: // write to HPIA register
	{	  
	  EP0BCL = 0;                         // re-arm EP0
	  while(EP01STAT & bmEP0BSY);         // wait until EP0 is available to be accessed by CPU
	  while(!HPI_RDY);                    // wait for HPI to complete internal portion of previous transfer
      IOA = bmHPIA;                       // select HPIA register
	  GPIFWFSELECT = 0x1E;                // point to waveforms that write first byte of HPI protocol
     // GPIF_SingleByteWrite(EP0BUF[0]);    // write LSB of DSP address
	  GPIF_SingleWordWrite(EP0BUF[1],EP0BUF[0]);
	  GPIFWFSELECT = 0x4E;                // point to waveforms that write second byte of HPI protocol
	  //GPIF_SingleByteWrite(EP0BUF[1]);    // write MSB of DSP address      	
	  GPIF_SingleWordWrite(EP0BUF[3],EP0BUF[2]);
	  break;
    }
	case VX_B8: // reset EP6 FIFO
	{	  
	  FIFORESET = 0x80; // set NAKALL bit to NAK all transfers from host
      SYNCDELAY;
      FIFORESET = 0x06; // reset EP6 FIFO
      SYNCDELAY;
      FIFORESET = 0x00; // clear NAKALL bit to resume normal operation
      SYNCDELAY;

      *EP0BUF = VX_B8;
  	  EP0BCH = 0;
	  EP0BCL = 1;
	  EP0CS |= bmHSNAK;

	  break;
    }
	case VX_B9: // read GPIFTRIG register
	{	  
	  EP0BUF[0] = VX_B9;
	  EP0BUF[1] = GPIFTRIG;
  	  EP0BCH = 0;
	  EP0BCL = 2;
	  EP0CS |= bmHSNAK;
	  break;
    }
	case VX_BA: // read GPIFTC registers
	{	  
	  EP0BUF[0] = VX_BA;
	  EP0BUF[1] = GPIFTCB1;
	  EP0BUF[2] = GPIFTCB0;
  	  EP0BCH = 0;
	  EP0BCL = 3;
	  EP0CS |= bmHSNAK;
	  break;
    }
	case VX_BB: // 		产生reset，写入0xFFE701*************
	{
	  EP0BCL = 0;                         // re-arm EP0
	  while(!HPI_RDY);
	  GPIFCTLCFG=0x00;
	  IOA = 0x10;
	  GPIFIDLECTL=0x02;		    
	  GPIFWFSELECT = 0x1E; 	 
	  GPIF_SingleWordWrite(EP0BUF[1],EP0BUF[0]);
	  IOA=0x00;
	  break;
    }
    default:
        return(TRUE);
  }

  return(FALSE);
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) interrupt 0
{
   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
   // whenever we get a USB reset, we should revert to full speed mode
   pConfigDscr = pFullSpeedConfigDscr;
   ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
   pOtherConfigDscr = pHighSpeedConfigDscr;
   ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
   Sleep = TRUE;
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
      pOtherConfigDscr = pFullSpeedConfigDscr;
      ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;
   }

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}
void ISR_Ep2inout(void) interrupt 0
{
}
void ISR_Ep4inout(void) interrupt 0
{
}
void ISR_Ep6inout(void) interrupt 0
{
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}

// ...debug LEDs: accessed via movx reads only ( through CPLD )
// it may be worth noting here that the default monitor loads at 0xC000
xdata volatile const BYTE LED0_ON  _at_ 0x8000;
xdata volatile const BYTE LED0_OFF _at_ 0x8100;
xdata volatile const BYTE LED1_ON  _at_ 0x9000;
xdata volatile const BYTE LED1_OFF _at_ 0x9100;
xdata volatile const BYTE LED2_ON  _at_ 0xA000;
xdata volatile const BYTE LED2_OFF _at_ 0xA100;
xdata volatile const BYTE LED3_ON  _at_ 0xB000;
xdata volatile const BYTE LED3_OFF _at_ 0xB100;
// use this global variable when (de)asserting debug LEDs...
BYTE xdata ledX_rdvar = 0x00;
BYTE xdata LED_State = 0;
void LED_Off (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_OFF;
		LED_State &= ~bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_OFF;
		LED_State &= ~bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_OFF;
		LED_State &= ~bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_OFF;
		LED_State &= ~bmBIT3;
	}
}

void LED_On (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_ON;
		LED_State |= bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_ON;
		LED_State |= bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_ON;
		LED_State |= bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_ON;
		LED_State |= bmBIT3;
	}
}

