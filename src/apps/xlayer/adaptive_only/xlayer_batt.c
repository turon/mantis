#include "mutex.h" 
#include "net.h"
#include "node_id.h"
#include "mos.h"
#include "msched.h"
#include "cc2420.h"
#include "inttypes.h"
#include "dev.h"
#include "printf.h"
#include "cond.h"
#include "led.h"
#include "clock.h"
#include "transport.h"
#include <stdlib.h>
#include <string.h>
#include "ctp_plus.h"

#include "msp-adc.h"
#define INTERNAL_VOLTAGE 11
#define USE_LOCAL_DRIVER_VERSION  


void appSend () {
	static uint8_t smpl_cnt = 10;  //number of samples we take to average on each channel
	uint8_t  count = 0;            //temp counter for samples taken
	uint8_t  i;
	uint8_t  nchannels=1;
	uint8_t  channels[1] = {INTERNAL_VOLTAGE};  //use only internal voltage for now  

	uint16_t adc_raw[8];
	uint8_t  ledcount = 0;
	uint8_t  ix;

	char     sendData[12];
	char     testStr[] = "test";
	char     iStr[3];
	uint16_t pause_time = 5000;

	// Avoid signal interference with default channel (26) and wi-fi 
	cc2420_set_channel(26);

	// Set transmit power to low-power mode 
	//com_ioctl(IFACE_RADIO, CC2420_LOW_POWER_MODE);
	com_ioctl_IFACE_RADIO (CC2420_TX_POWER, 30); //set the tx power from 0 (min) to 31 (max)  

    printf("Opening connection on port %d... \n", TRANSPORT_LISTENING_PORT);
    connect(TRANSPORT_LISTENING_PORT, 0);
    printf("Opened connection on port %d \n", TRANSPORT_LISTENING_PORT);
	
	//mos_thread_sleep(2000);
	while (true) {
		//--- MEASURING BATTERY VOLTAGE ---// 
		
		//mos_led_display( (ledcount++)%8 );  //see explanation in test_adc.c
		adc_on();  //turn on the ADC/Vref
		mos_thread_sleep(20);  //time to wait for the internal reference to settle in theory 

		// Measure each channel, possibly with multiple samples
		for(ix=0; ix<nchannels; ix++) {
			//take 10 samples and average the result
			//split this value out as the final channel reading
			count = 0;  //clear the samples counter
		 	adc_raw[ix]=0;  // clear adc reading accum

            //For this channel, do as many samples as are 'requested'
		    while(count < smpl_cnt) {
		    	adc_raw[ix] += adc_get_conversion16( channels[ix] );
	            count++;  //increment the no. of readings counter
	        }   //end of samples loop for this channel
		} //end channels loop 

		// Average the data for the samples taken on each channel
		for( ix=0; ix<nchannels; ix++ ) {
	    	adc_raw[ix] /= smpl_cnt;
        }

        // Report the results 
	   	//printf("Raw voltage reading = %x\n", adc_raw[0]); 
	    
		adc_off();
	    //mos_thread_sleep(4000);

		
		//--- SENDING DATA ---// 
		
		memset(sendData, 0, sizeof(sendData));
		strcpy(sendData, testStr);
		itoa(i, iStr, 10);
		strcat(sendData, iStr);

		printf("Calling sendPacket() for: %s \n", sendData);
		sendPacket(TRANSPORT_LISTENING_PORT, sendData, sizeof(sendData), 0);

		i++;
		// Reset counter
	    if (i >= 60) {
	    	i = 0;
	    }
		
		// Get raw voltage reading 
		printf("***** Raw voltage reading = %x *****\n", adc_raw[0]); 
		
		// Adjust pause time (send rate) for each specific voltage 
		if (adc_raw[0] > 0x950) {
			pause_time = 5000; 
		} else if (adc_raw[0] > 0x925) {
			pause_time = 7500;
		} else if (adc_raw[0] > 0x900) {
			pause_time = 10000;
		}
		// and so on... 
		// need to find the optimal values ... 
		
		mos_thread_sleep(pause_time); //adaptive send rate 
		
		// Close connection to destination (last) node; port 0
		closeConn(0, 3);
	    printf("Closed connection on port %d \n", TRANSPORT_LISTENING_PORT);
	}
}


#ifdef USE_LOCAL_DRIVER_VERSION
#ifdef PLATFORM_TELOSB
#define PULSE_SAMPLING_MODE  //this is the default in any event

void adc_init()
{
	ADC12CTL0 &= ~ENC;  // Clear the ENC bit to allow setting register bits
	ADC12IE = 0;	    // In this MOS application, don't use Interrupt Enable/Vectors
	P6SEL = 0xFF;	    // Disable Port6 Pin buffers for ADC in to elim. parasitics
					    // Note P6 Pins 6,7 have multiple use so these may need to
					    // be set/reset elsewhere.
					    // Should really just set the ones we're going to use

	// In this MOS application use the Internal Reference Voltage
	// The actual turn-on of the Vref is done in 'adc_on();' via REFON of CTL0 register.
	// Here we select set whether it will be Vcc (ie battery/supply) or the internal
	// reference of 2.5 or 1.5 volts

#ifndef VREF_Vcc
#ifdef VREF_1_5
	ADC12CTL0 &= ~REF2_5V;	//use 1.5V Vref, bit=0
#else
	ADC12CTL0 |= REF2_5V;	//use 2.5V Vref, bit=1: default setting
#endif
	// Not only do we need to enable the internal vref via the VREF_2
	// but also the conversion memory control registers
	
	ADC12MCTL0 |= SREF_1;	// This is for Vref+ to AVss on TelosB
							// Setting the '2.5' or '1.5' is done
#else						// If VREF_Vcc is defined all we need is to set it in memctl
	ADC12MCTL0 |= SREF_0;	// This is for Vcc to Gnd on TelosB
#endif	// VREF_Vcc

	// Sampling Mode: 1 of 2 options available
#ifdef EXTENDED_SAMPLING_MODE
   ADC12CTL1 &= ~SHP;
#else		// PULSE_SAMPLING_MODE is default
   ADC12CTL1 |= SHP;
#endif
}


uint8_t adc_channel;

void adc_set_channel(uint8_t ch)
{
   uint8_t addr = 0;

   //set the addr where conversion result will reside
   ADC12CTL1 |= ((addr & 0x0F) << 11);

   // set the addr to read from the corresponding channel
   // NOTE this does not set the SREF bits of the ADC12MCTLx
   // to select the reference source:....do that in the init routine
   // or elsewhere.  This method retains those bits set somewhere else
   ADC12MCTL0 &= 0xF0;		 // Clear out the previous channel setting
   ADC12MCTL0 |= (ch &0xF);	 // and add in the new setting
}


uint16_t adc_get_conversion16(uint8_t ch)
{
   ADC12CTL0 &= ~ENC;	// clear the ENC bit to allow setting control registers

   adc_set_channel(ch);
   
   //start the conversion
   ADC12CTL0 |= ENC | ADC12SC;

   //polling wait for conversion to complete
   while(ADC12CTL1 & ADC12BUSY);

   uint16_t sample = ADC12MEM0;
   return sample;
}

#endif
#endif // USE_LOCAL_DRIVER_VERSION


void start(void) 
{
	mos_node_id_set(6);
	transportInit(false);
	net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);

	mos_thread_new (appSend, 384, PRIORITY_NORMAL);
}

