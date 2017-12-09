/* 
 * File:   adc.h
 * Author: paulo
 *
 * Created on December 7, 2017, 1:43 AM
 */

#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

    
// ADC macros
// Detectors
#define FLD 6
#define SLD 7
#define FRD 4
#define SRD 5
#define NUM_DETECTORS 4
// Wall values
#define FRD_CLOSE 900 // GOOD
#define FLD_CLOSE 750 // kinda messed up
#define SRD_CLOSE 550 // GOOD
#define SLD_CLOSE 720 // GOOD
    
#define ADC_DT 1 // 1 ms
    
#define SELECT_DETECTOR(detector_id) (AD1CHS0bits.CH0SA = detector_id)
    
    
    /**
     * Setup all software modules required by ADC.
     * This includes the ADC software module, the interrupt to handle conversions.
     */
    void init_adc();
    
    /**
     * Turn on ADC. The ADC module should be self regulating. It handles its own
     * operations in an interrupt. The data captured from the ADC modules can be
     * read through volatile global variables in the adc.c file.
     */
    void enable_adc();
    
    /**
     * Stop all ADC operations
     */
    void disable_adc();



#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

