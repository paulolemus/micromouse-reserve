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
// Wall values
#define FRD_CLOSE 900
#define FLD_CLOSE
#define SRD_CLOSE 750
#define SLD_CLOSE 800
    
    
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

