/* 
 * File:   oscillator.h
 * Author: paulo
 *
 * Created on December 7, 2017, 12:56 AM
 */

#ifndef OSCILLATOR_H
#define	OSCILLATOR_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#define SYS_FREQ 32000000

    
    /**
     * @brief Set up the primary oscillator of the system.
     *        The system is currently set to 32MHz, where max is 40MHz.
     */
    void init_oscillator();


#ifdef	__cplusplus
}
#endif

#endif	/* OSCILLATOR_H */

