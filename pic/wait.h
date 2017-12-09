/* 
 * File:   wait.h
 * Author: paulo
 *
 * Created on December 8, 2017, 10:07 PM
 */

#ifndef WAIT_H
#define	WAIT_H

#ifdef	__cplusplus
extern "C" {
#endif


    /**
     * Wait holds in the function until the time ends.
     * @param ms
     */
    void wait_ms(const unsigned int ms);
    
    /**
     * Flips global variable when the timer is finished.
     * @param ms
     */
    void nonblock_wait_ms(const unsigned int ms);
    


#ifdef	__cplusplus
}
#endif

#endif	/* WAIT_H */

