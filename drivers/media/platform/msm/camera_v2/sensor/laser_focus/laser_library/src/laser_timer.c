/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "laser_timer.h"
#include "laser_log.h"

/** @brief Display current time
*	
*/
void O_get_current_time(struct timeval* now){
	do_gettimeofday(now);
}
struct timeval get_current_time(void){
	struct timeval now;
	
	do_gettimeofday(&now);

	return now;
}

/** @brief Check if timeout happen (ms) 
*	
*	@param start the start time
*	@param now the current time
*	@param timeout the timeout value(ms)
*
*/
bool is_timeout(struct timeval start, struct timeval now, int timeout){

       if((((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec)) > (timeout*1000)){
              LOG_Handler(LOG_ERR, "%s: Timeout!!\n", __func__);
        	return true;
       }
	return false;
}

void DeltaTime_ms(struct timeval start, struct timeval now, int *deltaT){

	*deltaT =( ((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec))/1000;
		
}
