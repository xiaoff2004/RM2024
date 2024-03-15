#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#include "main.h"

typedef struct
{
	uint8_t flag;
	uint32_t last_time;
}watch_dog_t;

extern watch_dog_t Remote_WatchDog_Info;
//extern watch_dog_t Can1_WatchDog_Info;
//extern watch_dog_t Can2_WatchDog_Info;

void WatchDogInfo_Init(void);
void WatchDogInfo_Updata(watch_dog_t *WatchDog);
uint8_t WatchDog_Detect(watch_dog_t *WatchDog);

#endif
