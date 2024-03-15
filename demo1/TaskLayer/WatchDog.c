#include "WatchDog.h"
#include "main.h"


watch_dog_t Remote_WatchDog_Info;
//watch_dog_t Can1_WatchDog_Info;
//watch_dog_t Can2_WatchDog_Info;

/**
 * \brief 看门狗初始化
 */
void WatchDogInfo_Init(void)
{
	WatchDogInfo_Updata(&Remote_WatchDog_Info);
//	WatchDogInfo_Updata(&Can1_WatchDog_Info);
//	WatchDogInfo_Updata(&Can2_WatchDog_Info);
}

/**
 * \brief 看门狗数据更新
 * \param WatchDog 看门狗结构体（Remote_WatchDog_Info、Can1_WatchDog_Info、Can2_WatchDog_Info）
 */
void WatchDogInfo_Updata(watch_dog_t *WatchDog)
{
	WatchDog->flag=1;
	WatchDog->last_time=uwTick;
}

/**
 * \brief 看门狗超时判断
 * \param WatchDog 看门狗结构体（Remote_WatchDog_Info、Can1_WatchDog_Info、Can2_WatchDog_Info）
 * \return 看门狗标志位
 */
uint8_t WatchDog_Detect(watch_dog_t *WatchDog)
{
	if(uwTick-WatchDog->last_time >2000)
		WatchDog->flag=0;
	return WatchDog->flag;
}
