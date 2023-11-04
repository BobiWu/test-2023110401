#include <stdbool.h>
#include "MC3416.h"
#include "esp_log.h"

//static const char *TAG = "mc3416";

#define	Imm_GrabItPosition_DebounceTime_First		1
#define	Imm_GrabItPosition_DebounceTime_AfterFirst	160//100

static 	uint16_t U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_First;

static  uint8_t	U8_GrabItPosition=0;

#define	Imm_Array_Number																(5+1)
static 	int16_t	U16_t_GrabItPosition_OnDebounce[Imm_Array_Number]={0};


//-------------------------New shake----------------------------
#define	Imm_Acce_Range	4//2	//2g=1	4g=2  8g=4  16g=8	//6663
						//4g		//8g		//4g 		//2g
#define Imm_NewShakeGSensorPulseValue	(20000)	//30000		//(20000)	//20000		//Before1207 //20000
#define Imm_NewShakeGSensorPulseTime	(30/5)	//50		//(30)		//30
static		uint16_t		U16_HoldAndDelayCheckNewShake=0; 

static		uint16_t		U16_NewShakeCounter=0;
static		uint16_t		U16_NewShakeTimeDelay=0;
static		uint16_t		U16_CountDownAndClearShakeRam=0;

static		uint16_t		U16_NewShakeZoutZhengOnDebounce=0;
static		uint16_t		U16_NewShakeZoutZhengOffDebounce=0;
static		uint16_t		U16_NewShakeZoutFanOnDebounce=0;
static		uint16_t		U16_NewShakeZoutFanOffDebounce=0;

static		uint16_t		U16_NewShakeYoutZhengOnDebounce=0;
static		uint16_t		U16_NewShakeYoutZhengOffDebounce=0;
static		uint16_t		U16_NewShakeYoutFanOnDebounce=0;
static		uint16_t		U16_NewShakeYoutFanOffDebounce=0;

static		uint16_t		U16_NewShakeXoutZhengOnDebounce=0;
static		uint16_t		U16_NewShakeXoutZhengOffDebounce=0;
static		uint16_t		U16_NewShakeXoutFanOnDebounce=0;
static		uint16_t		U16_NewShakeXoutFanOffDebounce=0;

static 		uint16_t 		U16_DelayAndThenCkNewShake=0;

static  	uint8_t			U8_PlayShakeHappen=0;

#define		Imm_ShakeUpDownTimes	6
//-------------------------New shake----------------------------

//----------------Grab it Position----------------
uint8_t	 CheckGrabItPosition(int16_t xAxisData, int16_t yAxisData, int16_t zAxisData)//在这个文件里面调用
{
	int U8_Counter = 0;

  //ESP_LOGI(TAG, "MC3416 x = %d      y = %d      z = %d", xAxisData, yAxisData, zAxisData);   
  
	//------------Check +X-------------			
  if(
      ((xAxisData>=(+4095-1002)))										&&
      ((yAxisData>=(0-1002))&&(yAxisData<(0+1002)))&&
      ((zAxisData>=(0-1002))&&(zAxisData<(0+1002)))
    )
  {
    for(U8_Counter = 0; U8_Counter < Imm_Array_Number; U8_Counter ++)
    {//clear 0
      if(U8_Counter!=(Imm_GrabItPosition_Xz))
      {
      	U16_t_GrabItPosition_OnDebounce[U8_Counter]=0;
      }
    }
    if(U8_GrabItPosition!=Imm_GrabItPosition_Xz)
    {
      U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xz]++;
      if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xz]>=U16_t_GrabItPosition_DebounceTime)
      {//Found Blue
      	U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_AfterFirst;
      	U8_GrabItPosition=Imm_GrabItPosition_Xz;
      	U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xz]=0;
        //ESP_LOGI(TAG, "MC3416 Position = %d", U8_GrabItPosition);         	
      }
    }
    else
    {
        U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xz] ++;
        if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xz] >= 200)
        {
            U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xz] = 0;
        }
    }
  }
	//------------Check +X-------------								

	//------------Check -X-------------										
  if(
      ((xAxisData< (-4095+1002)))										 &&
      ((yAxisData>=(0-1002))&&(yAxisData<(0+1002)))&&
      ((zAxisData>=(0-1002))&&(zAxisData<(0+1002)))
    )
  {
    for(U8_Counter = 0; U8_Counter < Imm_Array_Number; U8_Counter ++)
    {//clear 0
      if(U8_Counter!=(Imm_GrabItPosition_Xf))
      {
      	U16_t_GrabItPosition_OnDebounce[U8_Counter]=0;
      }
    }
    if(U8_GrabItPosition!=Imm_GrabItPosition_Xf)
    {
      U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xf]++;
      if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xf]>=U16_t_GrabItPosition_DebounceTime)
      {//Found Purple
      	U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_AfterFirst;
      	U8_GrabItPosition=Imm_GrabItPosition_Xf;
      	U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xf]=0;
        //ESP_LOGI(TAG, "MC3416 Position = %d", U8_GrabItPosition);         	
      }
    }
    else
    {
        U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xf] ++;
        if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xf] >= 200)
        {
            U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Xf] = 0;
        }
    }
  }
  //------------Check -X-------------										

	//------------Check +Y-------------
  if(
      ((yAxisData>=(+4095-1002)))										 &&
      ((xAxisData>=(0-1002))&&(xAxisData<(0+1002)))&&
      ((zAxisData>=(0-1002))&&(zAxisData<(0+1002)))
    )
  {
    for(U8_Counter = 0; U8_Counter < Imm_Array_Number; U8_Counter ++)
    {//clear 0
      if(U8_Counter!=(Imm_GrabItPosition_Yz))
      {
      	U16_t_GrabItPosition_OnDebounce[U8_Counter]=0;
      }
    }
    if(U8_GrabItPosition!=Imm_GrabItPosition_Yz)
    {
      U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yz]++;
      if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yz]>=U16_t_GrabItPosition_DebounceTime)
      {//Found Green
      	U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_AfterFirst;
      	U8_GrabItPosition=Imm_GrabItPosition_Yz;
      	U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yz]=0;
        //ESP_LOGI(TAG, "MC3416 Position = %d", U8_GrabItPosition);         	
      }
    }
    else
    {
        U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yz] ++;
        if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yz] >= 200)
        {
            U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yz] = 0;
        }
    }
  }
  //------------Check +Y-------------

	//------------Check -Y-------------
  if(
      ((yAxisData< (-4095+1002)))										 &&
      ((xAxisData>=(0-1002))&&(xAxisData<(0+1002)))&&
      ((zAxisData>=(0-1002))&&(zAxisData<(0+1002)))
    )
  {
    for(U8_Counter = 0; U8_Counter < Imm_Array_Number; U8_Counter ++)
    {//clear 0
      if(U8_Counter!=(Imm_GrabItPosition_Yf))
      {
      	U16_t_GrabItPosition_OnDebounce[U8_Counter]=0;
      }
    }
    if(U8_GrabItPosition!=Imm_GrabItPosition_Yf)
    {
      U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yf]++;
      if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yf]>=U16_t_GrabItPosition_DebounceTime)
      {//Found Orange
      	U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_AfterFirst;
      	U8_GrabItPosition=Imm_GrabItPosition_Yf;
      	U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yf]=0;
        //ESP_LOGI(TAG, "MC3416 Position = %d", U8_GrabItPosition);         	
      }
    }
    else
    {
        U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yf] ++;
        if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yf] >= 200)
        {
            U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Yf] = 0;
        }
    }
  }
  //------------Check -Y-------------

	//------------Check +Z-------------	
  if(
      ((zAxisData>=(+4095-1002)))										 &&
      ((xAxisData>=(0-1002))&&(xAxisData<(0+1002)))&&
      ((yAxisData>=(0-1002))&&(yAxisData<(0+1002)))
    )
  {
    for(U8_Counter = 0; U8_Counter < Imm_Array_Number; U8_Counter ++)
    {//clear 0
      if(U8_Counter!=(Imm_GrabItPosition_Zz))
      {
      	U16_t_GrabItPosition_OnDebounce[U8_Counter]=0;
      }
    }
    if(U8_GrabItPosition!=Imm_GrabItPosition_Zz)
    {
      U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zz]++;
      if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zz]>=U16_t_GrabItPosition_DebounceTime)
      {//Found Red
      	U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_AfterFirst;
      	U8_GrabItPosition=Imm_GrabItPosition_Zz;
      	U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zz]=0;
        //ESP_LOGI(TAG, "MC3416 Position = %d", U8_GrabItPosition);         	
      }
    }
    else
    {
        U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zz] ++;
        if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zz] >= 200)
        {
            U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zz] = 0;
        }
    }
  }
  //------------Check +Z-------------	

	//------------Check -Z-------------
  if(
      ((zAxisData< (-4095+1002)))										 &&
      ((xAxisData>=(0-1002))&&(xAxisData<(0+1002)))&&
      ((yAxisData>=(0-1002))&&(yAxisData<(0+1002)))
    )
  {
    for(U8_Counter = 0; U8_Counter < Imm_Array_Number; U8_Counter ++)
    {//clear 0
      if(U8_Counter!=(Imm_GrabItPosition_Zf))
      {
      	U16_t_GrabItPosition_OnDebounce[U8_Counter]=0;
      }
    }
    if(U8_GrabItPosition!=Imm_GrabItPosition_Zf)
    {
      U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zf]++;
      if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zf]>=U16_t_GrabItPosition_DebounceTime)
      {//Found Yellow
      	U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_AfterFirst;
      	U8_GrabItPosition=Imm_GrabItPosition_Zf;
      	U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zf]=0;
        //ESP_LOGI(TAG, "MC3416 Position = %d", U8_GrabItPosition);         	
      }
    }
    else
    {
        U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zf] ++;
        if(U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zf] >= 200)
        {
            U16_t_GrabItPosition_OnDebounce[Imm_GrabItPosition_Zf] = 0;
        }
    }
  }
  //------------Check -Z-------------	
  
  return U8_GrabItPosition;
}
//----------------Grab it Position----------------

//-------------------------New shake----------------------------
void ClearShakeRam(void)//就在这个文件里用
{//ccc4
  U16_NewShakeCounter=0;
  U16_NewShakeTimeDelay=0;

  U16_NewShakeZoutZhengOnDebounce=0;
  U16_NewShakeZoutZhengOffDebounce=0;
  U16_NewShakeZoutFanOnDebounce=0;
  U16_NewShakeZoutFanOffDebounce=0;

  U16_NewShakeYoutZhengOnDebounce=0;
  U16_NewShakeYoutZhengOffDebounce=0;
  U16_NewShakeYoutFanOnDebounce=0;
  U16_NewShakeYoutFanOffDebounce=0;

  U16_NewShakeXoutZhengOnDebounce=0;
  U16_NewShakeXoutZhengOffDebounce=0;
  U16_NewShakeXoutFanOnDebounce=0;
  U16_NewShakeXoutFanOffDebounce=0;
}//aaaaa5
//-------------------------New shake----------------------------

//-------------------------New shake----------------------------
void PlayNewShakeBodycon(void)//就在这个文件里用
{
  U16_NewShakeCounter++;	

  if(U16_NewShakeCounter>4)//4
  {
    U16_NewShakeTimeDelay=1000/5;
    U16_CountDownAndClearShakeRam=1000/5;
		//Set_Play1Voice(16); //Events:Imm_MotionEvent_Shake
		U8_PlayShakeHappen=1;
  }
  else
  {
    U16_NewShakeTimeDelay=500/5;
    U16_CountDownAndClearShakeRam=700/5;
  }

}
//-------------------------New shake----------------------------

//-------------------------New shake----------------------------
bool CheckNewShake(int16_t xAxisData, int16_t yAxisData, int16_t zAxisData)
//void CheckNewShake(void)//每1ms执行一次，就在这个文件里用
{
//------------------------------
  if(U16_DelayAndThenCkNewShake!=0)
  {
    U16_DelayAndThenCkNewShake--;
    ClearShakeRam();
  }
//------------------------------	
//----------------------
  if(U16_CountDownAndClearShakeRam!=0)
  {
    U16_CountDownAndClearShakeRam--;
    if(U16_CountDownAndClearShakeRam==0)
    {
      ClearShakeRam();
    }
  }
//----------------------
  if(U16_HoldAndDelayCheckNewShake!=0)
  {
    U16_HoldAndDelayCheckNewShake--;
    ClearShakeRam();
    return false;
  }
//----------------------
  if(zAxisData>=(Imm_NewShakeGSensorPulseValue/Imm_Acce_Range))
  {
    U16_NewShakeZoutZhengOnDebounce++;
    U16_NewShakeZoutZhengOffDebounce=0;
  }
  else
  {
    if(U16_NewShakeZoutZhengOnDebounce>=Imm_NewShakeGSensorPulseTime)
    {
      U16_NewShakeZoutZhengOffDebounce++;
      if(U16_NewShakeZoutZhengOffDebounce>=Imm_ShakeUpDownTimes)
      {
      	U16_NewShakeZoutZhengOffDebounce=0;
      	U16_NewShakeZoutZhengOnDebounce=0;
				if(U16_NewShakeTimeDelay!=0)
				{
	  			PlayNewShakeBodycon();
				}
				else
				{
	  			U16_NewShakeTimeDelay=500/5;
				}
      }
    }
    else
    {
      U16_NewShakeZoutZhengOnDebounce=0;
    }
  }
//--------------
  if(zAxisData<(-Imm_NewShakeGSensorPulseValue/Imm_Acce_Range))
  {
    U16_NewShakeZoutFanOnDebounce++;
    U16_NewShakeZoutFanOffDebounce=0;
  }
  else
  {
    if(U16_NewShakeZoutFanOnDebounce>=Imm_NewShakeGSensorPulseTime)
    {
      U16_NewShakeZoutFanOffDebounce++;
      if(U16_NewShakeZoutFanOffDebounce>=Imm_ShakeUpDownTimes)
      {
      	U16_NewShakeZoutFanOffDebounce=0;
      	U16_NewShakeZoutFanOnDebounce=0;
				if(U16_NewShakeTimeDelay!=0)
				{
	  			PlayNewShakeBodycon();
				}
				else
				{
	  			U16_NewShakeTimeDelay=500/5;
				}
      }
    }
    else
    {
      U16_NewShakeZoutFanOnDebounce=0;
    }
  }
//----------------------
  if(yAxisData>=(Imm_NewShakeGSensorPulseValue/Imm_Acce_Range))
  {
    U16_NewShakeYoutZhengOnDebounce++;
    U16_NewShakeYoutZhengOffDebounce=0;
  }
  else
  {
    if(U16_NewShakeYoutZhengOnDebounce>=Imm_NewShakeGSensorPulseTime)
    {
      U16_NewShakeYoutZhengOffDebounce++;
      if(U16_NewShakeYoutZhengOffDebounce>=Imm_ShakeUpDownTimes)
      {
      	U16_NewShakeYoutZhengOffDebounce=0;
      	U16_NewShakeYoutZhengOnDebounce=0;
				if(U16_NewShakeTimeDelay!=0)
				{
	  			PlayNewShakeBodycon();
				}
				else
				{
	  			U16_NewShakeTimeDelay=500/5;
				}
      }
    }
    else
    {
      U16_NewShakeYoutZhengOnDebounce=0;
    }
  }
//--------------
  if(yAxisData<(-Imm_NewShakeGSensorPulseValue/Imm_Acce_Range))
  {
    U16_NewShakeYoutFanOnDebounce++;
    U16_NewShakeYoutFanOffDebounce=0;
  }
  else
  {
    if(U16_NewShakeYoutFanOnDebounce>=Imm_NewShakeGSensorPulseTime)
    {
      U16_NewShakeYoutFanOffDebounce++;
      if(U16_NewShakeYoutFanOffDebounce>=Imm_ShakeUpDownTimes)
      {
      	U16_NewShakeYoutFanOffDebounce=0;
      	U16_NewShakeYoutFanOnDebounce=0;
				if(U16_NewShakeTimeDelay!=0)
				{
	  			PlayNewShakeBodycon();
				}
				else
				{
	  			U16_NewShakeTimeDelay=500/5;
				}
      }
    }
    else
    {
      U16_NewShakeYoutFanOnDebounce=0;
    }
  }
//----------------------
  if(xAxisData>=(Imm_NewShakeGSensorPulseValue/Imm_Acce_Range))
  {
    U16_NewShakeXoutZhengOnDebounce++;
    U16_NewShakeXoutZhengOffDebounce=0;
  }
  else
  {
    if(U16_NewShakeXoutZhengOnDebounce>=Imm_NewShakeGSensorPulseTime)
    {
      U16_NewShakeXoutZhengOffDebounce++;
      if(U16_NewShakeXoutZhengOffDebounce>=Imm_ShakeUpDownTimes)
      {
      	U16_NewShakeXoutZhengOffDebounce=0;
      	U16_NewShakeXoutZhengOnDebounce=0;
				if(U16_NewShakeTimeDelay!=0)
				{
	  			PlayNewShakeBodycon();
				}
				else
				{
	  			U16_NewShakeTimeDelay=500/5;
				}
      }
    }
    else
    {
      U16_NewShakeXoutZhengOnDebounce=0;
    }
  }
//--------------
  if(xAxisData<(-Imm_NewShakeGSensorPulseValue/Imm_Acce_Range))
  {
    U16_NewShakeXoutFanOnDebounce++;
    U16_NewShakeXoutFanOffDebounce=0;
  }
  else
  {
    if(U16_NewShakeXoutFanOnDebounce>=Imm_NewShakeGSensorPulseTime)
    {
      U16_NewShakeXoutFanOffDebounce++;
      if(U16_NewShakeXoutFanOffDebounce>=Imm_ShakeUpDownTimes)
      {
      	U16_NewShakeXoutFanOffDebounce=0;
      	U16_NewShakeXoutFanOnDebounce=0;
				if(U16_NewShakeTimeDelay!=0)
				{
	  			PlayNewShakeBodycon();
				}
				else
				{
	  			U16_NewShakeTimeDelay=500/5;
				}
      }
    }
    else
    {
      U16_NewShakeXoutFanOnDebounce=0;
    }
  }
//----------------------
	if(U8_PlayShakeHappen==1)
	{
		U8_PlayShakeHappen=0;
		return true;
	}	
	else
	{
		return false;		
	}	
}
//-------------------------New shake----------------------------
