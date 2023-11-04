#include <stdint.h>

#define	Imm_GrabItPosition_Xz														0
#define	Imm_GrabItPosition_Xf														1
#define	Imm_GrabItPosition_Yz														2
#define	Imm_GrabItPosition_Yf														3
#define	Imm_GrabItPosition_Zz														4
#define	Imm_GrabItPosition_Zf														5

uint8_t	 CheckGrabItPosition(int16_t XOUT_Data, int16_t YOUT_Data, int16_t ZOUT_Data);
bool CheckNewShake(int16_t xAxisData, int16_t yAxisData, int16_t zAxisData);