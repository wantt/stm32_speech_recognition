#line 1 "Src\\BSP\\Flash.C"









 
#line 1 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"






















 



 



 
    






  


 
  


 

#line 57 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
















 






 
   








            
#line 98 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"





 






 
#line 117 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 



 



 
#line 136 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

#line 195 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 216 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 244 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 270 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       


#line 360 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 406 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
} IRQn_Type;



 

#line 1 ".\\Src\\CM3_SYS\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 ".\\Src\\BSP\\stdint.h"
 
 





 









#line 25 ".\\Src\\BSP\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 ".\\Src\\BSP\\stdint.h"

     







     










     











#line 260 ".\\Src\\BSP\\stdint.h"



 


#line 91 ".\\Src\\CM3_SYS\\core_cm3.h"

















 

#line 117 ".\\Src\\CM3_SYS\\core_cm3.h"





 


 





 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                          
}  NVIC_Type;                                               
   





 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 














   





 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;

 












 



 



 








   





 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;                          
  volatile const  uint32_t PID6;                          
  volatile const  uint32_t PID7;                          
  volatile const  uint32_t PID0;                          
  volatile const  uint32_t PID1;                          
  volatile const  uint32_t PID2;                          
  volatile const  uint32_t PID3;                          
  volatile const  uint32_t CID0;                          
  volatile const  uint32_t CID1;                          
  volatile const  uint32_t CID2;                          
  volatile const  uint32_t CID3;                          
} ITM_Type;                                                

 



 
























 



 



 



 








   





 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 614 ".\\Src\\CM3_SYS\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 ".\\Src\\CM3_SYS\\core_cm3.h"

#line 728 ".\\Src\\CM3_SYS\\core_cm3.h"






   




 





#line 758 ".\\Src\\CM3_SYS\\core_cm3.h"


 


 




#line 783 ".\\Src\\CM3_SYS\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 933 ".\\Src\\CM3_SYS\\core_cm3.h"





 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





#line 1445 ".\\Src\\CM3_SYS\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
#line 413 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
#line 1 ".\\Src\\StdPeriph_Driver\\system_stm32f10x.h"


















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 414 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
#line 415 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;




typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    


typedef enum {FALSE = 0, TRUE = !FALSE} bool;


typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 861 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 









 




#line 1251 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 1274 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



#line 1293 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"




















 
  


   

#line 1393 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1454 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1630 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 1637 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 








 








 






#line 1673 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 











 











 













 






#line 1789 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"




#line 1809 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





#line 1822 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 1841 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 1850 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 1858 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



















#line 1883 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"







 













#line 1910 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"





#line 1924 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 1931 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"











 














#line 1963 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 1971 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



















#line 1996 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"







 













#line 2023 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"





#line 2037 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 2044 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"











 








 








   
#line 2083 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 2178 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 2205 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2367 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2385 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2403 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 2420 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2438 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2457 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 

 






 
#line 2484 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2559 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 










#line 2590 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





 
#line 2605 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2614 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

   
#line 2623 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2632 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





 
#line 2647 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2656 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

   
#line 2665 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2674 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





 
#line 2689 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2698 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

   
#line 2707 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2716 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





 
#line 2731 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2740 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

   
#line 2749 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2758 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 2767 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 2777 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2841 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2876 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2911 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2946 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 2981 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3048 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 



 









 
#line 3072 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"




 




 
#line 3088 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 





 
#line 3110 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 





 
#line 3125 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"
 
#line 3132 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3181 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3203 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3225 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3247 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3269 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3291 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 
 
 
 

 
#line 3327 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3357 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3367 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 
#line 3391 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 
#line 3415 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 
#line 3439 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 
#line 3463 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 
#line 3487 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 
#line 3511 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3612 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3621 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"















  
 
#line 3644 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3779 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3786 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3793 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3800 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"







 
#line 3814 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3821 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3828 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3835 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3842 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3849 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3857 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3864 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3871 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3878 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3885 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3892 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 3900 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3907 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3914 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 3921 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4063 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4073 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4121 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 

























 
#line 4164 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4178 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4188 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4306 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4341 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"





#line 4352 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4360 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 4367 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4389 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4451 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 
#line 4463 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4500 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 











#line 4522 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 











#line 4544 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 











#line 4566 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 4963 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4972 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4981 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 4992 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5002 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5012 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5022 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5033 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5043 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5053 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5063 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5074 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5084 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5094 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5104 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5115 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5125 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5135 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5145 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5156 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5166 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5176 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5186 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5197 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5207 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5217 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5227 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5238 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5248 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5258 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

#line 5268 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5316 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5386 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5401 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5427 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5648 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 5660 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 






 
#line 5677 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5821 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5833 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5845 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5857 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5869 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5881 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5893 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5905 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 

 


#line 5919 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5931 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5943 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5955 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5967 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5979 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 5991 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6003 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6015 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6027 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6039 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6051 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6063 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6075 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6087 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 


#line 6099 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6119 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6130 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6148 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"











 





 





 
#line 6186 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 












 
#line 6207 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6347 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6364 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6381 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6398 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6432 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6466 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6500 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6534 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6568 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6602 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6636 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6670 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6704 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6738 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6772 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6806 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6840 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6874 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6908 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6942 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 6976 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7010 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7044 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7078 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7112 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7146 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7180 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7214 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7248 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7282 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7316 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7350 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 
 
 
 

 









#line 7377 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7385 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7395 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7456 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7465 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"







 



#line 7486 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 



 


 
#line 7511 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7521 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7547 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 



 
#line 7571 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7580 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"







 
#line 7600 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
#line 7611 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 
 
 
 
 

 


#line 7640 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 









#line 7674 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7714 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8178 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"



 
#line 8198 ".\\Src\\StdPeriph_Driver\\stm32f10x.h"


 

  







 

















 









 

  

 

 
#line 12 "Src\\BSP\\Flash.C"
#line 1 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"


















  

 



#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 26 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\float.h"
 
 
 
 





 










#line 28 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\float.h"

 
 


     
#line 40 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\float.h"
    








 




     


    
 





    











 


 
 




     




     
     




     
     




     



     
     




     




     




     



 



unsigned _controlfp(unsigned, unsigned);
unsigned _clearfp(void);
unsigned _statusfp(void);










 

#line 151 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\float.h"










 





















 

#line 27 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"

 
 
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"



 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;                      

 

  FunctionalState ADC_ScanConvMode;       

 

  FunctionalState ADC_ContinuousConvMode; 

 

  uint32_t ADC_ExternalTrigConv;          

 

  uint32_t ADC_DataAlign;                 
 

  uint8_t ADC_NbrOfChannel;               

 
}ADC_InitTypeDef;


 



 










 

#line 103 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"

#line 114 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 128 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"




#line 138 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"

#line 153 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 







 



 

#line 191 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"




#line 204 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 228 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

















#line 265 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 281 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 

#line 296 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"

#line 304 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 











 



 

#line 337 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









 



 



 

 
#line 31 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
 
 
 
 
 
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_MemoryBaseAddr;      

  uint32_t DMA_DIR;                
 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_M2M;                
 
}DMA_InitTypeDef;



 



 

#line 106 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

#line 153 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 

#line 167 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 






 



 

#line 194 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 







 



 






#line 247 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"

#line 268 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"



#line 295 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"



 



 
#line 331 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"

#line 352 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"



#line 379 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG);
void DMA_ClearFlag(uint32_t DMA_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMA_IT);
void DMA_ClearITPendingBit(uint32_t DMA_IT);








 



 



 

 
#line 37 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_exti.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_exti.h"



 



 



 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;



 



 



 

#line 123 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_exti.h"
                                          
#line 135 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_exti.h"

                    


 



 



 



 



 

void EXTI_DeInit(void);
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);








 



 



 

 
#line 38 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

#line 76 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 

 
#line 117 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
#line 143 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
#line 210 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"











 



 







 



 







 



 





#line 269 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 


 
#line 290 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"






 



 
#line 332 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"





 
#line 345 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 



 

 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

 
void FLASH_UnlockBank1(void);
void FLASH_LockBank1(void);
FLASH_Status FLASH_EraseAllBank1Pages(void);
FLASH_Status FLASH_GetBank1Status(void);
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout);

#line 407 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_flash.h"








 



 



 

 
#line 39 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 



 



 

typedef struct
{
  uint32_t FSMC_AddressSetupTime;       


 

  uint32_t FSMC_AddressHoldTime;        


 

  uint32_t FSMC_DataSetupTime;          


 

  uint32_t FSMC_BusTurnAroundDuration;  


 

  uint32_t FSMC_CLKDivision;            

 

  uint32_t FSMC_DataLatency;            





 

  uint32_t FSMC_AccessMode;             
 
}FSMC_NORSRAMTimingInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Bank;                
 

  uint32_t FSMC_DataAddressMux;      

 

  uint32_t FSMC_MemoryType;          

 

  uint32_t FSMC_MemoryDataWidth;     
 

  uint32_t FSMC_BurstAccessMode;     

 

  uint32_t FSMC_WaitSignalPolarity;  

 

  uint32_t FSMC_WrapMode;            

 

  uint32_t FSMC_WaitSignalActive;    


 

  uint32_t FSMC_WriteOperation;      
 

  uint32_t FSMC_WaitSignal;          

 

  uint32_t FSMC_ExtendedMode;        
 

  uint32_t FSMC_WriteBurst;          
  

  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;    

  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;            
}FSMC_NORSRAMInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_SetupTime;      



 

  uint32_t FSMC_WaitSetupTime;  



 

  uint32_t FSMC_HoldSetupTime;  




 

  uint32_t FSMC_HiZSetupTime;   



 
}FSMC_NAND_PCCARDTimingInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Bank;              
 

  uint32_t FSMC_Waitfeature;      
 

  uint32_t FSMC_MemoryDataWidth;  
 

  uint32_t FSMC_ECC;              
 

  uint32_t FSMC_ECCPageSize;      
 

  uint32_t FSMC_TCLRSetupTime;    

 

  uint32_t FSMC_TARSetupTime;     

  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;     

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;  
}FSMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Waitfeature;    
 

  uint32_t FSMC_TCLRSetupTime;  

 

  uint32_t FSMC_TARSetupTime;   

  

  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;    
  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;    
}FSMC_PCCARDInitTypeDef;



 



 



 






 



   




 



     



 



















 



 








 



 

#line 312 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 








 



 







 



 








 



 








 



 








 



 





                              


 



 







 



 









 



 







 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 504 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 
  


 



 








 




 








 



 

#line 560 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"



 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 636 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"


 



 

#line 652 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_fsmc.h"





 



 



 



 



 



 

void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_PCCARDDeInit(void);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_PCCARDCmd(FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



 



  

 
#line 40 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 



 

#line 52 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;             
 

  GPIOSpeed_TypeDef GPIO_Speed;  
 

  GPIOMode_TypeDef GPIO_Mode;    
 
}GPIO_InitTypeDef;




 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

#line 143 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



#line 162 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

#line 203 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"







#line 216 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                                                       

#line 239 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 260 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

#line 268 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

#line 293 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

#line 310 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



  








                                                 


 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);








 



 



 

 
#line 41 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;   
  uint32_t HCLK_Frequency;     
  uint32_t PCLK1_Frequency;    
  uint32_t PCLK2_Frequency;    
  uint32_t ADCCLK_Frequency;   
}RCC_ClocksTypeDef;



 



 



 









  



 



#line 93 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



  



 
#line 125 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 140 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 
#line 174 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 




 
#line 195 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 

#line 282 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 

#line 294 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 316 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


  



 

#line 332 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 346 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 363 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 




 








 
#line 395 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


#line 422 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
  



 

#line 434 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 461 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 







#line 488 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 517 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




  



 

#line 552 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
 




 



 







#line 585 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 

#line 605 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 624 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);





#line 665 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);


 void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);




void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);






void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);





void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);








 



 



  

 
#line 45 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
 
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



 



 



 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;



  



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 221 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


  



 




 



 

#line 244 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 








 



 






  



 

#line 282 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 




 



 

#line 329 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"


 



 







 



 







 



 






 



 

#line 420 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"



#line 447 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_sdio.h"





 



 







 



 



 



 



 

void SDIO_DeInit(void);
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);








 



 



 

 
#line 47 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"



 



  



 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;



 



 










 
  
#line 135 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 219 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 247 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

#line 265 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

#line 281 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 319 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


  



 







 



 






 



 







 



 






 



 







 



 

#line 399 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 

#line 420 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_spi.h"


 



 




 



 



 



 



 

void SPI_I2S_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);








 



 



 

 
#line 48 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"



 



  



  




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint16_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef;       



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint16_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;



 

#line 185 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 



 






 
#line 204 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"
									                                 
 
#line 215 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

                                             
#line 224 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
#line 235 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
#line 248 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

                                         
#line 265 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

 
#line 278 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"
                                                                                                                                                                                                                          


  



 

#line 307 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 







  



 

#line 340 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 354 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


 



 

#line 372 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 496 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







 



 







  



 







  



 







  



 







  



 

#line 572 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 588 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 605 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"

#line 614 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 660 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 704 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 720 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"



  



 

#line 737 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 765 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 779 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



  






 



 







  



 







  



 

#line 828 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  




 

#line 846 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"



  



 

#line 861 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 922 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 

#line 938 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"


  



 







  



 

#line 982 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_tim.h"
                               
                               



  



 




  



 




  



 



 



  



 

void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx);
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);








  



  



 

 
#line 49 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"



 



  



  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            


 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;



  



  
  
















  
  


                                    




  



  
  
#line 145 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  
  
#line 159 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  
  





  



  
#line 186 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 263 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 335 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"
                              
#line 343 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"



  



  



  



  



 

void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
void USART_SendBreak(USART_TypeDef* USARTx);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);








  



  



  

 
#line 50 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"
 
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\misc.h"



















  

 







 
#line 32 ".\\Src\\StdPeriph_Driver\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 


 



 
























 



 



 



 







 



 

#line 132 ".\\Src\\StdPeriph_Driver\\inc\\misc.h"


 



 

#line 150 ".\\Src\\StdPeriph_Driver\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 



 

 
#line 52 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"

 
 

 
 

 
#line 75 ".\\Src\\StdPeriph_Driver\\stm32f10x_conf.h"



 
#line 13 "Src\\BSP\\Flash.C"
#line 1 "Src\\BSP\\Flash.H"



#line 1 ".\\Src\\Speech_Recog\\MFCC.H"
#line 4 ".\\Src\\Speech_Recog\\MFCC.H"
#line 1 ".\\Src\\Speech_Recog\\VAD.H"









typedef struct
{
	u32 mid_val;	
	u16	n_thl;		
	u16 z_thl;		
	u32 s_thl;		
}atap_tag;			

typedef struct
{
	u16 *start;	
	u16 *end;	
}valid_tag;	

void noise_atap(const u16* noise,u16 n_len,atap_tag* atap);
void VAD(const u16 *vc, u16 buf_len, valid_tag *valid_voice, atap_tag *atap_arg);

#line 5 ".\\Src\\Speech_Recog\\MFCC.H"


#line 14 ".\\Src\\Speech_Recog\\MFCC.H"




#pragma pack(1)
typedef struct
{
	u16 save_sign;						
	u16 frm_num;						
	s16 mfcc_dat[((1200-20)/(20-10)+1)*12];	
}v_ftr_tag;								
#pragma pack()

void get_mfcc(valid_tag *valid, v_ftr_tag *v_ftr, atap_tag *atap_arg);

#line 5 "Src\\BSP\\Flash.H"








#line 21 "Src\\BSP\\Flash.H"

u8 save_ftr_mdl(v_ftr_tag* ftr, u32 addr);



#line 14 "Src\\BSP\\Flash.C"
#line 1 "Src\\BSP\\USART.h"



void USART1_configuration(void);
void USART_SendStr(USART_TypeDef* USARTx, uint8_t *Data);
void USART_SendArray(USART_TypeDef* USARTx, uint8_t *Data ,u16 len);
void UASRT_DMA_TXConfiguration(USART_TypeDef* USARTx,u8 *BufferSRC, u32 BufferSize);
void USART1_printf (char *fmt, ...); 
void printf1 (char *fmt); 
void scanf1(char *str); 

#line 15 "Src\\BSP\\Flash.C"
#line 16 "Src\\BSP\\Flash.C"

u8 save_ftr_mdl(v_ftr_tag* ftr, u32 addr)
{
	u32 i;
	u32 ftr_size;
	
	if(((addr%2048)!=0)||(addr<(0x8080000-((4*(4*1024))*20)))||(addr>(0x8080000-(4*1024))))
	{
		USART1_printf("flash addr error");
		return 3;
	}
	ftr_size=2*12*ftr->frm_num;
	
	FLASH_UnlockBank1();
	FLASH_ClearFlag(((uint32_t)0x00000020) | ((uint32_t)0x00000004) | ((uint32_t)0x00000010));
	
	for(i=0;i<((4*1024)/2048);i++)
	{
		if(FLASH_ErasePage(addr+2048*i)!=FLASH_COMPLETE)
		{
			USART1_printf("flash Erase Error! ");
			return 3;
		}
	}
	
	
	if(FLASH_ProgramHalfWord(addr, 12345)!=FLASH_COMPLETE)
	{
		USART1_printf("flash Program Error! ");
		return 3;
	}
	addr+=2;
	
	if(FLASH_ProgramHalfWord(addr,*(u16 *)(&(ftr->frm_num)))!=FLASH_COMPLETE)
	{
		USART1_printf("flash Program Error! ");
		return 3;
	}
	addr+=2;
	
	for(i=0;i<ftr_size;i+=2)
	{
		if(FLASH_ProgramHalfWord(addr+i,*(u16 *)((u32)(ftr->mfcc_dat)+i))!=FLASH_COMPLETE)
		{
			USART1_printf("flash Program Error! ");
			return 3;
		}
	}
	
	FLASH_LockBank1();
	return 0;
}
