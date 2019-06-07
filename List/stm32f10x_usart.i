#line 1 "Src\\StdPeriph_Driver\\src\\stm32f10x_usart.c"


















  

 
#line 1 ".\\Src\\StdPeriph_Driver\\inc\\stm32f10x_usart.h"



















  

 







 
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








  



  



  

 
#line 23 "Src\\StdPeriph_Driver\\src\\stm32f10x_usart.c"
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








 



 



  

 
#line 24 "Src\\StdPeriph_Driver\\src\\stm32f10x_usart.c"



 




 



 



 



 





































 



 





 



 



 



 



 



 



 



 






 
void USART_DeInit(USART_TypeDef* USARTx)
{
   
  ((void)0);

  if (USARTx == ((USART_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3800)))
  {
    RCC_APB2PeriphResetCmd(((uint32_t)0x00004000), ENABLE);
    RCC_APB2PeriphResetCmd(((uint32_t)0x00004000), DISABLE);
  }
  else if (USARTx == ((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4400)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00020000), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00020000), DISABLE);
  }
  else if (USARTx == ((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4800)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00040000), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00040000), DISABLE);
  }    
  else if (USARTx == ((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4C00)))
  {
    RCC_APB1PeriphResetCmd(((uint32_t)0x00080000), ENABLE);
    RCC_APB1PeriphResetCmd(((uint32_t)0x00080000), DISABLE);
  }    
  else
  {
    if (USARTx == ((USART_TypeDef *) (((uint32_t)0x40000000) + 0x5000)))
    { 
      RCC_APB1PeriphResetCmd(((uint32_t)0x00100000), ENABLE);
      RCC_APB1PeriphResetCmd(((uint32_t)0x00100000), DISABLE);
    }
  }
}










 
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  uint32_t usartxbase = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;
   
  ((void)0);
  ((void)0);  
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
   
  if (USART_InitStruct->USART_HardwareFlowControl != ((uint16_t)0x0000))
  {
    ((void)0);
  }

  usartxbase = (uint32_t)USARTx;

 
  tmpreg = USARTx->CR2;
   
  tmpreg &= ((uint16_t)0xCFFF);
   
   
  tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
  
   
  USARTx->CR2 = (uint16_t)tmpreg;

 
  tmpreg = USARTx->CR1;
   
  tmpreg &= ((uint16_t)0xE9F3);
   
   
   
   
  tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
            USART_InitStruct->USART_Mode;
   
  USARTx->CR1 = (uint16_t)tmpreg;

   
  tmpreg = USARTx->CR3;
   
  tmpreg &= ((uint16_t)0xFCFF);
   
   
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
   
  USARTx->CR3 = (uint16_t)tmpreg;

 
   
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if (usartxbase == ((((uint32_t)0x40000000) + 0x10000) + 0x3800))
  {
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  }
  else
  {
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  }
  
   
  if ((USARTx->CR1 & ((u16)0x8000)) != 0)
  {
     
    integerdivider = ((25 * apbclock) / (2 * (USART_InitStruct->USART_BaudRate)));    
  }
  else  
  {
     
    integerdivider = ((25 * apbclock) / (4 * (USART_InitStruct->USART_BaudRate)));    
  }
  tmpreg = (integerdivider / 100) << 4;

   
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

   
  if ((USARTx->CR1 & ((u16)0x8000)) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else  
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
   
  USARTx->BRR = (uint16_t)tmpreg;
}






 
void USART_StructInit(USART_InitTypeDef* USART_InitStruct)
{
   
  USART_InitStruct->USART_BaudRate = 9600;
  USART_InitStruct->USART_WordLength = ((uint16_t)0x0000);
  USART_InitStruct->USART_StopBits = ((uint16_t)0x0000);
  USART_InitStruct->USART_Parity = ((uint16_t)0x0000) ;
  USART_InitStruct->USART_Mode = ((uint16_t)0x0004) | ((uint16_t)0x0008);
  USART_InitStruct->USART_HardwareFlowControl = ((uint16_t)0x0000);  
}










 
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  uint32_t tmpreg = 0x00;
   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  
 
  tmpreg = USARTx->CR2;
   
  tmpreg &= ((uint16_t)0xF0FF);
   
   
   
   
   
  tmpreg |= (uint32_t)USART_ClockInitStruct->USART_Clock | USART_ClockInitStruct->USART_CPOL | 
                 USART_ClockInitStruct->USART_CPHA | USART_ClockInitStruct->USART_LastBit;
   
  USARTx->CR2 = (uint16_t)tmpreg;
}






 
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct)
{
   
  USART_ClockInitStruct->USART_Clock = ((uint16_t)0x0000);
  USART_ClockInitStruct->USART_CPOL = ((uint16_t)0x0000);
  USART_ClockInitStruct->USART_CPHA = ((uint16_t)0x0000);
  USART_ClockInitStruct->USART_LastBit = ((uint16_t)0x0000);
}









 
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  
  if (NewState != DISABLE)
  {
     
    USARTx->CR1 |= ((uint16_t)0x2000);
  }
  else
  {
     
    USARTx->CR1 &= ((uint16_t)0xDFFF);
  }
}



















 
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState)
{
  uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
  uint32_t usartxbase = 0x00;
   
  ((void)0);
  ((void)0);
  ((void)0);
   
  if (USART_IT == ((uint16_t)0x096A))
  {
    ((void)0);
  }   
  
  usartxbase = (uint32_t)USARTx;

   
  usartreg = (((uint8_t)USART_IT) >> 0x05);

   
  itpos = USART_IT & ((uint16_t)0x001F);
  itmask = (((uint32_t)0x01) << itpos);
    
  if (usartreg == 0x01)  
  {
    usartxbase += 0x0C;
  }
  else if (usartreg == 0x02)  
  {
    usartxbase += 0x10;
  }
  else  
  {
    usartxbase += 0x14; 
  }
  if (NewState != DISABLE)
  {
    *(volatile uint32_t*)usartxbase  |= itmask;
  }
  else
  {
    *(volatile uint32_t*)usartxbase &= ~itmask;
  }
}














 
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);  
  ((void)0); 
  if (NewState != DISABLE)
  {
    
 
    USARTx->CR3 |= USART_DMAReq;
  }
  else
  {
    
 
    USARTx->CR3 &= (uint16_t)~USART_DMAReq;
  }
}








 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address)
{
   
  ((void)0);
  ((void)0); 
    
   
  USARTx->CR2 &= ((uint16_t)0xFFF0);
   
  USARTx->CR2 |= USART_Address;
}











 
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp)
{
   
  ((void)0);
  ((void)0);
  
  USARTx->CR1 &= ((uint16_t)0xF7FF);
  USARTx->CR1 |= USART_WakeUp;
}









 
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0); 
  
  if (NewState != DISABLE)
  {
     
    USARTx->CR1 |= ((uint16_t)0x0002);
  }
  else
  {
     
    USARTx->CR1 &= ((uint16_t)0xFFFD);
  }
}











 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength)
{
   
  ((void)0);
  ((void)0);
  
  USARTx->CR2 &= ((uint16_t)0xFFDF);
  USARTx->CR2 |= USART_LINBreakDetectLength;  
}









 
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  
  if (NewState != DISABLE)
  {
     
    USARTx->CR2 |= ((uint16_t)0x4000);
  }
  else
  {
     
    USARTx->CR2 &= ((uint16_t)0xBFFF);
  }
}








 
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
   
  ((void)0);
  ((void)0); 
  while (!(((USART_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x3800))->SR & ((uint16_t)0x0080)));   
   
  USARTx->DR = (Data & (uint16_t)0x01FF);
}









 
uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
   
  ((void)0);
  
   
  return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}







 
void USART_SendBreak(USART_TypeDef* USARTx)
{
   
  ((void)0);
  
   
  USARTx->CR1 |= ((uint16_t)0x0001);
}







 
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime)
{    
   
  ((void)0);
  
   
  USARTx->GTPR &= ((uint16_t)0x00FF);
   
  USARTx->GTPR |= (uint16_t)((uint16_t)USART_GuardTime << 0x08);
}









 
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler)
{ 
   
  ((void)0);
  
   
  USARTx->GTPR &= ((uint16_t)0xFF00);
   
  USARTx->GTPR |= USART_Prescaler;
}








 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    USARTx->CR3 |= ((uint16_t)0x0020);
  }
  else
  {
     
    USARTx->CR3 &= ((uint16_t)0xFFDF);
  }
}








 
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);  
  ((void)0);
  if (NewState != DISABLE)
  {
     
    USARTx->CR3 |= ((uint16_t)0x0010);
  }
  else
  {
     
    USARTx->CR3 &= ((uint16_t)0xFFEF);
  }
}









 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  
  if (NewState != DISABLE)
  {
     
    USARTx->CR3 |= ((uint16_t)0x0008);
  }
  else
  {
     
    USARTx->CR3 &= ((uint16_t)0xFFF7);
  }
}













 
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  
  if (NewState != DISABLE)
  {
     
    USARTx->CR1 |= ((u16)0x8000);
  }
  else
  {
     
    USARTx->CR1 &= ((u16)0x7FFF);
  }
}









 
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  
  if (NewState != DISABLE)
  {
     
    USARTx->CR3 |= ((u16)0x0800);
  }
  else
  {
     
    USARTx->CR3 &= ((u16)0xF7FF);
  }
}











 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode)
{
   
  ((void)0);
  ((void)0);
    
  USARTx->CR3 &= ((uint16_t)0xFFFB);
  USARTx->CR3 |= USART_IrDAMode;
}









 
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
    
  if (NewState != DISABLE)
  {
     
    USARTx->CR3 |= ((uint16_t)0x0002);
  }
  else
  {
     
    USARTx->CR3 &= ((uint16_t)0xFFFD);
  }
}



















 
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
  FlagStatus bitstatus = RESET;
   
  ((void)0);
  ((void)0);
   
  if (USART_FLAG == ((uint16_t)0x0200))
  {
    ((void)0);
  }  
  
  if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}


























 
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
   
  ((void)0);
  ((void)0);
   
  if ((USART_FLAG & ((uint16_t)0x0200)) == ((uint16_t)0x0200))
  {
    ((void)0);
  } 
   
  USARTx->SR = (uint16_t)~USART_FLAG;
}



















 
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint32_t bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
  ITStatus bitstatus = RESET;
   
  ((void)0);
  ((void)0);
    
  if (USART_IT == ((uint16_t)0x096A))
  {
    ((void)0);
  }   
  
   
  usartreg = (((uint8_t)USART_IT) >> 0x05);
   
  itmask = USART_IT & ((uint16_t)0x001F);
  itmask = (uint32_t)0x01 << itmask;
  
  if (usartreg == 0x01)  
  {
    itmask &= USARTx->CR1;
  }
  else if (usartreg == 0x02)  
  {
    itmask &= USARTx->CR2;
  }
  else  
  {
    itmask &= USARTx->CR3;
  }
  
  bitpos = USART_IT >> 0x08;
  bitpos = (uint32_t)0x01 << bitpos;
  bitpos &= USARTx->SR;
  if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  
  return bitstatus;  
}



























 
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
   
  ((void)0);
  ((void)0);
   
  if (USART_IT == ((uint16_t)0x096A))
  {
    ((void)0);
  }   
  
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->SR = (uint16_t)~itmask;
}


 



 



 

 
