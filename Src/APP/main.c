/********* main.C **********/

#include "includes.h"
#include "VAD.H"
#include "MFCC.H"
#include "DTW.H"
#include "GUI.H"
#include "flash.h"
#include "delay.h"

#define DEBUG
#ifdef DEBUG
#define DBP(fmt,arg...)  USART1_printf(fmt,##arg)
#define DBPLN(fmt,arg...)  USART1_printf_line(fmt,##arg)
#define DBPH(src, len)  USART1_print_hex(src, len)
#else
#define DBP(fmt,arg...)
#define DBPLN(fmt,arg...)
#define DBPH(src, len)
#endif

u16 		VcBuf[VcBuf_Len];
atap_tag	atap_arg;
valid_tag	valid_voice[max_vc_con];
v_ftr_tag	ftr;
typedef struct
{
	u8 str[3];
}comm_tag;

comm_tag commstr[]={"0 ","1 ","2 ","3 ","4 ","5 ","6 ","7 ","8 ","9 ","��","��","ǰ","��","��","��","��","С"};\
char buf1[32]="1234567890123456789012345678901\0";
uint16_t buf1Pos=0;
uint16_t ScanfFlag=0;//

#define sel_clor		BRED
#define dis_sel_clor	GRED
#define spk_clor		BRED
#define prc_clor		GRED

#define save_ok		0
#define VAD_fail	1
#define MFCC_fail	2
#define Flash_fail	3


void record(void)
{
	delay_ms(atap_len_t);	//��ʱ����������Ļ����������
	
	TIM_Cmd(TIM1, ENABLE); 	//������ʱ������ʼ�źŲɼ�
	
	//GUI_ClrArea(&(Label[G_ctrl]));					//��ʾ������ʾ
	//GUI_DispStr(&(Label[G_ctrl]),"¼����");		
	USART1_printf("recording   \n");
	
	//��ʼ˵��֮ǰ��¼��һС�α�������������ʵ�ֱ�����������Ӧ
	delay_ms(atap_len_t);		
	//��ʾ��ʼ˵��
//	set_label_backclor(&(Label[G_spk]), spk_clor);
	
	//�ȴ����������ݸ������
	while(DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET);
	
	
	//���ݲɼ��������رն�ʱ��
	TIM_Cmd(TIM1, DISABLE); 
	//�����ݴ�����ɱ�־���Ա��´�ʹ��
	DMA_ClearFlag(DMA1_FLAG_TC1);
	
	//��ʾ��ʼ����ɼ���������
	USART1_printf("start processing data");
//	set_label_backclor(&(Label[G_spk]), prc_clor);
}



u8 save_mdl(u16 *v_dat, u32 addr)
{
	noise_atap(v_dat,atap_len,&atap_arg);
	
	VAD(v_dat, VcBuf_Len, valid_voice, &atap_arg);
	if(valid_voice[0].end==((void *)0))
	{
		return VAD_fail;
	}
	
	get_mfcc(&(valid_voice[0]),&ftr,&atap_arg);
	USART1_printf("frame num  = %d",ftr.frm_num);
	if(ftr.frm_num==0)
	{
	//	USART1_printf("frame num  = %d",ftr.frm_num);
		return MFCC_fail;
	}
	
	return save_ftr_mdl(&ftr, addr);
}

void prc(void)
{
//	u32 i;
//	u8	prc_start=0;
	u8	comm=G_comm_fst;
	u8 	prc_count=0;
	u32 addr;
	u8 kk = 0;
	//u16 ii  = 0;
	USART1_printf("train start \n");
	while(kk<1)
	{
		kk+=1;
	//	USART1_printf("start record \n");
		
		//char c ;// wtt 
		//scanf1(&c);
		if(1){
		
			
			record();
			
		/*
			for (ii = 0; ii < VcBuf_Len; ii ++){
			
					USART1_printf("\t%d\t\n",VcBuf[ii]);
				//delay_ms(3000);
			} */
			
			addr=ftr_start_addr+(comm-G_comm_fst)*size_per_comm+prc_count*size_per_ftr;
			
			if(save_mdl(VcBuf, addr)==save_ok)
			{
				USART1_printf("start record okokokooook\n");
				prc_count++;
				if(prc_count==ftr_per_comm)
				{
					prc_count=0;
				}
			}
			USART1_printf("train %d times",prc_count);
		}
		
	}
}




u8* spch_recg(u16 *v_dat, u32 *mtch_dis)
{
	u16 i;
	u32 ftr_addr;
	u32 min_dis;
	u16 min_comm;
	u32 cur_dis;
	v_ftr_tag *ftr_mdl;
	
	noise_atap(v_dat, atap_len, &atap_arg);
	
	VAD(v_dat, VcBuf_Len, valid_voice, &atap_arg);
	if(valid_voice[0].end==((void *)0))
	{
		*mtch_dis=dis_err;
		USART1_printf("VAD fail ");
		return (void *)0;
	}
	
	get_mfcc(&(valid_voice[0]),&ftr,&atap_arg);
	if(ftr.frm_num==0)
	{
		*mtch_dis=dis_err;
		//USART1_printf("MFCC fail ");
		USART1_printf("mfcc fail\n");
		return (void *)0;
	}
	
	i=0;
	min_comm=0;
	min_dis=dis_max;
	for(ftr_addr=ftr_start_addr; ftr_addr<ftr_end_addr; ftr_addr+=size_per_ftr)
	{
		ftr_mdl=(v_ftr_tag*)ftr_addr;
		//USART1_printf("save_mask=%d ",ftr_mdl->save_sign);
		cur_dis=((ftr_mdl->save_sign)==save_mask)?dtw(&ftr,ftr_mdl):dis_err;
		//USART1_printf("cur_dis=%d ",cur_dis);
		if(cur_dis<min_dis)
		{
			min_dis=cur_dis;
			min_comm=i;
		}
		i++;
	}
	min_comm/=ftr_per_comm;
	//USART1_printf("recg end ");
	*mtch_dis=min_dis;
	return (commstr[min_comm].str);
}



void recg(void)
{
	u8 *res;
	u32 dis;
	u32 recg_count=0;

	USART1_printf("recog start\n");
	while(1)
	{
	
		
			record();
			
			USART1_printf("recognizing for recog ... \n");
			
			res=spch_recg(VcBuf, &dis);
			if(dis!=dis_err)
			{
				recg_count++;
		//		GUI_ClrArea(&(Label[G_recg_res]));
				USART1_printf("ʶ����:%s",(s8 *)res);
				USART1_printf("ƥ�����:%d",dis);
			}

	
		}
		
	
}
	char c[32];
//uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
uint16_t cc=0;
uint16_t cc1=0;
int main(void)
{
	BSP_Init();
	USART1_printf("sys init ok!\n");
	
	while(1)
	{
		cc = USART_ReceiveData(USART1);
		
		if(cc!=cc1){
			
			
		
		}
			
	}
}
