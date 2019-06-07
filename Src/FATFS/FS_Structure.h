
/**** ָ��AlignֵΪ1byte,��ֹ��������Ż� *******/
#pragma pack(1)
typedef struct { 
	vu8 	bJmpBoot[3];    	//ofs:0.���͵��磺0xEB,0x3E,0x90�� 
	vu8  	bOEMName[8];    	//ofs:3.���͵��磺 ��MSWIN4.1���� 
	vu16	BPB_BytesPerSec;  	//ofs:11.ÿ�����ֽ�����
	vu8 	BPB_SecPerClus;   	//ofs:13.ÿ���������� 
	vu16  	BPB_ReservedSec;  	//ofs:14.�������������� DBR�� FAT���������� 
	vu8 	BPB_NumFATs;   		//ofs:16.FAT�ĸ����� 
	vu16  	BPB_RootEntCnt;   	//ofs:17.FAT16 ��Ŀ¼������ 
	vu16  	BPB_TotalSec;    	//ofs:19.������������(<32Mʱ��)��
	vu8 	BPB_Media;    		//ofs:21.�������ʱ�ʶ,����һ���� 0xF8�� 
	vu16  	BPB_FATSz16;   		//ofs:22.FAT16��ÿ�� FATռ����������FAT32��Ϊ0 
	vu16  	BPB_SecPerTrk;   	//ofs:24.ÿ���������� 
	vu16  	BPB_NumHeads;    	//ofs:26.��ͷ���� 
	vu32 	BPB_HiddSec;   		//ofs:28.�������������� MBR�� DBR���������� 
	vu32 	BPB_TotalSec32;  	//ofs:32.������������(>=32Mʱ��)�� 
	vu32 	BPB_FATSz32;     	//ofs:36.FAT32��ÿ��FATռ����������FAT16��Ϊ0 
	vu16 	BPB_ExtFlags32;    	//ofs:40. 
	vu16 	BPB_FSVer32;    	//ofs:42.FAT32�汾�� 
	vu32 	BPB_RootClus;    	//ofs:44.FAT32�и�Ŀ¼���ڵ�һ�شغ� 
	vu16  	BPB_FSInfo;    		//ofs:48.FAT32��������FSInfo��ռ������  ����
	vu32 	BPB_BK32;			//ofs:50
	vu8  	FileSysType[8];   	//ofs:54.��FAT32   ���� 
	vu8 	ExecutableCode[448];//ofs:62.�������롣 
	vu16  	EndingFlag;    		//ofs:510.������ʶ:0xAA55�� 
}FAT32_BPB;

typedef struct { 
	vu32	FSI_LeadSig;		//ofs:0. 	FSI��־ 0x41615252
	vu8		FSI_Reserved1[480];	//ofs:4. 	������ ȫ��
	vu32	FSI_Strucsig;		//ofs:484	FSI��־ 0x61417272
	vu32	FSI_Free_Count;		//ofs:488	�������µ�ʣ�������
								// 			��Ϊ0xffffffff,�������¼���
	vu32	FSI_NxtFree;		//ofs:492	������һʣ��شغ�
								//			��Ϊ0xffffffff,�������¼���
	vu8		FSI_Reserved2[12];	//ofs:496	����
	vu32	FSI_Trailsig;		//ofs:508	FSI������־ 0xAA550000
}FAT32_FSinfo;
 
typedef struct{ 
	u8	FileName[8]; 	//ofs:0.�ļ��� OEM�ַ�
	u8  ExtName[3]; 	//ofs:8.��չ�� 
	u8 	Attribute; 		//ofs:11.�ļ����ԡ�����ֵ���浵(0x20)�����(0x08)�� 
	u8  NT_Res; 		//ofs:12.���� 
	u8	CtrTimeTeenth;  //ofs:13.����ʱ�䣨���룩 
	u16 CtrTime;   		//ofs:14.����ʱ�� 
	u16 CtrDate; 		//ofs:16.�������� 
	u16	LastAccDate;	//ofs:18.������ʱ��
	u16	FstClusHI;		//ofs:20.�ļ���ʼ�غŸ�λ
	u16	WrtTime;		//ofs:22.���дʱ��
	u16	WrtDate;		//ofs:24.���д����
	u16	FstClusLO;		//ofs:26.�ļ���ʼ�غŵ�λ
	u32 FileLength; 	//ofs:28.�ļ����� 
}DIR_tag;

typedef	struct{
	u8	Ord;		//ofs:0.�ó�Ŀ¼���ڱ����е����
	u8	Name1[10];	//ofs:1.���ļ��������1~5�ַ� Unicode�ַ�
	u8	Attr;		//ofs:11.���� ����ΪLong_Name 0x0F
	u8	Type;		//ofs:12.һ��Ϊ��
	u8	ChkSum;		//ofs:13.���ļ���У���
	u8	Name2[12];	//ofs:14.���ļ��������6~11�ַ� Unicode�ַ�	
	u16	FstClusLO;	//ofs:26.�˴�������	����Ϊ�� 
	u8	Name3[4];	//ofs:28.���ļ��������12~13�ַ� Unicode�ַ�
}LongDir_Ent;	
#pragma pack()
