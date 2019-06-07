% 8000Hz������ 20ms 160dot 50Hz  10ms 80dot 100Hz  16ms 128dot  62.5Hz 
% 32ms 256dot 31.25Hz
% Ԥ���� 6dB/��Ƶ�� һ�׸�ͨ�˲���  H(z)=1-uz^(-1) y(n)=x(n)-ux(n-1) u=0.94~0.97
% �˵��� ��ʱ���� ��ʱ������ 
% ����Ƚϵ�ʱ ��ʱ�����Ͷ�ʱ����ֵЧ������
% ����Ƚϸ�ʱ ��ʱ�������ڶ�ʱ����ֵ
clear

[y, fs, nbits] = wavread('H:\����ʶ��\��������\���������� �� 8KHz 16bit.wav');

data_max=32768;%���������ֱ��� ����������ֵ �з���

voice_end=length(y);%-104000;
voice_length=104000;
voice_start=voice_end-voice_length+1;


frame_time=20;						% ÿ֡ʱ�䳤�� ��λms
frame_mov_rtio=0.5;					% ֡��ϵ��
frame_len=frame_time*fs/1000;		% ֡�� 
frame_mov=frame_len*frame_mov_rtio;	% ֡�ƣ�����֡��������
frame_con=0; % ֡��

noise_val=0.02;						%��������С 0~1
noise_thl=0; %��������
n_thl_ratio=1;						%��������ϵ�� noise_thl=n_max_mean*n_thl_ratio
noise_time=20;						%����ȡ��ʱ�䳤�� ��λms
noise_len=noise_time*fs/1000;		%����ȡ������
noise_num=10;						%����������
noise_start=1;						%����ȡ����ʼ��
noise_max=0; %��������ֵ
n_max_mean=0; %��������ֵƽ��ֵ

valid_start=zeros(20,1); %��Ч������ʼ��
valid_end=zeros(20,1); %��Ч����������
valid_con=0; %��Ч�����μ���
cur_stus=0; %��ǰ����״̬��0������  1ǰ�˹��ɶ�  2������  3��˹��ɶ�
font_duration=0; %ǰ�˹��ɶγ�������ֵ����֡��
back_duration=0; %��˹��ɶε�������ֵ����֡��
v_durmin_t=50; 												%��Ч�������ʱ������ ms
v_durmin_f=v_durmin_t/(frame_time*(1-frame_mov_rtio)); 		%��Ч�������֡��
s_durmax_t=125; 											%�������ʱ������ ms
s_durmax_f=s_durmax_t/(frame_time*(1-frame_mov_rtio)); 		%�������֡��

s_thl_ratio=0.35;											% ��ʱ�����о�����ϵ�� ����
															% s_thl=frame_len*noise_thl*s_thl_ratio
s_thl=0;% ��ʱ���� �о�����
z_thl_ratio=2.2/160;										% ��ʱ������ �о�����ϵ�� ����
z_thl=frame_len*z_thl_ratio/n_thl_ratio; 					% ��ʱ�������о����� ֱ�����ڶ˵���

last_sig=0;%�����ʱ������ ��һ�ι���֮����������������ޱȽϵĽ�� 
           % 1 �����޴����� -1 �����޴�����

p_voice=y(voice_start:voice_end);%��������
n_voice=p_voice+noise_val*randn(voice_length,1);%����
%sound(n_voice,fs);
n_voice=n_voice*data_max;

% ����ֵ��ȡ
% ����ֵӦ�ܶԱ��������ı仯��������Ӧ����
% �ɼ����ɶα������� ����������ֵ Ȼ��ȡƽ��ֵ
% ��ƽ��ֵ�������о����޳�����
i=noise_start;
for h=1:noise_num
	for i=i:(i+noise_len)
		if abs(n_voice(i))>noise_max
			noise_max=abs(n_voice(i));
		end
	end
	n_max_mean=n_max_mean+noise_max;
    noise_max=0;
end
n_max_mean=n_max_mean/noise_num; %ȡ��ֵ
noise_thl=n_max_mean*n_thl_ratio; %ֱ�������жϹ���

s_thl=frame_len*noise_thl*s_thl_ratio; %��ʱ��������ֵ ֱ�����ڶ˵���

% ��ʱ���ȶ�ʱ��������ȡ��
% ��ʱ����ֱ���ۼ�
% ��ʱ�����ʸĽ�Ϊ�������ʣ�����������������ֵ��ȵ����ޡ�
% �������޴����������޴���������

%�˵��о���
% 1.�ж�������ʼ�㣬Ҫ���ܹ��˳�ͻ��������
%ͻ�����������������ʱ����������ʵ���ֵ�ܸߣ�������������ά���㹻����ʱ�䣬
%���Ŵ��Ŀ��أ��������ײ���������������Щ������ͨ���趨���ʱ���������б�
%����������֮һ��ȫ�������ҳ���ʱ�䳬����Ч�������ʱ�����ޣ�
%�����ʼ�������޵�ʱ��㣬������Ϊ��Ч������ʼ�㡣
%
% 2.�ж����������㣬Ҫ���ܶ��������м���ݵ��п��ܱ�������û�ġ��ž��Ρ�
%ͬʱ���������ޣ����ҳ���ʱ�䳬�������ʱ�����ޣ�
%�����ʼ�������޵�ʱ��㣬������Ϊ��Ч���������㡣
S=zeros((voice_length-frame_len)/(frame_len-frame_mov)+1,1);% ��ʱ����
Z=zeros((voice_length-frame_len)/(frame_len-frame_mov)+1,1);% ��ʱ������

frame_con=0;
for h=1:frame_len-frame_mov:(voice_length-frame_len+1)
    frame=n_voice(h:h+frame_len-1); 
	frame_con=frame_con+1;
    
    for i=1:frame_len
        S(frame_con)=S(frame_con)+abs(frame(i)); %�ۼ���ȡ��ʱ���Ⱥ�
    end
    
    for i=1:(frame_len-1) %��ȡ������
        if frame(i)>=noise_thl
            last_sig=1;
        elseif frame(i)<(0-noise_thl)
            last_sig=-1;
        end
        
        if last_sig==-1
            if frame(i+1)>=noise_thl
                Z(frame_con)=Z(frame_con)+1;
            end
        elseif last_sig==1
            if frame(i+1)<(0-noise_thl)
                Z(frame_con)=Z(frame_con)+1;
            end
        end
    end
    last_sig=0;
    
    if S(frame_con)>s_thl || Z(frame_con)>z_thl %������һ����������������ֵ
        if cur_stus==2 %�����ǰ��������
            %�ղ���
        elseif cur_stus==0 %�����ǰ��������
            cur_stus=1; %����ǰ�˹��ɶ� 
            font_duration=1; %ǰ�˹��ɶγ���֡����1 ��һ֡
        elseif cur_stus==1; %��ǰ��ǰ�˹��ɶ�
            font_duration=font_duration+1;
            if font_duration>=v_durmin_f %ǰ�˹��ɶ�֡�����������Ч����֡��
                cur_stus=2; %����������
                valid_con=valid_con+1; %�����μ���
                valid_start(valid_con)=frame_con-v_durmin_f; %��¼��ʼ֡λ��
                font_duration=0; %ǰ�˹��ɶγ���֡����0
            end
        elseif cur_stus==3 %�����ǰ�Ǻ�˹��ɶ�
            back_duration=0;
            cur_stus=2; %��Ϊ������
        end
    else %��������������ֵ����
        if cur_stus==0 %��ǰ��������
            %�ղ���
        elseif cur_stus==2 %��ǰ��������
            cur_stus=3;%��Ϊ��˹��ɶ�
            back_duration=1; %ǰ�˹��ɶγ���֡����1 ��һ֡
        elseif cur_stus==3
            back_duration=back_duration+1;
            if back_duration>=s_durmax_f %��˹��ɶ�֡�����������֡��
                cur_stus=0; %����������
                valid_end(valid_con)=frame_con-s_durmax_f; %��¼����֡λ��
                back_duration=0;
            end
        elseif cur_stus==1 %��ǰ��ǰ�˹��ɶ� �����������䵽����ֵ���� 
                           %����ʱ������������ʱ������ ��Ϊ��ʱ����
            font_duration=0;
            cur_stus=0; %��Ϊ������
        end
    end
end


%%%%%%%%%% ���ݿ��ӻ� %%%%%%%%% ���ݿ��ӻ� %%%%%%%%%% ���ݿ��ӻ� %%%%%%%%%%%%
x=(1:voice_length);
vert=linspace(-0.4,0.4,100);

i=1;
valid_start=valid_start*(frame_len-frame_mov);
valid_end=valid_end*(frame_len-frame_mov);
subplot(411)
plot(x,p_voice,valid_start(1),vert,...
               valid_end(1),vert,...
               valid_start(2),vert,...
               valid_end(2),vert,...
               valid_start(3),vert,...
               valid_end(3),vert,...
               valid_start(4),vert,...
               valid_end(4),vert,...
               valid_start(5),vert,...
               valid_end(5),vert,...
               valid_start(6),vert,...
               valid_end(6),vert,...
               valid_start(7),vert,...
               valid_end(7),vert,...
               valid_start(8),vert,...
               valid_end(8),vert,...
               valid_start(9),vert,...
               valid_end(9),vert,...
               valid_start(10),vert,...
               valid_end(10),vert)
ylabel('����������')
axis([0 voice_length -inf inf])
%grid on 

temp=zeros(voice_length,1);
temp=temp+noise_thl;

subplot(412)
plot(x,n_voice,x,temp,x,-temp)
ylabel('����������')
axis([0 voice_length -inf inf])
grid on


x=(1:frame_con)*voice_length/frame_con;

temp=zeros(frame_con,1);
temp=temp+s_thl;

subplot(413)
plot(x,S, x,temp)
ylabel('��ʱ����')
axis([0 voice_length -inf inf])
grid on

temp=zeros(frame_con,1);
temp=temp+z_thl;

subplot(414)
plot(x,Z, x,temp)
ylabel('��ʱ������')
axis([0 voice_length -inf inf])
grid on
