% 8000Hz������ 20ms 160dot 50Hz  10ms 80dot 100Hz  16ms 128dot  62.5Hz 
% 32ms 256dot 31.25Hz
% Ԥ���� 6dB/��Ƶ�� һ�׸�ͨ�˲���  H(z)=1-uz^(-1) y(n)=x(n)-ux(n-1) u=0.94~0.97
% �˵��� ��ʱ���� ��ʱ������ 
% ����Ƚϵ�ʱ ��ʱ�����Ͷ�ʱ����ֵЧ������
% ����Ƚϸ�ʱ ��ʱ�������ڶ�ʱ����ֵ
[y, fs, nbits] = wavread('H:\����ʶ��\��������\���������� �� 8KHz 16bit.wav');

voice_end=269480;
voice_length=8000;
voice_start=voice_end-voice_length+1;
noise_val=0.01;% ��������С 0~1
frame_len=80;% ֡�� 10ms
frame_mov=40;% ֡�ƣ�����֡��������
frame_con=0;% 

voice=y(voice_start:voice_end)+noise_val*randn(voice_length,1);%;%;
sound(voice,fs);
%voice=(voice+0.5)*65536;

E=zeros((voice_length-frame_len)/(frame_len-frame_mov)+1,1);% ��ʱ����
S=zeros((voice_length-frame_len)/(frame_len-frame_mov)+1,1);% ��ʱƽ������
Z=zeros((voice_length-frame_len)/(frame_len-frame_mov)+1,1);% ��ʱ������

for h=1:frame_len-frame_mov:(voice_length-frame_len+1)
    frame=voice(h:h+frame_len-1); 
	frame_con=frame_con+1;
    
    for i=1:frame_len
        E(frame_con)=E(frame_con)+frame(i)^2;
        S(frame_con)=S(frame_con)+abs(frame(i));
    end
    
    for i=1:(frame_len-1)
        if frame(i)>=0
            if frame(i+1)<0
                Z(frame_con)=Z(frame_con)+1;
            end
        else
            if frame(i+1)>=0
                Z(frame_con)=Z(frame_con)+1;
            end
        end
    end
    
end

subplot(411)
plot(voice)
title('������')
axis([0 voice_length -0.5 0.5])
grid on

x=(1:length(E))*voice_length/length(E);

subplot(412)
plot(x,E)
title('��ʱ����')
axis([0 voice_length 0 3])
grid on

subplot(413)
plot(x,S)
title('��ʱ����ֵ')
axis([0 voice_length 0 15])
grid on

subplot(414)
plot(x,Z)
title('��ʱ������')
axis([0 voice_length 0 50])
grid on
