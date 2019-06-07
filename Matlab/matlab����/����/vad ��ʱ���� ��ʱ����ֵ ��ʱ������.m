% 8000Hz������ 20ms 160dot 50Hz  10ms 80dot 100Hz  16ms 128dot  62.5Hz 
% 32ms 256dot 31.25Hz
% Ԥ���� 6dB/��Ƶ�� һ�׸�ͨ�˲���  H(z)=1-uz^(-1) y(n)=x(n)-ux(n-1) u=0.94~0.97
% �˵��� ��ʱ���� ��ʱ������ 
% ����Ƚϵ�ʱ ��ʱ�����Ͷ�ʱ����ֵЧ������
% ����Ƚϸ�ʱ ��ʱ�������ڶ�ʱ����ֵ
[y, fs, nbits] = wavread('F:\����ʶ��\��������\01234567989ʮ��ǧ��.wav');

len=3200;
v_start=19100;
voice=y(v_start:v_start+len-1)+0.02*randn(len,1);
%sound(voice,fs)
n=80;% ֡�� 10ms
m=0;% ֡�ƣ�����֡��������
E=zeros((length(voice)-n)/(n-m)+1,1);% ��ʱ����
S=zeros((length(voice)-n)/(n-m)+1,1);% ��ʱƽ������
Z=zeros((length(voice)-n)/(n-m)+1,1);% ��ʱ������
c=0;

for h=1:n-m:(length(voice)-n+1)
    samp=voice(h:h+n-1); 
	c=c+1;
    for i=1:n
        E(c)=E(c)+samp(i)^2;
        S(c)=S(c)+abs(samp(i));
    end
    for i=1:(n-1)
        if samp(i)>=0
            if samp(i+1)<0
                Z(c)=Z(c)+1;
            end
        else
            if samp(i+1)>=0
                Z(c)=Z(c)+1;
            end
        end
    end
end

subplot(411)
plot(voice)
title('������')
axis([0 length(voice) -0.5 0.5])
grid on

x=(1:length(E))*len/length(E);

subplot(412)
plot(x,E)
title('��ʱ����')
axis([0 length(voice) 0 3])
grid on

subplot(413)
plot(x,S)
title('��ʱ����ֵ')
axis([0 length(voice) 0 15])
grid on

subplot(414)
plot(x,Z)
title('��ʱ������')
axis([0 length(voice) 0 50])
grid on
