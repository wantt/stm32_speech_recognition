% 8000Hz������ 20ms 160dot 50Hz  10ms 80dot 100Hz  16ms 128dot  62.5Hz 
% 32ms 256dot 31.25Hz
% Ԥ���� 6dB/��Ƶ�� һ�׸�ͨ�˲���  H(z)=1-uz^(-1) y(n)=x(n)-ux(n-1) u=0.94~0.97
% �˵��� ��ʱ���� ��ʱ������ 
% ����Ƚϸ�ʱ ��ʱ�����Ͷ�ʱ����ֵЧ������
% ����Ƚϵ�ʱ ��ʱ�������ڶ�ʱ����ֵ
[y, fs, nbits] = wavread('F:\voice sample\01234567989ʮ��ǧ��.wav');

len=3200;
v_start=19100;
voice=y(v_start:v_start+len-1)+0.08*randn(len,1);
%sound(voice,fs)
n=80;% ֡�� 10ms
m=0;% ֡�ƣ�����֡��������
E=zeros(length(voice)/(n-m),1);% ��ʱ����
S=zeros(length(voice)/(n-m),1);% ��ʱƽ������
c=0;

for h=1:n-m:(length(voice)-n)
    samp=voice(h:h+n-1); 
	c=c+1;
    for i=1:n
        E(c)=E(c)+samp(i)^2;
        S(c)=S(c)+abs(samp(i));
    end
end

subplot(311)
plot(voice)
title('������')

x=(1:length(E))*len/length(E);

subplot(312)
plot(x,E)
title('��ʱ����')
axis([0 3500 0 2])

subplot(313)
plot(x,S)
title('��ʱ����ֵ')
axis([0 3500 0 15])

