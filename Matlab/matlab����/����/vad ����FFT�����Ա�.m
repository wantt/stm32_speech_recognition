% 8000Hz������ 20ms 160dot 50Hz  10ms 80dot 100Hz  16ms 128dot  62.5Hz 
% 32ms 256dot 31.25Hz
% Ԥ���� 6dB/��Ƶ�� һ�׸�ͨ�˲���  H(z)=1-uz^(-1) y(n)=x(n)-ux(n-1) u=0.94~0.97
% �˵��� ��ʱ���� ��ʱ������ 
[y, fs, nbits] = wavread('H:\����ʶ��\��������\01234567989ʮ��ǧ��.wav');

samp=y(41500:44600);
subplot(2,2,1)
plot(samp)
title('������')

n=80;
voice_samp=samp(1500:1579); % 10ms sample
subplot(2,2,2)
plot(voice_samp)
title('����')

sf=fft(voice_samp,128);
af=abs(sf);
af=af(1:64);
f=(0:63)*4000/64;
subplot(2,2,4)
plot(f,af);
title('Ƶ��')

sf=fft(voice_samp,256);
af=abs(sf);
af=af(1:128);
f=(0:127)*4000/128;
subplot(2,2,3)
plot(f,af);
title('Ƶ��')