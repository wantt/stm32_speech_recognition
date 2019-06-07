clear;
fs=8000;

noise=load('H:\����ʶ��\��������\STM32 noise.txt');
noise=(noise-2048)/2048;
sound(noise,fs);

Voice123=load('H:\����ʶ��\��������\STM32 123.txt');
Voice123=(Voice123-2048)/2048;
sound(Voice123,fs);

Voice456=load('H:\����ʶ��\��������\STM32 456.txt');
Voice456=(Voice456-2048)/2048;
sound(Voice456,fs);

subplot(311)
plot(noise)
ylabel('����')
grid on

subplot(312)
plot(Voice123)
ylabel('����')
grid on

subplot(313)
plot(Voice456)
ylabel('����')
grid on