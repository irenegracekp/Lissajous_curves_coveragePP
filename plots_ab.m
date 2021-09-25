t = 0:2*pi/100:2*pi;

lissajous = @(m,n,d) sin(m*t) + 1i*cos(n*t+pi*d);

f = lissajous(5,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl = real(f);
ylk = imag(f);

f1 = lissajous(1,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl1 = real(f1);
ylk1 = imag(f1);

f2 = lissajous(sqrt(2),1,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl2 = real(f2);
ylk2 = imag(f2);

f3 = lissajous(1,sqrt(2),180);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl3 = real(f3);
ylk3 = imag(f3);


length = 2;
r = 20*length/(2*pi);

theta1 = xl*2*pi/length;


x = 0; y = 0; z = 0;

x = r*sin(theta);
y = r*cos(theta);
z = yl;

path(1,:) = [0 0 0];
path(2:102,:) = [x' y' z'+2];
thetad = wrapToPi([0 theta]' + pi);
 
 figure
 sgtitle('Lissajous Curves with different a/b ratios')
 subplot(2,2,1)
 hold on
 plot(xl,ylk,'LineWidth', 1.5);
 xlabel('X');
 xlabel('Y');
 title('a/b = 3/1')
 grid on
 hold off
 subplot(2,2,2)
 hold on
 plot(xl1,ylk1,'LineWidth', 1.5);
 xlabel('X');
 xlabel('Y');
 title('a/b = 1/3')
 grid on
 hold off
 
 subplot(2,2,3)
 hold on
 plot(xl2,ylk2,'LineWidth', 1.5);
 xlabel('X');
 xlabel('Y');
 title('a/b = {\surd}2/1')
 grid on
 hold off
 
 subplot(2,2,4)
 hold on
 plot(xl3,ylk3,'LineWidth', 1.5);
 xlabel('X');
 xlabel('Y');
 title('a/b = 1/{\surd}2')
 grid on
 hold off