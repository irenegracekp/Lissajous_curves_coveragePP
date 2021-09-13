t = 0:2*pi/100:2*pi;

lissajous = @(m,n,d) sin(m*t) + 1i*cos(n*t+pi*d);

f = lissajous(5,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl = real(f);
yl = imag(f);

length = 2;
r = 10*length/(2*pi);
z = yl;
theta1 = xl*2*pi/length;

k = 1;
for i=1:101
    
  if(abs(theta1(i)- 3.1416) > 0)
      theta(k) = theta1(i);
      k = k + 1;
  end 
end

x = r*sin(theta);
y = r*cos(theta);

l = size(x);

scatter3(x(1,1:fix(l(2)/2)),y(1,1:fix(l(2)/2)),z(1,1:fix(l(2)/2)))
