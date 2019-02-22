clear;
clf;

function [t1, t2] = inverse_kinematics(x, y, l1, l2)
  l = sqrt(x^2+y^2);
  t1 = t2 = 0;
  if (l <= (l1+l2))
    t1 = atan2(y,x) + acos((l1^2+l^2-l2^2)/(2*l2*l));
    t2 = pi + acos((l1^2+l2^2-l^2)/(2*l1*l2));
  endif
endfunction

function [x, y] = bezier(P, t)
  B =[(1-t)^3, 3*t*(1-t)^2, 3*t^2*(1-t), t^3];
  X = B * P;
  x = X(1);
  y = X(2);
endfunction

function [xd, yd] = bezier_d(P, t)
  Bd =[-3*(1-t)^2, 3*(1-t)^2-6*t*(1-t), 6*t*(1-t)-3*t^2, 3*t^2];
  Xd = Bd * P;
  xd = Xd(1);
  yd = Xd(2);
endfunction

l1 = 0.108;
l2 = 0.108;
ratio = 0.1;
motor_velocity = 4.81; # 46rpm

P = [0, 0; 0, 0.1; 0.3, 0.1; 0.2, 0.0];
period = 0.32;

i = 0;
for t = 0.0: 0.2: 1.0
  [xt, yt] = bezier(P, t);
  x = xt - 0.1;
  y = yt - 0.18;

  [t1, t2] = inverse_kinematics(x, y, l1, l2);
  if (t1 != 0 || t2 != 0)
    i = i + 1;
    x1 = l1 * cos(t1);
    y1 = l1 * sin(t1);
    arm_x(i,:) = [0 x1 x];
    arm_y(i,:) = [0 y1 y];
    jaco = [-l1*sin(t1) -l2*sin(t2); l1*cos(t1) l2*cos(t2)];
    [U,S,V] = svd(jaco);
    mani(i,:) = [x y atan2(U(2),U(1)) S(1,1) S(2,2)];
	[xd, yd] = bezier_d(P, t);
    xdp(i,:) = [x x + xd / period / motor_velocity * ratio];
    ydp(i,:) = [y y + yd / period / motor_velocity * ratio];
  endif
endfor

hold on;
axis([-0.3,0.3,-0.3,0.3],"square");
for n = 1:i
  plot(arm_x(n,:), arm_y(n,:));
  ellipse(mani(n,1), mani(n,2), mani(n,4)*ratio, mani(n,5)*ratio, mani(n,3));
  plot(xdp(n,:), ydp(n,:), "r");
  if (mani(n,3) < 0)
    mani(n,3) += 2 * pi;
  endif
  printf("(velocity, angle): (%f, %f, %f)\n", mani(n,4) * motor_velocity, mani(n,5) * motor_velocity, mani(n,3)-pi);
end
hold off;

