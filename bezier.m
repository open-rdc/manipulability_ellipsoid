%clear;
%clf;

function [x, y] = bezier(P, t)
  B =[(1-t)^3, 3*t*(1-t)^2, 3*t^2*(1-t), t^3];
  X = B * P;
  x = X(1);
  y = X(2);
endfunction

%i = 0
%P = [0, 0; 0, 0.06; 0.1+0.07, 0.06; 0.1, 0.0]
%ttt = 0.0:0.1:1.0
%for tt = ttt
%  i = i + 1
%  [x(i,:), y(i,:)] = bezier(P, tt)
%endfor
%plot(x, y);

