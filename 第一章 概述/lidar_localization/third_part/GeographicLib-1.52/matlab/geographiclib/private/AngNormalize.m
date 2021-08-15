function x = AngNormalize(x)
%ANGNORMALIZE  Reduce angle to range (-180, 180]
%
%   x = ANGNORMALIZE(x) reduces angles to the range (-180, 180].  x can be
%   any shape.

  x = remx(x, 360);
  x(x == -180) = 180;
end
