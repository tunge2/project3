function [ktheta]=R2kth(R)
  
  sin_theta=norm(vee(R-R')/2);
  k=vee(R-R')/2/sin_theta;
  theta=atan2(sin_theta,(trace(R)-1)/2);
  ktheta = [k' theta]';
end