function Delta = DeltaTheta(s,d)
%DELTATHETA Summary of this function goes here
%   Detailed explanation goes here

% S ranges from [-360,360]
% d ranges from [-180,180]

d_o=d

d_p=(2*180)+d_o

s_p=(2*180)+s

Do=d_o-s
Dp=d_p-s
Di=d_o-s_p

Vout=[Do,Dp,Di]

aVout=abs(Vout)

if aVout(1)>aVout(2)
    if aVout(2)>aVout(3)
        Delta = Vout(3)
    else
        Delta = Vout(2)
    end
else
    if aVout(1)>aVout(3)
        Delta = Vout(3)
    else
        Delta = Vout(1)
    end
end


if Delta>=360
    Delta=Delta-360
else




end

