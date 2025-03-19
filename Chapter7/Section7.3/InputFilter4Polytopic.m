% Author: Olivier Sename
% 
% Description
%  Modify the generalized plants adding an input filter in order to comply with polytopic
%  synthesis conditions: so [B2 D12] and [C2 D21] must be parameter independent
%  see [Apkarian et al., 1995].
%
% Input
%  listP  : list of generalized plants
%  Xdot    A  B1  B2    X
%   Z   =  C1 D11 D12   W
%   Y      C2 D21 D22   U
%  nmeas  : number of measures
%  ncon   : number of control signals
%  filter= LTI filter 
% 
% Output
%  listPf  : list of  filtered generalized plants
%
% [listPf] = InputFilter4Polytopic(listP,nmeas,ncon,filter)

function [listPf] = InputFilter4Polytopic(listP,nmeas,ncon,filter)
%%% Size of the generalized plant
sizeX = size(listP{1}.a,1);
sizeZ = size(listP{1},1)-nmeas; % number of controlled output - number of measure
sizeY = nmeas;
sizeW = size(listP{1},2)-ncon;  % number of input - number of control
sizeU = ncon;

for i = 1:size(listP,2)
    A{i}   = listP{i}.a(1:sizeX,1:sizeX);
    B1{i}  = listP{i}.b(1:sizeX,1:sizeW);
    B2{i}  = listP{i}.b(1:sizeX,sizeW+1:sizeW+sizeU);
    C1{i}  = listP{i}.c(1:sizeZ,1:sizeX);
    D11{i} = listP{i}.d(1:sizeZ,1:sizeW);
    D12{i} = listP{i}.d(1:sizeZ,sizeW+1:sizeW+sizeU);
    C2{i}  = listP{i}.c(sizeZ+1:sizeZ+sizeY,1:sizeX);
    D21{i} = listP{i}.d(sizeZ+1:sizeZ+sizeY,1:sizeW);
    D22{i} = listP{i}.d(sizeZ+1:sizeZ+sizeY,sizeW+1:sizeW+sizeU);
end;

ERROR = 0;
for i = 1:size(listP,2)
    if (B2{1}==B2{i})
    else
        disp('To be corrected: System must be parameter independent on the input')
        ERROR = 1;
    end
    if (D12{1}==D12{i})
    else
        disp('To be corrected: System must be parameter independent on the input')
        ERROR = 1;
    end
    
    
    if (C2{1}==C2{i})
    else
        disp('To be corrected: System must be parameter independent on the output')
        ERROR = 1;
    end
    if (D21{1}==D21{i})
    else
        disp('To be corrected: System must be parameter independent on the output')
        ERROR = 1;
    end
    
    
    if (D22{i}==zeros(sizeY,sizeU))
    else
        disp('To be corrected: D22 should be null')
        ERROR = 1;
    end
end;

% extraction filter state space matrices
Fa=filter.a;
Fb=filter.b;
Fc=filter.c;
Fd=filter.d;
sizeF = size(Fb,2);

for i = 1:size(listP,2)
Af{i}=[A{i} B2{i}*Fc;zeros(sizeU,sizeX) Fa];
Bf1{i}=[B1{i}; zeros(sizeF,sizeW)];
Bf2{i}=[zeros(sizeX,1);Fb];
Cf1{i}  = [C1{i} D12{i}*Fc];
Df11{i} = D11{i};
Df12{i} = zeros(sizeZ,ncon);
Cf2{i}  = [C2{i}, zeros(sizeY,sizeU)];
Df21{i} = D21{i};
Df22{i} = zeros(sizeU,sizeU);
listPf{i}  = ss(Af{i},[Bf1{i} Bf2{i}],[Cf1{i};Cf2{i}],[Df11{i}, Df12{i};Df21{i}, Df22{i}]);
end
disp('-----------------------------------------------------------------------------------')
disp('Application of filtering method to comply with the polytopic method requirement')
disp('check results' )
ERROR = 0;
for i = 1:size(listPf,2)
    if (Bf2{1}==Bf2{i})
    else
        disp('Error: System must be parameter independent on the input')
        ERROR = 1;
    end
    if (Df12{1}==Df12{i})
    else
        disp('Error: System must be parameter independent on the input')
        ERROR = 1;
    end
    
    
    if (Cf2{1}==Cf2{i})
    else
        disp('Error: System must be parameter independent on the output')
        ERROR = 1;
    end
    if (Df21{1}==Df21{i})
    else
        disp('Error: System must be parameter independent on the output')
        ERROR = 1;
    end
    
    
    if (Df22{i}==zeros(sizeY,sizeU))
    else
        disp('Error: D22 should be null')
        ERROR = 1;
    end
end;

disp('check done')

end