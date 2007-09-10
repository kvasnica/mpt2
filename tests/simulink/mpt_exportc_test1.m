function mpt_exportc_test1

% tests that we can call mpt_exportc() with 2 inputs:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=1ea70666675c

load ctrl1

% this one should generate the file "mpt_getInput.h"
mpt_exportc(ctrl);
fileisthere = exist('mpt_getInput.h', 'file');
if fileisthere,
    % remove the file
    try
        delete('mpt_getInput.h');
    end
end
mbg_asserttrue(fileisthere);


% this one should generate the file "myctrl.h"
mpt_exportc(ctrl, 'myctrl.h');
fileisthere = exist('myctrl.h', 'file');
if fileisthere,
    % remove the file
    try
        delete('myctrl.h');
    end
end
mbg_asserttrue(fileisthere);
