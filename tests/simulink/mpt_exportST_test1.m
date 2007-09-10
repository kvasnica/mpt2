function mpt_exportST_test1

% tests the mpt_exportST() function which should generate a C-code which
% implements binary search trees:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=e8292128438b

%fprintf('no search tree for this controller, the function should fail\n');
load ctrl1
lasterr('');
try
    mpt_exportST(ctrl);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Search tree is not present in the controller. Use "mpt_searchTree(ctrl)" to compute it.');


fprintf('\nSearch tree should be written to the default file "searchtree.c"\n');
load ctrl_1d_searchtree
mpt_exportST(ctrl_st);
fileisthere = exist('searchtree.c', 'file');
if fileisthere,
    % remove the file
    try
        delete('searchtree.c');
    end
end
mbg_asserttrue(fileisthere);

fprintf('\nSearch tree should be written to a custom file "myctrl.c"\n');
load ctrl_1d_searchtree
mpt_exportST(ctrl_st, 'myctrl.c');
fileisthere = exist('myctrl.c', 'file');
if fileisthere,
    % remove the file
    try
        delete('myctrl.c');
    end
end
mbg_asserttrue(fileisthere);
