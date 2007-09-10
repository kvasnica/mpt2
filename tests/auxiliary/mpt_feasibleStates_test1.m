function mpt_feasibleStates_test1

Double_Integrator;
probStruct.Tconstraint = 0;
probStruct.N = 2;
ctrl = mpt_control(sysStruct, probStruct);

%---------------------------------------------------------
fprintf('First argument has to be a controller...\n');
lasterr('');
try
    x = mpt_feasibleStates(unitbox(2));
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Unknown type of first input argument.');


%---------------------------------------------------------
fprintf('\n\nReject on-line controllers...\n');
lasterr('');
Double_Integrator;
probStruct.Tconstraint = 0;
ctrl_onl = mpt_control(sysStruct, probStruct, 'online');
try
    x = mpt_feasibleStates(ctrl_onl);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'This function supports only explicit controllers!');


%---------------------------------------------------------
fprintf('\n\nAccept explicit controllers, gridpoints=30 by default...\n');
lasterr('');
x = mpt_feasibleStates(ctrl);
mbg_assertequal(size(x), [744 2]);


%---------------------------------------------------------
fprintf('\n\nAccept explicit controllers, custom gridpoints...\n');
lasterr('');
x = mpt_feasibleStates(ctrl, 10);
mbg_assertequal(size(x), [80 2]);


%---------------------------------------------------------
fprintf('\n\nAccept controller structures, custom gridpoints...\n');
lasterr('');
x = mpt_feasibleStates(struct(ctrl), 10);
mbg_assertequal(size(x), [80 2]);


%---------------------------------------------------------
fprintf('\n\nReject invalid controller structures...\n');
lasterr('');
cs = struct(ctrl);
cs.Pn = cs.Pn(1);
warning off
try
    x = mpt_feasibleStates(cs, 10);
    worked = 1;
catch
    worked = 0;
end
warning on
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'First argument has to be a valid controller structure');

