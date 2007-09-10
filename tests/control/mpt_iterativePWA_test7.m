function mpt_iterativePWA_test7

% using a control horizon of infinity should not cause any problems

pwa1d
probStruct.N = Inf;
probStruct.Nc = Inf;

ctrl = mpt_control(sysStruct,probStruct);
mbg_assertequal(length(ctrl), 6);
