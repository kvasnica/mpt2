function Matrices = mpt_yalmip2mpt(F, obj, parametric_vars, requested_vars)
% mpt_yalmip2mpt - expand YALMIP model and convert it to MPT format
%
% Objective(quadratic);
%   min U'*Matrices.H*U + Matrices.F'*x0*U
%
% Constraints:
%  Matrices.G*U <= Matrices.W + Matrices.E*x0
%  Matrices.Aeq*U + Matrices.Beq*x0 == Matrices.beq


global mptOptions
if ~isstruct(mptOptions),
    mpt_error;
end

% first expand the model
[Fexp, failure, cause] = expandmodel(F, obj);
if failure,
    fprintf('\n%s\n\n', cause);
    error('Cannot deal with given setup, see message above.');
end

% now export the expanded model into MPT format
yalmipOptions = mptOptions.sdpsettings;
yalmipOptions.expand = 0;
[a, b, c, model] = export(Fexp, obj, yalmipOptions);

% define which variables should be treated as parameters
model.parametric_variables = find(ismember(model.used_variables,getvariables(parametric_vars)));
model.requested_variables = find(ismember(model.used_variables,getvariables(requested_vars)));

% NOTE NOTE NOTE!!!
% this is a hack! export(), depending on the default solver, either sets
% "model.binary_variables" or "model.integer_variables". however the latter is
% not (currently) being used in yalmip2mpt(). since integer variables are not
% supported in YALMIP's interface to MPT, they are treated as binaries.
% therefore we have to update "model.binary_variables" accordingly, otherwise
% mpt_getInput() will think that the model has no binaries.
model.binary_variables = [model.binary_variables model.integer_variables];

Matrices = yalmip2mpt(model);
