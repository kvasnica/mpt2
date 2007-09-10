function mpt_init_hook(action)

persistent mptOptions_private
global mptOptions

if nargin < 1
    return
end

switch lower(action)
    case 'init'
        mptOptions_private = mpt_init('rehash');
        
    case 'restore'
        mptOptions = mptOptions_private;
        
    otherwise
        error(sprintf('Uknown action "%s"', action));
        
end
