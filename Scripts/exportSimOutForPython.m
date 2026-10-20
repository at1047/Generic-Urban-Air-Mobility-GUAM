function outputFile = exportSimOutForPython(inputFile, outputFile)
%EXPORTSIMOUTFORPYTHON Convert logged timeseries to plain Data/Time structs.
% Run once in MATLAB before using Scripts/plot_simout.py:
%   addpath('Scripts')
%   exportSimOutForPython('Exec_Scripts/SimOut_case6.mat')
% This creates SimOut_case6_python.mat and does not change the input file.
% Signal values, units, timestamps, and sample orientation are preserved.
if nargin < 2
    [folder, name] = fileparts(inputFile);
    outputFile = fullfile(folder, [name '_python.mat']);
end
if strcmp(char(java.io.File(inputFile).getCanonicalPath()), ...
          char(java.io.File(outputFile).getCanonicalPath()))
    error('Choose an output file different from the input file.');
end
loaded = load(inputFile, 'SimOut');
SimOut = plainData(loaded.SimOut);
save(outputFile, 'SimOut', '-v7');
fprintf('Saved Python-readable signals to %s\n', outputFile);
end

function out = plainData(in)
if isa(in, 'timeseries')
    out = struct('Data', in.Data, 'Time', in.Time, 'IsTimeFirst', in.IsTimeFirst);
elseif isstruct(in)
    out = in;
    names = fieldnames(in);
    for element = 1:numel(in)
        for k = 1:numel(names)
            out(element).(names{k}) = plainData(in(element).(names{k}));
        end
    end
elseif iscell(in)
    out = cellfun(@plainData, in, 'UniformOutput', false);
else
    out = in;
end
end
