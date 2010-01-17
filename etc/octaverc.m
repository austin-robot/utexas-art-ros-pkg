% Octave init file for the ART stacks (copy to ~/.octaverc).

% add ROS octave path
[status,rosoctpath] = system('rospack find rosoct');
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));

% add path for ART servo package
addpath(fullfile(rosoct_findpackage('art_servo'), 'octave'));

