function [is_ok, msg, data] = isel_cmd(command, varargin)
% ISEL_CMD Sends commands to the Isel Controller Python API.
%
% Usage:
%   [is_ok, msg] = isel_cmd('init');
%   [is_ok, msg] = isel_cmd('home', 'wait_ready', true);
%   [is_ok, msg, data] = isel_cmd('move_abs', 'x', 50, 'y', 20, 'speed', 15, 'accel', 800, 'wait_ready', true);
%   [is_ok, msg, data] = isel_cmd('get_pos');
%
% Note: The function keeps a persistent TCP connection open to minimize overhead.

persistent t;

% Initialize connection if it does not exist or was closed
if isempty(t) || ~isvalid(t)
    try
        t = tcpclient("127.0.0.1", 5000, "Timeout", 300);
    catch ME
        is_ok = false;
        msg = ['Connection failed: ', ME.message];
        data = [];
        return;
    end
end

% Flush leftover data in the buffer to prevent reading old responses
if t.NumBytesAvailable > 0
    read(t, t.NumBytesAvailable);
end

% Build the request structure
req = struct();
req.cmd = command;

% Parse name-value pairs
for i = 1:2:length(varargin)
    if i+1 <= length(varargin)
        req.(varargin{i}) = varargin{i+1};
    end
end

% Convert to JSON and send
try
    json_str = jsonencode(req);
    write(t, uint8([json_str, newline]));
    
    % Wait for the response (blocks until newline is received)
    response_str = readline(t);
    
    if isempty(response_str)
        is_ok = false;
        msg = 'No response from server.';
        data = [];
        return;
    end
    
    % Decode JSON
    data = jsondecode(response_str);
    
    is_ok = strcmp(data.status, 'ok');
    if isfield(data, 'msg')
        msg = data.msg;
    else
        msg = '';
    end
    
catch ME
    is_ok = false;
    msg = ['Communication error: ', ME.message];
    data = [];
end
end
