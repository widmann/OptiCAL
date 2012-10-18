% OptiCAL - Psychtoolbox IOPort interface to the CRS OptiCAL luminance
%           meter device
%
% Usage:
%   >> handle = OptiCAL('Open', port)
%   >> lum = OptiCAL('Read', handle)
%   >> OptiCAL('Close', handle)
%
% Inputs:
%   port      - string device (e.g. '/dev/ttyS0')
%   handle    - double handle
%
% Outputs:
%   handle    - double handle
%   lum       - luminance in cd/m^2
%
% Note:
%   Requires read and write access to the serial port (Ubuntu: add user to
%   the dialout group). Not yet tested with USB-to-serial adapters.
%
% References:
%   [1] Haenel, V. (2010). Pure python interface to CRS OptiCAL. Retrieved
%       October 18, 2012 from https://github.com/esc/pyoptical.
%   [2] Cambridge Research Systems. (1995). OptiCAL user's guide v4.02.
%       Retrieved October 18, 2012 from
%   	http://support.crsltd.com/ics/support/DLRedirect.asp?fileID=63194
%   [3] Cambridge Research Systems. (2009). OptiCAL user's guide v4.02
%       errata. Retrieved October 18, 2012 from
%   	http://support.crsltd.com/ics/support/DLRedirect.asp?fileID=63194
%
% Author: Andreas Widmann, University of Leipzig, 2012

%123456789012345678901234567890123456789012345678901234567890123456789012

% Copyright (C) 2012 Andreas Widmann, University of Leipzig, widmann@uni-leipzig.de
%
% This program is free software; you can redistribute it and/or modify it
% under the terms of the GNU General Public License as published by the
% Free Software Foundation; either version 2 of the License, or (at your
% option) any later version.
%
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details.
%
% You should have received a copy of the GNU General Public License along
% with this program; if not, write to the Free Software Foundation, Inc.,
% 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA

function [ ret ] = OptiCAL(command, handle)

persistent OptiCALCfg;

switch command

    case 'Open'
        handle = OptiCALOpen(handle);
        OptiCALCfg = OptiCALConfiguration(handle);
        ret = handle;

    case 'Close'
        OptiCALClose(handle);

    case 'Read'
        adc = OptiCALRead(handle);

        % Convert to luminance
        adc_adj = adc - OptiCALCfg.Z_count - 524288; % From OptiCAL manual
        lum = ((adc_adj / 524288) * OptiCALCfg.V_ref * 1e-6) / (OptiCALCfg.R_feed * OptiCALCfg.K_cal * 1e-15); % From pyoptical! See errata!!!
        ret = max([0 lum]);

end

end

function [handle, OptiCALCfg] = OptiCALOpen(port)

    % Open serial port
    OptiCALCfg.port = port;
    handle = IOPort('OpenSerialPort', OptiCALCfg.port);
    WaitSecs('YieldSecs', 0.1); % Why needed? Does not reliably work w/o.

    % Calibrate
    IOPort('Write', handle, 'C');
    data = IOPort('Read', handle, 1, 1);
    if isempty(data) || data ~= 6, error('Could not calibrate OptiCAL device.'), end

    % Set in current Mode
    IOPort('Write', handle, 'I');
    data = IOPort('Read', handle, 1, 1);
    if isempty(data) || data ~= 6, error('Could not set OptiCAL device into current mode.'), end

end

function OptiCALClose(handle)

     IOPort('Close', handle);

end

function OptiCALCfg = OptiCALConfiguration(handle)

    % Read configuration
    OptiCALCfg.prod_type  = OptiCALReadEEPROM(handle,  0,  2, 'int');
    OptiCALCfg.optical_sn = OptiCALReadEEPROM(handle,  2,  4, 'int');
    OptiCALCfg.fw_vers    = OptiCALReadEEPROM(handle,  6,  2, 'int');
    OptiCALCfg.V_ref      = OptiCALReadEEPROM(handle, 16,  4, 'int');
    OptiCALCfg.Z_count    = OptiCALReadEEPROM(handle, 32,  4, 'int');
    OptiCALCfg.R_feed     = OptiCALReadEEPROM(handle, 48,  4, 'int');
    OptiCALCfg.R_gain     = OptiCALReadEEPROM(handle, 64,  4, 'int');
    OptiCALCfg.probe_sn   = OptiCALReadEEPROM(handle, 80, 16, 'char');
    OptiCALCfg.K_cal      = OptiCALReadEEPROM(handle, 96,  4, 'int');

end

function val = OptiCALReadEEPROM(handle, address, nBytes, toType)

    offset = 128;

    % Read field from EEPROM
    data = zeros(1, nBytes);
    for iByte = 1:nBytes;
        IOPort('Write', handle, uint8(offset + address + iByte - 1));
        temp = IOPort('Read', handle, 1, 2);
        data(iByte) = temp(1);
    end

    % Cast to type
    if strcmp(toType, 'char')
        val = char(data);
    else
        val = sum(pow2(0:8:8 * (nBytes - 1)) .* data);
    end

end

function adc = OptiCALRead(handle)

    IOPort('Write', handle, 'L'); % See errata!!!
    temp = IOPort('Read', handle, 1, 4);
    adc = sum(pow2([0 8 16]) .* temp(1:3));

end
