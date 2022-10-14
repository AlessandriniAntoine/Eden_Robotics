function rosmsgOut = JointState(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Header = bus_conv_fcns.ros.busToMsg.std_msgs.Header(slBusIn.Header,rosmsgOut.Header(1));
    for iter=1:slBusIn.Name_SL_Info.CurrentLength
        rosmsgOut.Name{iter} = char(slBusIn.Name(iter).Data).';
        maxlen = length(slBusIn.Name(iter).Data);
        if slBusIn.Name(iter).Data_SL_Info.CurrentLength < maxlen
        rosmsgOut.Name{iter}(slBusIn.Name(iter).Data_SL_Info.CurrentLength+1:maxlen) = [];
        end
    end
    if slBusIn.Name_SL_Info.CurrentLength < numel(rosmsgOut.Name)
        rosmsgOut.Name(slBusIn.Name_SL_Info.CurrentLength+1:numel(rosmsgOut.Name)) = [];
    end
    rosmsgOut.Name = rosmsgOut.Name.';
    rosmsgOut.Position = double(slBusIn.Position(1:slBusIn.Position_SL_Info.CurrentLength));
    rosmsgOut.Velocity = double(slBusIn.Velocity(1:slBusIn.Velocity_SL_Info.CurrentLength));
    rosmsgOut.Effort = double(slBusIn.Effort(1:slBusIn.Effort_SL_Info.CurrentLength));
end
