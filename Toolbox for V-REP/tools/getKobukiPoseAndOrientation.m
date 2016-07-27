function [x, y, theta] = getKobukiPoseAndOrientation(odom_msg)
%GETKOBUKIPOSEANDORIENTATION retrieves the x, y and theta pose of Kobuki
    x      = odom_msg.Pose.Pose.Position.X;
    y      = odom_msg.Pose.Pose.Position.Y;
    quat   = odom_msg.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta  = rad2deg(angles(1));
end

