function  drawKinectImage(myTurtle, kinect_rgb_sub, kinect_depth_sub)
%DRAWKINECTIMAGE Summary of this function goes here
%   Detailed explanation goes here
    img_rgb = myTurtle.receive(kinect_rgb_sub);
    
    img_depth = myTurtle.receive(kinect_depth_sub);
    depth     = readImage(img_depth);
    img       = readImage(img_rgb);

    v = get(gca,'view');
    surf(depth, img, 'FaceColor', 'texturemap', 'EdgeColor', 'none');
    set(gca,'view',v);    
    drawnow;
end