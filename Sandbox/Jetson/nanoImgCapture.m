function nanoImgCapture()
%NANOIMGCAPTURE Summary of this function goes here
%   Detailed explanation goes here
%#codegen

hwobj = jetson;
%camlist = getCameraList(hwobj);
camName = 'vi-output, imx219 6-0010';
camRes = [1280 720];


camObj = camera(hwobj,camName,camRes);
dispObj = imageDisplay(hwobj); %for troubleshooting

for i = 1:100
    img = snapshot(camObj);
    image(dispObj, img);




    
end
