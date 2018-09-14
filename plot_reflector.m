% Plot reflectors and highlight in the map
% need to load detected reflector array
function plot_reflector(detected_reflector,detected_ID,color)

hold on;plot(detected_reflector(1,:),detected_reflector(2,:),'or')    
a = detected_ID'; b = num2str(a); c = cellstr(b);
dx = 5; dy = 5;
hold on;
h=text(detected_reflector(1,:)+dx,detected_reflector(2,:)+dy, c);
set(h, 'Color',color);% ??????