% getXYZ_fromZ

% parametros de calibracion kinect 2
fx = 365.775391;
fy = 365.775391;
cx = 255.634293;
cy = 207.1539;
k1 = 0.0935848877; 
k2 = -0.271052808; 
k3 = 0.0922934487; 
p1 = 0; 
p2 = 0; 


nf = 424; nc = 512; % numero de filas y columnas de la imagen

% LEER IMAGEN 10 (como ejemplo, aunque se puede leer cualquiera).
file_name = './calibration/calibration-000010.depth'; 
[fid message] = fopen(file_name); 
if fid == -1; 
    error(message);        
else 
    % lo devuelve en forma de fila
    depth = fread(fid, nc*nf, 'float=>float');
    fclose(fid);
    % se convierte en una matriz 
%     depth_im = reshape(fila,nc,nf)';    
end

% depth_im es directamente la coordenada Z
Z = double(depth); 

% Si no consideramos distorsión: 
% xi, yi -> coordenadas en plano imagen (1:512, 1:424)
% X, Y, Z, -> coordenadas 3D
% xi = fx*(X/Z) + cx --> X = Z*(xi-cx)/fx
% yi = fy*(Y/Z) + cy --> Y = Z*(yi-cy)/fy

% obtenemos las coordenadas en el plano imagen (xi, yi) para cada punto: 
% xi = [1 2 3 ... nc 1 2 3 ... nc ... ]' -> 1*(nf*nc)
% yi = [1 1 1 ... 1  2 2 2 ... 2  ... nf nf nf ... nf]' -> 1*(nf*nc)
xi = ones(nf,1)*[1:nc];  xi = reshape(xi, nf*nc, 1); 
yi = [1:nf]'*ones(1,nc); yi = reshape(yi, nf*nc, 1); 

% se obtienen las coordenadas métricas X e Y (en las mismas unidades 
X = Z.*(xi - cx)./fx; 
Y = Z.*(yi - cy)./fy; 
 
% se almacenan en un array para posteriormente guardarlas a disco: 
XYZ(1:3:nf*nc*3,1) = X; 
XYZ(2:3:nf*nc*3,1) = Y; 
XYZ(3:3:nf*nc*3,1) = Z; 

file_name_xyz = './calibration/calibration-000010.xyz'; 
fid = fopen(file_name_xyz, 'w'); 
count = fwrite(fid, XYZ, 'float'); 
fclose(fid);







