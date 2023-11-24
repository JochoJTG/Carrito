clear all
close all
clc

% Leer los datos de Excel
datos = xlsread('DatosMag.xlsx');
datos(any(isnan(datos), 2), :) = [];

% Extraer los datos de cada eje
x = datos(:,1);
y = datos(:,2);
z = datos(:,3);

% Ordenar los datos de cada eje
x_sorted = sort(x, 'ascend');
y_sorted = sort(y, 'ascend');
z_sorted = sort(z, 'ascend');

% Obtener los promedios de los 20 datos más grandes y más pequeños
x_prom_20_min = mean(x_sorted(1:20));
x_prom_20_max = mean(x_sorted(end-19:end));
y_prom_20_min = mean(y_sorted(1:20));
y_prom_20_max = mean(y_sorted(end-19:end));
z_prom_20_min = mean(z_sorted(1:20));
z_prom_20_max = mean(z_sorted(end-19:end));

% Calcular el centro de la esfera
x_center = (x_prom_20_max + x_prom_20_min) / 2;
y_center = (y_prom_20_max + y_prom_20_min) / 2;
z_center = (z_prom_20_max + z_prom_20_min) / 2;

% Centrar los datos
x_cent = x - x_center;
y_cent = y - y_center;
z_cent = z - z_center;

% Calcular el rango (distancia máxima desde el centro) para cada eje
x_range = x_prom_20_max - x_prom_20_min;
y_range = y_prom_20_max - y_prom_20_min;
z_range = z_prom_20_max - z_prom_20_min;

% Calcular el radio promedio de la esfera
radio_promedio = (x_range + y_range + z_range) / 6;

% Normalizar los datos a la esfera
x_norm = zeros(size(x_cent));
y_norm = zeros(size(y_cent));
z_norm = zeros(size(z_cent));
for i = 1:length(x)
    distancia_al_centro = sqrt(x_cent(i)^2 + y_cent(i)^2 + z_cent(i)^2);
    if distancia_al_centro ~= 0 % Evitar división por cero
        x_norm(i) = (x_cent(i) / distancia_al_centro); %* radio_promedio;
        y_norm(i) = (y_cent(i) / distancia_al_centro); %* radio_promedio;
        z_norm(i) = (z_cent(i) / distancia_al_centro); %* radio_promedio;
    end
end

% Graficar en 3D la esfera normalizada
scatter3(x_norm, y_norm, z_norm, 'filled');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Normalized Magnetometer Sphere');
grid on;
disp(x_center);
disp(y_center);
disp(z_center);