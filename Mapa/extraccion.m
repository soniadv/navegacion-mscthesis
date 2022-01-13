t=1311878193;
%% Descomentar para extraer imágenes rgb
% p=100001;
% for i = 1:count-1
%     newfilename = strcat(num2str(t),'.', num2str(p), '.png');
%     imwrite(imagenes(:,:,:,i), newfilename);
%     p=p+3;
% end

%% % Descomentar para extraer imágenes de profundidad
p=100010;
for i = 1:count-1
    newfilename = strcat(num2str(t),'.', num2str(p), '.png');
    imwrite(profundidad(:,:,i), newfilename);
    p=p+3;
end