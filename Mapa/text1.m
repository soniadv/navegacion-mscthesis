t=1311878193;

% fid = fopen('rgb.txt','wt');
% p=100001;
% for i = 1:count-1
%     fprintf(fid, strcat(num2str(t),'.', num2str(p),' rgb/',num2str(t),'.', num2str(p), '.png \n'));
%     p=p+3;
% end

fid = fopen('depth.txt','wt');
p=100010;
for i = 1:count-1
    fprintf(fid, strcat(num2str(t),'.', num2str(p),' depth/',num2str(t),'.', num2str(p), '.png \n'));
    p=p+3;
end

 fclose(fid);