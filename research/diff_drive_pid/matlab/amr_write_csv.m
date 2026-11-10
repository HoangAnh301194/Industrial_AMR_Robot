function amr_write_csv(filename, header, data)
%AMR_WRITE_CSV Lưu dữ liệu dạng CSV để kiểm tra bằng công cụ khác.
fid=fopen(filename,'w');
if fid < 0, error('Khong mo duoc file: %s',filename); end
cleaner=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',strjoin(header,','));
ncol=size(data,2);
fmt=[repmat('%.9g,',1,ncol-1),'%.9g\n'];
fprintf(fid,fmt,data.');
end
