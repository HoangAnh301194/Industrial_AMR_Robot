function amr_save_png(fig, filename)
%AMR_SAVE_PNG Xuất PNG, không cần toolbox ngoài MATLAB cơ bản.
set(fig,'PaperPositionMode','auto');
print(fig,filename,'-dpng','-r180');
close(fig);
end
