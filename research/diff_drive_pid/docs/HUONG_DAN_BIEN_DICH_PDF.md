# Biên dịch báo cáo PDF

Yêu cầu `pandoc`, XeLaTeX và font Tinos (hoặc thay font bằng Times New Roman có sẵn). Ở thư mục `research/diff_drive_pid/`:

```bash
pandoc docs/00_BAO_CAO_DAY_DU_MO_PHONG_MATLAB.md -o Robot_Vi_Sai_Dong_Hoc_Dong_Luc_PID.pdf \
  --pdf-engine=xelatex --resource-path=docs --toc \
  -V mainfont=Tinos -V fontsize=12pt -V linestretch=1.15 \
  -V geometry:margin=2.1cm -V colorlinks=true
```

Nếu PDF có các hình `extended_*.png`, chúng là kết quả Python minh họa. Không dùng PDF này như báo cáo MATLAB/Gazebo thực tế nếu chưa chạy kịch bản và lấy log.
