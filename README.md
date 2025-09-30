# Tinh chỉnh và Phân tích Đám mây điểm 3D

Phiên bản README được viết theo chuẩn GitHub: mô tả ngắn gọn, hướng dẫn cài đặt, cách sử dụng, ví dụ và thông tin đóng góp.

## Tổng quan

Repository chứa một script Python để tiền xử lý và phân tích đám mây điểm 3D (định dạng .ply). Các chức năng chính:

- Khử nhiễu (Statistical / Radius outlier removal)
- Phân đoạn mặt phẳng (RANSAC) để phát hiện sàn nhà
- Tính trị trung bình z (zmean) của sàn
- Xoay đám mây để sàn song song với mặt phẳng XY
- Xuất file đám mây đã xử lý và báo cáo phân tích

## Yêu cầu

- Python 3.7+
- Các thư viện trong `requirements.txt`

Bạn có thể cài nhanh bằng pip:

```powershell
pip install -r requirements.txt
```

## Cấu trúc dự án

```
pointcloud_processing/
├─ process_pointcloud.py    # Script chính (CLI)
├─ requirements.txt         # Thư viện cần thiết
├─ report.pdf               # Báo cáo (nếu có)
└─ ...
```

## Cách sử dụng

Chạy script cơ bản (PowerShell / CMD):

```powershell
python process_pointcloud.py --input path\to\input.ply --output path\to\output_folder
```

Ví dụ đầy đủ với các tuỳ chọn:

```powershell
python process_pointcloud.py --input data/input.ply --output results --visualize --noise_method radius --nb_neighbors 30 --std_ratio 0.05 --plane_threshold 0.01
```

### Các tham số CLI chính

- `--input` (bắt buộc): đường dẫn tới file .ply đầu vào
- `--output` (bắt buộc): thư mục lưu kết quả
- `--visualize` (tùy chọn): nếu có sẽ hiển thị đám mây điểm (gui)
- `--noise_method` (tùy chọn): `statistical` hoặc `radius` (mặc định: `statistical`)
- `--nb_neighbors` (tùy chọn): số điểm láng giềng cho khử nhiễu (mặc định: 20)
- `--std_ratio` (tùy chọn): tỷ lệ độ lệch chuẩn cho SOR (mặc định: 2.0)
- `--plane_threshold` (tùy chọn): ngưỡng khoảng cách cho RANSAC (mặc định: 0.02)
- `--help`: hiển thị trợ giúp

Gợi ý: dùng đường dẫn tuyệt đối hoặc đường dẫn tương đối đúng theo shell của bạn. Trong PowerShell, dùng `\` hoặc dấu `/` đều được.

## Kết quả đầu ra

Sau khi chạy, script sẽ tạo (ít nhất) các file sau trong thư mục `--output`:

- `output.ply` — đám mây điểm đã khử nhiễu và xoay đúng hướng
- `zmean.txt` — giá trị trung bình z của bề mặt được phát hiện
- `analysis_report.txt` — báo cáo tóm tắt các phép đo và tham số đã dùng

Ghi chú: Tên file có thể khác nếu script được cấu hình để ghi thêm thông tin thời gian hoặc hậu tố.

## Ví dụ luồng xử lý ngắn

1. Đọc file .ply
2. Khử nhiễu (Statistical hoặc Radius)
3. Phân đoạn mặt phẳng (RANSAC) để lấy các điểm thuộc sàn
4. Tính zmean của sàn
5. Xoay toàn bộ đám mây để sàn song song với mặt phẳng XY
6. Lưu kết quả và xuất báo cáo

## Kiểm tra nhanh (smoke test)

1. Chuẩn bị một file `sample.ply` nhỏ trong thư mục dự án hoặc `data/`
2. Chạy:  `python process_pointcloud.py --input sample.ply --output tmp_results`
3. Kiểm tra xem `tmp_results/output.ply`, `tmp_results/zmean.txt` và `tmp_results/analysis_report.txt` xuất hiện

## Phát triển & Đóng góp

- Muốn báo lỗi / yêu cầu tính năng: tạo issue trên GitHub
- Muốn gửi PR: fork repository, tạo branch mới, commit thay đổi kèm test nếu có, rồi gửi pull request
- Coding style: theo chuẩn Python (PEP8). Thêm tests nếu thay đổi logic xử lý đám mây điểm.

## License

Mặc định README này giả thiết license MIT. Nếu bạn muốn một license khác, hãy thêm file `LICENSE` vào repository. Hiện tại file license chưa được cung cấp.

## Liên hệ

Nếu cần trợ giúp thêm, bỏ thông tin liên hệ hoặc email tại đây (ví dụ: tên và email của tác giả / maintainer).

---

Cảm ơn bạn đã sử dụng/đóng góp cho dự án! Nếu muốn mình cập nhật README bằng tiếng Anh, hoặc thêm nhãn badges (build / pypi / license), nói mình biết.
