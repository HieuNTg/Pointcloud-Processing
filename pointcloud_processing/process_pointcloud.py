import open3d as o3d
import numpy as np
import argparse
import copy
import matplotlib.pyplot as plt
import os
import time

def load_point_cloud(file_path):
    """Đọc file đám mây điểm"""
    print(f"Đang đọc file đám mây điểm từ {file_path}...")
    pcd = o3d.io.read_point_cloud(file_path)
    print(f"Đám mây điểm ban đầu có {len(pcd.points)} điểm")
    return pcd

def visualize_point_cloud(pcd, title="Point Cloud"):
    """Hiển thị đám mây điểm"""
    vis = o3d.visualization.Visualizer()
    vis.create_window(title, 1024, 768)
    vis.add_geometry(pcd)
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.array([0.1, 0.1, 0.1])
    vis.run()
    vis.destroy_window()

def remove_noise(pcd, nb_neighbors=20, std_ratio=2.0, method="statistical"):
    """Khử nhiễu từ đám mây điểm"""
    print(f"Đang khử nhiễu bằng phương pháp {method}...")
    start_time = time.time()
    
    if method == "statistical":
        # Statistical outlier removal
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        pcd_filtered = pcd.select_by_index(ind)
        print(f"Đã loại bỏ {len(pcd.points) - len(pcd_filtered.points)} điểm nhiễu")
    elif method == "radius":
        # Radius outlier removal
        cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=std_ratio)
        pcd_filtered = pcd.select_by_index(ind)
        print(f"Đã loại bỏ {len(pcd.points) - len(pcd_filtered.points)} điểm nhiễu")
    else:
        print("Phương pháp không được hỗ trợ, sử dụng statistical outlier removal")
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        pcd_filtered = pcd.select_by_index(ind)
    
    print(f"Thời gian khử nhiễu: {time.time() - start_time:.2f} giây")
    return pcd_filtered

def segment_floor(pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000):
    """Phân đoạn sàn nhà từ đám mây điểm"""
    print("Đang phân đoạn sàn nhà...")
    start_time = time.time()
    
    # Tìm mặt phẳng lớn nhất (thường là sàn nhà)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )
    
    [a, b, c, d] = plane_model
    print(f"Phương trình mặt phẳng: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    
    # Tạo đám mây điểm cho sàn nhà và các điểm còn lại
    floor_cloud = pcd.select_by_index(inliers)
    floor_cloud.paint_uniform_color([1.0, 0, 0])  # Màu đỏ cho sàn nhà
    
    other_cloud = pcd.select_by_index(inliers, invert=True)
    
    print(f"Thời gian phân đoạn sàn nhà: {time.time() - start_time:.2f} giây")
    print(f"Số điểm thuộc sàn nhà: {len(floor_cloud.points)}")
    
    return floor_cloud, other_cloud, plane_model

def calculate_floor_zmean(floor_cloud):
    """Tính giá trị zmean của sàn nhà"""
    floor_points = np.asarray(floor_cloud.points)
    zmean = np.mean(floor_points[:, 2])
    print(f"Giá trị zmean của sàn nhà: {zmean}")
    return zmean

def correct_orientation(pcd, floor_cloud, plane_model):
    """Xoay đám mây điểm để sàn nhà song song với mặt phẳng XY"""
    print("Đang xoay đám mây điểm về đúng chiều...")
    start_time = time.time()
    
    [a, b, c, d] = plane_model
    
    # Vector pháp tuyến của sàn nhà
    normal_vector = np.array([a, b, c])
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    
    # Vector pháp tuyến của mặt phẳng XY (hướng xuống dưới theo trục Z)
    target_vector = np.array([0, 0, -1])
    
    # Tính ma trận xoay để đưa normal_vector về target_vector
    rotation = o3d.geometry.get_rotation_matrix_from_vectors(normal_vector, target_vector)
    
    # Xoay toàn bộ đám mây điểm
    pcd_rotated = copy.deepcopy(pcd)
    pcd_rotated.rotate(rotation, center=(0, 0, 0))
    
    # Tính lại zmean sau khi xoay
    floor_rotated = copy.deepcopy(floor_cloud)
    floor_rotated.rotate(rotation, center=(0, 0, 0))
    zmean_rotated = calculate_floor_zmean(floor_rotated)
    
    # Dịch chuyển đám mây điểm để sàn nhà nằm ở z=0
    translation = np.array([0, 0, -zmean_rotated])
    pcd_rotated.translate(translation)
    
    print(f"Thời gian xoay đám mây điểm: {time.time() - start_time:.2f} giây")
    
    return pcd_rotated

def analyze_point_cloud(pcd):
    """Phân tích đám mây điểm và trả về các thông tin quan trọng"""
    # Tính khoảng cách trung bình giữa các điểm
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    points = np.asarray(pcd.points)
    avg_distances = []
    
    for i in range(min(1000, len(pcd.points))):  # Lấy mẫu 1000 điểm để tăng tốc độ
        [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], 11)  # 10 điểm gần nhất + chính nó
        distances = np.linalg.norm(points[idx[1:]] - points[i], axis=1)
        avg_distances.append(np.mean(distances))
    
    avg_point_distance = np.mean(avg_distances)
    
    # Tính kích thước của đám mây điểm
    min_bound = pcd.get_min_bound()
    max_bound = pcd.get_max_bound()
    dimensions = max_bound - min_bound
    
    # Tính mật độ điểm
    volume = dimensions[0] * dimensions[1] * dimensions[2]
    point_density = len(pcd.points) / volume if volume > 0 else 0
    
    analysis = {
        "num_points": len(pcd.points),
        "dimensions": dimensions,
        "avg_point_distance": avg_point_distance,
        "point_density": point_density,
        "min_bound": min_bound,
        "max_bound": max_bound
    }
    
    return analysis

def save_analysis_report(analysis, output_dir):
    """Lưu báo cáo phân tích vào file"""
    report_path = os.path.join(output_dir, "analysis_report.txt")
    
    with open(report_path, "w") as f:
        f.write("=== BÁO CÁO PHÂN TÍCH ĐÁM MÂY ĐIỂM ===\n\n")
        f.write(f"Số lượng điểm: {analysis['num_points']}\n")
        f.write(f"Kích thước (x, y, z): {analysis['dimensions'][0]:.3f} x {analysis['dimensions'][1]:.3f} x {analysis['dimensions'][2]:.3f}\n")
        f.write(f"Khoảng cách trung bình giữa các điểm: {analysis['avg_point_distance']:.5f}\n")
        f.write(f"Mật độ điểm: {analysis['point_density']:.2f} điểm/đơn vị thể tích\n")
        f.write(f"Giới hạn dưới (min_bound): [{analysis['min_bound'][0]:.3f}, {analysis['min_bound'][1]:.3f}, {analysis['min_bound'][2]:.3f}]\n")
        f.write(f"Giới hạn trên (max_bound): [{analysis['max_bound'][0]:.3f}, {analysis['max_bound'][1]:.3f}, {analysis['max_bound'][2]:.3f}]\n")
    
    print(f"Đã lưu báo cáo phân tích vào {report_path}")

def main():
    parser = argparse.ArgumentParser(description="Xử lý đám mây điểm 3D")
    parser.add_argument("--input", required=True, help="Đường dẫn đến file đám mây điểm đầu vào (.ply)")
    parser.add_argument("--output", required=True, help="Đường dẫn đến file đám mây điểm đầu ra (.ply)")
    parser.add_argument("--visualize", action="store_true", help="Hiển thị đám mây điểm trong quá trình xử lý")
    parser.add_argument("--noise_method", choices=["statistical", "radius"], default="statistical", help="Phương pháp khử nhiễu")
    parser.add_argument("--nb_neighbors", type=int, default=20, help="Số lượng điểm lân cận cho khử nhiễu")
    parser.add_argument("--std_ratio", type=float, default=2.0, help="Tỷ lệ độ lệch chuẩn cho khử nhiễu")
    parser.add_argument("--plane_threshold", type=float, default=0.02, help="Ngưỡng khoảng cách cho phân đoạn mặt phẳng")
    
    args = parser.parse_args()
    
    # Tạo thư mục đầu ra nếu chưa tồn tại
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Đọc đám mây điểm
    pcd = load_point_cloud(args.input)
    
    # Hiển thị đám mây điểm ban đầu
    if args.visualize:
        visualize_point_cloud(pcd, "Đám mây điểm ban đầu")
    
    # Khử nhiễu
    pcd_filtered = remove_noise(pcd, args.nb_neighbors, args.std_ratio, args.noise_method)
    
    # Hiển thị đám mây điểm sau khi khử nhiễu
    if args.visualize:
        visualize_point_cloud(pcd_filtered, "Đám mây điểm sau khi khử nhiễu")
    
    # Phân đoạn sàn nhà
    floor_cloud, other_cloud, plane_model = segment_floor(pcd_filtered, args.plane_threshold)
    
    # Hiển thị sàn nhà
    if args.visualize:
        combined = copy.deepcopy(floor_cloud)
        other_cloud_colored = copy.deepcopy(other_cloud)
        other_cloud_colored.paint_uniform_color([0.5, 0.5, 0.5])  # Màu xám cho các điểm không phải sàn nhà
        combined += other_cloud_colored
        visualize_point_cloud(combined, "Phân đoạn sàn nhà")
    
    # Tính giá trị zmean của sàn nhà
    zmean = calculate_floor_zmean(floor_cloud)
    
    # Xoay đám mây điểm về đúng chiều
    pcd_corrected = correct_orientation(pcd_filtered, floor_cloud, plane_model)
    
    # Hiển thị đám mây điểm sau khi xoay
    if args.visualize:
        visualize_point_cloud(pcd_corrected, "Đám mây điểm sau khi xoay về đúng chiều")
    
    # Lưu đám mây điểm đã xử lý
    o3d.io.write_point_cloud(args.output, pcd_corrected)
    print(f"Đã lưu đám mây điểm đã xử lý vào {args.output}")
    
    # Lưu giá trị zmean vào file
    zmean_path = os.path.join(output_dir, "zmean.txt")
    with open(zmean_path, "w") as f:
        f.write(f"{zmean}")
    print(f"Đã lưu giá trị zmean vào {zmean_path}")
    
    # Phân tích đám mây điểm
    analysis = analyze_point_cloud(pcd_corrected)
    save_analysis_report(analysis, output_dir)
    
    print("Xử lý hoàn tất!")

if __name__ == "__main__":
    main()
