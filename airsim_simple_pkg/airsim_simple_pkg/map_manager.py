import os
import numpy as np
import airsim
from scipy.io import savemat, loadmat
import matplotlib.pyplot as plt
import numpy as np
if not hasattr(np, 'bool'):
    np.bool = bool
class AirSimMapManager:
    def __init__(self, config, client = None):
        """
        Initialize AirSim map manager
        
        Args:
            config: Configuration dictionary containing map-related parameters
        """
        # Read parameters from configuration
        self.center_point_in_airsim = config.get('simulation', {}).get('center_point_in_airsim', [0,0,0])
        self.xmin = config.get('simulation', {}).get('area_bounds', {}).get('xmin', -1000)
        self.xmax = config.get('simulation', {}).get('area_bounds', {}).get('xmax', 1000)
        self.ymin = config.get('simulation', {}).get('area_bounds', {}).get('ymin', -1000)
        self.ymax = config.get('simulation', {}).get('area_bounds', {}).get('ymax', 1000)
        self.zmin = config.get('simulation', {}).get('area_bounds', {}).get('zmin', -1000)
        self.zmax = config.get('simulation', {}).get('area_bounds', {}).get('zmax', 1000)
        self.resolution = config.get('simulation', {}).get('area_bounds', {}).get('resolution', 5.0)
        self.grid_size = config.get('simulation', {}).get('area_bounds', {}).get('grid_size', 1)
        self.max_x = self.xmax - self.xmin
        self.max_y = self.ymax - self.ymin
        # self.max_z = config.get('map_size', {}).get('max_z', 100)
        # This z parameter controls the height range when getting map from airsim
        
        
        # Map storage path
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # If relative path is provided in config, build absolute path based on current directory
        maps_dir = config.get('simulation', {}).get('map_dir', 'maps')
        self.map_file_name = config.get('simulation', {}).get('map_file_name', 'map_cloud.npy')
        if not os.path.isabs(maps_dir):
            self.maps_dir = os.path.abspath(os.path.join(current_dir, maps_dir))
        else:
            self.maps_dir = maps_dir
        print(f"[MapManager] maps_dir: {self.maps_dir}")

        if not os.path.exists(self.maps_dir):
            os.makedirs(self.maps_dir)
            
        # Point cloud data for storing map
        self.cloud_data = None
        
        # Connect to AirSim client
        if client is None:
            self.client = None
        else:
            self.client = client
        
    def get_map_from_airsim(self, force_update=False):
        """
        Get map data from AirSim
        
        Args:
            force_update: Whether to force update map even if cache exists
            
        Returns:
            numpy.ndarray: Point cloud data
        """

        # Check if cached map data exists
        cloud_path = os.path.join(self.maps_dir, self.map_file_name)
        if os.path.exists(cloud_path) and not force_update:
            self.cloud_data = np.load(cloud_path)
            print(f"load map from file: {cloud_path}, shape: {self.cloud_data.shape}")
            return self.cloud_data
        if self.client is None:
            self.client = airsim.VehicleClient()
            self.client.confirmConnection()
        print("Getting map data from AirSim")
        obstacle_points_whole = np.empty((0, 3))
        block_size_xy = 100  # Size of each block (meters)
        block_size_z = 50  # Height of each layer (meters)
        # Get map data layer by layer to reduce memory usage
        import binvox.binvox as binvox
        temp_map_path = os.path.join(self.maps_dir, 'temp_map.binvox')
        x_min = self.xmin
        x_max = self.xmax
        y_min = self.ymin
        y_max = self.ymax
        z_min = self.zmin
        z_max = self.zmax
        resolution = self.resolution
        x_idx_min = int(np.ceil(x_min / block_size_xy))
        x_idx_max = int(np.ceil(x_max / block_size_xy))
        y_idx_min = int(np.ceil(y_min / block_size_xy))
        y_idx_max = int(np.ceil(y_max / block_size_xy))
        z_idx_min = int(np.ceil(z_min / block_size_z))
        z_idx_max = int(np.ceil(z_max / block_size_z))
        try:
            # Divide into blocks on xy plane
            for x_idx in range(x_idx_min, x_idx_max + 1):
                # Check if termination signal received
                for y_idx in range(y_idx_min, y_idx_max + 1):
                        
                    x_center = x_idx * block_size_xy
                    y_center = y_idx * block_size_xy

                    
                    # Layer in z direction
                    for z_idx in range(z_idx_min, z_idx_max + 1):
                            
                        z_center = - block_size_z * z_idx
                        center_airsim = airsim.Vector3r(x_center, y_center, z_center)
                        
                        try:
                            # Get voxel data for current block
                            self.client.simCreateVoxelGrid(center_airsim, block_size_xy, block_size_xy, block_size_z, resolution, temp_map_path)
                            
                            # Read voxel data
                            convert_data = binvox.Binvox.read(temp_map_path, 'dense', True)
                            map_output = np.zeros_like(convert_data.data, dtype=int)
                            map_output[convert_data.data] = 1
                            
                            # Convert to point cloud
                            obstacle_indices = np.argwhere(map_output == 1)
                            obstacle_points = obstacle_indices.astype(float)
                            
                            if len(obstacle_points) == 0:
                                continue
                            
                            # Adjust coordinates
                            n_x = map_output.shape[0]
                            n_y = map_output.shape[1]
                            n_z = map_output.shape[2]
                            
                            # Convert local coordinates to global coordinates
                            obstacle_points[:, 0] = (obstacle_points[:, 0] - n_x/2 + y_center/resolution) * resolution
                            obstacle_points[:, 1] = (obstacle_points[:, 1] - n_y/2 + x_center/resolution) * resolution
                            obstacle_points[:, 2] = (obstacle_points[:, 2] - n_z/2 - z_center/resolution) * resolution

                            # Filter points outside x_min, x_max, y_min, y_max, z_min, z_max bounds
                            obstacle_points = obstacle_points[
                                (obstacle_points[:, 0] >= y_min) & 
                                (obstacle_points[:, 0] <= y_max) & 
                                (obstacle_points[:, 1] >= x_min) & 
                                (obstacle_points[:, 1] <= x_max) & 
                                (obstacle_points[:, 2] >= z_min) & 
                                (obstacle_points[:, 2] <= z_max)]
                            
                            # Add to total point cloud
                            obstacle_points_whole = np.vstack((obstacle_points_whole, obstacle_points))
                            
                            print(f"Block {x_center},{y_center} layer {z_center} added {len(obstacle_points)} points")
                            
                        except Exception as e:
                            print(f"Error processing block {x_center},{y_center},{z_center}: {str(e)}")
                            continue
        finally:
            # Ensure temporary file is deleted in any case
            if os.path.exists(temp_map_path):
                try:
                    os.remove(temp_map_path)
                except Exception as e:
                    print(f"Warning: Unable to delete temporary file {temp_map_path}: {e}")
            
        # Save point cloud data
        # Need to transform coordinates: swap xy, negate z
        R = np.array([[0, 1, 0],
                      [1, 0, 0],
                      [0, 0, -1]])
        obstacle_points_whole = np.dot(R, obstacle_points_whole.T)
        self.cloud_data = obstacle_points_whole.T
        self.save_map()
        
        return self.cloud_data
    
    def read_map_from_local(self):
        """
        从本地文件读取地图数据
        
        Returns:
            numpy.ndarray: 点云数据，如果文件不存在则返回None
        """

        cloud_path = os.path.join(self.maps_dir, self.map_file_name)
        if os.path.exists(cloud_path):
            self.cloud_data = np.load(cloud_path)
            print(f"load map from file: {cloud_path}, shape: {self.cloud_data.shape}")
            return self.cloud_data
        else:
            print(f"could not find map file at :{cloud_path}")
            return None
            
    def save_map(self):
        """
        保存地图数据到本地文件
        """
        if self.cloud_data is not None:
            try:
                # 计算地图尺寸（以分辨率为单位的格点数）
                x_size = int(self.xmax - self.xmin)
                y_size = int(self.ymax - self.ymin)
                z_size = int(self.zmax - self.zmin)
                shape_str = f"{x_size}x{y_size}x{z_size}"
                # resolution 保留两位小数，点替换为下划线，便于文件名
                res_str = f"{self.resolution:.2f}".replace('.', '_')
                # 拼接文件名
                npy_filename = f"map_cloud_{shape_str}_{res_str}m.npy"
                # mat_filename = f"map_cloud_mat_{shape_str}_{res_str}m.mat"
                npy_path = os.path.join(self.maps_dir, npy_filename)
                # mat_path = os.path.join(self.maps_dir, mat_filename)
                # 保存为numpy格式
                np.save(npy_path, self.cloud_data)
                # 保存为mat格式，方便其他工具读取
                # savemat(mat_path, {'data': self.cloud_data})
                print(f"地图数据已保存到:\n  {npy_path}")
                print(f"点云大小: {self.cloud_data.shape}, 地图尺寸: {shape_str}, 分辨率: {self.resolution}")
            except Exception as e:
                print(f"保存地图数据时出错: {e}")
        else:
            print("没有地图数据可保存")
            
    def get_grid_map(self,grid_size=None, height=None, height_tolerance=2.0):
        """
        获取指定高度的2D栅格地图
        
        Args:
            grid_size: 栅格大小，如果为None则使用默认值
            height: 指定高度（NED坐标系），如果为None则返回所有z<0的点云
                   否则返回所有z<=height的点云
            height_tolerance: 给一点点容差
            
        Returns:
            numpy.ndarray: 2D栅格地图，0表示空闲，1表示占据
        """
        if grid_size is None:
            grid_size = self.grid_size

        if self.cloud_data is None:
            self.read_map_from_local()
            if self.cloud_data is None:
                self.get_map_from_airsim()
                
        # 根据高度筛选点云
        if height is not None:
            # 选取所有高度小于等于指定高度的点（NED系中z越小表示越高）
            mask = self.cloud_data[:, 2] <= (height + height_tolerance)
            filtered_cloud = self.cloud_data[mask]
        else:
            # 如果没有指定高度，选取所有z<0的点
            mask = self.cloud_data[:, 2] < 0
            filtered_cloud = self.cloud_data[mask]
            
        # 创建2D栅格地图
        x_range = [self.xmin, self.xmax]
        y_range = [self.ymin, self.ymax]
        grid_map = self.points_to_gridmap(filtered_cloud, grid_size, x_range, y_range)
        
        return grid_map
        
    def points_to_gridmap(self, points, grid_size, x_range, y_range):
        """
        将点云数据转换为2D栅格地图
        
        Args:
            points: Nx3的点云数据
            grid_size: 栅格大小
            x_range: x轴范围 [min_x, max_x]
            y_range: y轴范围 [min_y, max_y]
            
        Returns:
            如果return_count为False:
                numpy.ndarray: 栅格地图，0表示空闲，1表示占据
        """
        # 计算地图尺寸
        map_width = int((x_range[1] - x_range[0]) / grid_size)
        map_height = int((y_range[1] - y_range[0]) / grid_size)
        
        # 创建空地图
        grid_map = np.zeros((map_height, map_width), dtype=np.int8)

        
        if len(points) == 0:
            return grid_map
            
        # 将点云坐标转换为栅格索引
        x_idx = np.floor((points[:, 0] - x_range[0])/grid_size).astype(int)
        y_idx = np.floor((points[:, 1] - y_range[0])/grid_size).astype(int)
        
        # 过滤掉超出范围的点
        valid_idx = (x_idx >= 0) & (x_idx < map_width) & (y_idx >= 0) & (y_idx < map_height)
        x_idx = x_idx[valid_idx]
        y_idx = y_idx[valid_idx]
        
        # 标记占据栅格
        grid_map[y_idx, x_idx] = 1
            
        return grid_map
        
    def expand_grid_map(self, grid_map, radius=0):
        """
        扩展栅格地图中的障碍物，基于给定的半径
        
        Args:
            grid_map: 原始栅格地图
            radius: 扩展半径（米）
            
        Returns:
            numpy.ndarray: 扩展后的栅格地图
        """
        if radius <= 0:
            return grid_map.copy()
            
        # 将半径从米转换为栅格单元并向上取整
        radius_cells = int(np.ceil(radius / self.grid_size))
        
        # 创建距离核
        y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
        kernel = x*x + y*y <= radius_cells*radius_cells
        
        # 使用二进制膨胀与圆形核
        from scipy import ndimage
        expanded_map = ndimage.binary_dilation(grid_map, kernel).astype(grid_map.dtype)
        
        return expanded_map
        
    def get_expanded_grid_map(self, radius, height=None, height_tolerance=10.0):
        """
        获取扩展后的栅格地图
        
        Args:
            radius: 扩展半径（米）
            height: 指定高度，如果为None则返回全部点云的栅格地图
            height_tolerance: 高度容差，默认10米
            
        Returns:
            numpy.ndarray: 扩展后的栅格地图
        """
        grid_map = self.get_grid_map(height=height, height_tolerance=height_tolerance)
        return self.expand_grid_map(grid_map, radius)

    def visualize_grid_map(self, grid_map, title="grid map", save_path=None):
        """
        可视化栅格地图
        
        Args:
            grid_map: 需要可视化的栅格地图（二维numpy数组，值为0或1）
            title: 图像标题
            save_path: 保存路径，如果为None则显示图像
        """
        # 创建图像
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 绘制原始栅格地图
        im1 = ax1.imshow(grid_map, origin='lower', cmap='binary')
        ax1.set_title(title)
        ax1.set_xlabel('X grid index')
        ax1.set_ylabel('Y grid index')
        plt.colorbar(im1, ax=ax1, label='occupied')
        
        # 添加栅格线
        ax1.grid(True, which='both', color='gray', linewidth=0.5, alpha=0.3)
        
        # 计算并显示占据区域的轮廓
        from scipy import ndimage
        edges = ndimage.sobel(grid_map)
        im2 = ax2.imshow(edges, origin='lower', cmap='hot')
        ax2.set_title(f"{title} - edge feature")
        ax2.set_xlabel('X grid index')
        ax2.set_ylabel('Y grid index')
        plt.colorbar(im2, ax=ax2, label='edge strength')
        
        # 显示统计信息
        occupied = np.sum(grid_map == 1)
        total = grid_map.size
        occupation_rate = occupied / total * 100
        stats_text = f'occupied grid number: {occupied}\ntotal grid number: {total}\noccupation rate: {occupation_rate:.2f}%'
        
        # 在两个子图之间添加统计信息
        fig.text(0.5, 0.02, stats_text, ha='center', bbox=dict(facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            plt.close()
        else:
            plt.show()
            plt.close()
        
        return fig

if __name__ == "__main__":
    import argparse
    import yaml
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='AirSim地图管理器测试程序')
    parser.add_argument('--config', type=str, default='config/config.yaml', help='配置文件路径')
    parser.add_argument('--force_update', action='store_true', help='强制从AirSim更新地图')
    parser.add_argument('--height', type=float, default=100.0, help='获取特定高度的栅格地图（米）')
    parser.add_argument('--radius', type=float, default=5.0, help='障碍物扩展半径（米）')
    parser.add_argument('--save_fig', action='store_true', help='保存图像到文件')
    args = parser.parse_args()
    
    # 加载配置文件
    try:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"找不到配置文件: {args.config}，使用默认配置")
        config = {
            'map_size': {
                'max_x': 2000,
                'max_y': 2000,
                'max_z': 100
            },
            'resolution': 5.0,
            'maps_dir': 'maps'
        }
    
    # 创建地图管理器实例
    map_manager = AirsimMapManager(config)
    
    # 获取地图数据
    if args.force_update:
        print("强制从AirSim更新地图...")
        cloud_data = map_manager.get_map_from_airsim(force_update=True)
    else:
        print("尝试从本地加载地图，如果不存在则从AirSim获取...")
        cloud_data = map_manager.get_grid_map()
    
    if cloud_data is None:
        print("获取地图数据失败！")
        exit(1)
    
    # 1. 点云俯视图
    plt.figure(figsize=(10, 8))
    plt.scatter(map_manager.cloud_data[:, 0], map_manager.cloud_data[:, 1], 
                c=map_manager.cloud_data[:, 2], s=1, cmap='viridis')
    plt.colorbar(label='height')
    plt.title('point cloud view')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    if args.save_fig:
        plt.savefig(os.path.join(map_manager.maps_dir, 'point_cloud_view.png'), 
                    dpi=300, bbox_inches='tight')
    grid_map = map_manager.get_grid_map(height=-100)
    map_manager.visualize_grid_map(grid_map, title="visualize grid map",save_path=os.path.join(map_manager.maps_dir, 'grid_map_view.png'))
    

    # 打印统计信息
    print("\n地图统计信息:")
    print(f"点云数据大小: {map_manager.cloud_data.shape}")
    print(f"栅格地图大小: {cloud_data.shape}")
    print(f"占据栅格数量: {np.sum(cloud_data == 1)}")
    
    # 显示所有图像
    plt.show()
    print("图像已显示")