import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import heapq
import json
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time

# 设置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class SimpleSLAMGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("简化界面SLAM仿真系统")
        self.root.geometry("1200x800")
        
        # 仿真状态变量
        self.simulation_running = False
        self.simulation_paused = False
        
        # 仿真参数
        self.params = {
            'resolution': 0.1,
            'speed': 100,
            'maze_file': 'exp3.json'
        }
        
        # 创建简化界面
        self.create_interface()
        
        # 加载默认迷宫
        self.load_maze_file()
        
    def create_interface(self):
        """创建简化界面"""
        # 顶部控制栏
        control_frame = tk.Frame(self.root, bg='lightgray', height=50)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        control_frame.pack_propagate(False)
        
        # 文件选择
        tk.Label(control_frame, text="文件:").pack(side=tk.LEFT, padx=5)
        self.file_var = tk.StringVar(value="exp3.json")
        tk.Label(control_frame, textvariable=self.file_var).pack(side=tk.LEFT, padx=5)
        tk.Button(control_frame, text="选择", command=self.load_maze_dialog).pack(side=tk.LEFT, padx=5)
        
        # 控制按钮
        self.start_btn = tk.Button(control_frame, text="开始", command=self.start_simulation, 
                                  bg='green', fg='white')
        self.start_btn.pack(side=tk.LEFT, padx=10)
        
        self.pause_btn = tk.Button(control_frame, text="暂停", command=self.pause_simulation, 
                                  bg='orange', fg='white')
        self.pause_btn.pack(side=tk.LEFT, padx=5)
        
        self.reset_btn = tk.Button(control_frame, text="重置", command=self.reset_simulation, 
                                  bg='red', fg='white')
        self.reset_btn.pack(side=tk.LEFT, padx=5)
        
        # 速度控制
        tk.Label(control_frame, text="每步时间:").pack(side=tk.RIGHT, padx=5)
        self.speed_var = tk.IntVar(value=0.001)
        speed_scale = tk.Scale(control_frame, from_=0.001, to=100, orient=tk.HORIZONTAL,
                              variable=self.speed_var, length=100)
        speed_scale.pack(side=tk.RIGHT, padx=5)
        
        # 状态显示
        self.status_var = tk.StringVar(value="准备就绪")
        tk.Label(control_frame, textvariable=self.status_var).pack(side=tk.RIGHT, padx=20)
        
        # 地图显示区域
        self.create_map_area()
        
    def create_map_area(self):
        """创建地图显示区域"""
        # 创建matplotlib图形
        self.fig = Figure(figsize=(12, 6), facecolor='white')
        self.fig.suptitle("迷宫SLAM仿真 - 原始地图 vs SLAM构建地图", fontsize=14)
        
        # 创建两个子图
        self.ax1 = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)
        
        self.ax1.set_title('原始迷宫地图 - 机器人真实轨迹')
        self.ax2.set_title('SLAM扫描构建地图')
        
        # 创建画布
        self.canvas = FigureCanvasTkAgg(self.fig, self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
    def load_maze_dialog(self):
        """加载迷宫文件"""
        filename = filedialog.askopenfilename(
            title="选择迷宫文件",
            filetypes=[("JSON文件", "*.json"), ("所有文件", "*.*")]
        )
        if filename:
            self.params['maze_file'] = filename
            self.file_var.set(filename.split('/')[-1])
            self.load_maze_file()
            
    def load_maze_file(self):
        """加载迷宫文件"""
        try:
            with open(self.params['maze_file'], "r", encoding='utf-8') as f:
                self.maze_data = json.load(f)
            
            self.segments = [(tuple(seg['start']), tuple(seg['end'])) 
                            for seg in self.maze_data['segments']]
            self.start_point = tuple(self.maze_data['start_point'])
            
            self.status_var.set("迷宫加载成功")
            self.display_static_map()
            
        except Exception as e:
            messagebox.showerror("错误", f"加载失败: {str(e)}")
            self.status_var.set("加载失败")
            
    def display_static_map(self):
        """显示静态地图"""
        self.ax1.clear()
        self.ax1.set_title('原始迷宫地图 - 机器人真实轨迹')
        
        # 使用demo4.py的地图构建方式
        resolution = self.params['resolution']
        occupancy_grid = self.build_occupancy_grid_from_segments(self.segments, resolution)
        
        # 显示占用栅格地图
        self.ax1.imshow(occupancy_grid.T, cmap='gray_r', origin='lower', alpha=0.8)
        
        # 绘制迷宫墙壁线段
        for seg_start, seg_end in self.segments:
            x_coords = [seg_start[0]/resolution, seg_end[0]/resolution]
            y_coords = [seg_start[1]/resolution, seg_end[1]/resolution] 
            self.ax1.plot(x_coords, y_coords, 'k-', linewidth=2, alpha=0.8)
        
        # 标记起点
        start_grid = (int(self.start_point[0] / resolution) + 1,
                     int(self.start_point[1] / resolution) + 1)
        self.ax1.scatter(*start_grid, c='green', s=100, marker='s', label='起点')
        
        # 设置坐标轴显示范围
        grid_size = int(21 / resolution)
        self.ax1.set_xlim(0, grid_size)
        self.ax1.set_ylim(0, grid_size)
        self.ax1.grid(True, alpha=0.3)
        #self.ax1.legend()
        
        # 初始化SLAM地图
        self.ax2.clear()
        self.ax2.set_title('SLAM扫描构建地图')
        initial_slam_map = np.full_like(occupancy_grid, 0.5, dtype=float)
        self.ax2.imshow(initial_slam_map.T, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        self.ax2.scatter(*start_grid, c='green', s=100, marker='s', label='起点')
        self.ax2.set_xlim(0, grid_size)
        self.ax2.set_ylim(0, grid_size)
        self.ax2.grid(True, alpha=0.3)
        #self.ax2.legend()
        
        self.canvas.draw()
        
    def start_simulation(self):
        """开始仿真"""
        if not hasattr(self, 'maze_data'):
            messagebox.showwarning("警告", "请先加载迷宫文件！")
            return
            
        if self.simulation_running:
            return
            
        self.simulation_running = True
        self.start_btn.config(state='disabled')
        self.pause_btn.config(state='normal')
        self.reset_btn.config(state='normal')
        
        self.status_var.set("仿真运行中...")
        
        # 在后台线程中运行仿真
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
        
    def pause_simulation(self):
        """暂停仿真"""
        if self.simulation_running:
            self.simulation_paused = not self.simulation_paused
            if self.simulation_paused:
                self.pause_btn.config(text="继续")
                self.status_var.set("仿真暂停")
            else:
                self.pause_btn.config(text="暂停")
                self.status_var.set("仿真运行中...")
                self.init_scan_output()

                
    def reset_simulation(self):
        """重置仿真"""
        self.simulation_running = False
        self.simulation_paused = False
        
        self.start_btn.config(state='normal')
        self.pause_btn.config(state='disabled', text="暂停")
        self.reset_btn.config(state='disabled')
        
        self.status_var.set("仿真重置")
        
        if hasattr(self, 'maze_data'):
            self.display_static_map()
            
    def run_simulation(self):
        """运行仿真 - 使用demo4.py的完整算法"""
        try:
            self.init_scan_output()

            # 使用demo4.py的完整仿真逻辑
            resolution = self.params['resolution']
            grid_size = int(21 / resolution)
            updated_grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
            trajectory = []
            scan_grids = []
            
            # 获取终点信息
            goal_line_info = self.get_goal_area_range(self.maze_data, resolution)
            start_grid = (int(self.start_point[0] / resolution) + 1, 
                         int(self.start_point[1] / resolution))
            
            robot_pos = start_grid
            robot_dir = 'N'
            trajectory.append(robot_pos)
            
            step = 0
            has_left_start = False
            
            print(f"开始探索，起点位置: {start_grid}")
            
            # 第一阶段：SLAM探索
            while self.simulation_running and step < 5000:
                if self.simulation_paused:
                    time.sleep(0.01)
                    continue
                    
                step += 1
                
                # 执行仿真步骤
                real_pos = (robot_pos[0] * resolution, robot_pos[1] * resolution)
                scan_pts, scan_hits = self.simple_lidar_scan(real_pos, self.segments)
                self.update_occupancy_grid(updated_grid, real_pos, scan_pts, scan_hits, resolution)
                scan_grids.append(updated_grid.copy())
                
                # 只在SLAM探索阶段保存雷达扫描数据到dat文件
                self.save_scan_to_dat(step, real_pos, scan_pts)
                
                # 检查是否回到起点
                if has_left_start and robot_pos == start_grid:
                    print(f"步骤 {step}: 机器人回到起点，探索完成！")
                    # SLAM探索阶段结束，关闭雷达数据文件
                    if hasattr(self, 'scan_file'):
                        self.scan_file.close()
                        saved_scans = (self.scan_counter + 2) // 3  # 实际保存的扫描数
                        print(f"SLAM雷达数据保存完成: all_scans.dat")
                        print(f"总扫描次数: {self.scan_counter}, 实际保存: {saved_scans} 帧 (保留率: {saved_scans/self.scan_counter*100:.1f}%)")
                    break
                    
                if robot_pos != start_grid:
                    has_left_start = True
                
                # 左手法则导航
                left = self.turn_left(robot_dir)
                moved = False
                
                if self.can_move_with_goal_blocking(updated_grid, robot_pos, left, goal_line_info, resolution):
                    robot_dir = left
                    moved = True
                elif self.can_move_with_goal_blocking(updated_grid, robot_pos, robot_dir, goal_line_info, resolution):
                    moved = True
                elif self.can_move_with_goal_blocking(updated_grid, robot_pos, self.turn_right(robot_dir), goal_line_info, resolution):
                    robot_dir = self.turn_right(robot_dir)
                    moved = True
                elif self.can_move_with_goal_blocking(updated_grid, robot_pos, self.turn_back(robot_dir), goal_line_info, resolution):
                    robot_dir = self.turn_back(robot_dir)
                    moved = True
                else:
                    if self.can_move(updated_grid, robot_pos, left):
                        robot_dir = left
                        moved = True
                    elif self.can_move(updated_grid, robot_pos, robot_dir):
                        moved = True
                    elif self.can_move(updated_grid, robot_pos, self.turn_right(robot_dir)):
                        robot_dir = self.turn_right(robot_dir)
                        moved = True
                    elif self.can_move(updated_grid, robot_pos, self.turn_back(robot_dir)):
                        robot_dir = self.turn_back(robot_dir)
                        moved = True
                
                if moved:
                    dx, dy = self.DIR_DELTAS[robot_dir]
                    new_pos = (robot_pos[0] + dx, robot_pos[1] + dy)
                    robot_pos = new_pos
                    if robot_pos not in trajectory:
                        trajectory.append(robot_pos)
                
                # 每3步更新一次显示
                if step % 5 == 0:
                    self.root.after(0, self.update_display, trajectory, scan_grids, start_grid, step, False, None, None, None)
                
                # 控制速度
                time.sleep(self.speed_var.get() / 1000.0)
                    
            # 第二阶段：A*路径规划和多路径比较
            if self.simulation_running:
                self.root.after(0, self.status_var.set, "计算多条A*路径...")
                
                # 计算起点和终点
                start_real = (start_grid[0] * resolution, start_grid[1] * resolution)
                end_real = (goal_line_info['center'][0] * resolution, goal_line_info['center'][1] * resolution)
                
                # 计算三条不同的A*路径
                print("正在计算多条候选路径...")
                
                # 路径1：标准A*算法（曼哈顿距离）
                path1 = self.astar_limited(start_real, end_real, 40, self.segments, resolution, 
                                         heuristic_weight=1.0, search_type="manhattan")
                
                # 路径2：更保守的搜索（欧几里得距离 + 大搜索窗口）
                path2 = self.astar_limited(start_real, end_real, 60, self.segments, resolution, 
                                         heuristic_weight=1.5, search_type="euclidean")
                
                # 路径3：更激进的搜索（对角线距离 + 小搜索窗口）
                path3 = self.astar_limited(start_real, end_real, 25, self.segments, resolution, 
                                         heuristic_weight=0.6, search_type="diagonal")
                
                # 收集所有有效路径
                candidate_paths = []
                if path1:
                    path1_length = self.calculate_path_length(path1)
                    candidate_paths.append({'path': path1, 'length': path1_length, 'name': '曼哈顿A*', 'color': 'blue'})
                    print(f"路径1 (曼哈顿A*): {len(path1)} 点, 长度: {path1_length:.2f}m")
                
                if path2:
                    path2_length = self.calculate_path_length(path2)
                    candidate_paths.append({'path': path2, 'length': path2_length, 'name': '欧几里得A*', 'color': 'green'})
                    print(f"路径2 (欧几里得A*): {len(path2)} 点, 长度: {path2_length:.2f}m")
                
                if path3:
                    path3_length = self.calculate_path_length(path3)
                    candidate_paths.append({'path': path3, 'length': path3_length, 'name': '对角线A*', 'color': 'purple'})
                    print(f"路径3 (对角线A*): {len(path3)} 点, 长度: {path3_length:.2f}m")
                
                if not candidate_paths:
                    print("❌ 所有路径计算失败！")
                    self.root.after(0, self.status_var.set, "路径计算失败")
                else:
                    # 选择最短路径
                    best_path_info = min(candidate_paths, key=lambda x: x['length'])
                    optimal_path = best_path_info['path']
                    
                    print(f"🎯 选择最优路径: {best_path_info['name']} (长度: {best_path_info['length']:.2f}m)")
                    
                    # 显示所有候选路径
                    self.root.after(0, self.display_multiple_paths, trajectory, scan_grids, start_grid, 
                                   candidate_paths, best_path_info, step)
                    
                    # 等待用户查看路径比较
                    time.sleep(3.0)
                    
                    # 执行最优路径
                    if optimal_path:
                        print(f"开始执行最优路径，包含 {len(optimal_path)} 个点")
                        
                        # 将路径分解成多个阶段
                        stage_distance = 3.0
                        stages = self.create_path_stages(optimal_path, stage_distance)
                        print(f"最优路径分解为 {len(stages)} 个阶段")
                        
                        # 逐个阶段执行
                        current_pos = start_grid
                        for stage_idx, stage_goal in enumerate(stages):
                            if not self.simulation_running:
                                break
                                
                            stage_goal_grid = (int(stage_goal[0] / resolution), int(stage_goal[1] / resolution))
                            
                            print(f"执行阶段 {stage_idx + 1}/{len(stages)}, 目标: {stage_goal_grid}")
                            self.root.after(0, self.status_var.set, f"最优路径执行 - 阶段 {stage_idx + 1}/{len(stages)}")
                            
                            # 计算当前阶段的路径
                            current_real = (current_pos[0] * resolution, current_pos[1] * resolution)
                            stage_path = self.astar_limited(current_real, stage_goal, 20, self.segments, resolution)
                            
                            if not stage_path:
                                print(f"阶段 {stage_idx + 1} 路径计算失败")
                                break
                                
                            # 执行当前阶段的路径
                            for i, (real_x, real_y) in enumerate(stage_path[1:], 1):
                                if not self.simulation_running:
                                    break
                                    
                                if self.simulation_paused:
                                    time.sleep(0.01)
                                    continue
                                
                                path_grid_pos = (int(real_x / resolution), int(real_y / resolution))
                                trajectory.append(path_grid_pos)
                                current_pos = path_grid_pos
                                
                                # 生成激光雷达扫描
                                real_pos = (real_x, real_y)
                                scan_pts, scan_hits = self.simple_lidar_scan(real_pos, self.segments)
                                self.update_occupancy_grid(updated_grid, real_pos, scan_pts, scan_hits, resolution)
                                scan_grids.append(updated_grid.copy())
                                
                                # 每5步更新一次显示（显示最优路径执行）
                                if i % 5 == 0 or i == len(stage_path) - 1:
                                    self.root.after(0, self.update_display, trajectory, scan_grids, start_grid, 
                                                   step + i, True, stage_path, stage_goal_grid, stage_idx + 1)
                                
                                time.sleep(self.speed_var.get() / 1000.0)
                            
                            print(f"阶段 {stage_idx + 1} 完成")
                            time.sleep(0.5)
                            
                        print(f"🎉 最优A*路径执行完成!")
                    else:
                        print("❌ 最优路径执行失败")
            
            # 仿真完成
            self.root.after(0, self.simulation_completed)

            
        except Exception as e:
            self.root.after(0, messagebox.showerror, "错误", f"仿真错误: {str(e)}")
            self.root.after(0, self.reset_simulation)
    
    def create_path_stages(self, full_path, stage_distance):
        """将完整路径分解为多个阶段"""
        if not full_path or len(full_path) < 2:
            return []
        
        stages = []
        current_pos = full_path[0]
        accumulated_distance = 0.0
        
        for i in range(1, len(full_path)):
            point = full_path[i]
            segment_distance = math.sqrt((point[0] - current_pos[0])**2 + (point[1] - current_pos[1])**2)
            accumulated_distance += segment_distance
            
            # 如果累计距离达到阶段距离，或者这是最后一个点
            if accumulated_distance >= stage_distance or i == len(full_path) - 1:
                stages.append(point)
                accumulated_distance = 0.0
            
            current_pos = point
        
        # 确保最后一个点被包含
        if stages and stages[-1] != full_path[-1]:
            stages.append(full_path[-1])
        
        return stages
            
    def update_display(self, trajectory, scan_grids, start_grid, step, is_astar_phase, stage_path, current_goal, stage_number):
        """更新显示"""
        if not scan_grids:
            return
            
        # 更新右侧SLAM地图
        self.ax2.clear()
        title = f'SLAM扫描构建地图 (步数: {step})'
        if is_astar_phase and stage_number:
            title = f'A*路径执行 - 阶段 {stage_number} (步数: {step})'
        self.ax2.set_title(title)
        
        # 显示SLAM构建的地图
        current_slam = scan_grids[-1].copy().astype(float)
        display_map = np.full_like(current_slam, 0.5, dtype=float)
        display_map[current_slam == 1] = 1.0  # 自由空间为白色
        display_map[current_slam == 2] = 0.0  # 障碍物为黑色
        
        self.ax2.imshow(display_map.T, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # 绘制轨迹
        if len(trajectory) > 1:
            traj_x, traj_y = zip(*trajectory)
            self.ax2.plot(traj_x, traj_y, 'r-', linewidth=2, alpha=0.8, label='SLAM轨迹')
            
        # 标记起点和当前位置
        self.ax2.scatter(*start_grid, c='green', s=100, marker='s', label='起点', zorder=5)
        if trajectory:
            current_pos = trajectory[-1]
            self.ax2.scatter(*current_pos, c='blue', s=80, marker='o', label='当前位置', zorder=5)
        
        # 如果是A*阶段，显示当前目标和计划路径
        if is_astar_phase and current_goal and stage_path:
            # 显示当前目标点（星形标记）
            self.ax2.scatter(*current_goal, c='red', s=200, marker='*', 
                           label=f'目标-阶段{stage_number}', zorder=6, edgecolors='yellow', linewidth=2)
            
            # 显示计划路径（虚线）
            if len(stage_path) > 1:
                path_x = [p[0] / self.params['resolution'] for p in stage_path]
                path_y = [p[1] / self.params['resolution'] for p in stage_path]
                self.ax2.plot(path_x, path_y, 'orange', linewidth=2, linestyle='--', 
                            alpha=0.8, label=f'计划路径-阶段{stage_number}')
        
        self.ax2.set_xlim(0, 210)
        self.ax2.set_ylim(0, 210)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend()
        
        # 更新左侧原始地图的轨迹
        if len(trajectory) > 1:
            self.ax1.clear()
            self.ax1.set_title('原始迷宫地图 - 机器人真实轨迹')
            
            # 创建原始地图的显示
            resolution = self.params['resolution']
            original_map = self.build_occupancy_grid_from_segments(self.segments, resolution)
            self.ax1.imshow(original_map.T, cmap='gray_r', origin='lower', alpha=0.8)
            
            # 绘制迷宫墙壁线段
            for seg_start, seg_end in self.segments:
                x_coords = [seg_start[0]/resolution, seg_end[0]/resolution]
                y_coords = [seg_start[1]/resolution, seg_end[1]/resolution] 
                self.ax1.plot(x_coords, y_coords, 'k-', linewidth=2, alpha=0.8)
            
            # 绘制轨迹
            traj_x, traj_y = zip(*trajectory)
            self.ax1.plot(traj_x, traj_y, 'blue', linewidth=3, alpha=0.8, label='机器人真实轨迹')
            
            # 标记位置
            self.ax1.scatter(*start_grid, c='green', s=100, marker='s', label='起点', zorder=5)
            current_pos = trajectory[-1]
            self.ax1.scatter(*current_pos, c='red', s=100, marker='o', label='当前位置', zorder=5)
            
            # 如果是A*阶段，也在左图显示目标和路径
            if is_astar_phase and current_goal and stage_path:
                # 显示当前目标点（星形标记）
                self.ax1.scatter(*current_goal, c='red', s=200, marker='*', 
                               label=f'目标-阶段{stage_number}', zorder=6, edgecolors='yellow', linewidth=2)
                
                # 显示计划路径（虚线）
                if len(stage_path) > 1:
                    path_x = [p[0] / resolution for p in stage_path]
                    path_y = [p[1] / resolution for p in stage_path]
                    self.ax1.plot(path_x, path_y, 'orange', linewidth=2, linestyle='--', 
                                alpha=0.8, label=f'计划路径-阶段{stage_number}')
            
            self.ax1.set_xlim(0, 210)
            self.ax1.set_ylim(0, 210)
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend()
        
        self.canvas.draw()
    
    def init_scan_output(self):
        """初始化一个总的 dat 文件用于记录所有扫描数据"""
        print("初始化 all_scans.dat 文件")
        self.scan_file = open("all_scans.dat", "w")
        self.scan_file.write("# Format: step, robot_x, robot_y, point_x, point_y\n")
        # 添加扫描计数器，用于控制数据保存频率
        self.scan_counter = 0

    def save_scan_to_dat(self, step, real_pos, scan_pts):
        """
        保存为 BreezySLAM 格式：
        timestamp_usec dx_mm dy_mm dist1 dist2 ... dist360
        只保留每3个扫描中的1个，减少数据量到三分之一
        """
        if not hasattr(self, 'scan_file'):
            return

        # 扫描计数器递增
        self.scan_counter += 1
        
        # 只保留每3个扫描中的1个（保留三分之一）
        if self.scan_counter % 3 != 1:
            return

        # 初始化前一帧位置
        if not hasattr(self, 'prev_real_pos'):
            self.prev_real_pos = real_pos

        dx = real_pos[0] - self.prev_real_pos[0]
        dy = real_pos[1] - self.prev_real_pos[1]
        q1 = int(dx * 1000)
        q2 = int(dy * 1000)

        self.prev_real_pos = real_pos  # 更新前一帧位置

        # 时间戳（微秒）
        timestamp = step * 10000

        # 计算雷达距离
        distances = []
        for pt in scan_pts:
            dx = pt[0] - real_pos[0]
            dy = pt[1] - real_pos[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            dist_mm = min(int(dist * 1000), 9999)
            distances.append(str(dist_mm))

        line = f"{timestamp} {q1} {q2} " + " ".join(distances) + "\n"
        self.scan_file.write(line)
        
        # 打印保存信息（可选）
        if self.scan_counter % 30 == 1:  # 每保存10个数据点打印一次
            print(f"已保存第 {(self.scan_counter + 2) // 3} 个雷达扫描数据（总扫描次数: {self.scan_counter}）")

    def simulation_completed(self):
        """仿真完成"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.pause_btn.config(state='disabled', text="暂停")
        self.reset_btn.config(state='disabled')
        
        self.status_var.set("仿真完成")
        messagebox.showinfo("完成", "仿真已完成！机器人成功探索迷宫并执行了A*路径。")
        
    # ==================== 以下为demo4.py的所有核心算法函数 =====================
    
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar_limited(self, start, goal, window=40, segments=None, resolution=0.1, heuristic_weight=1.0, search_type="manhattan"):
        """A*路径规划算法"""
        # 计算迷宫边界限制
        maze_bounds = None
        if segments:
            all_points = []
            for seg_start, seg_end in segments:
                all_points.extend([seg_start, seg_end])
            
            if all_points:
                min_x_real = min(point[0] for point in all_points)
                max_x_real = max(point[0] for point in all_points)
                min_y_real = min(point[1] for point in all_points)
                max_y_real = max(point[1] for point in all_points)
                
                maze_bounds = {
                    'min_x': int(min_x_real / resolution),
                    'max_x': int(max_x_real / resolution),
                    'min_y': int(min_y_real / resolution),
                    'max_y': int(max_y_real / resolution)
                }
        
        # 转换实际坐标到栅格坐标
        start_grid = (int(start[0] / resolution), int(start[1] / resolution))
        goal_grid = (int(goal[0] / resolution) - 1, int(goal[1] / resolution) - 1)
        
        # 计算搜索区域的边界
        min_x = start_grid[0] - window
        max_x = start_grid[0] + window
        min_y = start_grid[1] - window
        max_y = start_grid[1] + window
        
        # 确保包含目标点
        min_x = min(min_x, goal_grid[0] - 10)
        max_x = max(max_x, goal_grid[0] + 10)
        min_y = min(min_y, goal_grid[1] - 10)
        max_y = max(max_y, goal_grid[1] + 10)
        
        # 计算地图尺寸
        map_width = max_x - min_x + 1
        map_height = max_y - min_y + 1
        
        # 创建局部地图 (0=自由空间, 1=障碍物)
        local_grid = np.zeros((map_width, map_height), dtype=np.uint8)
        
        # 在局部地图中绘制墙壁
        if segments:
            def draw_line_on_local_grid(seg_start, seg_end):
                x0 = int(seg_start[0] / resolution) - min_x
                y0 = int(seg_start[1] / resolution) - min_y
                x1 = int(seg_end[0] / resolution) - min_x
                y1 = int(seg_end[1] / resolution) - min_y
                
                # Bresenham直线算法
                dx = abs(x1 - x0)
                dy = abs(y1 - y0)
                x, y = x0, y0
                sx = 1 if x0 < x1 else -1
                sy = 1 if y0 < y1 else -1
                
                if dx > dy:
                    err = dx / 2.0
                    while x != x1:
                        if 0 <= x < map_width and 0 <= y < map_height:
                            local_grid[x, y] = 1
                        err -= dy
                        if err < 0:
                            y += sy
                            err += dx
                        x += sx
                else:
                    err = dy / 2.0
                    while y != y1:
                        if 0 <= x < map_width and 0 <= y < map_height:
                            local_grid[x, y] = 1
                        err -= dx
                        if err < 0:
                            x += sx
                            err += dy
                        y += sy
                
                if 0 <= x1 < map_width and 0 <= y1 < map_height:
                    local_grid[x1, y1] = 1
            
            for seg_start, seg_end in segments:
                draw_line_on_local_grid(seg_start, seg_end)
        
        # 转换起点和终点到局部坐标系
        local_start = (start_grid[0] - min_x, start_grid[1] - min_y)
        local_goal = (goal_grid[0] - min_x, goal_grid[1] - min_y)
        
        # 检查起点和终点是否有效
        if not (0 <= local_start[0] < map_width and 0 <= local_start[1] < map_height):
            return []
        if not (0 <= local_goal[0] < map_width and 0 <= local_goal[1] < map_height):
            return []
        
        if local_grid[local_start[0], local_start[1]] == 1:
            return []
        if local_grid[local_goal[0], local_goal[1]] == 1:
            return []
        
        if local_start == local_goal:
            return [start]
        
        # A*算法核心
        def is_within_maze_bounds(global_x, global_y):
            if maze_bounds is None:
                return True
            return (maze_bounds['min_x'] < global_x < maze_bounds['max_x'] and 
                    maze_bounds['min_y'] < global_y < maze_bounds['max_y'])
        
        # 选择启发式函数
        def get_heuristic(current, goal, heuristic_type):
            dx = abs(current[0] - goal[0])
            dy = abs(current[1] - goal[1])
            
            if heuristic_type == "manhattan":
                return dx + dy
            elif heuristic_type == "euclidean":
                return math.sqrt(dx*dx + dy*dy)
            elif heuristic_type == "diagonal":
                return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
            else:
                return dx + dy  # 默认曼哈顿距离
        
        open_set = []
        initial_h = get_heuristic(local_start, local_goal, search_type)
        heapq.heappush(open_set, (initial_h * heuristic_weight, 0, local_start, [local_start]))
        visited = {}
        visited[local_start] = 0
        
        # 八个方向
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        while open_set:
            f_score, g_score, current, path = heapq.heappop(open_set)
            
            if current in visited and visited[current] < g_score:
                continue
            
            if current == local_goal:
                real_path = []
                for local_x, local_y in path:
                    real_x = (local_x + min_x) * resolution
                    real_y = (local_y + min_y) * resolution
                    real_path.append((real_x, real_y))
                return real_path
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                
                if not (0 <= nx < map_width and 0 <= ny < map_height):
                    continue
                
                if local_grid[nx, ny] == 1:
                    continue
                
                global_x = nx + min_x
                global_y = ny + min_y
                
                if not is_within_maze_bounds(global_x, global_y):
                    continue
                
                cost = 1.0 if dx == 0 or dy == 0 else math.sqrt(2)
                new_g_score = g_score + cost
                
                if neighbor in visited and visited[neighbor] <= new_g_score:
                    continue
                
                visited[neighbor] = new_g_score
                h_score = get_heuristic(neighbor, local_goal, search_type)
                f_score = new_g_score + h_score * heuristic_weight
                new_path = path + [neighbor]
                
                heapq.heappush(open_set, (f_score, new_g_score, neighbor, new_path))
        
        return []

    # 方向相关函数
    DIR_DELTAS = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
    DIRECTIONS = list(DIR_DELTAS.keys())

    def turn_left(self, dir):
        return self.DIRECTIONS[(self.DIRECTIONS.index(dir) - 1) % 4]

    def turn_right(self, dir):
        return self.DIRECTIONS[(self.DIRECTIONS.index(dir) + 1) % 4]

    def turn_back(self, dir):
        return self.DIRECTIONS[(self.DIRECTIONS.index(dir) + 2) % 4]

    def update_occupancy_grid(self, grid, origin, scan_points, scan_hits, resolution):
        origin_x = int(origin[0] / resolution)
        origin_y = int(origin[1] / resolution)
        for i, point in enumerate(scan_points):
            end_x = int(point[0] / resolution)
            end_y = int(point[1] / resolution)
            dx = end_x - origin_x
            dy = end_y - origin_y
            steps = max(abs(dx), abs(dy))
            if steps == 0:
                continue
            for j in range(steps):
                x = origin_x + int(j * dx / steps)
                y = origin_y + int(j * dy / steps)
                if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                    if grid[x, y] == 0:
                        grid[x, y] = 1
            if scan_hits[i] and 0 <= end_x < grid.shape[0] and 0 <= end_y < grid.shape[1]:
                grid[end_x, end_y] = 2

    def build_occupancy_grid_from_segments(self, segments, resolution=0.1, map_size_m=21):
        grid_size = int(map_size_m / resolution)
        grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

        def draw_line_on_grid(start, end):
            x0, y0 = [int(round(c / resolution)) for c in start]
            x1, y1 = [int(round(c / resolution)) for c in end]
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            x, y = x0, y0
            sx = -1 if x0 > x1 else 1
            sy = -1 if y0 > y1 else 1
            if dx > dy:
                err = dx / 2.0
                while x != x1:
                    if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                        grid[x, y] = 2
                    err -= dy
                    if err < 0:
                        y += sy
                        err += dx
                    x += sx
            else:
                err = dy / 2.0
                while y != y1:
                    if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                        grid[x, y] = 2
                    err -= dx
                    if err < 0:
                        x += sx
                        err += dy
                    y += sy
            if 0 <= x1 < grid.shape[0] and 0 <= y1 < grid.shape[1]:
                grid[x1, y1] = 2

        for seg in segments:
            draw_line_on_grid(seg[0], seg[1])

        return grid

    def ray_segment_intersection(self, ray_origin, ray_angle, seg_start, seg_end):
        x1, y1 = ray_origin
        x2 = x1 + math.cos(ray_angle)
        y2 = y1 + math.sin(ray_angle)
        x3, y3 = seg_start
        x4, y4 = seg_end
        denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
        if denom == 0:
            return None
        t = ((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4)) / denom
        u = -((x1 - x2)*(y1 - y3) - (y1 - y2)*(x1 - x3)) / denom
        if t >= 0 and 0 <= u <= 1:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            if math.hypot(ix - x1, iy - y1) <= 10.0:
                return (ix, iy)
        return None

    def simple_lidar_scan(self, real_pos, float_segments):
        scan_pts = []
        scan_hits = []
        for deg in range(0, 360, 1):
            angle_rad = math.radians(deg)
            closest_pt = None
            min_dist = 10.0
            for seg_start, seg_end in float_segments:
                pt = self.ray_segment_intersection(real_pos, angle_rad, seg_start, seg_end)
                if pt:
                    dist = math.hypot(pt[0] - real_pos[0], pt[1] - real_pos[1])
                    if dist < min_dist:
                        min_dist = dist
                        closest_pt = pt
            if closest_pt:
                scan_pts.append(closest_pt)
                scan_hits.append(True)
            else:
                scan_pts.append((real_pos[0] + 10 * math.cos(angle_rad), real_pos[1] + 10 * math.sin(angle_rad)))
                scan_hits.append(False)
        return scan_pts, scan_hits

    def can_move(self, grid, pos, dir):
        dx, dy = self.DIR_DELTAS[dir]
        nx, ny = pos[0] + dx, pos[1] + dy
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
            return grid[nx, ny] != 2
        return False

    def can_move_with_goal_blocking(self, grid, pos, dir, goal_line_info, resolution=0.1):
        dx, dy = self.DIR_DELTAS[dir]
        nx, ny = pos[0] + dx, pos[1] + dy
        
        if not (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]):
            return False
        
        if grid[nx, ny] == 2:
            return False
        
        real_pos = (nx * resolution, ny * resolution)
        if self.is_in_goal_area(real_pos, goal_line_info):
            return False
        
        return True

    def get_goal_area_range(self, maze_data, resolution=0.1):
        """动态分析迷宫数据找出终点线段"""
        segments = maze_data['segments']
        start_point = maze_data['start_point']
        
        # 第一步：找到迷宫的边界范围
        all_points = []
        for seg in segments:
            all_points.extend([seg['start'], seg['end']])
        
        if not all_points:
            raise ValueError("迷宫数据中没有找到任何线段")
        
        min_x = min(point[0] for point in all_points)
        max_x = max(point[0] for point in all_points)
        min_y = min(point[1] for point in all_points)
        max_y = max(point[1] for point in all_points)
        
        # 第二步：分析四个边界的完整性，找出所有缺口
        def get_boundary_segments(boundary_type):
            """获取特定边界上的所有线段"""
            boundary_segs = []
            
            if boundary_type == 'bottom':  # 下边界 y = min_y
                for seg in segments:
                    if seg['start'][1] == min_y and seg['end'][1] == min_y:
                        boundary_segs.append((seg['start'][0], seg['end'][0]))
            elif boundary_type == 'top':  # 上边界 y = max_y
                for seg in segments:
                    if seg['start'][1] == max_y and seg['end'][1] == max_y:
                        boundary_segs.append((seg['start'][0], seg['end'][0]))
            elif boundary_type == 'left':  # 左边界 x = min_x
                for seg in segments:
                    if seg['start'][0] == min_x and seg['end'][0] == min_x:
                        boundary_segs.append((seg['start'][1], seg['end'][1]))
            elif boundary_type == 'right':  # 右边界 x = max_x
                for seg in segments:
                    if seg['start'][0] == max_x and seg['end'][0] == max_x:
                        boundary_segs.append((seg['start'][1], seg['end'][1]))
            
            return boundary_segs
        
        def find_gaps_in_boundary(boundary_segs, total_min, total_max):
            """找出边界中的缺口"""
            if not boundary_segs:
                return [(total_min, total_max)]  # 整个边界都是缺口
            
            # 合并重叠的线段
            sorted_segs = []
            for seg in boundary_segs:
                x1, x2 = seg
                if x1 > x2:
                    x1, x2 = x2, x1
                sorted_segs.append((x1, x2))
            
            sorted_segs.sort()
            
            # 找出缺口
            gaps = []
            current_pos = total_min
            
            for seg_start, seg_end in sorted_segs:
                if current_pos < seg_start:
                    gaps.append((current_pos, seg_start))
                current_pos = max(current_pos, seg_end)
            
            if current_pos < total_max:
                gaps.append((current_pos, total_max))
            
            return gaps
        
        # 第三步：检查每个边界的缺口
        exits = []
        
        # 检查下边界 (y = min_y)
        bottom_segs = get_boundary_segments('bottom')
        bottom_gaps = find_gaps_in_boundary(bottom_segs, min_x, max_x)
        for gap in bottom_gaps:
            if gap[1] - gap[0] > 0.5:  # 缺口足够大
                exits.append({
                    'start': (gap[0], min_y),
                    'end': (gap[1], min_y),
                    'type': 'horizontal',
                    'boundary': 'bottom'
                })
        
        # 检查上边界 (y = max_y)
        top_segs = get_boundary_segments('top')
        top_gaps = find_gaps_in_boundary(top_segs, min_x, max_x)
        for gap in top_gaps:
            if gap[1] - gap[0] > 0.5:  # 缺口足够大
                exits.append({
                    'start': (gap[0], max_y),
                    'end': (gap[1], max_y),
                    'type': 'horizontal',
                    'boundary': 'top'
                })
        
        # 检查左边界 (x = min_x)
        left_segs = get_boundary_segments('left')
        left_gaps = find_gaps_in_boundary(left_segs, min_y, max_y)
        for gap in left_gaps:
            if gap[1] - gap[0] > 0.5:  # 缺口足够大
                exits.append({
                    'start': (min_x, gap[0]),
                    'end': (min_x, gap[1]),
                    'type': 'vertical',
                    'boundary': 'left'
                })
        
        # 检查右边界 (x = max_x)
        right_segs = get_boundary_segments('right')
        right_gaps = find_gaps_in_boundary(right_segs, min_y, max_y)
        for gap in right_gaps:
            if gap[1] - gap[0] > 0.5:  # 缺口足够大
                exits.append({
                    'start': (max_x, gap[0]),
                    'end': (max_x, gap[1]),
                    'type': 'vertical',
                    'boundary': 'right'
                })
        
        # 第四步：找出终点出口（排除起点附近的出口）
        start_x, start_y = start_point
        goal_exit = None
        
        for exit_info in exits:
            # 检查这个出口是否离起点太近（起点出口）
            start_pos = exit_info['start']
            end_pos = exit_info['end']
            
            # 计算出口中心到起点的距离
            center_x = (start_pos[0] + end_pos[0]) / 2
            center_y = (start_pos[1] + end_pos[1]) / 2
            distance_to_start = ((center_x - start_x) ** 2 + (center_y - start_y) ** 2) ** 0.5
            
            if distance_to_start > 3.0:  # 距离起点超过3米的出口才认为是终点
                goal_exit = exit_info
                break
        
        if not goal_exit:
            raise ValueError("未找到合适的终点出口")
        
        # 计算线段中心
        center_x = (goal_exit['start'][0] + goal_exit['end'][0]) / 2
        center_y = (goal_exit['start'][1] + goal_exit['end'][1]) / 2
        
        return {
            'start': goal_exit['start'],
            'end': goal_exit['end'],
            'type': goal_exit['type'],
            'center': (center_x / resolution, center_y / resolution),
            'boundary': goal_exit['boundary']
        }

    def is_in_goal_area(self, pos, goal_line_info, tolerance=0.5):
        px, py = pos
        start_x, start_y = goal_line_info['start']
        end_x, end_y = goal_line_info['end']
        
        if goal_line_info['type'] == 'horizontal':
            if abs(py - start_y) <= tolerance:
                min_x = min(start_x, end_x) - tolerance
                max_x = max(start_x, end_x) + tolerance
                return min_x <= px <= max_x
        elif goal_line_info['type'] == 'vertical':
            if abs(px - start_x) <= tolerance:
                min_y = min(start_y, end_y) - tolerance
                max_y = max(start_y, end_y) + tolerance
                return min_y <= py <= max_y
        
        return False

    def calculate_path_length(self, path):
        length = 0.0
        for i in range(1, len(path)):
            length += math.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)
        return length

    def display_multiple_paths(self, trajectory, scan_grids, start_grid, candidate_paths, best_path_info, step):
        """显示多条路径"""
        if not scan_grids:
            return
            
        # 更新右侧SLAM地图
        self.ax2.clear()
        self.ax2.set_title(f'多路径比较 - 步数: {step}')
        
        # 显示SLAM构建的地图
        current_slam = scan_grids[-1].copy().astype(float)
        display_map = np.full_like(current_slam, 0.5, dtype=float)
        display_map[current_slam == 1] = 1.0  # 自由空间为白色
        display_map[current_slam == 2] = 0.0  # 障碍物为黑色
        
        self.ax2.imshow(display_map.T, cmap='gray_r', origin='lower', vmin=0, vmax=1)
        
        # 绘制轨迹
        if len(trajectory) > 1:
            traj_x, traj_y = zip(*trajectory)
            self.ax2.plot(traj_x, traj_y, 'r-', linewidth=2, alpha=0.8, label='SLAM轨迹')
            
        # 标记起点和当前位置
        self.ax2.scatter(*start_grid, c='green', s=100, marker='s', label='起点', zorder=5)
        if trajectory:
            current_pos = trajectory[-1]
            self.ax2.scatter(*current_pos, c='blue', s=80, marker='o', label='当前位置', zorder=5)
        
        # 绘制所有候选路径
        resolution = self.params['resolution']
        for path_info in candidate_paths:
            path = path_info['path']
            # 转换路径坐标到栅格坐标
            path_x = [p[0] / resolution for p in path]
            path_y = [p[1] / resolution for p in path]
            
            # 判断是否为最优路径
            line_style = '-' if path_info == best_path_info else '--'
            line_width = 3 if path_info == best_path_info else 2
            alpha = 1.0 if path_info == best_path_info else 0.7
            
            label = f"{path_info['name']} ({path_info['length']:.1f}m)"
            if path_info == best_path_info:
                label += " ★最优"
            
            self.ax2.plot(path_x, path_y, color=path_info['color'], linewidth=line_width, 
                         linestyle=line_style, alpha=alpha, label=label)
        
        self.ax2.set_xlim(0, 210)
        self.ax2.set_ylim(0, 210)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend(loc='upper right')
        
        # 更新左侧原始地图
        if len(trajectory) > 1:
            self.ax1.clear()
            self.ax1.set_title('原始迷宫地图 - 候选路径比较')
            
            # 创建原始地图的显示
            resolution = self.params['resolution']
            original_map = self.build_occupancy_grid_from_segments(self.segments, resolution)
            self.ax1.imshow(original_map.T, cmap='gray_r', origin='lower', alpha=0.8)
            
            # 绘制迷宫墙壁线段
            for seg_start, seg_end in self.segments:
                x_coords = [seg_start[0]/resolution, seg_end[0]/resolution]
                y_coords = [seg_start[1]/resolution, seg_end[1]/resolution] 
                self.ax1.plot(x_coords, y_coords, 'k-', linewidth=2, alpha=0.8)
            
            # 绘制SLAM轨迹
            traj_x, traj_y = zip(*trajectory)
            self.ax1.plot(traj_x, traj_y, 'red', linewidth=2, alpha=0.8, label='SLAM轨迹')
            
            # 绘制所有候选路径
            for path_info in candidate_paths:
                path = path_info['path']
                path_x = [p[0] / resolution for p in path]
                path_y = [p[1] / resolution for p in path]
                
                line_style = '-' if path_info == best_path_info else '--'
                line_width = 3 if path_info == best_path_info else 2
                alpha = 1.0 if path_info == best_path_info else 0.7
                
                label = f"{path_info['name']} ({path_info['length']:.1f}m)"
                if path_info == best_path_info:
                    label += " ★最优"
                
                self.ax1.plot(path_x, path_y, color=path_info['color'], linewidth=line_width,
                             linestyle=line_style, alpha=alpha, label=label)
            
            # 标记位置
            self.ax1.scatter(*start_grid, c='green', s=100, marker='s', label='起点', zorder=5)
            current_pos = trajectory[-1]
            self.ax1.scatter(*current_pos, c='orange', s=100, marker='o', label='当前位置', zorder=5)
            
            self.ax1.set_xlim(0, 210)
            self.ax1.set_ylim(0, 210)
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend(loc='upper right')
        
        self.canvas.draw()


def main():
    """主程序"""
    root = tk.Tk()
    app = SimpleSLAMGUI(root)
    
    def on_closing():
        if app.simulation_running:
            if messagebox.askokcancel("退出", "确定要退出吗？"):
                app.simulation_running = False
                root.destroy()
        else:
            root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()




if __name__ == "__main__":
    print("启动简化界面SLAM仿真系统...")
    print("保留完整算法功能：SLAM探索 + 多路径A*优化")
    print("新功能：计算多条候选路径，自动选择最优路径执行")
    main() 