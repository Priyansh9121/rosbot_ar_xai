
"""
GUI控制模块
提供图形化控制界面
"""

import tkinter as tk
import threading

class GUIController:
    """GUI控制器"""
    
    def __init__(self, navigation_controller):
        self.navigation_controller = navigation_controller
        self.root = None
        
    def start_gui(self):
        """启动GUI线程"""
        gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        gui_thread.start()
        return gui_thread
    
    def _run_gui(self):
        """运行GUI界面"""
        self.root = tk.Tk()
        self.root.title("🤖 Control Panel")
        self.root.geometry("350x300")
        
        self._create_control_buttons()
        self._create_backup_controls()
        self._create_parameter_controls()
        
        self.root.mainloop()
    
    def _create_control_buttons(self):
        """创建基本控制按钮"""
        # 方向控制
        tk.Button(self.root, text="⬆️ Forward", 
                 command=lambda: self.navigation_controller.send_manual_command(1, 0), 
                 width=10).grid(row=0, column=1, padx=5, pady=5)
        
        tk.Button(self.root, text="⬅️ Left", 
                 command=lambda: self.navigation_controller.send_manual_command(0, 1), 
                 width=10).grid(row=1, column=0, padx=5, pady=5)
        
        tk.Button(self.root, text="⏹ STOP", fg='white', bg='red', 
                 command=self.navigation_controller.stop_robot, 
                 width=10).grid(row=1, column=1, padx=5, pady=5)
        
        tk.Button(self.root, text="➡️ Right", 
                 command=lambda: self.navigation_controller.send_manual_command(0, -1), 
                 width=10).grid(row=1, column=2, padx=5, pady=5)
        
        tk.Button(self.root, text="⬇️ Back", 
                 command=lambda: self.navigation_controller.send_manual_command(-1, 0), 
                 width=10).grid(row=2, column=1, padx=5, pady=5)
        
        tk.Button(self.root, text="▶️ Resume", 
                 command=self.navigation_controller.resume_navigation, 
                 width=10).grid(row=3, column=1, padx=5, pady=5)
    
    def _create_backup_controls(self):
        """创建后退控制按钮"""
        tk.Label(self.root, text="--- Backup Controls ---", 
                font=("Arial", 10, "bold")).grid(row=4, column=0, columnspan=3, pady=10)
        
        tk.Button(self.root, text="🔙 Force Backup", fg='white', bg='orange', 
                 command=self.navigation_controller.force_backup, 
                 width=15).grid(row=5, column=0, columnspan=2, padx=5, pady=5)
        
        tk.Button(self.root, text="❌ Cancel Backup", 
                 command=self.navigation_controller.cancel_backup, 
                 width=15).grid(row=5, column=2, padx=5, pady=5)
    
    def _create_parameter_controls(self):
        """创建参数调整控件"""
        tk.Label(self.root, text="--- Parameters ---", 
                font=("Arial", 10, "bold")).grid(row=6, column=0, columnspan=3, pady=10)
        
        # 后退距离调整
        tk.Label(self.root, text="Backup Distance (m):").grid(row=7, column=0, padx=5, pady=2)
        
        from config import Config
        self.backup_distance_var = tk.StringVar(value=str(Config.BACKWARD_DISTANCE))
        backup_distance_entry = tk.Entry(self.root, textvariable=self.backup_distance_var, width=8)
        backup_distance_entry.grid(row=7, column=1, padx=5, pady=2)
        
        tk.Button(self.root, text="Update", command=self._update_backup_distance, 
                 width=8).grid(row=7, column=2, padx=5, pady=2)
        
        # 触发距离调整
        tk.Label(self.root, text="Trigger Distance (m):").grid(row=8, column=0, padx=5, pady=2)
        
        self.trigger_distance_var = tk.StringVar(value=str(Config.BACKWARD_TRIGGER_DISTANCE))
        trigger_distance_entry = tk.Entry(self.root, textvariable=self.trigger_distance_var, width=8)
        trigger_distance_entry.grid(row=8, column=1, padx=5, pady=2)
        
        tk.Button(self.root, text="Update", command=self._update_trigger_distance, 
                 width=8).grid(row=8, column=2, padx=5, pady=2)
    
    def _update_backup_distance(self):
        """更新后退距离"""
        try:
            new_distance = float(self.backup_distance_var.get())
            self.navigation_controller.update_backup_distance(new_distance)
        except ValueError:
            pass  # 日志已在navigation_controller中处理
    
    def _update_trigger_distance(self):
        """更新触发距离"""
        try:
            new_distance = float(self.trigger_distance_var.get())
            self.navigation_controller.update_trigger_distance(new_distance)
        except ValueError:
            pass  # 日志已在navigation_controller中处理
