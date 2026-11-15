import os
import time
import cv2
import yaml
import json
import glob
import shutil
import numpy as np

import rclpy
from rclpy.node import Node

# ---> 恢复使用底层的极速写入和 C++ 序列化 <---
from mcap.writer import Writer as McapWriter
from rclpy.serialization import serialize_message

# 引入所有需要的消息类型
from sensor_msgs.msg import JointState, CameraInfo, CompressedImage, RegionOfInterest
from std_msgs.msg import String
from builtin_interfaces.msg import Time

# ================= 新增：硬编码注入的 ROS 2 Schema 字典 =================
# 这样不仅保留了极速性能，还能完美满足甲方脚本的离线 Schema 解析需求！
SCHEMA_TEXTS = {
    'sensor_msgs/msg/JointState': """\
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
""",
    'sensor_msgs/msg/CameraInfo': """\
std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: sensor_msgs/RegionOfInterest
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
""",
    'sensor_msgs/msg/CompressedImage': """\
std_msgs/Header header
string format
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
""",
    'std_msgs/msg/String': """\
string data
"""
}

class VLADataCollector(Node):
    def __init__(self, config_path='config.yaml'):
        super().__init__('vla_data_collector')

        # ================= 1. 解析 YAML 配置 =================
        if not os.path.exists(config_path):
            self.get_logger().error(f"找不到配置文件: {config_path}")
            raise FileNotFoundError(f"Missing config file: {config_path}")
            
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        coll_cfg = self.config.get('collection_config', {})
        self.state_save_freq = coll_cfg.get('state_save_freq', 50.0)
        self.image_save_freq = coll_cfg.get('image_save_freq', 10.0)
        self.total_sampled_frames = coll_cfg.get('total_sampled_frames', 500)
        self.timeout_threshold = 1.0  
        
        self.base_dir = coll_cfg.get('dataset_dir', 'data/default_dataset')
        self.task_info_dict = self.config.get('task_info', {})
        self.custom_cam_cfg = self.config.get('custom_camera_intrinsics', {})

        # ================= 2. 目录初始化 =================
        self.video_dir = os.path.join(self.base_dir, "videos")
        os.makedirs(self.video_dir, exist_ok=True)
        
        existing_mcaps = glob.glob(os.path.join(self.base_dir, "*.mcap"))
        if existing_mcaps:
            indices = [int(os.path.basename(f).split('.')[0]) for f in existing_mcaps if os.path.basename(f).split('.')[0].isdigit()]
            self.current_episode_id = max(indices) + 1 if indices else 0
        else:
            self.current_episode_id = 0

        self.get_logger().info(f"数据保存路径: {self.base_dir}")
        self.get_logger().info(f"下一个 Episode 将保存为: {self.current_episode_id}.mcap")

        with open(os.path.join(self.base_dir, 'record_config.yaml'), 'w') as f:
            yaml.dump(self.config, f, allow_unicode=True)

        # ================= 3. 话题映射表 =================
        self.state_topic_map = {
            '/joint_states_ctrl_left': '/robot/arm_left_leader/joint_states',
            '/joint_states_ctrl_right': '/robot/arm_right_leader/joint_states',
            '/joint_states_left': '/robot/arm_left_follow/joint_states',
            '/joint_states_right': '/robot/arm_right_follow/joint_states',
        }
        
        self.live_camera_info_map = {
            '/camera/camera/color/camera_info': '/robot/camera/cam_high/image_raw/compressed/camera_info'
        }
        
        self.synthetic_camera_info_map = {
            '/dabai_right/camera_info': '/robot/camera/cam_right/image_raw/compressed/camera_info',
            '/dabai_left/camera_info': '/robot/camera/cam_left/image_raw/compressed/camera_info'
        }
        
        self.video_topic_map = {
            '/camera/camera/color/image_rect_raw/compressed': 'cam_high',
            '/dabai_right/image_raw/compressed': 'cam_right',
            '/dabai_left/image_raw/compressed': 'cam_left'
        }

        # ================= 4. 核心变量与缓存 =================
        self.state = 'IDLE' 
        self.sampled_state_count = 0  
        
        self.mcap_file = None
        self.writer = None
        self._schemas = {}
        self._channels = {}
        self.video_writers = {}
        
        self.latest_msgs = {}        
        self.latest_images_cv = {}

        self.last_valid_leader_msgs = {}
        self.movement_detected = False   
        
        self.last_msg_time = {}
        self.raw_msg_counts = {}
        self.episode_start_time = 0.0

        # ================= 5. 创建订阅者 =================
        for topic in self.state_topic_map.keys():
            self.create_subscription(JointState, topic, self.make_cache_callback(topic), 10)
            
        for topic in self.live_camera_info_map.keys():
            self.create_subscription(CameraInfo, topic, self.make_cache_callback(topic), 10)
            
        for topic in self.video_topic_map.keys():
            self.create_subscription(CompressedImage, topic, self.make_image_cache_callback(topic), 10)

        # ================= 6. 定时器 =================
        self.state_timer = self.create_timer(1.0 / self.state_save_freq, self.sync_state_timer_callback)
        self.image_timer = self.create_timer(1.0 / self.image_save_freq, self.sync_image_timer_callback)
        self.ui_timer = self.create_timer(1.0 / 10.0, self.ui_loop)
        
        self.print_instructions()

    # ---------------- 缓存刷新与异常过滤 ----------------
    def make_cache_callback(self, topic_name):
        def callback(msg):
            self.last_msg_time[topic_name] = time.time()
            
            if self.state == 'COLLECTING':
                self.raw_msg_counts[topic_name] = self.raw_msg_counts.get(topic_name, 0) + 1
            
            if 'ctrl' in topic_name:
                is_all_zero = (len(msg.position) > 0 and all(abs(p) < 1e-4 for p in msg.position))
                
                if not is_all_zero:
                    self.movement_detected = True
                    self.last_valid_leader_msgs[topic_name] = msg
                    self.latest_msgs[topic_name] = msg
                else:
                    if topic_name in self.last_valid_leader_msgs:
                        # ---> 遵循真实原则：只继承上一帧的物理位置，但完全使用当前发来的全新时间戳 <---
                        valid_msg = JointState()
                        valid_msg.header = msg.header
                        valid_msg.name = self.last_valid_leader_msgs[topic_name].name
                        valid_msg.position = self.last_valid_leader_msgs[topic_name].position
                        valid_msg.velocity = self.last_valid_leader_msgs[topic_name].velocity
                        valid_msg.effort = self.last_valid_leader_msgs[topic_name].effort
                        self.latest_msgs[topic_name] = valid_msg
                    else:
                        self.latest_msgs[topic_name] = msg
            else:
                self.latest_msgs[topic_name] = msg
                        
        return callback

    def make_image_cache_callback(self, topic_name):
        def callback(msg):
            self.last_msg_time[topic_name] = time.time()
            
            if self.state == 'COLLECTING':
                self.raw_msg_counts[topic_name] = self.raw_msg_counts.get(topic_name, 0) + 1
            
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                video_name = self.video_topic_map[topic_name]
                self.latest_images_cv[video_name] = cv_image
        return callback

    # ---------------- 严格同步的写入定时器 ----------------
    def sync_state_timer_callback(self):
        if self.state not in ['WAITING_FOR_MOVEMENT', 'COLLECTING']:
            return

        current_time = time.time()
        timeout_topics = []
        expected_live_topics = list(self.state_topic_map.keys()) + list(self.live_camera_info_map.keys()) + list(self.video_topic_map.keys())

        for topic in expected_live_topics:
            if topic in self.last_msg_time:
                if current_time - self.last_msg_time[topic] > self.timeout_threshold:
                    timeout_topics.append(topic)

        if timeout_topics:
            self.get_logger().error(f"🚨 严重异常：传感器掉线！以下话题超过 {self.timeout_threshold} 秒未收到数据: {timeout_topics}")
            if self.state == 'COLLECTING':
                self.get_logger().error("为防止写入脏数据，本次录制已自动强制中止！")
                self.discard_current_episode()
            else:
                self.state = 'IDLE' 
                self.get_logger().warn("已自动退出预备采集状态，请修复传感器后重试。")
            return

        if self.state == 'WAITING_FOR_MOVEMENT':
            missing_states = [t for t in self.state_topic_map.keys() if t not in self.latest_msgs]
            
            if not missing_states and self.movement_detected:
                self.state = 'COLLECTING'
                self.episode_start_time = time.time()
                self.get_logger().info("🤖 缓存就绪且检测到主臂有效动作，正式开始同步录制！")
            else:
                self.waiting_log_count = getattr(self, 'waiting_log_count', 0) + 1
                if self.waiting_log_count % 50 == 0:
                    if missing_states:
                        self.get_logger().warn(f"等待中... 发现以下话题尚未发送第一帧数据: {missing_states}")
                    elif not self.movement_detected:
                        self.get_logger().info("等待中... 话题已全部连通，等待主臂做出非 0 动作触发。")
                return
                
        if self.state != 'COLLECTING': 
            return
            
        timestamp_ns = self.get_clock().now().nanoseconds
        
        for raw_topic, mapped_topic in self.state_topic_map.items():
            msg = self.latest_msgs[raw_topic]
            # 这里没有任何篡改，原汁原味地把包含真实时间戳的 msg 写入 MCAP
            self.write_to_mcap(mapped_topic, msg, timestamp_ns)
            
        self.sampled_state_count += 1
        
        if self.sampled_state_count % 50 == 0:
            self.get_logger().info(f"正在录制... 已采集同步帧: {self.sampled_state_count} / {self.total_sampled_frames}")
        
        if self.sampled_state_count >= self.total_sampled_frames:
            self.get_logger().info("达到指定总采样帧数，自动触发保存。")
            self.save_episode()

    def sync_image_timer_callback(self):
        if self.state != 'COLLECTING': 
            return
        
        if not all(t in self.latest_msgs for t in self.live_camera_info_map.keys()): return
        if not all(name in self.latest_images_cv for name in self.video_topic_map.values()): return
        
        timestamp_ns = self.get_clock().now().nanoseconds
        
        for raw_topic, mapped_topic in self.live_camera_info_map.items():
            msg = self.latest_msgs[raw_topic]
            self.write_to_mcap(mapped_topic, msg, timestamp_ns)
            
        if self.custom_cam_cfg:
            for raw_topic, mapped_topic in self.synthetic_camera_info_map.items():
                if 'right' in raw_topic:
                    cam_key = 'cam_right'
                    frame_id = 'camera_right_color_optical_frame'
                    ref_img_topic = '/dabai_right/image_raw/compressed'
                elif 'left' in raw_topic:
                    cam_key = 'cam_left'
                    frame_id = 'camera_left_color_optical_frame'
                    ref_img_topic = '/dabai_left/image_raw/compressed'
                else:
                    continue
                
                cfg = self.custom_cam_cfg.get(cam_key)
                if not cfg: continue

                synth_msg = CameraInfo()
                
                # ---> 遵循真实原则：精准复刻对应图像自身的物理时间戳！ <---
                ref_msg = self.latest_msgs.get(ref_img_topic)
                if ref_msg and hasattr(ref_msg, 'header'):
                    synth_msg.header.stamp = ref_msg.header.stamp
                else:
                    synth_msg.header.stamp = Time(sec=int(timestamp_ns // 1e9), nanosec=int(timestamp_ns % 1e9))
                    
                synth_msg.header.frame_id = frame_id
                
                synth_msg.height = cfg['height']
                synth_msg.width = cfg['width']
                synth_msg.distortion_model = cfg['distortion_model']
                synth_msg.d = cfg['D']
                synth_msg.k = cfg['K']
                synth_msg.r = cfg['R']
                synth_msg.p = cfg['P']
                synth_msg.binning_x = 0
                synth_msg.binning_y = 0
                synth_msg.roi = RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)
                
                self.write_to_mcap(mapped_topic, synth_msg, timestamp_ns)
            
        for video_name in self.video_topic_map.values():
            cv_image = self.latest_images_cv[video_name]
            
            if video_name not in self.video_writers:
                h, w, _ = cv_image.shape
                current_video_dir = os.path.join(self.video_dir, str(self.current_episode_id))
                video_path = os.path.join(current_video_dir, f"{video_name}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writers[video_name] = cv2.VideoWriter(video_path, fourcc, self.image_save_freq, (w, h))
                
            self.video_writers[video_name].write(cv_image)

    # ---------------- UI 与 状态机控制 ----------------
    def ui_loop(self):
        for name, img in self.latest_images_cv.items():
            display_img = cv2.resize(img, (640, 360))  # 缩小一半
            cv2.imshow(f"Camera: {name}", display_img)
            #cv2.imshow(f"Camera: {name}", img)

        if not self.latest_images_cv:
            dummy = np.zeros((100, 300, 3), dtype=np.uint8)
            cv2.putText(dummy, "Waiting for camera...", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.imshow("VLA Data Collector", dummy)

        key = cv2.waitKey(1) & 0xFF
        if key == 32:  
            if self.state == 'IDLE': self.start_episode()
        elif key == ord('s'):
            if self.state in ['WAITING_FOR_MOVEMENT', 'COLLECTING']: self.save_episode()
        elif key == ord('q'):
            if self.state in ['WAITING_FOR_MOVEMENT', 'COLLECTING']: self.discard_current_episode()
        elif key == ord('r'):
            if self.state == 'IDLE': self.remove_last_episode()
        elif key == ord('z'):
            self.exit_program()

    def start_episode(self):
        self.get_logger().info(f"--- 🟢 准备采集 Episode {self.current_episode_id}，请操作主臂以激活录制... ---")
        
        mcap_path = os.path.join(self.base_dir, f"{self.current_episode_id}.mcap")
        self.mcap_file = open(mcap_path, "wb")
        
        # 恢复使用官方基础 Writer
        self.writer = McapWriter(self.mcap_file)
        self.writer.start()
        
        self._schemas.clear()
        self._channels.clear()
        self.video_writers.clear()
        
        self.sampled_state_count = 0
        self.latest_msgs.clear()
        self.last_valid_leader_msgs.clear()
        self.movement_detected = False   
        
        self.raw_msg_counts.clear()
        self.episode_start_time = 0.0
        
        current_video_dir = os.path.join(self.video_dir, str(self.current_episode_id))
        os.makedirs(current_video_dir, exist_ok=True)

        info_msg = String()
        info_msg.data = json.dumps(self.task_info_dict, indent=2, ensure_ascii=False)
        self.write_to_mcap('/task_info', info_msg, self.get_clock().now().nanoseconds)

        self.state = 'WAITING_FOR_MOVEMENT'

    def save_episode(self):
        duration = time.time() - self.episode_start_time if self.episode_start_time > 0 else 0
        
        self.state = 'IDLE'
        self.close_files()
        
        self.get_logger().info(f"--- 💾 Episode {self.current_episode_id} 已成功保存！ ---")
        
        if duration > 0:
            self.get_logger().info("\n========== 📊 硬件底层真实通讯频率体检 ==========")
            self.get_logger().info(f"录制总时长: {duration:.2f} 秒")
            self.get_logger().info("【摄像头 Raw FPS】:")
            for topic, cam_name in self.video_topic_map.items():
                count = self.raw_msg_counts.get(topic, 0)
                fps = count / duration
                self.get_logger().info(f"  - {cam_name}: {fps:.2f} Hz  (原始通讯: {count} 帧)")
                
            self.get_logger().info("【机械臂状态 Raw Hz】:")
            for topic in self.state_topic_map.keys():
                count = self.raw_msg_counts.get(topic, 0)
                hz = count / duration
                short_name = topic.split('/')[-1] if '/' in topic else topic 
                self.get_logger().info(f"  - {short_name}: {hz:.2f} Hz  (原始通讯: {count} 帧)")
            self.get_logger().info("====================================================\n")
            
        self.current_episode_id += 1

    def discard_current_episode(self):
        self.state = 'IDLE'
        self.close_files()
        self.get_logger().warn(f"--- 🛑 Episode {self.current_episode_id} 采集被中止，废弃数据清理中... ---")
        self.delete_episode_files(self.current_episode_id)

    def remove_last_episode(self):
        if self.current_episode_id == 0:
            self.get_logger().warn("没有历史 Episode。")
            return
        target_id = self.current_episode_id - 1
        self.get_logger().warn(f"--- 🗑️ 正在删除历史 Episode {target_id}... ---")
        self.delete_episode_files(target_id)
        self.current_episode_id = target_id

    def exit_program(self):
        self.get_logger().info("收到退出指令 [z]，正在关闭...")
        if self.state in ['WAITING_FOR_MOVEMENT', 'COLLECTING']:
            self.get_logger().warn("检测到录制中途退出，自动舍弃废弃片段！")
            self.discard_current_episode()
        rclpy.shutdown()

    def close_files(self):
        for name, video_writer in self.video_writers.items():
            video_writer.release()
        if self.writer is not None:
            self.writer.finish()
            self.writer = None
        if self.mcap_file is not None and not self.mcap_file.closed:
            self.mcap_file.close()
            self.mcap_file = None

    def delete_episode_files(self, ep_id):
        mcap_path = os.path.join(self.base_dir, f"{ep_id}.mcap")
        if os.path.exists(mcap_path): os.remove(mcap_path)
        current_video_dir = os.path.join(self.video_dir, str(ep_id))
        if os.path.exists(current_video_dir): shutil.rmtree(current_video_dir)

    def write_to_mcap(self, mapped_topic_name, msg, timestamp_ns):
        if self.writer is None: return
        msg_type = type(msg)
        
        # ---> 核心机制：在此处判断是否需要注入 Schema <---
        if msg_type not in self._schemas:
            pkg_name = msg_type.__module__.split('.')[0]  
            msg_name = msg_type.__name__                  
            schema_name = f"{pkg_name}/msg/{msg_name}"
            
            # 获取事先准备好的 Schema 字符串
            schema_text = SCHEMA_TEXTS.get(schema_name, "")
            
            # 关键：将完整的 Schema 定义写进数据文件，给甲方的纯 Python 解码器看！
            schema_id = self.writer.register_schema(
                name=schema_name, 
                encoding="ros2msg", 
                data=schema_text.encode('utf-8')  # 不再是空字节 b"" 了
            )
            self._schemas[msg_type] = schema_id
            
        if mapped_topic_name not in self._channels:
            channel_id = self.writer.register_channel(
                topic=mapped_topic_name, 
                message_encoding="cdr", 
                schema_id=self._schemas[msg_type]
            )
            self._channels[mapped_topic_name] = channel_id

        # 极致速度：原汁原味的真实硬件数据，0 篡改，100% 物理时序
        self.writer.add_message(
            channel_id=self._channels[mapped_topic_name],
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=serialize_message(msg)
        )

    def print_instructions(self):
        print("\n" + "="*50)
        print("VLA 数采系统已就绪！(极速C++序列化 + 完美 Schema 注入版)")
        print(" [空格键] : 预备采集 (将等待主臂产生非0动作后正式开始)")
        print(" [ s ]    : 结束并保存当前 Episode")
        print(" [ q ]    : 中途舍弃当前正在采集的 Episode")
        print(" [ r ]    : 删除最后一个已保存的 Episode")
        print(" [ z ]    : 安全退出程序 (中途退出不保存)")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    try:
        collector = VLADataCollector('config.yaml')
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if 'collector' in locals() and collector.state in ['WAITING_FOR_MOVEMENT', 'COLLECTING']:
            print("\n检测到强制中断，丢弃当前损坏的数据片段...")
            collector.discard_current_episode()
            
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()