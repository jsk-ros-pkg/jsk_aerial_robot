#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import Vector3

def load_yaml_file(file_path):
    """YAMLファイルを読み込み、パラメータを設定する関数"""
    try:
        with open(file_path, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            if 'parameters' in yaml_data:
                params = yaml_data['parameters']
                p1_value = float(params['p1_value'])
                p2_value = float(params['p2_value'])
                rospy.set_param('/p1_value', params['p1_value'])
                rospy.set_param('/p2_value', params['p2_value'])
                rospy.loginfo(f"Parameters set: p1_value={p1_value}, p2_value={p2_value}")
                return p1_value, p2_value
            else:
                rospy.logwarn("YAML file does not contain 'parameters' field.")
                return None, None
    except (FileNotFoundError, ValueError) as e:
        rospy.logwarn(f"YAML file not found or invalid format: {file_path}. Using default parameters.")
        return None, None
    except yaml.YAMLError as e:
        rospy.logerr(f"Error loading YAML file: {e}")
        return None, None

def main():
    # ROSノードの初期化
    rospy.init_node('param_loader', anonymous=True)

    # デフォルトのパラメータを設定
    default_p1 = 0.3
    default_p2 = 0.3

    rospy.set_param('/p1_value', default_p1)  # デフォルトのp1_value
    rospy.set_param('/p2_value', default_p2)  # デフォルトのp2_value

    rospy.loginfo(f"Default parameters set: p1_value={default_p1}, p2_value={default_p2}")

    # ユーザーが指定したモード (YAMLファイル) のパスを引数で取得
    yaml_file_path = rospy.get_param('~yaml_file_path', None)

    if yaml_file_path:
        rospy.loginfo(f"Loading parameters from: {yaml_file_path}")
        # YAMLファイルを読み込んでパラメータを設定
        p1_value, p2_value = load_yaml_file(yaml_file_path)
    else:
        rospy.loginfo("No YAML file provided.")
        p1_value, p2_value = default_p1, default_p2

    # 値がNoneでないことを確認してfloat型に変換
    if p1_value is None or p2_value is None:
        p1_value = default_p1
        p2_value = default_p2
    else:
        p1_value = float(p1_value)
        p2_value = float(p2_value)

    v1_value = p1_value * 4096 / 0.9
    v2_value = p2_value * 4096 / 0.9

    # Vector3メッセージをmpa_cmdトピックにパブリッシュ
    pub_v1v2 = rospy.Publisher('mpa_cmd', Vector3, queue_size=10)

    mpa_cmd_msg = Vector3()
    mpa_cmd_msg.x = v1_value
    mpa_cmd_msg.y = v2_value

    # 1回だけPublishして終了
    rospy.sleep(1)  
    pub_v1v2.publish(mpa_cmd_msg)
    rospy.loginfo(f"Published V1={v1_value}, V2={v2_value}")

    rospy.sleep(1)  # 終了前に少し待機

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
