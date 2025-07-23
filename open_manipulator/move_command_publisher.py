import re
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def extract_keywords(text):
    result = {
        "action": None,
        "direction": None,
        "value": None,
        "unit": None
    }

    lower_text = text.lower()

    # ✅ gripper 관련 문장인지 먼저 확인
    if re.search(r"\b(gripper|grip|gripping|end effector)\b", lower_text):
        # ✅ gripper 동작 탐지
        gripper_phrases = {
            "open": [r"\bopen\b", r"\brelease\b", r"\blet go\b", r"\bunclamp\b"],
            "close": [r"\bclose\b", r"\bgrip\b", r"\bgrab\b", r"\bhold\b", r"\bclamp\b", r"\bsqueeze\b"],
            "wide": [r"\bwide\b", r"\bopen fully\b", r"\bmaximum\b", r"\bwidest\b"],
            "reset": [r"\breset\b", r"\binitialize\b"]
        }

        for direction, patterns in gripper_phrases.items():
            for pattern in patterns:
                if re.search(pattern, lower_text):
                    return {
                        "action": "gripper",
                        "direction": direction,
                        "value": None,
                        "unit": None
                    }



    # ✅ 초기화
    init_keywords = [
        "initial pose", "reset position", "go back to start", "return to initial",
        "home position", "back to home", "original position", "go to default"
    ]
    if any(phrase in lower_text for phrase in init_keywords):
        return {
            "action": "initialize",
            "direction": None,
            "value": 0,
            "unit": None
        }

    # ✅ 방향 키워드 탐지
    direction_map = {
        "left": "left",
        "right": "right",
        "forward": "forward",
        "backward": "backward",
        "up": "up",
        "down": "down"
    }

    reverse_direction = {
        "left": "right",
        "right": "left",
        "forward": "backward",
        "backward": "forward",
        "up": "down",
        "down": "up"
    }

    for keyword, value in direction_map.items():
        if keyword in lower_text:
            result["direction"] = value
            break

    # ✅ 수치 값 및 단위 추출
    value_match = re.search(r'(-?[0-9]+(?:\.[0-9]+)?)\s*(degrees?|rads?|cm|centimeter|m|meter|inch|inches)?', lower_text)
    if value_match:
        val = float(value_match.group(1))
        unit = value_match.group(2)

        if unit in ['degree', 'degrees']:
            result["unit"] = "degree"
            result["value"] = abs(val)
            result["action"] = "rotate"
        elif unit in ['rad', 'rads']:
            result["unit"] = "rad"
            result["value"] = abs(val)
            result["action"] = "rotate"
        elif unit in ['cm', 'centimeter']:
            result["unit"] = "cm"
            result["value"] = abs(val)
            result["action"] = "move"
        elif unit in ['m', 'meter']:
            result["unit"] = "cm"
            result["value"] = abs(val * 100)
            result["action"] = "move"
        elif unit in ['inch', 'inches']:
            result["unit"] = "cm"
            result["value"] = abs(val * 2.54)
            result["action"] = "move"

        # 음수면 방향 반대로
        if val < 0 and result["direction"] in reverse_direction:
            result["direction"] = reverse_direction[result["direction"]]

    # ✅ action 추론
    if result["action"] is None:
        if any(word in lower_text for word in ['rotate', 'turn']):
            result["action"] = "rotate"
            result["unit"] = result["unit"] or "degree"
            result["value"] = result["value"] or 10
        elif any(word in lower_text for word in ['move', 'go', 'forward', 'backward', 'up', 'down']):
            result["action"] = "move"
            result["unit"] = result["unit"] or "cm"
            result["value"] = result["value"] or 5

    return result

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, '/command_topic', 10)

    def publish_command(self, command_dict):
        msg = String()
        msg.data = json.dumps(command_dict, ensure_ascii=False)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main():
    rclpy.init()
    node = CommandPublisher()

    print("💬 자연어로 명령을 입력하세요 (예: 'grip', 'rotate left 10 degrees', 'move up')")
    print("종료하려면 'exit' 입력")

    while rclpy.ok():
        text = input(">>> ").strip()
        if text.lower() == "exit":
            print("종료합니다.")
            break

        structured_command = extract_keywords(text)
        print("\n[🧠 구조화된 명령]")
        print(json.dumps(structured_command, indent=2, ensure_ascii=False))

        # 퍼블리시
        node.publish_command(structured_command)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
