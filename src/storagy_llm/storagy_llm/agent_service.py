import rclpy
from rclpy.node import Node
from storagy_llm.robot_tools import ToolSet, create_tools
from storagy_interfaces.srv import Agent
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import yaml
import cv2
import base64
import re
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.agents import create_agent
from langgraph.checkpoint.memory import MemorySaver


llm_dir = get_package_share_directory('storagy_llm')
env_file_path = Path(llm_dir) / '.env'
load_dotenv(dotenv_path=env_file_path)

prompt_file = Path(llm_dir) / 'params/prompt.yaml'
with open(prompt_file, 'r', encoding='utf-8') as f:
    prompt_data = yaml.safe_load(f)    


TABLE_ALIASES = {
    "table1": [
        r"\bt\s*1\b", r"\btable\s*1\b", r"\btable1\b",
        "t1", "T1", "1번", "1 번", "테이블1", "테이블 1",
        "첫번째", "첫 번째", "일번",
    ],
    "table2": [
        r"\bt\s*2\b", r"\btable\s*2\b", r"\btable2\b",
        "t2", "T2", "2번", "2 번", "테이블2", "테이블 2",
        "두번째", "두 번째", "이번",
    ],
    "table3": [
        r"\bt\s*3\b", r"\btable\s*3\b", r"\btable3\b",
        "t3", "T3", "3번", "3 번", "테이블3", "테이블 3",
        "세번째", "세 번째", "삼번",
    ],
    "table4": [
        r"\bt\s*4\b", r"\btable\s*4\b", r"\btable4\b",
        "t4", "T4", "4번", "4 번", "테이블4", "테이블 4",
        "네번째", "네 번째", "사번",
    ],
}

NON_NAVIGATION_KEYWORDS = (
    "앞에", "보여", "카메라", "사진", "현재 위치", "어디", "목록",
    "취소", "멈춰", "정지", "그만", "배회", "속도", "회전",
)

NAVIGATION_KEYWORDS = (
    "안내", "이동", "가", "데려", "목적", "도착", "테이블", "자리",
    "손님", "시각장애", "준비", "출발", "guide", "go", "move",
    "navigate", "start", "table",
)


class AgentLLM(Node):
    def __init__(self):
        super().__init__('agent_llm')

        self.srv = self.create_service(Agent, 'llm_agent', self.handle_question)
        self.llm = ChatOpenAI(model="gpt-4o-mini", temperature=0)
        
        self.bridge = CvBridge()
        self.latest_image_msg = None
        
        # 1. Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        yaml_file = Path(llm_dir) / 'params/points.yaml'
        with open(yaml_file, 'r') as f:
            config = yaml.safe_load(f)

        places = {name: (info["x"], info["y"], info["qz"], info["qw"]) for name, info in config["places"].items()}
        self.tool_set = ToolSet(places, explain_fn=self.explain_current_view)
        tool_list = create_tools(self.tool_set)

        # langchain v1.2+ 의 create_agent API 사용
        # checkpointer 를 지정하면 session 별 대화 히스토리를 자동 관리합니다.
        self.checkpointer = MemorySaver()
        self.agent_graph = create_agent(
            model=self.llm,
            tools=tool_list,
            system_prompt=prompt_data["system"],
            checkpointer=self.checkpointer,
        )
        
        self.get_logger().info("agent service start")

    def destination_from_prompt(self, query: str):
        text = (query or "").strip()
        lowered = text.lower()

        for place, aliases in TABLE_ALIASES.items():
            for alias in aliases:
                if alias.startswith(r"\b"):
                    if re.search(alias, lowered):
                        return place
                elif alias in text or alias.lower() in lowered:
                    return place

        if any(keyword in text or keyword in lowered
               for keyword in NON_NAVIGATION_KEYWORDS):
            return None

        if any(keyword in text or keyword in lowered
               for keyword in NAVIGATION_KEYWORDS):
            return "table4"

        return "table4"

    def route_destination_prompt(self, query: str):
        destination = self.destination_from_prompt(query)
        if destination is None:
            return None

        result = self.tool_set.move_to_location(destination)
        self.get_logger().info(
            f"Destination prompt routed to {destination}: {result}")
        return (
            f"{destination}로 안내를 시작합니다. "
            "목적지가 명확하지 않으면 기본 목적지는 table4입니다."
        )

    def image_callback(self, msg: Image):
        self.latest_image_msg = msg

    def explain_current_view(self) -> str:
        if self.latest_image_msg is None:
            return "현재 전방 카메라 이미지 데이터를 수신하지 못했습니다. 카메라 토픽이 켜져 있는지 확인해 주세요."
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return f"이미지 변환 중 오류가 발생했습니다: {e}"
        
        try:
            from langchain_core.messages import HumanMessage
            message = HumanMessage(
                content=[
                    {
                        "type": "text", 
                        "text": "로봇의 전방 카메라에 찍힌 사진입니다. 현재 앞에 무엇이 보이는지 한국어로 정중하고 친근하게 설명해주세요."
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"},
                    },
                ]
            )
            response = self.llm.invoke([message])
            return response.content
        except Exception as e:
            self.get_logger().error(f"OpenAI call failed: {e}")
            return f"OpenAI API 호출 중 오류가 발생했습니다: {e}"

    def process_query(self, query):
        routed = self.route_destination_prompt(query)
        if routed is not None:
            return routed

        # session_id 기반으로 대화 히스토리를 유지합니다.
        config = {"configurable": {"thread_id": "storagy"}}
        result = self.agent_graph.invoke(
            {"messages": [{"role": "user", "content": query}]},
            config=config,
        )
        # 최종 AI 메시지를 반환
        messages = result.get("messages", [])
        if messages:
            return messages[-1].content
        return str(result)

    def handle_question(self, request, response):
        self.get_logger().info(f"💬: {request.question}"+"\n")
        try:
            answer = self.process_query(request.question)
            response.answer = answer
        except Exception as e:
            self.get_logger().info(str(e))
            response.answer = "잘 이해하지 못했어요.. 자세하게 물어봐 주시겠어요?"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    agent = AgentLLM()
    try:
        rclpy.spin(agent) 
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
