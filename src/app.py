import random
import rclpy
from rclpy.node import Node
from flask import Flask, render_template, request, redirect, url_for, jsonify
from src.robot_interface import send_feedback_to_robot
from std_msgs.msg import String

app = Flask(__name__, static_folder='../static', template_folder='../templates')

score = 0
difficulty = 1
correct_answer = None

# ROS2 Node for publishing to /face_state
class MathPublisher(Node):
    def __init__(self):
        super().__init__('math_publisher')
        self.publisher_ = self.create_publisher(String, 'face_state', 10)

    def publish_result(self, result):
        msg = String()
        msg.data = result
        self.publisher_.publish(msg)

rclpy.init()
ros_node = MathPublisher()

def generate_question(difficulty_level):
    global correct_answer
    if difficulty_level == 1:
        num1 = random.randint(1, 10)
        num2 = random.randint(1, 10)
    elif difficulty_level == 2:
        num1 = random.randint(10, 50)
        num2 = random.randint(10, 50)
    else:
        num1 = random.randint(50, 100)
        num2 = random.randint(50, 100)

    operation = random.choice(['+', '-', '*'])
    question = f"{num1} {operation} {num2}"
    correct_answer = str(eval(question))
    return question, correct_answer

@app.route('/')
def index():
    global difficulty
    question, correct_answer = generate_question(difficulty)
    return render_template('index.html', question=question, correct_answer=correct_answer, score=score)

@app.route('/submit', methods=['POST'])
def submit_answer():
    global score, difficulty, correct_answer
    answer = request.form.get('answer')

    if answer == correct_answer:
        if difficulty == 1:
            score += 1  # 1 point for easy
        elif difficulty == 2:
            score += 2  # 2 points for medium
        else:
            score += 3  # 3 points for hard

        if score % 3 == 0 and difficulty < 3:
            difficulty += 1 
        result = 'Correct! 😊'
    else:
        score = max(0, score - 1)
        if difficulty > 1:
            difficulty -= 1
        result = 'Incorrect! 😢'

    # Generate a new question for the next round
    question, correct_answer = generate_question(difficulty)
    
    return jsonify({
        'result': result,
        'score': score,
        'difficulty': difficulty,
        'question': question,
        'correct_answer': correct_answer
    })

@app.route('/reset')
def reset():
    global score, difficulty
    score = 0
    difficulty = 1
    return redirect(url_for('index'))

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
