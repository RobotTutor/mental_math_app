import random
import rclpy
from rclpy.node import Node
from flask import Flask, render_template, request, jsonify, redirect
from src.robot_interface import send_feedback_to_robot
from std_msgs.msg import String
import json

app = Flask(__name__, static_folder='../static', template_folder='../templates')

# Variables for score and difficulty
score = 0
difficulty = 2  # Start at medium level
correct_answer = None
current_trick = None

# Placeholder for user data
user_data = {
    'total_questions': 0,
    'correct_answers': 0,
    'incorrect_answers': 0,
    'trick_views': 0,
    'answers_per_difficulty': {2: {'correct': 0, 'incorrect': 0}, 3: {'correct': 0, 'incorrect': 0}},
    'recommendations': {
        'focus_on': 'General math practice',
        'message': 'Keep practicing to improve your skills!'
    }
}

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

# Randomly generate question based on difficulty and match the appropriate trick
def generate_question_and_trick(difficulty_level):
    global correct_answer, current_trick
    if difficulty_level == 2:
        num1 = random.randint(20, 50)
        num2 = random.randint(80, 100)
        question = f"{num1} * {num2}"
        correct_answer = str(num1 * num2)
        tens1, units1 = divmod(num1, 10)
        tens2, units2 = divmod(num2, 10)
        # Dynamically generate the trick steps
        trick_steps = {
            'step1': f"Break down the numbers: {num1} = {tens1} * 10 + {units1}, {num2} = {tens2} * 10 + {units2}.",
            'step2': f"Apply distributive property: ({tens1} * 10 + {units1}) * ({tens2} * 10 + {units2}) = "
                     f"({tens1} * {tens2} * 100) + ({tens1} * {units2} * 10) + ({units1} * {tens2} * 10) + ({units1} * {units2}).",
            'step3': f"Multiply each part: {tens1} * {tens2} * 100 = {tens1 * tens2 * 100}, "
                     f"{tens1} * {units2} * 10 = {tens1 * units2 * 10}, "
                     f"{units1} * {tens2} * 10 = {units1 * tens2 * 10}, "
                     f"{units1} * {units2} = {units1 * units2}.",
            'step4': f"Add them: {tens1 * tens2 * 100} + {tens1 * units2 * 10} + {units1 * tens2 * 10} + {units1 * units2} = {correct_answer}."
        }
        current_trick = trick_steps

    elif difficulty_level == 3:
        num1 = random.randint(100, 999)
        num2 = random.randint(50, 99)
        question = f"{num1} - {num2}"
        correct_answer = str(num1 - num2)
        current_trick = {
            'step1': f"Round {num1} to the nearest hundred and {num2} to the nearest ten.",
            'step2': f"Subtract the rounded numbers and adjust for the difference.",
            'step3': f"After rounding, subtract and adjust for the rounding error.",
            'step4': f"Your final answer is {correct_answer}."
        }

    return question, correct_answer, current_trick

# Track user progress and save data
def update_user_data(correct, difficulty):
    global user_data
    user_data['total_questions'] += 1
    
    if correct:
        user_data['correct_answers'] += 1
        user_data['answers_per_difficulty'][difficulty]['correct'] += 1
    else:
        user_data['incorrect_answers'] += 1
        user_data['answers_per_difficulty'][difficulty]['incorrect'] += 1
    
    generate_training_suggestions()

# Analyze data and generate training suggestions
def generate_training_suggestions():
    global user_data
    if user_data['answers_per_difficulty'][2]['incorrect'] > 2:
        user_data['recommendations'] = {
            "focus_on": "Distributive property multiplication",
            "message": "You answered distributive property questions incorrectly multiple times. Review the steps and tricks provided to improve."
        }
    elif user_data['answers_per_difficulty'][3]['incorrect'] > 2:
        user_data['recommendations'] = {
            "focus_on": "Advanced subtraction tricks",
            "message": "You seem to struggle with subtraction. Review the steps to improve your performance."
        }
    else:
        user_data['recommendations'] = {
            "focus_on": "Keep practicing",
            "message": "You are doing well! Keep practicing to master all levels."
        }

# Save user data
def save_user_data():
    with open('user_data.json', 'w') as f:
        json.dump(user_data, f)

@app.route('/')
def index():
    global difficulty
    question, correct_answer, current_trick = generate_question_and_trick(difficulty)
    return render_template('index.html', question=question, correct_answer=correct_answer, score=score)

@app.route('/submit', methods=['POST'])
def submit_answer():
    global score, difficulty, correct_answer
    answer = request.form.get('answer')
    
    correct = (answer == correct_answer)
    update_user_data(correct, difficulty)

    if correct:
        if difficulty == 2:
            score += 2
        else:
            score += 3
        
        if score % 5 == 0 and difficulty < 3:
            difficulty += 1
        result = 'Correct! 😊'
    else:
        score = max(0, score - 1)
        if difficulty > 2:
            difficulty -= 1
        result = 'Incorrect! 😢'

    question, correct_answer, current_trick = generate_question_and_trick(difficulty)
    save_user_data()

    return jsonify({
        'result': result,
        'score': score,
        'difficulty': difficulty,
        'question': question,
        'correct_answer': correct_answer
    })

@app.route('/get_trick', methods=['GET'])
def get_trick():
    global current_trick

    # If the trick explanation is already generated (stored in current_trick), return it
    if current_trick:
        return jsonify({'trick': current_trick})
    
    # If no trick is available, return an error message
    return jsonify({'error': 'No trick available'}), 400

@app.route('/reset')
def reset():
    global score, difficulty, correct_answer, current_trick
    
    # Resetting variables to initial state
    score = 0
    difficulty = 2
    correct_answer = None
    current_trick = None
    
    user_data['total_questions'] = 0
    user_data['correct_answers'] = 0
    user_data['incorrect_answers'] = 0
    user_data['trick_views'] = 0
    user_data['answers_per_difficulty'] = {2: {'correct': 0, 'incorrect': 0}, 3: {'correct': 0, 'incorrect': 0}}
    
    # Redirecting to home page
    return redirect('/')

@app.route('/get_recommendations', methods=['GET'])
def get_recommendations():
    return jsonify(user_data['recommendations'])

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
