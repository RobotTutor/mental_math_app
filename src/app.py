import random
from flask import Flask, render_template, request, jsonify, redirect, url_for
import json
import time
import requests  # Add the requests module
import socket

app = Flask(__name__, template_folder='../templates', static_folder='../static')

# Variables for score, difficulty, mode, correct answer, and timer
score = 0
difficulty = 1  # Start with smaller numbers
correct_answer = None
current_trick = None
MODE = "DEMO MODE"  # Default mode
user_start_time = None  # Track the start time for each question
question_category = 'simple_division'  # Default category


# Function to send result to external IP
def send_result_to_ip(result):
    ip_address = '192.168.0.101'  # IP address of the receiver
    port = 65432  # Port that the receiver is listening on

    try:
        # Create a socket object
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Connect to the receiver
            s.connect((ip_address, port))
            # Send the result (convert to bytes before sending)
            s.sendall(str(result).encode())

            # Optionally, receive an echo response from the server
            data = s.recv(1024)
            print(f"Received echo: {data.decode()}")

    except Exception as e:
        print(f"Error sending result to {ip_address}: {e}")


@app.route('/submit', methods=['POST'])
def submit_answer():
    global score, difficulty, correct_answer, MODE, current_trick
    answer = request.form.get('answer')

    correct = (answer.lower() == correct_answer.lower())

    if correct:
        score += 1 if difficulty == 1 else 2 if difficulty == 2 else 3
        result = 'Correct! 😊'
        hint = None  # No trick needed for a correct answer
        send_result_to_ip(1)  # Send 'correct' to the external IP
    else:
        score = max(0, score - 1)
        result = 'Incorrect! 😢'
        send_result_to_ip(0)  # Send 'incorrect' to the external IP

        # If the answer is incorrect, always show a trick in DEMO MODE
        if MODE == "DEMO MODE":
            if current_trick is not None:
                hint = current_trick['steps'].get('step1', 'No hint available.')
            else:
                hint = 'No hint available.'

    # Generate new question for both correct and incorrect answers
    if question_category == 'simple_division':
        question, correct_answer, current_trick = generate_valid_division_question(difficulty)
    else:
        question, correct_answer, current_trick = generate_multi_division_question(difficulty)

    # Restart the timer for the next question
    start_question_timer()

    return jsonify({
        'result': result,
        'hint': hint,  # Send the hint back if answer is incorrect
        'score': score,
        'question': question,
        'correct_answer': correct_answer,
        'animate': current_trick['animation'] if current_trick else False
    })

def generate_valid_division_question(difficulty_level):
    global correct_answer, current_trick

    if difficulty_level == 1:
        num1 = random.randint(100, 200)
    elif difficulty_level == 2:
        num1 = random.randint(200, 500)
    else:
        num1 = random.randint(500, 1000)

    divisors = [3, 5, 6, 9]
    valid_divisors = [d for d in divisors if num1 % d == 0]

    if not valid_divisors:
        return generate_valid_division_question(difficulty_level)

    div = random.choice(valid_divisors)
    question = f"What is {num1} divided by {div}?"
    correct_answer = str(num1 // div)

    # Better Trick Explanation with step-by-step calculation for each divisor
    if div == 3:
        trick_steps = {
            'step1': f"Sum the digits of {num1}. In this case, {sum([int(digit) for digit in str(num1)])}. Since this sum is divisible by 3, {num1} is divisible by 3.",
            'step2': f"Break down the number {num1} for easier division. Divide the hundreds place separately. For example, divide {num1 - (num1 % 100)} by 3, which equals {(num1 - (num1 % 100)) // 3}.",
            'step3': f"Now divide the remaining {num1 % 100} by 3, which gives {(num1 % 100) // 3}.",
            'step4': f"Add the two results to get the final answer: {correct_answer}."
        }
    elif div == 5:
        trick_steps = {
            'step1': f"Look at the last digit of {num1}. If it's 0 or 5, then it's divisible by 5. The last digit of {num1} is {str(num1)[-1]}.",
            'step2': f"Divide the hundreds and tens first. For example, divide {num1 - (num1 % 10)} by 5 to get {(num1 - (num1 % 10)) // 5}.",
            'step3': f"Then, divide the remainder {num1 % 10} by 5. The remainder is {num1 % 10}, and {num1 % 10} ÷ 5 = {(num1 % 10) // 5}.",
            'step4': f"Add the results: {(num1 - (num1 % 10)) // 5} + {(num1 % 10) // 5} = {correct_answer}."
        }
    elif div == 6:
        trick_steps = {
            'step1': f"Check divisibility by both 2 (last digit even) and 3 (sum of digits divisible by 3). Since both conditions are true, {num1} is divisible by 6.",
            'step2': f"Divide {num1} by 6. Start by dividing {num1 - (num1 % 10)} by 6 to get {(num1 - (num1 % 10)) // 6}.",
            'step3': f"Then, divide the remainder {num1 % 10} by 6 to get {(num1 % 10) // 6}.",
            'step4': f"Add the results: {(num1 - (num1 % 10)) // 6} + {(num1 % 10) // 6} = {correct_answer}."
        }
    elif div == 9:
        trick_steps = {
            'step1': f"Sum the digits of {num1}: {sum([int(digit) for digit in str(num1)])}. Since the sum is divisible by 9, {num1} is divisible by 9.",
            'step2': f"Start by dividing the hundreds place. Divide {num1 - (num1 % 100)} by 9 to get {(num1 - (num1 % 100)) // 9}.",
            'step3': f"Now divide the remainder {num1 % 100} by 9 to get {(num1 % 100) // 9}.",
            'step4': f"Add the two results together to get the final answer: {correct_answer}."
        }

    current_trick = {'steps': trick_steps, 'animation': True}
    return question, correct_answer, current_trick

# Function to generate questions testing divisibility by multiple divisors (e.g., 3 and 5, 5 and 9)
def generate_multi_division_question(difficulty_level):
    global correct_answer, current_trick

    if difficulty_level == 1:
        num1 = random.randint(100, 200)
    elif difficulty_level == 2:
        num1 = random.randint(200, 500)
    else:
        num1 = random.randint(500, 1000)

    # Choose a random combination of two divisors from [3, 5, 6, 9]
    divisor_combinations = [(3, 5), (5, 9), (3, 6), (6, 9)]
    div1, div2 = random.choice(divisor_combinations)

    if num1 % div1 == 0 and num1 % div2 == 0:
        question = f"Is {num1} divisible by both {div1} and {div2}?"
        correct_answer = 'Yes'
        trick_steps = {
            'step1': f"Check if {num1} is divisible by {div1} (e.g., by summing the digits or checking the last digit).",
            'step2': f"Then check if it’s divisible by {div2}. If both conditions are met, {num1} is divisible by both {div1} and {div2}. Answer: Yes."
        }
    else:
        question = f"Is {num1} divisible by both {div1} and {div2}?"
        correct_answer = 'No'
        trick_steps = {
            'step1': f"Check if {num1} is divisible by {div1}. If not, it’s not divisible by both.",
            'step2': f"Even if it is divisible by {div1}, check if it’s divisible by {div2}. If not, the answer is No."
        }

    current_trick = {'steps': trick_steps, 'animation': True}
    return question, correct_answer, current_trick


# Track user progress and adjust difficulty
def update_user_difficulty(correct):
    global difficulty
    if MODE == "DEMO MODE":
        if correct:
            difficulty = min(3, difficulty + 1)
        else:
            difficulty = max(1, difficulty - 1)


# Start timer for the question
def start_question_timer():
    global user_start_time
    user_start_time = time.time()


# Check if user has taken too long to answer (for DEMO mode)
def check_user_time():
    if user_start_time:
        elapsed_time = time.time() - user_start_time
        return elapsed_time > 3  # If more than 10 seconds, show hint
    return False


@app.route('/')
def index():
    global difficulty, question_category
    if question_category == 'simple_division':
        question, correct_answer, current_trick = generate_valid_division_question(difficulty)
    else:
        question, correct_answer, current_trick = generate_multi_division_question(difficulty)
    
    start_question_timer()  # Start timer when question is generated
    return render_template('index.html', question=question, correct_answer=correct_answer, score=score, mode=MODE)

@app.route('/reset', methods=['GET'])
def reset():
    global score, difficulty, correct_answer, current_trick
    score = 0
    difficulty = 1  # Reset difficulty
    correct_answer = None
    current_trick = None
    return redirect(url_for('index'))


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
