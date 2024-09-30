document.addEventListener("DOMContentLoaded", function () {
    const submitButton = document.getElementById("submit-btn");
    const showTrickButton = document.getElementById("show-trick-btn");
    const problemText = document.getElementById("problem-text");
    const answerInput = document.getElementById("answer");
    const resultText = document.getElementById("result");
    const trickText = document.getElementById("trick");
    const scoreText = document.getElementById("score");

    submitButton.addEventListener("click", function (event) {
        event.preventDefault();
        const answer = answerInput.value;

        fetch('/submit', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded',
            },
            body: new URLSearchParams({
                'answer': answer,
                'correct_answer': document.querySelector('input[name="correct_answer"]').value
            })
        })
            .then(response => response.json())
            .then(data => {
                resultText.innerHTML = `<span class="result-message">${data.result}</span>`;
                scoreText.innerHTML = `Score: ${data.score}`;
                problemText.innerHTML = `<h3>Math Problem:</h3><p>${data.question}</p>`;
                answerInput.value = '';
                document.querySelector('input[name="correct_answer"]').value = data.correct_answer;

                if (data.result.includes("Correct")) {
                    resultText.classList.add("correct");
                    resultText.classList.remove("incorrect");
                } else {
                    resultText.classList.add("incorrect");
                    resultText.classList.remove("correct");
                }
            })
            .catch(error => console.error('Error submitting answer:', error));
    });

    showTrickButton.addEventListener("click", function () {
        fetch('/get_trick', { method: 'GET' })
            .then(response => response.json())
            .then(data => {
                trickText.innerHTML = `<p>${data.trick}</p>`;
                animateTrick();
            })
            .catch(error => console.error('Error fetching trick:', error));
    });

    function animateTrick() {
        anime({
            targets: '#trick p',
            translateY: [50, 0],
            opacity: [0, 1],
            duration: 800,
            easing: 'easeOutElastic'
        });
    }
});
