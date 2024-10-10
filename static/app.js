document.addEventListener("DOMContentLoaded", function () {
    const submitButton = document.getElementById("submit-btn");
    const resultText = document.getElementById("result");
    const trickText = document.getElementById("trick");
    const scoreText = document.getElementById("score");

    submitButton.addEventListener("click", function (event) {
        event.preventDefault();
        const answer = document.getElementById("answer").value;

        fetch('/submit', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded',
            },
            body: new URLSearchParams({
                'answer': answer
            })
        })
            .then(response => response.json())
            .then(data => {
                resultText.innerHTML = `<p>${data.result}</p>`;
                scoreText.innerHTML = `Score: ${data.score}`;
                document.getElementById('problem-text').innerHTML = `<p>${data.question}</p>`;
                document.querySelector('input[name="correct_answer"]').value = data.correct_answer;

                // If there's a hint (in DEMO mode), display it automatically
                if (data.hint) {
                    trickText.innerHTML = `
                    <p class="step">${data.hint}</p>
                `;

                    // Add animation for the trick explanation
                    anime({
                        targets: '#trick .step',
                        translateY: [50, 0],
                        opacity: [0, 1],
                        duration: 800,
                        easing: 'easeOutElastic'
                    });
                } else {
                    trickText.innerHTML = ''; // Clear the trick if no hint
                }
            });
    });
});
