document.addEventListener("DOMContentLoaded", function () {
    const submitButton = document.getElementById("submit-btn");
    const showTrickButton = document.getElementById("show-trick-btn");
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
            });
    });

    showTrickButton.addEventListener("click", function () {
        fetch('/get_trick', { method: 'GET' })
        .then(response => response.json())
        .then(data => {
            const trickText = document.getElementById('trick');
            
            trickText.innerHTML = `
                <p class="step">1. ${data.trick.step1}</p>
                <p class="step">2. ${data.trick.step2}</p>
                <p class="step">3. ${data.trick.step3}</p>
                <p class="step">4. ${data.trick.step4}</p>
            `;
            
            // Add a fun animation for the trick explanation
            anime({
                targets: '#trick .step',
                translateY: [50, 0],
                opacity: [0, 1],
                duration: 800,
                delay: anime.stagger(200),
                easing: 'easeOutElastic'
            });
        });
});

    // Fetch and display recommendations
    fetch('/get_recommendations')
        .then(response => response.json())
        .then(data => {
            document.getElementById('recommendations').innerHTML = `
                <h4>Training Focus</h4>
                <p>${data.message}</p>
            `;
        });
});
