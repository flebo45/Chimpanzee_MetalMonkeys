const socket = io();
let timeIndex = 0;
const TARGET_AREA = 25000;

// --- CONFIGURAZIONE GRAFICI ---
// maintainAspectRatio: false è il segreto per farli stare nella card!
const commonOptions = {
    responsive: true,
    maintainAspectRatio: false, 
    animation: false,
    interaction: { mode: 'index', intersect: false },
    scales: {
        x: { display: false },
        y: { type: 'linear', display: true, position: 'left', title: { display: true, text: 'Error' } },
        y1: { type: 'linear', display: true, position: 'right', grid: { drawOnChartArea: false }, title: { display: true, text: 'Vel' } }
    },
    plugins: {
        legend: {
            labels: { color: '#bbb', boxWidth: 10, padding: 10 }
        }
    }
};

// Inizializza Grafico Angolare
const angChart = new Chart(document.getElementById('telemetryChart').getContext('2d'), {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            { label: 'Vis. Error X', borderColor: '#ff6384', borderWidth: 2, yAxisID: 'y', data: [], pointRadius: 0, tension: 0.1 },
            { label: 'Ang. Vel Z', borderColor: '#36a2eb', borderWidth: 2, yAxisID: 'y1', data: [], pointRadius: 0, tension: 0.1 }
        ]
    },
    options: commonOptions
});

// Inizializza Grafico Lineare
const linChart = new Chart(document.getElementById('linearChart').getContext('2d'), {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            { label: 'Dist. Error', borderColor: '#ff9f40', borderWidth: 2, yAxisID: 'y', data: [], pointRadius: 0, tension: 0.1 },
            { label: 'Lin. Vel X', borderColor: '#4bc0c0', borderWidth: 2, yAxisID: 'y1', data: [], pointRadius: 0, tension: 0.1 }
        ]
    },
    options: commonOptions
});

// --- SOCKET EVENTI ---

socket.on('connect', () => {
    document.getElementById('connectionStatus').className = "badge bg-success";
    document.getElementById('connectionStatus').innerText = "ONLINE";
});

socket.on('disconnect', () => {
    document.getElementById('connectionStatus').className = "badge bg-danger";
    document.getElementById('connectionStatus').innerText = "OFFLINE";
});

socket.on('update_video', (data) => {
    document.getElementById('videoFeed').src = "data:image/jpeg;base64," + data;
});

socket.on('update_gui', (data) => {
    let distError = TARGET_AREA - data.area; 
    let batteryLevel = data.battery_level;
    

    // Battery Update
    const batteryBadge = document.getElementById('batteryLevel');
    batteryBadge.innerText = batteryLevel.toFixed(0) + "%";
    if (batteryLevel > 50) {
        batteryBadge.className = "badge bg-success";
    } else if (batteryLevel > 20) {
        batteryBadge.className = "badge bg-warning text-dark";
    } else {
        batteryBadge.className = "badge bg-danger";
    }

    // UI Update
    const badge = document.getElementById('targetBadge');
    if (data.visible) {
        badge.innerText = "TRACKING";
        badge.className = "badge bg-success fs-6 px-3 py-2";
    } else {
        badge.innerText = "SEARCHING";
        badge.className = "badge bg-warning text-dark fs-6 px-3 py-2";
    }

    const dist = parseFloat(data.distance);
    document.getElementById('distValue').innerText = dist.toFixed(2) + " m";
    const bar = document.getElementById('safetyBar');
    let percentage = Math.min((dist / 2.0) * 100, 100);
    bar.style.width = percentage + "%";
    
    if (dist < 0.6) bar.className = "progress-bar bg-danger progress-bar-striped progress-bar-animated";
    else if (dist < 1.0) bar.className = "progress-bar bg-warning";
    else bar.className = "progress-bar bg-success";

    // Chart Update
    if (angChart.data.labels.length > 100) {
        angChart.data.labels.shift();
        angChart.data.datasets[0].data.shift();
        angChart.data.datasets[1].data.shift();
        linChart.data.labels.shift();
        linChart.data.datasets[0].data.shift();
        linChart.data.datasets[1].data.shift();
    }

    angChart.data.labels.push(timeIndex);
    linChart.data.labels.push(timeIndex);
    timeIndex++;

    angChart.data.datasets[0].data.push(data.error_x);
    angChart.data.datasets[1].data.push(data.cmd_vel_z);
    
    linChart.data.datasets[0].data.push(distError);
    linChart.data.datasets[1].data.push(data.cmd_vel_x);

    angChart.update();
    linChart.update();
});